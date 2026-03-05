/**
 ******************************************************************************
 * @file    vcu_safety.h
 * @brief   Public interface for the VCU Safety module.
 *
 *  This module owns three safety-critical functions:
 *
 *  1. APPS PLAUSIBILITY (Process_APPS_Safety_Logic)
 *     Implements Rules T11.8.8 / T11.8.9 – redundant pedal sensor checking.
 *
 *  2. BRAKE LIGHT CONTROLLER & IMU (vBrakeLightTask)
 *     Implements Rule T6.3.1 – brake light must activate on hydraulic brake
 *     OR regenerative deceleration > 1 m/s².
 *
 *  3. READY-TO-DRIVE SUPERVISOR (vR2DLogicTask)
 *     Implements Rules EV4.11.7 and EV4.12.1 – R2D activation sequence and
 *     mandatory acoustic signal.
 *
 *  Dependency overview:
 *    - Reads adc_dma_buffer[]  (owned by DMA, started in main.c).
 *    - Writes vcu_data         (global state struct defined here).
 *    - Receives task notifications from HAL_I2C_MemRxCpltCallback (main.c).
 *    - Controls BRAKE_LIGHT (PB12), DRIVE_ENABLE (PB15), FAULT_LED.
 *    - Drives TIM3_CH1 for RTD buzzer (sole owner of TIM3).
 ******************************************************************************
 */

#ifndef VCU_SAFETY_H

#define VCU_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* ============================================================================
 * SAFETY THRESHOLDS
 * Centralised here so that future team members can locate and modify them
 * without hunting through task implementations.
 * ========================================================================= */

/** Rule T11.8.8/T11.8.9 – APPS implausibility threshold (10% of full scale). */
#define APPS_IMPLAUSIBILITY_THRESHOLD   0.10f

/** Rule T11.8.9 – Grace period before torque cut [ms]. Must be > 100 ms. */
#define APPS_CONFLICT_TIMER_MS          100U

/** Rule EV2.3.1 – Simultaneous brake+throttle (> 25%) threshold [%].      */
#define BRAKE_PLAUSIBILITY_THRESHOLD    25.0f

/** Rule EV2.3.1 – Brake plausibility persist time before torque cut [ms].  */
#define BRAKE_PLAUSIBILITY_TIMER_MS     500U

/** Rule EV2.3.2 – APPS must fall below 5% before torque is restored [%].  */
#define BRAKE_RESET_THRESHOLD           5.0f

/** Hydraulic brake detection raw ADC threshold (~0.1 V on 12-bit ADC).    */
#define HYDRAULIC_PRESS_THRESHOLD       50U

/** Rule T6.3.1 – Regen deceleration threshold for brake light [g].
 *  1.0 m/s² = 0.102g.  Hysteresis prevents rapid toggling.                */
#define BRAKE_DECEL_THRESHOLD_G         0.102f
#define BRAKE_DECEL_HYSTERESIS_G        0.020f

/** APPS low-pass filter coefficient (0.1 = very smooth, 1.0 = raw).       */
#define APPS_FILTER_ALPHA               0.2f

/** IMU low-pass filter coefficient.                                         */
#define ACCEL_FILTER_ALPHA              0.1f

/** Throttle deadzone for regen brake light detection [%].
 *  If both APPS sensors read below this value the driver is not pressing the
 *  throttle. Used only to qualify the IMU-based regen brake light condition.
 *  The VCU does not send torque commands — the DTI controls its own output. */     */
#define APPS_DEADZONE_PERCENT           5.0f

/* --- APPS ADC calibration (tune to hardware) ----------------------------- */
#define APPS1_MIN_ADC   0
#define APPS1_MAX_ADC   4000
#define APPS2_MIN_ADC   0
#define APPS2_MAX_ADC   4000

/* --- R2D / SAFETY PIN MAPPING --------------------------------------------
 *
 * NO hardcoded GPIO defines here. All pin assignments are owned by the
 * STM32CubeIDE .ioc file and auto-generated into main.h as:
 *
 *   #define <LABEL>_Pin         GPIO_PIN_x
 *   #define <LABEL>_GPIO_Port   GPIOy
 *
 * This module uses those generated names directly. Never redeclare them here —
 * doing so risks silently overriding the .ioc value with a stale constant.
 *
 * Current pin assignments (from .ioc — update comment here if .ioc changes):
 *   RTD_BUTTON   → PB14   input,  GPIO_PULLDOWN
 *   DRIVE_ENABLE → PB15   output, active HIGH
 *   SDC_SENSE    → PA9    input,  GPIO_PULLDOWN  (Rule EV4.11.8)
 *   BRAKE_LIGHT  → PB12   output, active HIGH
 *   FAULT_LED    → PA1    output, active HIGH
 *
 * SDC_SENSE pull-down rationale (Rule EV4.11.8):
 *   SDC closed (normal) → upstream circuit drives PA9 HIGH → VCU reads HIGH.
 *   SDC open   (fault)  → PA9 floats → pull-DOWN pulls to GND → VCU reads LOW
 *                       → DRIVE_ENABLE immediately pulled LOW.
 *   Broken sense wire   → PA9 floats → pull-DOWN → LOW → same safe response.
 *   A wiring fault cannot hold the inverter enabled.
 * ----------------------------------------------------------------------- */

/**
 * Rule EV4.12.1 – RTD sound: "continuously for at least 1 s and a
 * maximum of 3 s". We use 2 s (compliant middle ground).
 */
#define RTD_SOUND_DURATION_MS  2000U

/**
 * TIM3 PWM period for 2 kHz buzzer tone.
 * TIM3 clock: 72 MHz / (PSC+1 = 72) = 1 MHz tick.
 * 2 kHz → ARR = 1 000 000 / 2000 = 500 ticks.
 */
#define RTD_BUZZER_PWM_PERIOD  500U

/* ============================================================================
 * MPU-6050 CONSTANTS
 * ========================================================================= */
#define MPU6050_ADDR              0xD0   /**< I2C address (AD0=0 → 0x68<<1). */
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_ACCEL_RANGE_2G    0x00
#define MPU6050_ACCEL_SENSITIVITY 16384.0f  /**< LSB/g for ±2g range.       */

/* ============================================================================
 * VCU INTEGRATED SENSOR DATA STRUCT
 * Single source of truth for all run-time vehicle state visible to tasks,
 * telemetry, or a future CAN broadcast task.
 * ========================================================================= */
typedef struct {

    /* --- Raw DMA ADC values ---------------------------------------------- */
    uint16_t adc_raw_apps1;          /**< APPS1 ADC count (rank 1, CH2).    */
    uint16_t adc_raw_apps2;          /**< APPS2 ADC count (rank 2, CH3).    */
    uint16_t adc_raw_pressure;       /**< Brake pressure (rank 3, CH4).     */

    /* --- Processed throttle channels ------------------------------------- */
    float    apps1_percent;          /**< Filtered APPS1 position [%].       */
    float    apps2_percent;          /**< Filtered APPS2 position [%].       */
    float    apps_percent;           /**< Average of both sensors [%].       */

    /* --- IMU (MPU-6050 Y-axis) ------------------------------------------ */
    int16_t  accel_y_raw;            /**< Raw 16-bit ADC reading.            */
    float    accel_y_g;              /**< Converted to g.                    */
    float    accel_y_filtered;       /**< Low-pass filtered [g].             */
    float    deceleration_g;         /**< |accel_y_filtered| for threshold.  */

    /* --- Logic output flags ---------------------------------------------- */
    uint8_t  brake_light_active;         /**< 1 = brake light is on.        */
    uint8_t  apps_fault_active;          /**< 1 = APPS implausibility fault. */
    uint8_t  brake_plausibility_active;  /**< 1 = brake override latched.   */
    char     trigger_source[15];         /**< "HYDRAULIC"/"REGEN_DECEL"/"NONE". */

    /* --- R2D state ------------------------------------------------------- */
    uint8_t  ready_to_drive_active; /**< 0 = standby, 1 = drive enabled.   */
    uint8_t  rtd_button_state;      /**< Raw button reading (1 = pressed).  */
    uint8_t  buzzer_is_dc_mode;     /**< 0 = passive piezo, 1 = active DC. */

    /**
     * Rule EV4.11.8 – Shutdown Circuit state.
     * 0 = SDC closed (normal).  1 = SDC open (fault – inverter must be off).
     * Written by vR2DLogicTask every 20 ms from the SDC_SENSE GPIO.
     * Exposed here for live-watch debugging and future telemetry broadcast.
     */
    uint8_t  shutdown_circuit_open;

} VcuSensorData_t;

/* ============================================================================
 * GLOBAL INSTANCE DECLARATION
 * ========================================================================= */
extern volatile VcuSensorData_t vcu_data;

/* ============================================================================
 * EXTERN HANDLES & BUFFERS (owned by main.c)
 * ========================================================================= */
extern volatile uint16_t adc_dma_buffer[3];   /**< DMA destination buffer.  */
extern I2C_HandleTypeDef hi2c1;               /**< I2C peripheral handle.    */
extern TIM_HandleTypeDef htim3;               /**< TIM3 handle (buzzer PWM). */
extern TaskHandle_t      xBrakeTaskHandle;    /**< For ISR → task notify.    */

/* ============================================================================
 * PUBLIC FUNCTION PROTOTYPES
 * ========================================================================= */

/**
 * @brief  Initialise MPU-6050 accelerometer over I2C (blocking, at boot only).
 * @note   Call from main() before vTaskStartScheduler().
 */
void MPU6050_Init(void);

/**
 * @brief  APPS dual-sensor plausibility check + brake plausibility check.
 *
 *  Rules enforced:
 *    T11.8.8 / T11.8.9 – dual APPS agreement within 10 % for > 100 ms.
 *    EV2.3.1            – simultaneous brake + APPS > 25 % for > 500 ms.
 *    EV2.3.2            – brake latch released only when APPS < 5 %.
 *
 * @note   Called from vBrakeLightTask every 10 ms. Must be < 1 ms execution.
 */
void Process_APPS_Safety_Logic(void);

/**
 * @brief  FreeRTOS task: IMU data acquisition, brake light control, APPS logic.
 * @note   Stack: 256 words. Priority: 3 (safety critical).
 *           xTaskCreate(vBrakeLightTask, "BrakeLight", 256, NULL, 3, &xBrakeTaskHandle);
 *         xBrakeTaskHandle MUST be captured so HAL_I2C_MemRxCpltCallback can
 *         notify it.
 * @param  argument  Unused.
 */
void vBrakeLightTask(void *argument);

/**
 * @brief  FreeRTOS task: Ready-to-Drive supervisor (EV4.11 / EV4.12).
 * @note   Stack: 256 words. Priority: 3 (safety critical).
 *           xTaskCreate(vR2DLogicTask, "R2D_Logic", 256, NULL, 3, &xR2DTaskHandle);
 * @param  argument  Unused.
 */
void vR2DLogicTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* VCU_SAFETY_H */
