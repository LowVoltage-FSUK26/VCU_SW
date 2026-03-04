/**
 ******************************************************************************
 * @file    imd_monitor.h
 * @brief   Public interface for the Bender IMD (Insulation Monitoring Device)
 *          PWM-decoding module.
 *
 *  The Bender ISOMETER encodes isolation resistance onto a PWM signal:
 *    - Frequency encodes operating state (10 / 20 / 30 / 40 / 50 Hz).
 *    - Duty cycle encodes measured insulation resistance (at 10 Hz normal mode).
 *
 *  Hardware:
 *    - TIM2 in PWM Input Capture mode (CH1 rising, CH2 falling).
 *    - Slave-reset mode: CH1 resets counter on rising edge → CCR1 = period,
 *      CCR2 = pulse width. Both updated atomically by hardware.
 *    - TIM2 clock: 72 MHz / (PSC+1=144) = 500 kHz → 2 µs resolution.
 *
 *  Task notification flow:
 *    ISR (HAL_TIM_IC_CaptureCallback) → vTaskNotifyGiveFromISR(xPwmTaskHandle)
 *    StartPwmTask wakes, copies isr_buffer under critical section, processes.
 ******************************************************************************
 */

#ifndef IMD_MONITOR_H
#define IMD_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* ============================================================================
 * IMD STATE ENUMERATION
 * Bender ISOMETER frequency-to-state mapping (from device datasheet).
 * ========================================================================= */
typedef enum {
    IMD_STATE_UNKNOWN      = 0,  /**< Frequency out of all known bands.     */
    IMD_STATE_NORMAL,            /**< 10 Hz – measuring; duty = resistance. */
    IMD_STATE_UNDERVOLTAGE,      /**< 20 Hz – DC bus below threshold.       */
    IMD_STATE_SST,               /**< 30 Hz – Speed Start (self-test).      */
    IMD_STATE_DEVICE_ERROR,      /**< 40 Hz – Internal device error.        */
    IMD_STATE_GROUND_FAULT,      /**< 50 Hz – Isolation fault detected.     */
    IMD_STATE_DISCONNECTED       /**< No signal received within timeout.    */
} ImdState_t;

/* ============================================================================
 * IMD LIVE STATUS STRUCT
 * Written by StartPwmTask; read-only for all other consumers (e.g. telemetry).
 * ========================================================================= */
typedef struct {
    float      Frequency_Hz;        /**< Decoded PWM frequency.              */
    float      Duty_Cycle_Percent;  /**< Decoded duty cycle (0-100 %).       */
    float      Resistance_kOhm;     /**< Calculated isolation resistance.    */
    ImdState_t CurrentState;        /**< Current IMD operating state.        */
    uint32_t   Raw_Period_Ticks;    /**< TIM2 CCR1 – raw period count.       */
    uint32_t   Raw_Pulse_Ticks;     /**< TIM2 CCR2 – raw pulse count.        */
} ImdDebugStatus_t;

/* ============================================================================
 * ISR SHARED BUFFER
 * Written atomically in the TIM2 capture ISR; read under critical section
 * in StartPwmTask to avoid torn reads.
 * ========================================================================= */
typedef struct {
    uint32_t period;   /**< TIM2->CCR1 snapshot (full period in ticks).     */
    uint32_t pulse;    /**< TIM2->CCR2 snapshot (pulse width in ticks).     */
} IsrBuffer_t;

/* ============================================================================
 * GLOBAL INSTANCE DECLARATIONS
 * Both defined in imd_monitor.c.
 * ========================================================================= */
extern volatile ImdDebugStatus_t imd_live_status;
extern volatile IsrBuffer_t      isr_buffer;

/* ============================================================================
 * EXTERN HANDLE (owned by main.c, used by TIM2 ISR in main.c)
 * ========================================================================= */
extern TaskHandle_t xPwmTaskHandle;

/* ============================================================================
 * PUBLIC FUNCTION PROTOTYPES
 * ========================================================================= */

/**
 * @brief  FreeRTOS task: processes TIM2 PWM captures to decode IMD state.
 * @note   Stack: 256 words.  Priority: 2.
 *         xPwmTaskHandle MUST be captured at task creation so the ISR can
 *         notify it:
 *           xTaskCreate(StartPwmTask, "PWM_Proc", 256, NULL, 2, &xPwmTaskHandle);
 * @param  argument  Unused (pass NULL).
 */
void StartPwmTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* IMD_MONITOR_H */
