/**
 ******************************************************************************
 * @file    can_dti.h
 * @brief   Public interface for the DTI HV500 CAN decoder module.
 *
 *  This module owns:
 *    - vCanDecodeTask()  : FreeRTOS task that drains xCanRxQueue and populates
 *                          the global dti_data struct.
 *    - DtiData_t         : Consolidated struct replacing ~30 individual globals.
 *    - CAN_Msg_t         : Queue payload type shared with main.c ISR.
 *
 *  Packet mapping (DTI CANBus protocol v2.5):
 *    0x1F >> General Data 6  (control mode, target Iq, motor position)
 *    0x20 >> General Data 1  (ERPM, duty, DC bus voltage)
 *    0x21 >> General Data 2  (AC current, DC current)
 *    0x22 >> General Data 3  (temperatures, fault code)
 *    0x23 >> General Data 4  (Id, Iq flux/torque currents)
 *    0x24 >> General Data 5  (throttle, brake, digital I/O, limits)
 ******************************************************************************
 */

#ifndef CAN_DTI_H
#define CAN_DTI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "main.h"          /* HAL types, CAN_HandleTypeDef                   */
#include "FreeRTOS.h"
#include "queue.h"

/* ============================================================================
 * QUEUE MESSAGE TYPE
 * Shared between the CAN RX ISR (main.c) and vCanDecodeTask (can_dti.c).
 * Declared here so that main.c can #include this header and the type is
 * defined in exactly one place.
 * ========================================================================= */
typedef struct {
    uint32_t StdId;    /**< CAN Standard ID (11-bit).                        */
    uint8_t  Data[8];  /**< Raw payload bytes.                                */
} CAN_Msg_t;

/* ============================================================================
 * DTI CONSOLIDATED DATA STRUCT
 *
 * Previously ~30 individual volatile globals (dti_erpm, dti_duty, etc.).
 * Grouping them here:
 *   1. Reduces namespace pollution.
 *   2. Makes the live-watch debugger view cleaner (single struct expansion).
 *   3. Enables atomic snapshot copies in future with a mutex if needed.
 * ========================================================================= */
typedef struct {

    /* --- Packet 0x1F : General Data 6 --- */
    uint8_t  control_mode;    /**< 0=Current, 1=Speed, 2=Duty, 3=Position.  */
    float    target_iq;       /**< Target quadrature current [A].            */
    float    motor_pos;       /**< Rotor position [degrees].                 */
    uint8_t  is_still;        /**< 1 if motor is stationary.                 */

    /* --- Packet 0x20 : General Data 1 --- */
    int32_t  erpm;            /**< Electrical RPM (mechanical = erpm/poles). */
    float    duty;            /**< Duty cycle [%].                           */
    int16_t  dc_volt;         /**< DC bus voltage [V].                       */

    /* --- Packet 0x21 : General Data 2 --- */
    float    ac_curr;         /**< AC (phase) current RMS [A].               */
    float    dc_curr;         /**< DC bus current [A].                       */

    /* --- Packet 0x22 : General Data 3 --- */
    float    temp_ctrl;       /**< Controller (IGBT) temperature [°C].       */
    float    temp_motor;      /**< Motor temperature [°C].                   */
    uint8_t  fault_code;      /**< DTI fault code (0 = no fault).            */

    /* --- Packet 0x23 : General Data 4 --- */
    float    id;              /**< D-axis (flux) current [A].                */
    float    iq;              /**< Q-axis (torque) current [A].              */

    /* --- Packet 0x24 : General Data 5 --- */
    int8_t   throttle;        /**< Throttle position echoed by DTI [-128,127]. */
    int8_t   brake;           /**< Brake position echoed by DTI [-128,127].  */

    /* Digital I/O (Byte 2 of 0x24) */
    uint8_t  dig_in_1;
    uint8_t  dig_in_2;
    uint8_t  dig_in_3;
    uint8_t  dig_in_4;
    uint8_t  dig_out_1;
    uint8_t  dig_out_2;
    uint8_t  dig_out_3;
    uint8_t  dig_out_4;

    /* Drive Enable (Byte 3 of 0x24) */
    uint8_t  drive_enable;    /**< 1 if DTI reports drive enable active.     */

    /* Active Limits (Bytes 4-5 of 0x24) */
    uint8_t  lim_dc_curr;
    uint8_t  lim_motor_temp;

    /* Map version (Byte 7 of 0x24) */
    uint8_t  map_ver;

} DtiData_t;

/* ============================================================================
 * GLOBAL INSTANCE DECLARATION
 * Defined (allocated) in can_dti.c; all other modules use 'extern'.
 * ========================================================================= */
extern volatile DtiData_t dti_data;

/* ============================================================================
 * EXTERN HANDLES (owned/created in main.c)
 * ========================================================================= */
extern QueueHandle_t xCanRxQueue;  /**< CAN RX FIFO queue; fed by ISR.     */

/* ============================================================================
 * PUBLIC FUNCTION PROTOTYPES
 * ========================================================================= */

/**
 * @brief  FreeRTOS task: drains xCanRxQueue and decodes DTI CAN frames.
 * @note   Stack: 512 words.  Priority: 2 (normal).
 *         Create with: xTaskCreate(vCanDecodeTask, "DTI_Dec", 512, NULL, 2, &xCanTaskHandle);
 * @param  pvParameters  Unused (pass NULL).
 */
void vCanDecodeTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DTI_H */
