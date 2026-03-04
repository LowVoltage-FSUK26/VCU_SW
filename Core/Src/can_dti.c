/**
 ******************************************************************************
 * @file    can_dti.c
 * @brief   DTI HV500 CAN decoder – FreeRTOS task implementation.
 *
 *  Packet ID extraction:
 *    The DTI multiplexes multiple packets onto the same physical bus using
 *    the upper 6 bits of the 11-bit Standard ID as a packet type field:
 *      packet_id = StdId >> 5
 *    The lower 5 bits encode the device node address (default 0x00).
 *
 *  Data format conventions (DTI protocol v2.5):
 *    - All multi-byte integers are BIG-ENDIAN (MSB first).
 *    - Signed integers use two's complement.
 *    - Fixed-point scaling is applied after cast to signed type.
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "can_dti.h"
#include "task.h"

/* ============================================================================
 * GLOBAL STRUCT INSTANCE
 * Allocated here; extern declared in can_dti.h for all other translation units.
 * ========================================================================= */
volatile DtiData_t dti_data = {0};

/* ============================================================================
 * TASK IMPLEMENTATION
 * ========================================================================= */

/**
 * @brief  CAN decode task – runs at priority 2, blocks indefinitely on queue.
 *
 * Design notes:
 *   - portMAX_DELAY: task sleeps with zero CPU cost when the bus is idle.
 *   - The LED toggle on PC13 gives a visual heartbeat that CAN frames are
 *     being received without a scope or debugger attached.
 *   - All fields are written atomically (32-bit aligned writes on Cortex-M3
 *     are single-cycle bus transactions; no mutex needed for read-only consumers
 *     that tolerate one frame of latency).
 */
void vCanDecodeTask(void *pvParameters)
{
    CAN_Msg_t rxMsg;

    for (;;)
    {
        /* Block here until a message arrives; releases CPU to other tasks. */
        if (xQueueReceive(xCanRxQueue, &rxMsg, portMAX_DELAY) == pdTRUE)
        {
            /* Visual heartbeat on onboard LED (PC13, active-LOW on Blue Pill). */
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

            /*
             * DTI packet type = upper 6 bits of the 11-bit Standard ID.
             * Lower 5 bits = node address; default node = 0, so:
             *   StdId 0x3E0 >> 5 = 0x1F  (General Data 6)
             *   StdId 0x400 >> 5 = 0x20  (General Data 1)
             *   etc.
             */
            uint8_t packet_id = (uint8_t)(rxMsg.StdId >> 5);

            switch (packet_id)
            {
                /* --------------------------------------------------------
                 * Packet 0x1F – General Data 6 (v2.5)
                 * Byte 0       : Control mode
                 * Bytes 1-2    : Target Iq  [/10 → A]
                 * Bytes 3-4    : Motor pos  [/10 → degrees]
                 * Byte 5       : Is-still flag
                 * ------------------------------------------------------- */
                case 0x1F:
                    dti_data.control_mode = rxMsg.Data[0];
                    dti_data.target_iq    = (float)(int16_t)(rxMsg.Data[1] << 8 | rxMsg.Data[2]) / 10.0f;
                    dti_data.motor_pos    = (float)(int16_t)(rxMsg.Data[3] << 8 | rxMsg.Data[4]) / 10.0f;
                    dti_data.is_still     = rxMsg.Data[5];
                    break;

                /* --------------------------------------------------------
                 * Packet 0x20 – General Data 1
                 * Bytes 0-3    : ERPM          [raw int32, no scaling]
                 * Bytes 4-5    : Duty cycle    [/10 → %]
                 * Bytes 6-7    : DC bus voltage [raw int16, V]
                 * ------------------------------------------------------- */
                case 0x20:
                    dti_data.erpm    = (int32_t)((uint32_t)rxMsg.Data[0] << 24 |
                                                 (uint32_t)rxMsg.Data[1] << 16 |
                                                 (uint32_t)rxMsg.Data[2] <<  8 |
                                                 (uint32_t)rxMsg.Data[3]);
                    dti_data.duty    = (float)(int16_t)(rxMsg.Data[4] << 8 | rxMsg.Data[5]) / 10.0f;
                    dti_data.dc_volt = (int16_t)(rxMsg.Data[6] << 8 | rxMsg.Data[7]);
                    break;

                /* --------------------------------------------------------
                 * Packet 0x21 – General Data 2
                 * Bytes 0-1    : AC (phase) current [/10 → A]
                 * Bytes 2-3    : DC bus current     [/10 → A]
                 * ------------------------------------------------------- */
                case 0x21:
                    dti_data.ac_curr = (float)(int16_t)(rxMsg.Data[0] << 8 | rxMsg.Data[1]) / 10.0f;
                    dti_data.dc_curr = (float)(int16_t)(rxMsg.Data[2] << 8 | rxMsg.Data[3]) / 10.0f;
                    break;

                /* --------------------------------------------------------
                 * Packet 0x22 – General Data 3
                 * Bytes 0-1    : Controller temp [/10 → °C]
                 * Bytes 2-3    : Motor temp      [/10 → °C]
                 * Byte 4       : Fault code (0 = no fault)
                 * ------------------------------------------------------- */
                case 0x22:
                    dti_data.temp_ctrl  = (float)(int16_t)(rxMsg.Data[0] << 8 | rxMsg.Data[1]) / 10.0f;
                    dti_data.temp_motor = (float)(int16_t)(rxMsg.Data[2] << 8 | rxMsg.Data[3]) / 10.0f;
                    dti_data.fault_code = rxMsg.Data[4];
                    break;

                /* --------------------------------------------------------
                 * Packet 0x23 – General Data 4
                 * Bytes 0-3    : Id (flux current)   [/100 → A]
                 * Bytes 4-7    : Iq (torque current) [/100 → A]
                 * ------------------------------------------------------- */
                case 0x23:
                    dti_data.id = (float)(int32_t)((uint32_t)rxMsg.Data[0] << 24 |
                                                   (uint32_t)rxMsg.Data[1] << 16 |
                                                   (uint32_t)rxMsg.Data[2] <<  8 |
                                                   (uint32_t)rxMsg.Data[3]) / 100.0f;
                    dti_data.iq = (float)(int32_t)((uint32_t)rxMsg.Data[4] << 24 |
                                                   (uint32_t)rxMsg.Data[5] << 16 |
                                                   (uint32_t)rxMsg.Data[6] <<  8 |
                                                   (uint32_t)rxMsg.Data[7]) / 100.0f;
                    break;

                /* --------------------------------------------------------
                 * Packet 0x24 – General Data 5
                 *
                 * Byte 0       : Throttle position (int8, echoed from DTI)
                 * Byte 1       : Brake position    (int8, echoed from DTI)
                 *
                 * Byte 2 – Digital I/O bitmask:
                 *   Bit 0-3 = Digital Inputs  1-4
                 *   Bit 4-7 = Digital Outputs 1-4
                 *
                 * Byte 3 – Status flags:
                 *   Bit 0 = Drive Enable active
                 *
                 * Byte 4 – Active limit flags (selected; extend as needed):
                 *   Bit 1 = DC current limit active
                 *   Bit 7 = Motor temperature limit active
                 *
                 * Byte 7 – Firmware map version
                 * ------------------------------------------------------- */
                case 0x24:
                    dti_data.throttle    = (int8_t)rxMsg.Data[0];
                    dti_data.brake       = (int8_t)rxMsg.Data[1];

                    /* Byte 2: digital I/O */
                    dti_data.dig_in_1    = (rxMsg.Data[2] >> 0) & 0x01;
                    dti_data.dig_in_2    = (rxMsg.Data[2] >> 1) & 0x01;
                    dti_data.dig_in_3    = (rxMsg.Data[2] >> 2) & 0x01;
                    dti_data.dig_in_4    = (rxMsg.Data[2] >> 3) & 0x01;
                    dti_data.dig_out_1   = (rxMsg.Data[2] >> 4) & 0x01;
                    dti_data.dig_out_2   = (rxMsg.Data[2] >> 5) & 0x01;
                    dti_data.dig_out_3   = (rxMsg.Data[2] >> 6) & 0x01;
                    dti_data.dig_out_4   = (rxMsg.Data[2] >> 7) & 0x01;

                    /* Byte 3: drive enable bit */
                    dti_data.drive_enable   = (rxMsg.Data[3] >> 0) & 0x01;

                    /* Byte 4: active limits */
                    dti_data.lim_dc_curr    = (rxMsg.Data[4] >> 1) & 0x01;
                    dti_data.lim_motor_temp = (rxMsg.Data[4] >> 7) & 0x01;

                    /* Byte 7: map version */
                    dti_data.map_ver        = rxMsg.Data[7];
                    break;

                default:
                    /* Unknown / unhandled packet – silently discard. */
                    break;
            }
        }
    }
}
