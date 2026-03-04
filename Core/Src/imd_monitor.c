/**
 ******************************************************************************
 * @file    imd_monitor.c
 * @brief   Bender IMD PWM-decoding task implementation.
 *
 *  Resistance calculation (NORMAL state only, 10 Hz signal):
 *
 *    The Bender ISOMETER® encodes isolation resistance (R_iso) onto the
 *    PWM duty cycle using the following relationship:
 *
 *      duty = 0.05 + (R_internal * 0.90) / (R_iso + R_internal)
 *
 *    Rearranging for R_iso:
 *
 *      R_iso = ((R_internal * 0.90) / (duty - 0.05)) - R_internal
 *
 *    Where R_internal = 1200 kΩ (Bender internal reference resistor).
 *
 *    Clamped to 0 on negative result (full fault) and to 999999 kΩ
 *    when denominator is near zero (open circuit / infinite resistance).
 *
 *  Timeout:
 *    If no notification arrives within 250 ms the state is set to
 *    IMD_STATE_DISCONNECTED, which should trigger a shutdown in a
 *    full implementation (not in scope for this module).
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "imd_monitor.h"
#include "task.h"

/* ============================================================================
 * CONSTANTS (local – not needed by other modules)
 * ========================================================================= */
#define TIM2_CLOCK_FREQ      500000.0f   /**< TIM2 tick frequency [Hz].     */
#define BENDER_INTERNAL_R    1200.0f     /**< Internal reference [kΩ].      */
#define BENDER_MAX_R_FACTOR  0.90f       /**< Scaling factor from datasheet. */

/* ============================================================================
 * GLOBAL INSTANCE DEFINITIONS
 * ========================================================================= */
volatile ImdDebugStatus_t imd_live_status = {0};
volatile IsrBuffer_t      isr_buffer      = {0, 0};

/* ============================================================================
 * TASK IMPLEMENTATION
 * ========================================================================= */

/**
 * @brief  IMD PWM monitoring task.
 *
 * Execution model:
 *   1. Block on task notification (sent by HAL_TIM_IC_CaptureCallback in main.c).
 *   2. Copy ISR buffer under critical section to avoid torn 32-bit reads.
 *   3. Compute frequency and duty from raw tick counts.
 *   4. Classify IMD state by frequency band (±1 Hz tolerance).
 *   5. If state is NORMAL, compute insulation resistance from duty cycle.
 *   6. On timeout (250 ms), mark as DISCONNECTED.
 */
void StartPwmTask(void *argument)
{
    const TickType_t xBlockTime = pdMS_TO_TICKS(250);
    uint32_t   ulNotificationValue;
    IsrBuffer_t local_data;

    for (;;)
    {
        /*
         * Wait for notification from TIM2 capture ISR.
         * pdTRUE = clear notification count on exit (binary semaphore behaviour).
         * 250 ms timeout → disconnected detection.
         */
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, xBlockTime);

        if (ulNotificationValue > 0)
        {
            /* ----------------------------------------------------------------
             * Copy ISR data atomically.
             * taskENTER_CRITICAL disables interrupts; keeps the window tiny.
             * --------------------------------------------------------------- */
            taskENTER_CRITICAL();
            local_data.period = isr_buffer.period;
            local_data.pulse  = isr_buffer.pulse;
            taskEXIT_CRITICAL();

            /* Store raw ticks for live-watch inspection. */
            imd_live_status.Raw_Period_Ticks = local_data.period;
            imd_live_status.Raw_Pulse_Ticks  = local_data.pulse;

            /* ----------------------------------------------------------------
             * Compute frequency and duty cycle.
             * period == 0 guard prevents divide-by-zero on startup glitch.
             * --------------------------------------------------------------- */
            imd_live_status.Frequency_Hz = (local_data.period > 0) ?
                (TIM2_CLOCK_FREQ / (float)local_data.period) : 0.0f;

            imd_live_status.Duty_Cycle_Percent = (local_data.period > 0) ?
                ((float)local_data.pulse / (float)local_data.period * 100.0f) : 0.0f;

            float freq = imd_live_status.Frequency_Hz;
            float duty = imd_live_status.Duty_Cycle_Percent;

            /* ----------------------------------------------------------------
             * State classification by frequency band (±1 Hz tolerance).
             * Values from Bender ISOMETER® application note.
             * --------------------------------------------------------------- */
            if (freq > 9.0f && freq < 11.0f)
            {
                /* 10 Hz → NORMAL operating mode. Resistance is valid. */
                imd_live_status.CurrentState = IMD_STATE_NORMAL;

                /*
                 * Resistance calculation from duty cycle:
                 *
                 *   denom = (duty% / 100) - 0.05
                 *
                 * If denom ≤ 0.001 the formula diverges → set to max (open circuit).
                 */
                float denom = (duty / 100.0f) - 0.05f;
                if (denom > 0.001f)
                {
                    imd_live_status.Resistance_kOhm =
                        ((BENDER_MAX_R_FACTOR * BENDER_INTERNAL_R) / denom) - BENDER_INTERNAL_R;

                    if (imd_live_status.Resistance_kOhm < 0.0f)
                    {
                        imd_live_status.Resistance_kOhm = 0.0f;
                    }
                }
                else
                {
                    imd_live_status.Resistance_kOhm = 999999.0f; /* Effectively infinite. */
                }
            }
            else if (freq > 19.0f && freq < 21.0f)
            {
                /* 20 Hz → DC bus undervoltage. No resistance information. */
                imd_live_status.CurrentState    = IMD_STATE_UNDERVOLTAGE;
                imd_live_status.Resistance_kOhm = 0.0f;
            }
            else if (freq > 29.0f && freq < 31.0f)
            {
                /* 30 Hz → Speed Start Test (SST / self-test). */
                imd_live_status.CurrentState    = IMD_STATE_SST;
                imd_live_status.Resistance_kOhm = 0.0f;
            }
            else if (freq > 39.0f && freq < 41.0f)
            {
                /* 40 Hz → Internal device error. */
                imd_live_status.CurrentState    = IMD_STATE_DEVICE_ERROR;
                imd_live_status.Resistance_kOhm = 0.0f;
            }
            else if (freq > 49.0f && freq < 51.0f)
            {
                /* 50 Hz → Ground / isolation fault confirmed. */
                imd_live_status.CurrentState    = IMD_STATE_GROUND_FAULT;
                imd_live_status.Resistance_kOhm = 0.0f;
            }
            else
            {
                /* Frequency out of all known bands – treat as unknown glitch. */
                imd_live_status.CurrentState = IMD_STATE_UNKNOWN;
            }
        }
        else
        {
            /* ----------------------------------------------------------------
             * 250 ms elapsed with no capture interrupt → signal disconnected.
             * A full implementation should escalate this to a shutdown latch.
             * --------------------------------------------------------------- */
            imd_live_status.CurrentState       = IMD_STATE_DISCONNECTED;
            imd_live_status.Frequency_Hz       = 0.0f;
            imd_live_status.Duty_Cycle_Percent = 0.0f;
            imd_live_status.Resistance_kOhm    = 0.0f;
        }
    }
}
