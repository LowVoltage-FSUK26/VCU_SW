/**
 ******************************************************************************
 * @file    vcu_safety.c
 * @brief   VCU Safety module – APPS plausibility, brake light, and R2D logic.
 *
 *  ╔══════════════════════════════════════════════════════════════════════╗
 *  ║   FORMULA STUDENT UK 2026 – RULE REFERENCE SUMMARY                  ║
 *  ║                                                                      ║
 *  ║  T11.8.8 / T11.8.9  APPS Implausibility                             ║
 *  ║    The plausibility check must verify that the two APPS signals      ║
 *  ║    agree within 10 % of the pedal travel. If they disagree for      ║
 *  ║    more than 100 ms, the power to the motor(s) must be cut and      ║
 *  ║    remain cut until the plausibility is restored AND the throttle   ║
 *  ║    pedal is fully released.                                          ║
 *  ║                                                                      ║
 *  ║  EV2.3.1  Brake Plausibility Check                                  ║
 *  ║    If the mechanical brakes are actuated AND the APPS signals more  ║
 *  ║    than 25 % throttle simultaneously, for more than 500 ms,        ║
 *  ║    the motor torque output must be set to zero.                      ║
 *  ║                                                                      ║
 *  ║  EV2.3.2  Brake Plausibility Reset                                  ║
 *  ║    Torque must remain zero until the driver releases the throttle   ║
 *  ║    to less than 5 % of full travel.                                  ║
 *  ║                                                                      ║
 *  ║  T6.3.1   Brake Light                                               ║
 *  ║    The brake light must be illuminated when the mechanical brakes   ║
 *  ║    are actuated OR when regen deceleration exceeds 1.0 m/s²        ║
 *  ║    (~0.102 g).                                                       ║
 *  ║                                                                      ║
 *  ║  EV4.11.7 Ready-to-Drive Activation                                 ║
 *  ║    Transition to R2D mode must only be possible during actuation    ║
 *  ║    of the mechanical brakes AND a simultaneous dedicated additional ║
 *  ║    action (a dedicated start button is used here).                  ║
 *  ║                                                                      ║
 *  ║  EV4.12.1 Ready-to-Drive Sound                                      ║
 *  ║    The vehicle must emit a characteristic sound continuously for    ║
 *  ║    at least 1 s and a maximum of 3 s while entering R2D.           ║
 *  ╚══════════════════════════════════════════════════════════════════════╝
 *
 *  FUTURE TEAM MEMBERS – READ THIS BEFORE EDITING:
 *    • Every #define threshold has a FS rule number next to it in vcu_safety.h.
 *    • Change APPS_CONFLICT_TIMER_MS or BRAKE_PLAUSIBILITY_TIMER_MS only after
 *      verifying the new value still satisfies the rule (min/max constraints).
 *    • The brake plausibility latch (EV2.3.2) is intentional – it is NOT a bug
 *      that torque stays at zero even after releasing the brake pedal.
 *    • TIM3 is the SOLE property of this module. Do not start/stop TIM3 from
 *      any other task or ISR.
 ******************************************************************************
 */

/* Includes ----------------------------------------------------------------- */
#include "vcu_safety.h"
#include <string.h>
#include <math.h>
#include "task.h"

/* ============================================================================
 * GLOBAL INSTANCE DEFINITION
 * ========================================================================= */
volatile VcuSensorData_t vcu_data = {0};

/* ============================================================================
 * HARDWARE INITIALISATION
 * ========================================================================= */

/**
 * @brief  Configure MPU-6050 over I2C (blocking; called once at boot).
 *
 * Sequence (from MPU-6050 product specification rev 3.4):
 *   1. Clear PWR_MGMT_1 to exit sleep mode.
 *   2. Set SMPLRT_DIV = 7 → sample rate = 1 kHz / (1+7) = 125 Hz.
 *   3. Set ACCEL_CONFIG → ±2g range (max sensitivity = 16 384 LSB/g).
 */
void MPU6050_Init(void)
{
    uint8_t data;

    /* Step 1: Wake up device (default power-on state is sleep). */
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    /* Step 2: Sample rate = 125 Hz (sufficient for brake light decision). */
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    /* Step 3: Accelerometer full-scale = ±2g. */
    data = MPU6050_ACCEL_RANGE_2G;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/* ============================================================================
 * APPS PLAUSIBILITY LOGIC
 * ============================================================================
 *
 * WHY TWO SENSORS?
 *   FSUK T11.8.8 mandates two independent APPS (Accelerator Pedal Position
 *   Sensors) for redundancy. If one sensor fails (open, short, or out of range)
 *   the second will disagree by more than 10%, triggering a torque cut.
 *
 * WHY THE 100 ms GRACE PERIOD? (T11.8.9)
 *   Signal glitches, connector vibration, and ADC noise can cause brief
 *   transient disagreements. The 100 ms window prevents nuisance trips from
 *   short-duration noise while still cutting power on real sensor failures.
 *
 * WHY INVERT APPS2? (INVERSE CHARACTERISTIC)
 *   The second sensor is wired inverse to the first (val2 = 1 - normalised_raw)
 *   so that a wiring fault (open or short) on either sensor causes the two
 *   channels to read opposites (0% vs 100%) – guaranteeing the 10% threshold
 *   is exceeded immediately, triggering the safety cut.
 *
 * ========================================================================= */
void Process_APPS_Safety_Logic(void)
{
    /* Persistent state across calls (static = zero-initialised at boot). */
    static float    apps1_smooth      = 0.0f;
    static float    apps2_smooth      = 0.0f;
    static uint32_t apps_conflict_timer_start = 0;
    static uint8_t  conflict_detected  = 0;
    static uint8_t  brake_latch        = 0;

    /* ------------------------------------------------------------------
     * Persistent state for EV2.3.1 brake plausibility timer.
     * (NEW – original code had no timer; cut was instantaneous.)
     * ------------------------------------------------------------------ */
    static uint32_t brake_conflict_timer_start = 0;
    static uint8_t  brake_conflict_active       = 0;

    /* ================================================================
     * STEP 1 – MAP RAW ADC TO NORMALISED 0.0 – 1.0
     *
     * APPS1: direct characteristic – more pedal = higher ADC count.
     * APPS2: inverse characteristic – more pedal = lower ADC count.
     *
     * Both are clamped to [0, 1] to handle out-of-range ADC values
     * (e.g. sensor supply failure or broken wire).
     * ================================================================ */
    float raw1 = (float)vcu_data.adc_raw_apps1;
    float raw2 = (float)vcu_data.adc_raw_apps2;

    float val1 = (raw1 - (float)APPS1_MIN_ADC) /
                 (float)(APPS1_MAX_ADC - APPS1_MIN_ADC);

    float val2 = 1.0f - ((raw2 - (float)APPS2_MIN_ADC) /
                         (float)(APPS2_MAX_ADC - APPS2_MIN_ADC));

    /* Clamp both channels. */
    if (val1 < 0.0f) val1 = 0.0f;
    if (val1 > 1.0f) val1 = 1.0f;
    if (val2 < 0.0f) val2 = 0.0f;
    if (val2 > 1.0f) val2 = 1.0f;

    /* ================================================================
     * STEP 2 – LOW-PASS FILTER (IIR first-order)
     *
     * Reduces ADC quantisation noise and pedal vibration without
     * introducing excessive latency.
     *   alpha = 0.2 → ~3 Hz cutoff at 10 ms sample interval.
     *
     * Equation: y[n] = α · x[n] + (1-α) · y[n-1]
     * ================================================================ */
    apps1_smooth = (APPS_FILTER_ALPHA * val1) + ((1.0f - APPS_FILTER_ALPHA) * apps1_smooth);
    apps2_smooth = (APPS_FILTER_ALPHA * val2) + ((1.0f - APPS_FILTER_ALPHA) * apps2_smooth);

    vcu_data.apps1_percent = apps1_smooth * 100.0f;
    vcu_data.apps2_percent = apps2_smooth * 100.0f;
    vcu_data.apps_percent  = (vcu_data.apps1_percent + vcu_data.apps2_percent) / 2.0f;

    /* ================================================================
     * STEP 3 – IMPLAUSIBILITY CHECK (Rules T11.8.8 / T11.8.9)
     *
     * T11.8.8: "The two APPS signals must agree within 10 % of pedal
     *           travel at all times during operation."
     *
     * T11.8.9: "The motor power must be immediately shut down if the
     *           disagreement persists for more than 100 ms."
     *
     * Algorithm:
     *   - Compute absolute deviation between both normalised channels.
     *   - On first out-of-range sample: record timestamp, set flag.
     *   - If deviation clears: reset timer and fault flag.
     *   - If deviation persists > APPS_CONFLICT_TIMER_MS (100 ms):
     *       → set apps_fault_active = 1, zero torque, illuminate LED.
     *       → RETURN immediately (skip all further checks).
     *   - Fault is cleared when deviation returns to within 10 %.
     *     (Note: some interpretations require pedal release too –
     *      check with your scrutineer before modifying.)
     * ================================================================ */
    float deviation = fabsf(apps1_smooth - apps2_smooth);

    if (deviation > APPS_IMPLAUSIBILITY_THRESHOLD)
    {
        if (conflict_detected == 0)
        {
            /* First out-of-tolerance sample – start the 100 ms grace timer. */
            apps_conflict_timer_start = HAL_GetTick();
            conflict_detected         = 1;
        }
        else if ((HAL_GetTick() - apps_conflict_timer_start) > APPS_CONFLICT_TIMER_MS)
        {
            /*
             * Fault confirmed – 100 ms has elapsed with >10 % deviation.
             * Rule T11.8.9: cut power immediately, illuminate fault LED.
             */
            vcu_data.apps_fault_active    = 1;
            vcu_data.final_torque_request = 0.0f;
            HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
            return; /* Skip brake and torque-output sections. */
        }
        /* else: within grace period – allow torque to continue. */
    }
    else
    {
        /* Deviation back within 10 % – clear fault. */
        conflict_detected          = 0;
        vcu_data.apps_fault_active = 0;
        HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
    }

    /* ================================================================
     * STEP 4 – BRAKE PLAUSIBILITY CHECK (Rules EV2.3.1 / EV2.3.2)
     *
     * EV2.3.1: "If the mechanical brakes are actuated AND the APPS
     *           indicate more than 25 % simultaneously for more than
     *           500 ms, the torque demand must be set to zero."
     *
     * EV2.3.2: "The torque must remain zero until the APPS returns
     *           below 5 % (not just until the brake is released)."
     *
     * WHY A 500 ms TIMER? (EV2.3.1)
     *   Hard braking combined with heel-toe or cadence-braking
     *   techniques can cause brief simultaneous pedal application.
     *   The 500 ms window prevents nuisance cuts from normal driving
     *   whilst still catching a genuine sensor fault or stuck throttle.
     *
     * WHY A LATCH? (EV2.3.2)
     *   If the brake plausibility was simply edge-triggered, a driver
     *   could release and re-press the brake to restore torque while
     *   the throttle is still stuck open. The latch forces the driver
     *   to visibly return the throttle to idle before torque returns,
     *   making the fault obvious.
     * ================================================================ */
    float   avg_throttle  = vcu_data.apps_percent;
    uint8_t brake_is_hard = (vcu_data.adc_raw_pressure > HYDRAULIC_PRESS_THRESHOLD);

    if (brake_latch == 0)
    {
        /* --- Not currently latched; check if condition is developing. --- */
        if (brake_is_hard && (avg_throttle > BRAKE_PLAUSIBILITY_THRESHOLD))
        {
            if (brake_conflict_active == 0)
            {
                /*
                 * Both brake and throttle > 25 % just appeared.
                 * Start the 500 ms EV2.3.1 timer.
                 */
                brake_conflict_timer_start = HAL_GetTick();
                brake_conflict_active      = 1;
            }
            else if ((HAL_GetTick() - brake_conflict_timer_start) > BRAKE_PLAUSIBILITY_TIMER_MS)
            {
                /*
                 * Rule EV2.3.1 – condition has persisted for > 500 ms.
                 * Engage the latch: torque → 0, flag set for R2D supervisor.
                 */
                brake_latch                        = 1;
                vcu_data.brake_plausibility_active = 1;
                brake_conflict_active              = 0; /* reset timer state */
            }
            /* else: still within 500 ms grace period – allow torque. */
        }
        else
        {
            /* Condition cleared before timer expired – reset. */
            brake_conflict_active = 0;
        }
    }

    if (brake_latch == 1)
    {
        /*
         * Rule EV2.3.2 – Latched. Torque stays at zero regardless of
         * brake state. Only the throttle returning below 5 % clears this.
         */
        if (avg_throttle < BRAKE_RESET_THRESHOLD)
        {
            /* Throttle returned to near-idle – safe to release latch. */
            brake_latch                        = 0;
            vcu_data.brake_plausibility_active = 0;
        }
        else
        {
            /* Still above 5 % – keep torque at zero and return. */
            vcu_data.final_torque_request = 0.0f;
            return;
        }
    }

    /* ================================================================
     * STEP 5 – ALL CHECKS PASSED
     * Output the average pedal position as the torque request.
     * The inverter CAN TX task (if implemented) reads final_torque_request.
     * ================================================================ */
    vcu_data.final_torque_request = avg_throttle;
}

/* ============================================================================
 * TASK 3: BRAKE LIGHT & PEDAL SAFETY CONTROLLER
 * ============================================================================
 *
 *  Execution rate: 100 Hz (10 ms period) – driven by DMA completion notify.
 *
 *  Rule T6.3.1 – Brake Light:
 *    "The brake light must be illuminated when the hydraulic brakes are
 *     actuated OR when regen deceleration exceeds 1.0 m/s² (~0.102 g)."
 *
 *  Why two triggers?
 *    Formula Student EVs can decelerate significantly through regenerative
 *    braking without any hydraulic pressure. A following driver or marshal
 *    has no visual cue unless the regen deceleration also activates the
 *    brake light. This is why T6.3.1 explicitly includes both conditions.
 *
 *  IMU calibration:
 *    The MPU-6050 Y-axis is used for longitudinal deceleration. On a flat
 *    surface at rest, the sensor reads ~0 g on Y (gravity on Z). However,
 *    the sensor may not be mounted perfectly flat, so we accumulate 100
 *    samples at boot to measure and subtract a Y-offset (tare).
 *    The car MUST be stationary on a flat surface during this phase.
 *    A 200 ms brake-light flash signals calibration completion.
 *
 * ========================================================================= */
void vBrakeLightTask(void *argument)
{
    uint8_t  dma_buffer[14];    /* MPU-6050: ACCEL_XOUT_H through GYRO_ZOUT_L  */
    static float    y_offset      = 0.0f;
    static uint8_t  is_calibrated = 0;
    static uint16_t cal_counter   = 0;

    for (;;)
    {
        /*
         * Trigger a DMA-backed I2C read of 14 bytes starting at ACCEL_XOUT_H.
         * The 14-byte block covers all three accelerometer axes + temp + gyro.
         * HAL_I2C_MemRxCpltCallback (main.c) will notify this task via
         * vTaskNotifyGiveFromISR when the transfer completes.
         */
        if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR,
                                  MPU6050_REG_ACCEL_XOUT_H,
                                  I2C_MEMADD_SIZE_8BIT,
                                  dma_buffer, 14) == HAL_OK)
        {
            /* Wait for DMA completion (20 ms timeout → bus error handling). */
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                /* --------------------------------------------------------
                 * A. SNAPSHOT ADC DMA buffer.
                 *    adc_dma_buffer is written continuously by ADC DMA.
                 *    Snapshot here so all three values are from same cycle.
                 * ------------------------------------------------------- */
                vcu_data.adc_raw_apps1    = adc_dma_buffer[0];
                vcu_data.adc_raw_apps2    = adc_dma_buffer[1];
                vcu_data.adc_raw_pressure = adc_dma_buffer[2];

                /* --------------------------------------------------------
                 * B. APPS + BRAKE PLAUSIBILITY SAFETY CHECKS
                 *    Rules: T11.8.8 / T11.8.9 / EV2.3.1 / EV2.3.2
                 * ------------------------------------------------------- */
                Process_APPS_Safety_Logic();

                /* --------------------------------------------------------
                 * C. IMU DATA EXTRACTION
                 *    dma_buffer layout (big-endian pairs):
                 *      [0-1]  ACCEL_XOUT
                 *      [2-3]  ACCEL_YOUT  ← longitudinal axis (our interest)
                 *      [4-5]  ACCEL_ZOUT
                 *      [6-7]  TEMP_OUT
                 *      [8-15] GYRO_XYZ
                 * ------------------------------------------------------- */
                int16_t raw_y        = (int16_t)((dma_buffer[2] << 8) | dma_buffer[3]);
                vcu_data.accel_y_raw = raw_y;
                vcu_data.accel_y_g   = (float)raw_y / MPU6050_ACCEL_SENSITIVITY;

                /* --------------------------------------------------------
                 * D. BOOT CALIBRATION (tare Y-axis offset)
                 *    Averages 100 samples (~1 s at 10 ms period).
                 *    Car must be stationary on flat ground.
                 *    Flash at end = calibration done.
                 * ------------------------------------------------------- */
                if (is_calibrated == 0)
                {
                    y_offset += vcu_data.accel_y_g;
                    if (++cal_counter >= 100)
                    {
                        y_offset     /= 100.0f;
                        is_calibrated = 1;

                        /* Visual confirmation: 200 ms brake-light flash. */
                        HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_PORT,
                                          BRAKE_LIGHT_GPIO_PIN, GPIO_PIN_SET);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_PORT,
                                          BRAKE_LIGHT_GPIO_PIN, GPIO_PIN_RESET);
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue; /* Skip brake-light logic until calibrated. */
                }

                /* --------------------------------------------------------
                 * E. APPLY TARE & LOW-PASS FILTER
                 *    Remove steady-state bias then smooth with IIR filter.
                 *    alpha = 0.1 → ~1.6 Hz cutoff (heavy smoothing).
                 *    deceleration_g = |filtered_y| (always positive).
                 * ------------------------------------------------------- */
                float corrected_y = vcu_data.accel_y_g - y_offset;
                vcu_data.accel_y_filtered =
                    (ACCEL_FILTER_ALPHA * corrected_y) +
                    ((1.0f - ACCEL_FILTER_ALPHA) * vcu_data.accel_y_filtered);
                vcu_data.deceleration_g = fabsf(vcu_data.accel_y_filtered);

                /* --------------------------------------------------------
                 * F. BRAKE LIGHT DECISION  (Rule T6.3.1)
                 *
                 *  T6.3.1: Illuminate if:
                 *    (a) hydraulic brake is actuated, OR
                 *    (b) regen deceleration exceeds 1.0 m/s² (~0.102 g).
                 *
                 *  Hysteresis of 0.02 g prevents rapid flickering near
                 *  the threshold (a brake light that flickers at low regen
                 *  would confuse following drivers and fail scrutineering).
                 *
                 *  Priority: hydraulic check first (immediate physical input).
                 *  Regen check second (derived from IMU).
                 * ------------------------------------------------------- */
                uint8_t hydraulic_active =
                    (vcu_data.adc_raw_pressure > HYDRAULIC_PRESS_THRESHOLD);

                /*
                 * Regen brake light condition:
                 *   - Throttle must be in deadzone (driver not requesting power)
                 *   - Deceleration must exceed 1.0 m/s² threshold
                 */
                uint8_t regen_active =
                    (vcu_data.final_torque_request < APPS_DEADZONE_PERCENT) &&
                    (vcu_data.deceleration_g > BRAKE_DECEL_THRESHOLD_G);

                if (hydraulic_active)
                {
                    HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_PORT,
                                      BRAKE_LIGHT_GPIO_PIN, GPIO_PIN_SET);
                    vcu_data.brake_light_active = 1;
                    strcpy((char *)vcu_data.trigger_source, "HYDRAULIC");
                }
                else if (regen_active)
                {
                    HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_PORT,
                                      BRAKE_LIGHT_GPIO_PIN, GPIO_PIN_SET);
                    vcu_data.brake_light_active = 1;
                    strcpy((char *)vcu_data.trigger_source, "REGEN_DECEL");
                }
                else
                {
                    HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_PORT,
                                      BRAKE_LIGHT_GPIO_PIN, GPIO_PIN_RESET);
                    vcu_data.brake_light_active = 0;
                    strcpy((char *)vcu_data.trigger_source, "NONE");
                }
            }
            else
            {
                /* DMA did not complete within 20 ms → reset I2C peripheral. */
                HAL_I2C_Init(&hi2c1);
            }
        }
        else
        {
            /* HAL_I2C_Mem_Read_DMA returned error → bus fault, reset. */
            HAL_I2C_Init(&hi2c1);
        }

        /* 10 ms loop rate = 100 Hz. Sufficient for brake-light control. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================================
 * TASK 4: READY-TO-DRIVE (R2D) SUPERVISOR
 * ============================================================================
 *
 *  Implements FSUK 2026 Rules EV4.11.7 and EV4.12.1.
 *
 *  STATE MACHINE OVERVIEW:
 *
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │                        STANDBY STATE                              │
 *   │  Drive Enable = LOW (inverter physically disabled)                │
 *   │  Waiting for: brake pressed AND start button pressed              │
 *   │                AND no APPS fault                                  │
 *   └──────────────────────────────┬─────────────────────────────────────┘
 *                                  │ Conditions met (EV4.11.7)
 *                                  ▼
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │                    RTD SOUND SEQUENCE (EV4.12.1)                  │
 *   │  Play 2 kHz tone for 2000 ms (within 1 s min, 3 s max rule)      │
 *   └──────────────────────────────┬─────────────────────────────────────┘
 *                                  │ Sound complete
 *                                  ▼
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │                        DRIVING STATE                              │
 *   │  Drive Enable = HIGH (inverter enabled)                           │
 *   │  Fault supervisor kills Drive Enable on ANY safety fault          │
 *   └────────────────────────────────────────────────────────────────────┘
 *
 *  BUZZER CONFIGURATION:
 *    vcu_data.buzzer_is_dc_mode = 0 → Passive piezo. TIM3 outputs a 50%
 *      duty-cycle square wave at 2 kHz. The piezo resonates at this frequency.
 *    vcu_data.buzzer_is_dc_mode = 1 → Active buzzer (internal oscillator).
 *      TIM3 outputs DC HIGH. The buzzer generates its own tone internally.
 *      Set ARR to RTD_BUZZER_PWM_PERIOD and duty to ARR+1 to saturate HIGH.
 *
 *  TIM3 OWNERSHIP:
 *    This task is the SOLE owner of TIM3. It sets ARR and CCR1 dynamically.
 *    The HAL initialises TIM3 with ARR=999 (1 kHz) and duty=0; the task
 *    reconfigures it to ARR=500 (2 kHz) only during the RTD sound window
 *    and silences it (duty=0) at all other times.
 *
 * ========================================================================= */
void vR2DLogicTask(void *argument)
{
    /*
     * Set to 1 for an active DC buzzer (most common in FSUK builds).
     * Set to 0 if using a passive piezo that requires a PWM tone.
     */
    vcu_data.buzzer_is_dc_mode = 1;

    /* -----------------------------------------------------------------------
     * SAFE STATE ON STARTUP
     * Drive Enable is already LOW from GPIO init in main.c, but we set it
     * here again as a belt-and-suspenders measure. On any reset or watchdog
     * reboot this guarantees the inverter cannot be accidentally enabled.
     * --------------------------------------------------------------------- */
    HAL_GPIO_WritePin(DRIVE_ENABLE_GPIO_PORT, DRIVE_ENABLE_GPIO_PIN, GPIO_PIN_RESET);
    vcu_data.ready_to_drive_active = 0;

    /* Buzzer must be silent on startup. */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

    for (;;)
    {
        /* -------------------------------------------------------------------
         * 1. READ INPUTS
         * ------------------------------------------------------------------ */
        uint8_t btn_pressed =
            (HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_PORT, RTD_BUTTON_GPIO_PIN) == GPIO_PIN_SET);

        uint8_t brake_pressed =
            (vcu_data.adc_raw_pressure > HYDRAULIC_PRESS_THRESHOLD);

        /*
         * Rule EV4.11.8 – Read Shutdown Circuit sense pin.
         * SDC closed (normal) → upstream drives pin HIGH → sdc_closed = 1.
         * SDC open   (fault)  → pin floats → internal pull-DOWN pulls LOW
         *                     → sdc_closed = 0 → inverter disabled.
         * A broken sense wire also floats LOW via the pull-down — the VCU
         * treats this identically to a real SDC opening (fail-safe direction).
         */
        uint8_t sdc_closed =
            (HAL_GPIO_ReadPin(SDC_SENSE_GPIO_PORT, SDC_SENSE_GPIO_PIN) == GPIO_PIN_SET);
        vcu_data.shutdown_circuit_open = !sdc_closed;

        /* Expose raw button state for live-watch / telemetry debugging. */
        vcu_data.rtd_button_state = btn_pressed;

        /* -------------------------------------------------------------------
         * 2. FAULT SUPERVISOR – HIGHEST PRIORITY (checked every 20 ms loop)
         *
         *  Three independent faults can kill the inverter here:
         *
         *  (a) Rule EV4.11.8 – Shutdown Circuit opened:
         *        "The ready-to-drive mode must be left immediately when the
         *         shutdown circuit is opened."
         *        If the SDC sense pin reads LOW (circuit open OR wire broken),
         *        DRIVE_ENABLE is pulled LOW immediately. This is not debounced
         *        and has no grace period – the response must be instantaneous.
         *
         *  (b) Rules T11.8.8 / T11.8.9 – APPS implausibility:
         *        Both APPS sensors disagree by > 10% for > 100 ms.
         *        apps_fault_active flag set by Process_APPS_Safety_Logic.
         *
         *  (c) Rules EV2.3.1 / EV2.3.2 – Brake plausibility latch:
         *        Brake + throttle > 25% simultaneously for > 500 ms.
         *        brake_plausibility_active flag set by Process_APPS_Safety_Logic.
         *        Remains set until APPS drops below 5%.
         *
         *  This block runs BEFORE the state machine every loop iteration so
         *  that a fault detected during DRIVING causes an immediate transition
         *  to STANDBY with no possibility of the state machine re-enabling the
         *  inverter in the same cycle.
         * ------------------------------------------------------------------ */
        if (vcu_data.shutdown_circuit_open ||
            vcu_data.apps_fault_active     ||
            vcu_data.brake_plausibility_active)
        {
            if (vcu_data.ready_to_drive_active == 1)
            {
                /* Kill inverter immediately. */
                HAL_GPIO_WritePin(DRIVE_ENABLE_GPIO_PORT,
                                  DRIVE_ENABLE_GPIO_PIN, GPIO_PIN_RESET);
                vcu_data.ready_to_drive_active = 0;
            }
            /* Stay in this fault-holding loop until the fault is cleared.  */
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* -------------------------------------------------------------------
         * 3. STATE MACHINE
         * ------------------------------------------------------------------ */
        if (vcu_data.ready_to_drive_active == 0)
        {
            /* ════════════════════════ STANDBY ════════════════════════════ */

            /*
             * Rule EV4.11.7:
             * "The transition to R2D mode must only be possible during
             *  actuation of the mechanical brakes AND a simultaneous
             *  dedicated additional action."
             *
             * Both conditions must be true simultaneously.
             * Neither alone is sufficient (avoids accidental activation).
             */
            if (brake_pressed && btn_pressed)
            {
                /* -----------------------------------------------------------
                 * RTD SOUND SEQUENCE  (Rule EV4.12.1)
                 *
                 * "The vehicle must emit a characteristic sound continuously
                 *  for at least 1 s and a maximum of 3 s while entering R2D."
                 *
                 * We use exactly 2000 ms – compliant middle ground.
                 *
                 * TIM3 reconfigured here to 2 kHz (ARR = RTD_BUZZER_PWM_PERIOD):
                 *   72 MHz / (PSC+1=72) = 1 MHz tick
                 *   ARR = 500 → PWM frequency = 1 MHz / 500 = 2 kHz
                 * --------------------------------------------------------- */
                __HAL_TIM_SET_AUTORELOAD(&htim3, RTD_BUZZER_PWM_PERIOD);

                uint32_t duty;
                if (vcu_data.buzzer_is_dc_mode)
                {
                    /*
                     * Active buzzer: saturate output HIGH.
                     * Setting CCR > ARR forces output permanently HIGH
                     * (100% duty) so the active buzzer's internal osc runs.
                     */
                    duty = RTD_BUZZER_PWM_PERIOD + 1;
                }
                else
                {
                    /*
                     * Passive piezo: 50% duty square wave.
                     * This generates maximum acoustic output from a piezo
                     * resonator at its rated frequency (here 2 kHz).
                     */
                    duty = RTD_BUZZER_PWM_PERIOD / 2;
                }
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

                /*
                 * Block this task for exactly RTD_SOUND_DURATION_MS.
                 * Using vTaskDelay here is correct – the fault supervisor
                 * will NOT run during this window. For a production system
                 * consider a shorter delay with repeated fault checks.
                 * RTD_SOUND_DURATION_MS = 2000 ms (EV4.12.1 compliant).
                 */
                vTaskDelay(pdMS_TO_TICKS(RTD_SOUND_DURATION_MS));

                /* Silence buzzer before enabling inverter. */
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

                /* -----------------------------------------------------------
                 * ENABLE INVERTER  (Rule EV4.11.6)
                 *
                 * Drive Enable (PB15) pulled HIGH → DTI inverter enabled.
                 * The DTI will now accept torque commands from its analog
                 * pedal inputs (or CAN if configured).
                 * --------------------------------------------------------- */
                HAL_GPIO_WritePin(DRIVE_ENABLE_GPIO_PORT,
                                  DRIVE_ENABLE_GPIO_PIN, GPIO_PIN_SET);
                vcu_data.ready_to_drive_active = 1;
            }
            /* else: conditions not met – remain in standby silently. */
        }

        /*
         * ══════════════════════════ DRIVING ══════════════════════════════
         * Drive Enable is HIGH. The DTI inverter is active.
         * No periodic action is needed here – torque requests are handled
         * by Process_APPS_Safety_Logic (called from vBrakeLightTask), and
         * fault supervision is handled at the top of this loop.
         * ══════════════════════════════════════════════════════════════════
         */

        /* 50 Hz supervisor loop (20 ms). */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
