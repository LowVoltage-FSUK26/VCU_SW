# FSUK VCU — Formula Student UK 2026 Vehicle Control Unit Firmware

> **STM32F103 · FreeRTOS · DTI HV500 · Bender IMD · MPU-6050**
> Safety-critical embedded firmware for an electric Formula Student car.

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Hardware Platform](#hardware-platform)
4. [Module Breakdown](#module-breakdown)
5. [Safety Rules Enforced](#safety-rules-enforced)
6. [FreeRTOS Task Map](#freertos-task-map)
7. [Pin Reference](#pin-reference)
8. [Signal & Data Flow](#signal--data-flow)
9. [Key Tunable Parameters](#key-tunable-parameters)
10. [Build & Flash](#build--flash)
11. [Debugging Tips](#debugging-tips)
12. [Future Work](#future-work)

---

## Overview

This repository contains the complete embedded C firmware for the VCU (Vehicle Control Unit) of an FSUK 2026 electric racing car. The VCU is the central safety brain of the powertrain — it monitors every sensor, enforces all relevant FS rulebook safety requirements, and is the sole authority that enables or disables the inverter.

**Design Philosophy:** The VCU *does not* compute or transmit CAN torque commands. The DTI HV500 inverter is configured to handle its own internal torque curves based on direct pedal inputs. The VCU acts strictly as a hardware-level safety supervisor, possessing ultimate authority over the powertrain via the physical `DRIVE_ENABLE` pin.

The codebase is structured around **four FreeRTOS tasks** running on an STM32F103 microcontroller (72 MHz Cortex-M3). Peripheral data acquisition is handled by DMA wherever possible, keeping safety-critical task CPU budgets minimal.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                       STM32F103 VCU                                 │
│                                                                     │
│  ┌────────────────┐  ┌──────────────┐  ┌───────────────────────────┐│
│  │ vBrakeLightTask│  │ vR2DLogicTask│  │       vCanDecodeTask      ││
│  │  Priority: 3   │  │  Priority: 3 │  │        Priority: 2        ││
│  │  Period: 10ms  │  │  Period: 20ms│  │    Blocks on CAN queue    ││
│  └──────┬─────────┘  └────┬─────────┘  └──────────┬────────────────┘│
│         │                 │                       │                 │
│         │                 │               ┌───────▼──────────────┐  │
│         │                 │               │     xCanRxQueue      │  │
│         │                 │               │     (depth 10)       │  │
│         │                 │               └───────▲──────────────┘  │
│         │                 │                       │                 │
│         ▼                 │               ┌───────┴──────────────┐  │
│  ┌─────────────────────────────────────┐  │  HAL_CAN_RxFifo0ISR  │  │
│  │          Global State               │  └──────────────────────┘  │
│  │  vcu_data (VcuSensorData_t)         │                            │
│  │  dti_data (DtiData_t)               │  ┌──────────────────────┐  │
│  │  imd_live_status (ImdDebugStatus_t) │  │  HAL_TIM_IC_Capture  │  │
│  └──────────────────▲──────────────────┘  │  (TIM2 → IMD PWM ISR)│  │
│                     │                     └───────┬──────────────┘  │
│                     │                             │                 │
│              ┌──────┴───────┐             ┌───────▼──────────────┐  │
│              │  StartPwmTask│             │  HAL_I2C_MemRxCplt   │  │
│              │  Priority: 2 │             │  (DMA → MPU-6050 ISR)│  │
│              │ Notify-driven│             └──────────────────────┘  │
│              └──────────────┘                                       │
└─────────────────────────────────────────────────────────────────────┘
```

All hardware ISRs live in `main.c`. Their only job is to place data into FreeRTOS primitives (queue, task notification) and immediately yield. No business logic runs at interrupt level.

---

## Hardware Platform

| Component | Part | Interface |
|---|---|---|
| Microcontroller | STM32F103C8 ("Blue Pill") | — |
| Motor Inverter | DTI HV500 | CAN 500 kbps |
| Insulation Monitor | Bender ISOMETER® | PWM (TIM2 input capture) |
| IMU | InvenSense MPU-6050 | I2C + DMA (hi2c1) |
| Pedal sensors | Bosch APM (0 280 755 250) | ADC1 DMA scan |
| Brake sensor | BD SENSORS 26.600 G | ADC1 DMA scan |
| Buzzer | Passive piezo **or** active DC | TIM3 CH1 PWM (PA6) |
| Brake light | External relay/LED driver | GPIO PB12 |
| Drive enable | DTI enable line | GPIO PB15 |
| SDC sense | Shutdown circuit monitor | GPIO PA9 (pull-down) |

---

## Module Breakdown

### `main.c` — Boot, Peripherals, ISRs

The intentionally thin top-level file. STM32CubeIDE regenerates all `MX_*` init functions here on every `.ioc` save, so no application logic lives here. Its responsibilities are:

- System clock configuration (72 MHz PLL from 8 MHz HSE).
- Peripheral initialisation: ADC1, CAN, TIM2, TIM3, I2C1, DMA.
- FreeRTOS primitive creation (`xCanRxQueue`) and task spawning.
- Hardware start-up sequence (CAN start → TIM2 IC → TIM3 PWM → MPU-6050 init → ADC DMA → scheduler).
- Three HAL ISR callbacks: CAN RX, TIM2 capture, I2C DMA complete.

**ISR priority ladder** (lower number = higher HW priority):

| Priority | Peripheral | Purpose |
|---|---|---|
| 4 | DMA1 Ch1 | ADC DMA (APPS + brake pressure) |
| 5 | DMA1 Ch7 | I2C1 RX DMA (MPU-6050 accel data) |
| 6 | TIM2 | IMD PWM input capture |
| ≥5 (SW) | FreeRTOS kernel | `configMAX_SYSCALL_INTERRUPT_PRIORITY` |

All ISR priorities sit **at or below** the FreeRTOS syscall threshold, allowing the use of `FromISR` API variants safely.

---

### `vcu_safety.c / .h` — Safety Module

The most safety-critical file in the project. Owns three distinct concerns:

#### 1. APPS Plausibility — `Process_APPS_Safety_Logic()`

Implements the dual-sensor pedal safety chain required by T11.8.5, T11.8.9, and T11.8.8:

- **T11.8.5** mandates at least two independent APPS sensors that do not share supply or signal lines. This is why two separate ADC channels are used.
- APPS1 has a **direct** characteristic (more pedal → higher ADC count).
- APPS2 has an **inverse** characteristic (more pedal → lower ADC count). This is intentional and directly supports **T11.9** System Critical Signal requirements: a wiring fault (open circuit, short to ground, or short to supply) on either channel immediately causes the two readings to oppose each other (0% vs 100%), guaranteeing the 10 percentage point disagreement threshold defined in T11.9 (implausibility due to out-of-range signals) is instantly exceeded rather than silently passing. See T11.9.7: the ESF must document all potential failure modes for each SCS, the detection strategy, and test evidence.
- Both channels pass through a first-order IIR low-pass filter (`APPS_FILTER_ALPHA = 0.2`) before comparison, suppressing connector vibration and ADC quantisation noise.
- **T11.8.9** defines implausibility as a deviation of more than ten percentage points of pedal travel between any two APPSs, or any failure according to T11.9.
- **T11.8.8** requires that if implausibility persists for more than 100 ms, motor power must be immediately shut down. Disagreement persisting beyond `APPS_CONFLICT_TIMER_MS` (100 ms) latches `apps_fault_active = 1`. This timer must never be increased beyond 100 ms.
- Simultaneously, **EV2.3.1** brake plausibility is checked: if brake pressure AND throttle > 25% (or > 5 kW equivalent, whichever is lower) persist for more than `BRAKE_PLAUSIBILITY_TIMER_MS` (500 ms), `brake_plausibility_active` is latched. Per **EV2.3.2**, this latch only clears when APPS falls below 5% **and** the desired motor torque is 0 Nm.

#### 2. Brake Light & IMU — `vBrakeLightTask()`

Runs at 100 Hz (10 ms period). On each cycle:

1. Copies the three ADC DMA values from `adc_dma_buffer` into `vcu_data`.
2. Calls `Process_APPS_Safety_Logic()` to evaluate pedal plausibility and update the safety fault flags.
3. Initiates a non-blocking DMA read of 14 bytes from the MPU-6050 (all 3 axes + temperature), then blocks waiting for the DMA-complete notification from `HAL_I2C_MemRxCpltCallback`.
4. Converts the raw 16-bit Y-axis reading to g, applies a second IIR filter (`ACCEL_FILTER_ALPHA = 0.1`), and computes `deceleration_g = |accel_y_filtered|`.
5. Applies **T6.3.1** brake light logic. T6.3.1 requires illumination when any of the following are true:
   - **(a)** Hydraulic brake pressure > `HYDRAULIC_PRESS_THRESHOLD` (≈ 0.1 V).
   - **(b)** The regenerative braking system is actuated in accordance with T6.1.10.
   - **(c)** Regen is actuated on accelerator release and deceleration exceeds 1 m/s² ± 0.3 m/s² (0.071 g to 0.133 g; nominal threshold `BRAKE_DECEL_THRESHOLD_G` = 0.102 g).

   In this implementation, conditions (b) and (c) are both detected via IMU: because the DTI manages regen internally and does not expose a dedicated regen-active signal to the VCU, the VCU infers regen from measured deceleration while APPS is below the deadzone. Hysteresis (`BRAKE_DECEL_HYSTERESIS_G` = 0.020 g) prevents rapid toggling near the threshold.

#### 3. Ready-to-Drive Supervisor — `vR2DLogicTask()`

Runs at 50 Hz (20 ms period). Implements a two-state machine:

```
  ┌──────────┐   brake + button pressed     ┌───────────┐
  │  STANDBY │ ──────────────────────────►  │   BUZZER  │
  │          │   (EV4.11.7)                 │  2000 ms  │
  │          │                              │ (EV4.12.1)│
  │          │ ◄── any fault ────────────   │           │
  │          │                              └─────┬─────┘
  │          │                                    │ buzzer complete
  │          │ ◄── SDC open ───────────────────   │
  │          │ ◄── APPS fault ──────────────────  │
  │          │ ◄── brake plausibility ─────────── ▼
  └──────────┘                             ┌───────────┐
                                           │  DRIVING  │
                                           │  ENABLE   │
                                           │  HIGH     │
                                           └───────────┘
```

Fault supervision runs **before** the state machine on every tick. If the SDC sense pin reads LOW (open circuit or broken wire — both fail safe to LOW via external pull-down), or either APPS fault flag is set, `DRIVE_ENABLE` is pulled LOW immediately with no debouncing or grace period.

The RTD buzzer supports both passive piezo (50% duty, 2 kHz square wave) and active DC buzzers (CCR > ARR forces 100% duty), selected by `vcu_data.buzzer_is_dc_mode`.

EV4.11.6 defines ready-to-drive as the state in which the motor(s) will respond to APPS input. Pulling DRIVE_ENABLE HIGH after the buzzer sequence is what satisfies that definition. The gate condition authorising this transition is EV4.11.7 (brake held + button pressed simultaneously).

---

### `can_dti.c / .h` — DTI HV500 CAN Decoder

Runs at FreeRTOS priority 2, blocking indefinitely on `xCanRxQueue`. On each received frame:

- Extracts the DTI packet type from the **upper 6 bits** of the 11-bit Standard ID (`packet_id = StdId >> 5`), with the lower 5 bits encoding the node address.
- Decodes six packet types (0x1F–0x24) into a single `DtiData_t` struct, replacing what was previously ~30 individual global variables.
- All multi-byte fields are big-endian; signed integers use two's-complement with fixed-point scaling as documented in DTI CANBus protocol v2.5.
- Toggles the heartbeat LED (PC13, active-LOW) on every received frame — useful for CAN health monitoring without a scope.

Decoded fields include: ERPM, duty cycle, DC bus voltage, AC/DC currents, controller and motor temperatures, fault codes, d/q axis currents, echoed throttle/brake positions, digital I/O state, and firmware map version.

---

### `imd_monitor.c / .h` — Bender IMD PWM Decoder

Runs at FreeRTOS priority 2, woken by direct task notification from the TIM2 input capture ISR.

The Bender ISOMETER® encodes operating state into PWM **frequency** and isolation resistance into **duty cycle**:

| Frequency | State |
|---|---|
| 10 Hz | Normal — resistance encoded in duty |
| 20 Hz | DC bus undervoltage |
| 30 Hz | Speed Start Test (self-test) |
| 40 Hz | Internal device error |
| 50 Hz | **Ground / isolation fault** |
| Timeout (250 ms) | Disconnected |

In NORMAL state, isolation resistance is calculated from duty cycle using the Bender datasheet formula:

```c
R_iso = ((1200 kΩ × 0.90) / (duty − 0.05)) − 1200 kΩ
```

Results are clamped: 0 kΩ (hard fault) to 999,999 kΩ (open circuit / infinite). Raw TIM2 capture counts (`CCR1`, `CCR2`) are preserved in `imd_live_status` for oscilloscope-free debugging via a live-watch session.

TIM2 is configured in **PWM Input Capture** / slave-reset mode: CH1 captures rising edges (resetting the counter), CH2 captures falling edges. At the moment the CH1 ISR fires, both CCR registers are already updated atomically by hardware — no race condition between period and pulse reads.

---

## Safety Rules Enforced

| Rule | Description | Implementation |
| --- | --- | --- |
| **T11.8.5** | At least two independent APPS sensors required; must not share supply or signal lines | Two ADC channels, inverse characteristic wiring |
| **T11.8.9** | Implausibility defined as >10 percentage point deviation between any two APPSs, or any T11.9 failure | `APPS_IMPLAUSIBILITY_THRESHOLD`, `Process_APPS_Safety_Logic()` |
| **T11.8.8** | If APPS implausibility persists for more than 100 ms, motor power must be immediately shut down | `APPS_CONFLICT_TIMER_MS` (must not exceed 100 ms), `apps_fault_active` flag |
| **T11.9** | APPS are System Critical Signals; open circuit, short to ground, short to supply, and out-of-range failures must result in a safe state. T11.9.7 requires ESF documentation of all failure modes and detection strategies | Inverse-wiring scheme ensures all T11.9 failures cause immediate >10 pp disagreement; see ESF |
| **EV2.3.1** | Commanded torque must be 0 Nm if brakes actuated AND APPS signals >25% torque or >5 kW (whichever is lower) for more than 500 ms | `BRAKE_PLAUSIBILITY_TIMER_MS`, `BRAKE_PLAUSIBILITY_THRESHOLD` |
| **EV2.3.2** | Torque must remain 0 Nm until APPS signals <5% pedal travel AND desired motor torque is 0 Nm, regardless of brake state | `brake_latch` cleared only when both conditions met at `BRAKE_RESET_THRESHOLD` |
| **T6.3.1** | Brake light on: hydraulic brake actuated; OR regen actuated per T6.1.10; OR regen on accelerator release with decel >1 m/s² ±0.3 m/s² | `vBrakeLightTask()`, MPU-6050 Y-axis deceleration, `BRAKE_DECEL_THRESHOLD_G` |
| **EV4.11.7** | R2D transition only possible with mechanical brakes held AND simultaneous dedicated action | `vR2DLogicTask()` STANDBY → DRIVING guard |
| **EV4.11.8** | R2D must exit immediately when shutdown circuit is opened | SDC_SENSE pull-down, instantaneous kill in fault supervisor |
| **EV4.12.1** | RTD characteristic sound continuously for 1–3 s while entering drive | 2000 ms at 2 kHz, `RTD_SOUND_DURATION_MS` |

---

## FreeRTOS Task Map

| Task | Function | Stack | Priority | Period | Wakeup Mechanism |
|---|---|---|---|---|---|
| `BrakeLight` | `vBrakeLightTask()` | 256 words | **3** | 10 ms | `vTaskDelay` + I2C DMA notification |
| `R2D_Logic` | `vR2DLogicTask()` | 256 words | **3** | 20 ms | `vTaskDelay` |
| `DTI_Dec` | `vCanDecodeTask()` | 512 words | 2 | Event-driven | `xQueueReceive` (portMAX_DELAY) |
| `PWM_Proc` | `StartPwmTask()` | 256 words | 2 | Event-driven | `ulTaskNotifyTake` (250 ms timeout) |

**Priority policy:** Safety-critical real-time tasks run at priority 3. Monitoring/decoding tasks run at priority 2. Background tasks (telemetry, logging — not yet implemented) are reserved for priority 1.

Stack sizes were selected empirically using `uxTaskGetStackHighWaterMark()` during development.

---

## Pin Reference

All pin assignments are owned by the STM32CubeIDE `.ioc` file and auto-generated into `main.h`. **Do not re-declare them in application code** — this can silently shadow the generated values with stale constants.

| Label | Pin | Direction | Config | Purpose |
|---|---|---|---|---|
| `APPS1_ADC` | PA2 | Input | ADC1 CH2, Rank 1 | Throttle sensor 1 (direct) |
| `APPS2_ADC` | PA3 | Input | ADC1 CH3, Rank 2 | Throttle sensor 2 (inverse) |
| `BRAKE_PRESSURE` | PA4 | Input | ADC1 CH4, Rank 3 | Hydraulic brake pressure |
| `IMD_OKHS` | PA5 | Input | GPIO, no pull | Bender OK_HS digital output |
| `RTD_BUZZER_T3C1` | PA6 | Output | TIM3 CH1 PWM | RTD acoustic signal |
| `SDC_SENSE` | PA9 | Input | GPIO, **pull-down** | Shutdown circuit monitor |
| `IMD_PWM_IN_T2C1` | PA15 | Input | TIM2 CH1 IC | Bender IMD PWM signal |
| `FAULT_LED` | PA1 | Output | GPIO | Fault indicator |
| `BRAKE_LIGHT` | PB12 | Output | GPIO | Brake light relay |
| `RTD_BUTTON` | PB14 | Input | GPIO, pull-down | Ready-to-drive button |
| `DRIVE_ENABLE` | PB15 | Output | GPIO | DTI inverter enable |
| `FAN_BATTERY` | PB6 | Output | GPIO | Battery cooling fan |
| `FAN_MOTOR` | PB7 | Output | GPIO | Motor cooling fan |
| `CAN_HEART_LED` | PC13 | Output | GPIO, active-LOW | CAN RX heartbeat |

**SDC_SENSE pull-down rationale (EV4.11.8):** SDC closed (normal) → upstream circuit drives PA9 HIGH. SDC open (fault) or broken sense wire → PA9 floats → internal/external pull-down pulls to GND → VCU reads LOW → DRIVE_ENABLE immediately pulled LOW. A wiring fault cannot hold the inverter enabled.

---

## Signal & Data Flow

```
ADC DMA (continuous scan)
  PA2 APPS1 ──► adc_dma_buffer[0]  ┐
  PA3 APPS2 ──► adc_dma_buffer[1]  ├──► vBrakeLightTask ──► Process_APPS_Safety_Logic
  PA4 BRAKE ──► adc_dma_buffer[2]  ┘                          │
                                                              vcu_data.apps_fault_active
                                                              vcu_data.brake_plausibility_active
                                                                  │
I2C DMA (initiated every 10 ms)                                   ▼
  MPU-6050 ──► i2c_rx_buffer[6] ──► vBrakeLightTask ──► vcu_data.accel_y_filtered
                                          │                       │
                                          ▼                       ▼
                                    BRAKE_LIGHT (PB12) ◄──── T6.3.1 logic


CAN 500 kbps
  DTI HV500 ──► FIFO0 ISR ──► xCanRxQueue ──► vCanDecodeTask ──► dti_data.*


TIM2 Input Capture
  PA15 IMD PWM ──► CCR1 (period) ┐
                  CCR2 (pulse)  ─┴──► ISR ──► isr_buffer ──► StartPwmTask ──► imd_live_status.*


R2D Supervisor (20 ms tick)
  PB14 RTD_BUTTON ──────────┐
  PA4  BRAKE_PRESSURE ───────┼──► vR2DLogicTask ──► DRIVE_ENABLE (PB15)
  PA9  SDC_SENSE ────────────┤            │
  vcu_data.apps_fault_active ┤            └──► TIM3 CH1 ──► Buzzer (PA6)
  vcu_data.brake_plaus.      ┘
```

---

## Key Tunable Parameters

> **⚠️ CRITICAL:** Do not alter safety timer thresholds without consulting the FSUK rulebook and the Chief Electrical Engineer.

All safety thresholds are centralised in `vcu_safety.h` with their governing rule numbers. **Read the rule comment before changing any value.**

| `#define` | Default | Rule | Notes |
| --- | --- | --- | --- |
| `APPS_IMPLAUSIBILITY_THRESHOLD` | 0.10 (10%) | T11.8.9 | Maximum sensor disagreement before implausibility is declared |
| `APPS_CONFLICT_TIMER_MS` | 100 ms | T11.8.8 | Must **not exceed** 100 ms — increasing beyond this value is non-compliant |
| `BRAKE_PLAUSIBILITY_THRESHOLD` | 25.0% | EV2.3.1 | Simultaneous brake + throttle trip point. Note: >5 kW may be the binding limit on low-power motors — verify for your installation |
| `BRAKE_PLAUSIBILITY_TIMER_MS` | 500 ms | EV2.3.1 | Must be ≤ 500 ms (allows trail-braking) |
| `BRAKE_RESET_THRESHOLD` | 5.0% | EV2.3.2 | APPS must fall below this AND desired motor torque must be 0 Nm to clear the brake plausibility latch |
| `HYDRAULIC_PRESS_THRESHOLD` | 50 (ADC counts) | T6.3.1 | ≈ 0.1 V on 12-bit ADC |
| `BRAKE_DECEL_THRESHOLD_G` | 0.102 g | T6.3.1 | 1.0 m/s² equivalent. T6.3.1 permits 0.7–1.3 m/s² (0.071–0.133 g); do not set outside this band |
| `BRAKE_DECEL_HYSTERESIS_G` | 0.020 g | T6.3.1 | Prevents rapid brake light toggling near threshold |
| `RTD_SOUND_DURATION_MS` | 2000 ms | EV4.12.1 | Must be 1000–3000 ms |
| `APPS_FILTER_ALPHA` | 0.2 | — | IIR coefficient (0.1 = smooth, 1.0 = raw) |
| `ACCEL_FILTER_ALPHA` | 0.1 | — | IMU IIR coefficient |
| `APPS1_MIN_ADC` / `APPS1_MAX_ADC` | 0 / 4000 | — | Calibrate to your pedal box |
| `APPS2_MIN_ADC` / `APPS2_MAX_ADC` | 0 / 4000 | — | Calibrate to your pedal box |

---

## Build & Flash

### Prerequisites

- STM32CubeIDE (tested on v1.15+)
- STM32CubeMX `.ioc` file (used to regenerate `MX_*` peripheral init code)
- ST-Link V2 programmer

### Steps

1. Clone this repository into your STM32CubeIDE workspace.
2. Open the project (`File → Open Projects from File System`).
3. Build: `Project → Build All` or `Ctrl+B`.
4. Flash: `Run → Debug` (ST-Link) or use STM32CubeProgrammer with the `.hex` from `Debug/`.

### `.ioc` regeneration note

When regenerating from the `.ioc` file, STM32CubeIDE rewrites `main.c` but **preserves** all `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks. All application code in `main.c` lives inside these guards. Never place code outside them.

---

## Debugging Tips

**Live-watch the global structs** in STM32CubeIDE's Expressions view:

- `vcu_data` — all pedal, brake, IMU, fault, and R2D state in one place.
- `dti_data` — full inverter telemetry from the DTI.
- `imd_live_status` — IMD operating state, resistance, and raw TIM2 ticks.

**CAN heartbeat:** The onboard PC13 LED (active-LOW on Blue Pill) toggles on every decoded CAN frame. No LED activity = no CAN frames reaching the decoder (check termination, baud rate, or FIFO0 notification enable).

**IMD timeout:** If `imd_live_status.CurrentState` reads `IMD_STATE_DISCONNECTED`, the TIM2 ISR has not fired in 250 ms. Check PA15 signal integrity and that `HAL_TIM_IC_Start_IT` was called for both channels before the scheduler started.

**APPS calibration:** With the pedal at rest and fully pressed, read `vcu_data.adc_raw_apps1` and `adc_raw_apps2` from the live-watch window and update `APPS1_MIN_ADC` / `APPS1_MAX_ADC` (and APPS2 equivalents) in `vcu_safety.h` accordingly.

**Stack headroom:** Call `uxTaskGetStackHighWaterMark(NULL)` from within a task to verify remaining stack words. Current allocations were validated at 256 words for all tasks except `DTI_Dec` (512 words, larger due to switch-case frame depth).

---

## Future Work

- **Telemetry broadcast** — `vcu_data`, `dti_data`, and `imd_live_status` are structured for easy CAN or UART telemetry framing. Priority-1 background task placeholder is reserved.
- **IMD fault escalation** — `StartPwmTask` correctly sets `IMD_STATE_GROUND_FAULT` and `IMD_STATE_DISCONNECTED` but does not yet command `DRIVE_ENABLE` LOW. While the Bender's physical `OK_HS` pin is directly hardwired to the Shutdown Circuit to handle the actual high-voltage cut, the VCU should mirror this fault in software for telemetry and redundancy purposes.
- **Fault logging** — a ring buffer in SRAM or external Flash to record the fault type, timestamp (FreeRTOS tick), and sensor values at the moment of each safety cut.
- **APPS sensor validation on boot** — check that both APPS channels are within a plausible ADC range before enabling R2D, catching dead sensors before the driver enters the car.
- **Fan control logic** — `FAN_BATTERY` (PB6) and `FAN_MOTOR` (PB7) are GPIO-initialised but not yet driven by temperature thresholds from `dti_data.temp_motor` / `temp_ctrl`.
- **ESF documentation (T11.9.7)** — the ESF must contain a detailed description of all potential failure modes for each System Critical Signal (including APPS), the strategy used to detect each failure, and test evidence. The inverse-wiring scheme's coverage of open circuit, short to ground, and short to supply failures should be explicitly documented and tested there.

---

> **For future team members:** Every `#define` threshold in `vcu_safety.h` has its governing FS rule number beside it. The brake plausibility latch (`EV2.3.2`) is intentional — `DRIVE_ENABLE` staying LOW after releasing the brake is correct behaviour, not a bug. The latch only clears when APPS drops below 5% **and** desired motor torque is 0 Nm. TIM3 is the sole property of `vcu_safety.c`; do not start, stop, or reconfigure it from any other task.
