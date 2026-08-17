# Carding-MainBoard_v8 — Carding Machine Main Control Board Firmware

STM32CubeIDE firmware for the **Carding machine main/mother control board** on GEN4 machines. 
## Hardware target

- **MCU:** STM32G431CBT3 (LQFP48), STM32G4 family
- **Peripherals in use (per `.ioc`):** ADC2, DAC1, FDCAN1, I2C2, TIM6, TIM7, TIM15, TIM16, TIM17, USART1 (+ RX DMA), USART2 (+ TX DMA), GPIO/EXTI, NVIC
- **Toolchain:** STM32CubeIDE / GCC, generated from `Carding-MainBoard_v8.ioc`

## What this board does

- Runs the machine-wide **state machine** — idle, run, debug, diagnostics, settings, error — dispatched every main-loop iteration. (Unlike the Flyer Frame board, there's no homing/GB-calibration/finished state here — carding has no lift motors.)
- Acts as **FDCAN master** for **8 motor/inverter stations**: Carding Cylinder, Beater Cylinder, Cage, Card Feed, BTR (Beater) Feed, Coiler, AF Picker Cylinder, and AF Feed. Sends setup/run/change-target commands and diagnostics requests; parses run data, ACKs, and errors coming back. Two of these stations (Cylinder, Beater Cylinder — see `ExtendedRunTime_TypeDef ER[2]`) report **extended FOC/VFD-inverter telemetry** (bus voltage, Id/Iq axis current, peak phase current/voltage, fault codes) rather than the simpler six-sector run-time frame used by the rest.
- Bridges the machine to the **mobile app over Bluetooth** (USART1, TLV-framed protocol): settings sync, diagnostics, run/state broadcasts, and machine parameters.
- Owns **machine/user settings** persistence in I2C EEPROM — delivery rate, length limit, cylinder/beater/picker RPMs, feed RPMs, card-feed delivery ratio — with default-load/fallback logic, and derives per-station RPMs (`setupCardingMCType`, `ReadySetupCommand_AllMotors`) from machine geometry (gearbox ratios, cage/coiler/tongue-groove circumferences).
- Monitors machine-level **errors** (ACK failures, SMPS faults) and per-motor errors relayed over CAN.
- Reads **duct-level sensors** for the card-feed duct and the auto-feed (AF) duct via an MCP23017 I/O expander, plus a coiler TG (tongue-and-groove) optical sensor — each with configurable hysteresis and dead-time to avoid chatter, and duct-fill state reporting to the app.
- Reads an **analog tension-draft potentiometer** (`TensionDraft_Pot`) on ADC2, banding the raw reading into discrete draft levels and deriving a max coiler RPM/draft relationship, with a debounce/change-counter and an audible beep on a confirmed setting change.
- Drives an **AC solid-state relay** (`AC_SSR`, via the MCP23017's port A) — used for a mains-switched machine accessory — with state verification readback.
- Drives the **SMPS** power supply, **tower lamp**, status **LEDs**, and reads the panel **user buttons/rotary switch** (same drivers as the Flyer Frame board).
- Provides a **serial logging** channel (USART2) for run-time telemetry.

## Application flow (`Core/Src/main.c`)

1. `HAL_Init()` → `SystemClock_Config()` → `MX_*_Init()` for all peripherals.
2. Application-level init: diagnostics/motherboard-error/BT structs, internal settings.
3. Loads **user settings** from EEPROM (`ReadUserSettingsFromEeprom` → `CheckUserSettings`); if invalid, loads and writes defaults (delivery rate, length limit, cylinder/BTR/picker RPMs, feed RPMs, card-feed delivery ratio). Derives the full `CardingMc` machine-parameter struct (`setupCardingMCType`) and prepares all-motor setup commands (`ReadySetupCommand_AllMotors`). Initializes the tension-draft pot from the derived setting.
4. Initializes the MCP23017, sets the tower lamp to its off state, reads initial button/rotary states, and enters `IDLE_STATE`.
5. Sets up Bluetooth (manual setup, with a reboot-retry path if setup fails), configures duct-sensor hysteresis/dead-time for both the card-feed and AF ducts from stored internal settings, reads their initial states, and sets the coiler TG sensor hysteresis.
6. Turns the AC SSR off, sets up UART1 (Bluetooth) RX via DMA + idle/TX-complete interrupts and UART2 (logging) TX-complete interrupt.
7. Starts the 100 ms state-broadcast timer (TIM7) and 1 s CAN-health-check timer (TIM16, currently disabled — see notes below), then powers up the SMPS (with a 1 s settle delay for the contactor).
8. **Main loop** dispatches over `S.current_state` to `IdleState()`, `RunState()`, `DebugState()`, `DiagnosticsState()`, `SettingsState()`, or `ErrorState()` in `Core/Src/MachineStates/`. `RunState` further tracks carding-specific run sub-modes: `RUN_RAMPUP` → `RUN_FILL_DUCT` → `RUN_CARDING_SECTION` → `RUN_PAUSED`/`RUN_STOPPED`/`RUN_OVER` — the machine ramps up, waits for the feed duct to fill, then runs the carding section proper.
9. ISRs handle time-critical work: `HAL_TIM_PeriodElapsedCallback` (ACK timeout/result, per-motor diagnostics streaming since carding diagnoses one motor at a time, 100 ms BT tick + tension-pot check + duct sensor timers), `HAL_GPIO_EXTI_Callback` (SMPS fault detection, panel buttons), and the FDCAN RX callback hands every received frame to `FDCAN_parseForMotherBoard()`.

## Module map (`Drivers/`)

| Module | Files | Responsibility |
|---|---|---|
| **FDCAN** | `FDCAN/FDCAN.c/h` | Low-level FDCAN peripheral setup, the 8 station addresses (Cylinder 0x02, Beater Cylinder 0x03, Cage 0x04, Card Feed 0x05, BTR Feed 0x06, Coiler 0x07, Picker Cylinder 0x08, AF Feed 0x09), function IDs, priority levels |
| **CAN_MotherBoard** | `FDCAN/MotherBoard/CAN_MotherBoardFns.c`, `CAN_MotherBoard.h` | Application-layer CAN traffic: send setup/commands/diagnostics/change-target to each station, parse incoming ACKs, run data (including the extended cylinder-inverter frame), and errors |
| **MotorComms** | `MotorComms/` | Builds per-station `SetupMotor` structs (RPM, ramp times, gear-ratio-derived targets) from machine settings for all 8 stations |
| **MachineSettings** | `MachineSettings/machineSettings.*`, `machineEepromFns.c`, `machineEepromSettings.h` | User/machine settings model (delivery rate, length limit, per-station RPMs, gearbox ratios and circumferences for cylinder/cage/coiler/tongue-groove), EEPROM read/write, defaults, `CardingMc` parameter derivation |
| **MachineErrors** | `MachineErrors/` | Machine/motor error taxonomy and error-latching logic (shared design with the Flyer Frame board) |
| **MachineSensors** | `MachineSensors/` | Card-feed duct, AF duct, and coiler TG sensor state with hysteresis and dead-time, duct open/closed/reset and fill-level (low/correct/high) reporting |
| **TensionDraft_Pot** | `TensionDraft_Pot/TD_Pot.c/h` | Reads the analog tension-draft pot on ADC2, bands it into 10 draft levels between `MIN_TD_DRAFT`/`MAX_TD_DRAFT`, derives max coiler RPM from delivery rate, debounces changes with a counter and an audible beep |
| **AC_SSR** | `AC_SSR/AC_SSR.c/h` | Sets and verifies the state of an AC solid-state relay output via the MCP23017's port A |
| **Eeprom** | `Eeprom/` | I2C EEPROM byte/int/float read-write primitives |
| **MCP23017** | `MCP23017/` | I2C GPIO expander driver — shared between duct sensors, tower lamp, and the AC SSR output |
| **SysObserver** | `SysObserver/` | Per-station CAN traffic health monitoring (the 1 s timer that drives this is currently commented out in `main.c` — see notes) |
| **DataRequest** | `DataRequest/` | Request/response tracking for pulling settings from a station board, with timeout |
| **SMPS** | `SMPS/` | Switch-mode power supply on/off control and fault-signal handling |
| **TowerLamp** | `TowerLamp/` | Buzzer/red/green/amber tower lamp control via MCP23017 |
| **MB_LEDs** | `MB_LEDs/` | Status LED toggling tied to machine state |
| **userButtons** | `userButtons/` | Panel push-buttons (red/green/yellow) and run/setup rotary switch |
| **Bluetooth** | `Bluetooth/BT_Fns.*`, `BT_Machine.*`, `BT_Console.*` | TLV-framed serial protocol to the mobile app: settings sync, diagnostics, run/state broadcasts, plus a console/manual BT-module setup path |
| **SerialLogging** | `SerialLogging/Log.c/h` | USART2 telemetry logging |

## Core application logic (`Core/`)

- **`Struct.h` / `InitializeTypeDefs.c`** — shared cross-module structs: `DiagnosticsTypeDef`, the standard `RunTime_TypeDef` (used by 6 of the 8 stations), and the carding-specific `ExtendedRunTime_TypeDef` (used by the 2 cylinder inverter stations — bus voltage, Id/Iq, peak phase current/voltage, fault codes).
- **`CommonConstants.h`** — global state-machine IDs, motor command codes, shared gain constants (kept identical to the Flyer Frame board even though carding doesn't use the homing/GB-calib states defined here).
- **`StateMachine.h/.c`** — `StateTypeDef` (current/prev state, run sub-mode, BT flags, SMPS control, `AC_SSR_On`, `TD_POT_check`, piecing-mode flags) and state-transition helpers. Run sub-modes are carding-specific: `RUN_RAMPUP`, `RUN_FILL_DUCT`, `RUN_CARDING_SECTION`, `RUN_PAUSED`, `RUN_STOPPED`, `RUN_OVER`.
- **`Ack.h/.c`** — CAN command acknowledgement tracking (expected vs. received ACKs, critical vs non-critical).
- **`MachineStates/`** — `IdleState.c`, `RunState.c` (largest — the ramp-up/fill-duct/carding-section sequencing), `DebugState.c`, `DiagnosticsState.c`, `SettingsState.c`, `ErrorState.c`. No `HomingState`, `GB_Calibration`, or `FinishState` — this machine has no lift motors.

## Repository layout

```
Carding-MainBoard_v8/
├── Carding-MainBoard_v8.ioc          CubeMX peripheral/pin configuration
├── STM32G431CBTX_FLASH.ld            Linker script
├── Core/
│   ├── Inc/                          main.h, Struct.h, CommonConstants.h, StateMachine.h, Ack.h, stm32g4xx_it.h, HAL config
│   └── Src/
│       ├── main.c                    Entry point, init, ISRs, state-machine dispatcher
│       ├── StateMachine.c, Ack.c, InitializeTypeDefs.c
│       └── MachineStates/            IdleState, RunState, DebugState, DiagnosticsState, SettingsState, ErrorState
├── Drivers/
│   ├── FDCAN/                        FDCAN.c/h + MotherBoard/ (CAN_MotherBoardFns.c, CAN_MotherBoard.h)
│   ├── MotorComms/, MachineSettings/, MachineErrors/, MachineSensors/
│   ├── TensionDraft_Pot/, AC_SSR/    Carding-specific: analog draft pot, AC SSR output
│   ├── Eeprom/, MCP23017/, SysObserver/, DataRequest/
│   ├── SMPS/, TowerLamp/, MB_LEDs/, userButtons/, Bluetooth/, SerialLogging/
│   ├── CMSIS/                        ARM CMSIS core + STM32G4 device headers
│   └── STM32G4xx_HAL_Driver/         ST HAL driver sources
└── Debug/                            Build output (generated by STM32CubeIDE)
```
