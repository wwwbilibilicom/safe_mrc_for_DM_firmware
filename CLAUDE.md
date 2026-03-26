# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

- **IDE**: Keil MDK-ARM v5.37+. Open `MDK-ARM/safeMRC_for_DM_firmware.uvprojx`.
- **Build**: Press F7 in Keil, or use `UV4.exe` from the MDK install directory.
- **Peripheral config**: Use STM32CubeMX with `safeMRC_for_DM_firmware.ioc`. After regenerating, manually restore all `/* USER CODE BEGIN */` blocks — CubeMX does not overwrite them.
- **Flash**: Use ST-Link via Keil debug/download, or STM32CubeProgrammer.
- There are no automated tests or linting steps.

## Host Python UI (scripts/)

```bash
conda env create -f scripts/environment.yml
conda activate safemrc_host
python scripts/main.py
```

Requires a USB-RS485 adapter at up to 4 Mbps. The Python SDK is in `scripts/safeMRC_sdk.py`.

## Architecture

### Layering

```
Core/           HAL-generated peripheral init (CubeMX output)
Common/         Reusable utilities: PID, filters, flash, sys_clock
Devices/        MRC application drivers (the primary logic layer)
Applications/   Callbacks wiring HAL interrupts to device logic
scripts/        PC-side Python UI + SDK
```

### Single Global Device Instance

`main.c` declares one global `Device_MRC_t MRC`. All application code operates on this instance. There is no RTOS — all concurrency is managed via flags set in timer ISRs and polled in the main loop.

### Timing Architecture

| Timer | Rate   | Purpose |
|-------|--------|---------|
| TIM6  | 1 kHz  | Sets `MRC.control_loop_flag`; triggers `MRC_StateMachine_Task1ms()` + RS485 feedback TX |
| TIM4  | 10 kHz | Sets `MRC.coil_current_update_flag`; triggers `MRC_CoilCurrentControl_Update()`, `MRC_Com_Process()`, encoder filter |

### Communication

- **RS485 (USART2)**: Primary control bus. 4 Mbps, DMA+IDLE-interrupt receive. Binary protocol: 10-byte command (host→device), 17-byte feedback (device→host). Both frames: `0xFE 0xEE` header + device ID + payload + CRC-16-CCITT (poly=0x1021, init=0xFFFF, no reflection, no XOR-out). Multi-byte fields are little-endian.
- **Debug CLI (USART1)**: `printf` via DMA (`retarget_print.c`). Interactive CLI initialized with `MRC_DebugCLI_Init(&huart1)`. Supports `DEBUG ON/OFF`, `MODE VOLTAGE/CURRENT`, `SET VOLTAGE/CURRENT <val>`, `ID CHECK/CHANGE <n>`, `RES CHECK/CHANGE <ohm>`.
- **FDCAN1**: Added in `class_can` branch. Initialized via `bsp_can_init()` / `can_filter_init()` in `Devices/Src/bsp_fdcan.c`. RX via FIFO0 watermark interrupt.

### State Machine (`Devices/Src/mrc_statemachine.c`)

`MRC_StateMachine_t` inside `Device_MRC_t` manages operating modes: `FREE`, `FIX_LIMIT`, `ADAPTATION`, `DEBUG`. Collision detection in ADAPTATION mode triggers automatic demagnetization. `MRC_StateMachine_Task1ms()` is called every 1 ms from the TIM6 ISR.

### Control Loop (`Devices/Src/drv_mrc.c`)

- **Voltage mode** (`MRC_VOLTAGE_CONTROL`): Directly maps target voltage through the LUT in `LookupMeasuredVoltageByTarget()` to compensate H-bridge dead-zone nonlinearity below 5 V, then drives `Device_VNH7040_t` PWM.
- **Current mode** (`MRC_CURRENT_CONTROL`): Feedforward (V = I_ref * R_coil) + PI on current error. 10 kHz update. Tuning macros in `Devices/Inc/drv_mrc.h`: `COIL_PID_KP`, `COIL_PID_KI`, `COIL_PID_TS`, `COIL_PID_MAX_OUT`, `COIL_PID_MIN_OUT`.

### Persistent Flash Parameters (STM32H723xG — 1 MB, 128 KB sectors)

| Parameter       | Address        | Sector |
|-----------------|----------------|--------|
| Device ID       | `0x080E0000`   | Last   |
| Coil resistance | `0x080C0000`   | Second-to-last |

On first boot (erased flash), firmware writes defaults and persists them. `ID CHANGE` / `RES CHANGE` CLI commands update both RAM and flash immediately.

## Key Files

| File | Role |
|------|------|
| [Core/Src/main.c](Core/Src/main.c) | Entry point, global `MRC` instance, peripheral init order |
| [Devices/Inc/drv_mrc.h](Devices/Inc/drv_mrc.h) | `Device_MRC_t` struct, control-mode enums, PID tuning macros |
| [Devices/Src/drv_mrc.c](Devices/Src/drv_mrc.c) | `MRC_Init`, `MRC_CoilCurrentControl_Update`, voltage/current control, collision detect |
| [Devices/Src/mrc_com.c](Devices/Src/mrc_com.c) | RS485 pack/unpack/send functions |
| [Devices/Inc/mrc_protocol.h](Devices/Inc/mrc_protocol.h) | Frame structs, `MRC_Mode` enum, buffer sizes |
| [Devices/Src/mrc_statemachine.c](Devices/Src/mrc_statemachine.c) | Mode transitions, demagnetization sequencing |
| [Devices/Src/mrc_debugcli.c](Devices/Src/mrc_debugcli.c) | CLI command parser for USART1 |
| [Devices/Src/bsp_fdcan.c](Devices/Src/bsp_fdcan.c) | FDCAN1 init, filter config, baud-rate tables |
| [Common/Src/flash.c](Common/Src/flash.c) | Sector-erase + flashword write helpers for ID/resistance persistence |
| [Common/Inc/filter.h](Common/Inc/filter.h) | `SimpleLowPassFilter`, `movingAverage_t`, `FirstOrderKalmanFilter`, `BandPassFilter` |
| [scripts/safeMRC_sdk.py](scripts/safeMRC_sdk.py) | Python SDK mirroring the embedded protocol |

## Protocol Consistency Rule

The embedded firmware (`mrc_protocol.h`, `crc_ccitt.c`) and the Python host (`scripts/safeMRC_sdk.py`) must always use identical frame formats and CRC parameters. Any protocol change requires updating both sides.
