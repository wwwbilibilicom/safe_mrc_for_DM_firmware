# API Reference

## Control Modes

| Enum Name             | Value | Description               |
|-----------------------|-------|---------------------------|
| `MRC_VOLTAGE_CONTROL` | 0     | Voltage control mode      |
| `MRC_CURRENT_CONTROL` | 1     | Coil current control mode |

### Key Reaction Logic

- **KEY1**: Voltage mode: -1 V per press / Current mode: -0.1 A per press
- **KEY2**: Voltage mode: +1 V per press / Current mode: +0.1 A per press
- **LED1/LED2**: Toggle on key press for feedback
- **UART**: Prints current set value after each key press

## `Device_MRC_t` Structure

| Member                 | Type                  | Description                        |
|------------------------|-----------------------|------------------------------------|
| `state_phase`          | `MRC_State`           | State machine phase                |
| `collision_threshold`  | `float`               | Collision detection threshold      |
| `COLLISION_REACT_FLAG` | `uint8_t`             | Collision reaction flag            |
| `com`                  | `MRC_Com_t`           | RS-485 communication state         |
| `can_com`              | `MRC_Can_Com_t`       | CAN communication state            |
| `LED1`, `LED2`         | `device_led_t`        | LEDs for feedback                  |
| `KEY1`, `KEY2`         | `device_key_t`        | Keys for user input                |
| `Encoder`              | `Device_encoder_t`    | Encoder feedback                   |
| `VNH7040`              | `Device_VNH7040_t`    | H-bridge driver                    |
| `coil_pid`             | `PID_Controller`      | Coil current PI controller         |
| `coil_current_filter`  | `SimpleLowPassFilter` | Coil current low-pass filter       |
| `control_mode`         | `MRC_ControlMode`     | Control mode (voltage/current)     |
| `statemachine`         | `MRC_StateMachine_t`  | State machine for safety/mode mgmt |

All members and functions are documented in the header files with Doxygen-style comments.

## MRC Device Functions

### `MRC_Init(dev_name, mrc, id)`

Initialize MRC device with communication support.

- `dev_name`: Device name string
- `mrc`: `Device_MRC_t *` - MRC device structure pointer
- `id`: Device ID (0-255)

### `MRC_set_voltage(mrc)`

Set coil voltage based on target value.

### `MRC_Com_Process(mrc)`

Handle communication exchange using request-response pattern.

### `MRC_collision_detect(mrc)`

Detect collision for the MRC device.

## Communication Functions

| Function | Description |
|----------|-------------|
| `MRC_Com_Init(mrc_com, huart, id)` | Initialize communication module with DMA idle reception |
| `MRC_Com_UnpackCmd(mrc_com)` | Unpack and validate command from DMA buffer |
| `MRC_Com_PackFbk(mrc_com, mode, encoder, velocity, current, collision)` | Pack feedback message with device status |
| `MRC_Com_SendFbk(mrc_com)` | Send feedback response using DMA |
| `MRC_Com_Reset(mrc_com)` | Reset communication status and restart DMA reception |

## Coil Current Control (Feedforward + PI)

**Function:** `float MRC_CoilCurrentControl_Update(Device_MRC_t *MRC)`

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| `i_ref` | Target current (A) |
| `i_meas` | Measured current (A) |
| `R_coil` | Coil resistance (Ohm, DC value from multimeter) |
| `L_coil` | Coil inductance (H, 20 kHz value from LCR meter) |
| `Ts` | Sample time (seconds) |

**PI Tuning** (macros in `Devices/Inc/drv_mrc.h`):

```c
COIL_PID_KP      // Proportional gain
COIL_PID_KI      // Integral gain
COIL_PID_KD      // Derivative gain
COIL_PID_TS      // Sample time
COIL_PID_MAX_OUT // Maximum output
COIL_PID_MIN_OUT // Minimum output
```

**Tuning Advice:** Start with recommended values. Increase `Kp` for faster response, then add `Ki` for zero steady-state error. Reduce `Kp`/`Ki` if overshoot occurs.

## Filters

| Filter | Type | Use Case |
|--------|------|----------|
| `SimpleLowPassFilter` | Exponential smoothing | General noise reduction (supports cutoff freq or alpha init) |
| `movingAverage_t` | Moving average | Removes high-frequency noise |
| `FirstOrderKalmanFilter` | First-order Kalman | Noisy signal estimation |
| `BandPassFilter` | Band-pass | Extracts specific frequency bands |

See `Common/Inc/filter.h` for initialization and update functions.

## PWM Voltage Lookup Table

**Function:** `float LookupMeasuredVoltageByTarget(float v_target)`

- For |v_target| <= 5 V: Uses LUT and linear interpolation
- For |v_target| > 5 V: Uses v_target directly
- **Purpose**: Compensates for H-bridge dead-zone and nonlinearity at low voltages

Sample data:

| Target Voltage (V) | Measured Voltage (V) | Duty Cycle (0~1) | Output Current (A) |
|---------------------|----------------------|-------------------|---------------------|
| 0                   | 0.0017               | 0                 | -0.062              |
| 0.1                 | 0.0017               | 0.00833           | -0.062              |
| 0.2                 | 0.0017               | 0.01667           | 0.062               |
| ...                 | ...                  | ...               | ...                 |
| 5.0                 | 5.752                | 0.41667           | 1.200               |

The full table is in code and can be updated as needed.

## Timer Resource Allocation

| Timer | Rate | Purpose | Notes |
|-------|------|---------|-------|
| TIM1  | -    | Encoder PWM capture (input capture) | CH4, reads encoder PWM signal |
| TIM2  | -    | PWM generation for VNH7040 H-bridge | CH1, main coil drive output |
| TIM4  | **10 kHz** | Coil current control loop | Sets `MRC.coil_current_update_flag` |
| TIM6  | **1 kHz** | State machine & feedback TX | Sets `MRC.control_loop_flag` |

## Example: Main Loop

```c
Device_MRC_t mrc_device;
MRC_Init("MRC_Device", &mrc_device, 0x01);

while(1) {
    MRC_Com_Process(&mrc_device);
    MRC_Key1_Reaction(&mrc_device);
    MRC_Key2_Reaction(&mrc_device);
    if (mrc_device.control_mode == MRC_CURRENT_CONTROL) {
        float i_meas = MRC_Update_Coil_Current(&mrc_device);
        float v_cmd = MRC_CoilCurrentControl_Update(&mrc_device);
    } else {
        MRC_set_voltage(&mrc_device);
    }
}
```

## Configuration Defines

```c
#define MRC_COIL_MAX_VOLTAGE 12.0f    // Maximum coil voltage
#define PWM_FREQ 20000                // PWM frequency
#define ENCODER_RESOLUTION 4096       // Encoder pulses per revolution
```
