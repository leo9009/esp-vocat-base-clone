# ESP-EchoEar Rotating Base

[中文](README_CN.md) | **English**

## Project Introduction

This project is the firmware for the rotating base of Espressif's EchoEar intelligent interactive device, developed based on the ESP32-C61 chip. The project implements precise stepper motor control, magnetic slide switch detection, UART communication, and other features, providing vivid interactive actions and sensitive input detection for smart devices.

## Main Features

### Stepper Motor Control
- **Multiple Predefined Actions**:
  - Head shaking action (fixed amplitude/gradual decay)
  - Look around observation action
  - Cat nuzzle action
  - Beat swing action following rhythm

All actions are parameterizable for easy integration with LLMs like Doubao and Xiaozhi.
- **Precise Angle Control**: Supports arbitrary angle rotation with precision up to 0.1 degrees
- **Smooth Acceleration/Deceleration**: Uses acceleration/deceleration algorithms for more natural and fluid motion
- **Half-Step Drive Mode**: Doubles precision for smoother movement
- **Auto Homing Calibration**: Automatically returns to zero position via limit switch on startup

### Magnetic Slide Switch
- **Multiple Sensor Support**:
  - BMM150 3-axis magnetometer
  - QMC6309 3-axis magnetometer
  - Linear Hall sensor

Choose one of the above three sensors, BMM150 is selected by default.
- **Rich Event Detection**:
  - Slider up/down sliding (SLIDE_UP / SLIDE_DOWN)
  - Remove from up/down position (REMOVE_FROM_UP / REMOVE_FROM_DOWN)
  - Place back from up/down position (PLACE_FROM_UP / PLACE_FROM_DOWN)
  - Single click event (SINGLE_CLICK)
- **Smart Calibration Function**:
  - Auto-guided calibration on first use
  - Calibration data persistent storage (NVS Flash)
  - Support UART command triggered recalibration
  - Support long press Boot button triggered recalibration
  - Auto-adapt to different magnetic field environments

### UART Communication
- **UART Protocol Communication**: Data communication with EchoEar
- **Command Types**:
  - Angle control command (CMD_BASE_ANGLE_CONTROL)
  - Preset action control command (CMD_BASE_ACTION_CONTROL)
  - Preset action completion notification (CMD_ACTION_COMPLETE)
  - Magnetic switch event reporting (CMD_MAGNETIC_SWITCH_EVENT)
  - Calibration progress notification (MAG_SWITCH_CALIB_*)
- **Data Frame Format**: `AA 55 [LEN_H] [LEN_L] [CMD] [DATA...] [CHECKSUM]`
- **Verification Mechanism**: Supports **frame header** verification and **checksum** validation

## Hardware Requirements

### Main Control Chip
- **ESP32-C61-WROOM-1** or **ESP32-C61-WROOM-1U** module

ESP32-C61-WROOM-1 is used by default

### Peripheral Connection and Pin Assignment

#### Stepper Motor (28BYJ-48 + ULN2003 Driver IC)
- IN1 → GPIO28
- IN2 → GPIO27
- IN3 → GPIO26
- IN4 → GPIO25

#### Magnetometer Sensor (I2C)
- SCL → GPIO3
- SDA → GPIO2
- Supports BMM150 (I2C address: 0x10)
- Supports QMC6309 (I2C address: 0x7C)

#### Linear Hall Sensor (ADC)
- ADC_PIN → GPIO5 (ADC_CHANNEL_3)

#### Turntable Homing Calibration Limit Switch
- Homing calibration limit switch detection pin → GPIO1 (active low)

#### BOOT Button
- BOOT button → GPIO9 (active low)

Used by default for long press to trigger magnetic slide switch calibration.

#### UART Communication
- TXD → GPIO29
- RXD → GPIO8
- Baud rate: 115200

## Software Architecture

### Main Components

#### 1. stepper_motor (Stepper Motor Control)
- Implements half-step drive control algorithm
- Provides acceleration/deceleration curves
- Encapsulates multiple predefined actions
- Supports precise angle rotation

#### 2. magnetic_slide_switch (Magnetic Slide Switch)
- Sliding window filtering algorithm
- State machine implements event detection
- Single click detection (peak detection method)
- Auto calibration process
- Calibration data NVS storage

#### 3. control_serial (UART Communication)
- UART TX/RX tasks
- Protocol frame parsing
- Command dispatch processing
- Event reporting

#### 4. BMM150_SensorAPI (Sensor Driver)
- BMM150 official API adaptation
- I2C bus encapsulation

### Task Structure
```
app_main
├── base_calibration_task     // Base angle calibration task (on startup)
├── uart_cmd_receive_task     // UART command receive task
├── uart_cmd_send_task        // UART command send task
└── magnetometer_read_task    // Magnetometer read task
```

## Quick Start

### Environment Preparation

1. **ESP-IDF Version**

- ESP-IDF release/v5.5(v5.5-445-g1191f4c4f91-dirty)

### Project Configuration

Use `idf.py menuconfig` to select the sensor for the magnetic slide switch:

 `Component config → Magnetic Slide Switch → Sensor Type`

Select sensor type: BMM150 / QMC6309 / Linear Hall Sensor

### Build and Flash

```bash
# Build project
idf.py build
# Flash firmware
idf.py flash
# View logs
idf.py monitor
```

## Usage Instructions

### First Startup

1. **Base Angle Calibration**:
   - After power-on, the motor will automatically rotate left to find the limit switch
   - After touching the limit switch, the motor will rotate right 95° to the center position
   - After calibration is complete, the motor powers off

2. **Magnetic Switch Calibration** (if first use):
   - System prompt: `Please keep the slider in current position and wait for calibration...`, and send `0x11` to EchoEar to notify it that the calibration has started
   - Keep the slider in the current position, wait for data to stabilize (about 1 second)
   - After the system records the first position, prompt: `Please move the slider to another position...`
   - Move the slider to another position (up/down/removed), wait for data to stabilize
   - After the system records the second position, send `0x12` to EchoEar, EchoEar gives voice prompt for the third step
   - Move the slider to the third position, wait for data to stabilize
   - After the system records the third position, calibration is complete, send `0x13` to EchoEar to notify EchoEar calibration is complete
   - Calibration data is automatically saved to Flash, automatically loaded on next startup

### Recalibration

**Method 1: Long Press Boot Button**
- Long press the Boot button on GPIO9 (about 1.5 seconds)
- System automatically enters recalibration mode

**Method 2: UART Command**
- Send command: `AA 55 00 03 03 00 10 16`
- System enters recalibration mode after receiving the command

### UART Commands

#### 1. Magnetic Attachment Notification Command（0x00）
`
AA 55 00 03 00 00 01 01
`

The base periodically sends this command. Once EchoEar is magnetically attached and successfully receives the command, it can confirm that the connection to the base has been established and then start playing the connection animation.

#### 2. Angle Control Command (0x01)
**Format**: `AA 55 00 03 01 [ANGLE_H] [ANGLE_L] [CHECKSUM]`

**Example**: Rotate to 45°
```
AA 55 00 03 01 00 2D 2E
```
- `00 2D` = 45 (decimal)
- `2E` = `01 + 00 + 2D` (checksum)

#### 3. Action Control Command (0x02)
**Format**: `AA 55 00 03 02 00 [ACTION] [CHECKSUM]`

**Action Codes**:
- `0x01`: Shake head (SHAKE_HEAD)
- `0x02`: Look around (LOOK_AROUND)
- `0x03`: Beat swing - drum (BEAT_SWING_DRUM)
- `0x04`: Cat nuzzle (CAT_NUZZLE)
- `0x05`: Beat swing - other instruments (BEAT_SWING_OTHER)

**Example**: Execute look around action
```
AA 55 00 03 02 00 02 04
```

#### 4. Magnetic Switch Event Reporting (0x03)
**Format**: `AA 55 00 03 03 00 [EVENT] [CHECKSUM]`

**Event Codes**:
- `0x01`: Slider slides down
- `0x02`: Slider slides up
- `0x03`: Remove from top
- `0x04`: Remove from bottom
- `0x05`: Place back from top
- `0x06`: Place back from bottom
- `0x07`: Single click event
- `0x11`: Second position calibration complete (EchoEar gives voice prompt after receiving)
- `0x12`: Calibration complete

#### 5. Action Completion Notification (0x02)
**Format**: `AA 55 00 03 02 00 10 12`

- Base automatically sends after completing any specific action
- EchoEar can synchronize status according to this command

## Stepper Motor Action Details

### 1. stepper_shake_head (Shake Head)
```c
stepper_shake_head(float amplitude, int cycles, int speed_us);
```
- `amplitude`: Swing amplitude (degrees)
- `cycles`: Number of swings
- `speed_us`: Speed (microseconds/step)

### 2. stepper_shake_head_decay (Gradual Decay Shake Head)
```c
stepper_shake_head_decay(float initial_amplitude, float decay_rate, int speed_us);
```
- `initial_amplitude`: Initial amplitude (degrees)
- `decay_rate`: Decay coefficient (0.0-1.0, e.g., 0.8 means 20% decay each time)
- `speed_us`: Speed (microseconds/step)

### 3. stepper_look_around (Look Around)
```c
stepper_look_around(float left_angle, float right_angle, float scan_angle, 
                    int pause_ms, int large_speed_us, int small_speed_us);
```
- `left_angle`: Maximum left angle
- `right_angle`: Maximum right angle
- `scan_angle`: Small amplitude scan angle
- `pause_ms`: Pause time after each rotation
- `large_speed_us`: Large amplitude rotation speed
- `small_speed_us`: Small amplitude scan speed

### 4. stepper_cat_nuzzle (Cat Nuzzle)
```c
stepper_cat_nuzzle(float angle, int cycles, int speed_us);
```
- `angle`: Nuzzle amplitude (degrees)
- `cycles`: Number of nuzzles
- `speed_us`: Speed (microseconds/step)

### 5. stepper_beat_swing (Beat Swing)
```c
stepper_beat_swing(float angle, int speed_us);
```
- `angle`: Swing angle (degrees)
- `speed_us`: Speed (microseconds/step)
- **Smart Swing Logic**:
  - Same instrument type continuously triggered → Swing 15 degrees to the other side

## Magnetic Switch Calibration Principle

### Calibration Flow State Machine

```
CALIBRATION_FIRST_POSITION    // Wait for first position to stabilize
         ↓                    // (Send 0x11 to notify EchoEar)
CALIBRATION_WAIT_SECOND       // Wait to move to second position
         ↓                    // (Enter settling period after detecting movement)
CALIBRATION_SECOND_POSITION   // Wait for second position to stabilize
         ↓                    // (Send 0x12 to notify EchoEar)
CALIBRATION_WAIT_THIRD        // Wait to move to third position
         ↓                    // (Enter settling period after detecting movement)
CALIBRATION_THIRD_POSITION    // Wait for third position to stabilize
         ↓
CALIBRATION_COMPLETED         // Auto sort and save to Flash
                              // (Send 0x13 to notify EchoEar)
```

### Key Technical Points

1. **Sliding Window Filtering**: 5-point moving average to reduce noise interference
2. **Stability Detection**: Only considered stable after consecutive multiple times (15 times) with value changes less than threshold
3. **Settling Period Mechanism**: After detecting movement, wait for data to fully stabilize (about 500ms) before recording
4. **Auto Sorting**: After calibration is complete, automatically assign to REMOVED/UP/DOWN according to magnetic field strength
5. **NVS Persistence**: Calibration data saved to Flash, not lost after power off

### Magnetic Field Value Reference Range (BMM150)
- **REMOVED** (removed): ~1230 ± 100
- **UP** (up position): ~1619 ± 100
- **DOWN** (down position): ~1894 ± 100

*Note: Actual values vary due to magnet strength and installation position, calibration is required*

### Common Issues

**Q1: Motor vibrates or doesn't rotate**
- Check if power supply is sufficient (at least 5V 1A)
- Check if wiring is correct
- Try reducing speed (increase `speed_us` parameter)

**Q2: Magnetic switch calibration fails**
- Ensure sensor I2C connection is normal
- Check if sensor type configuration matches
- Move the slider with clear actions, wait for stability prompt before moving
- For slow movement, check if settling period mechanism has been optimized

**Q3: UART communication abnormal**
- Check if baud rate is 115200
- Confirm TX/RX wiring is correct
- Check if checksum is calculated correctly

**Q4: Motor rotates in wrong direction on first power-on**
- Check limit switch installation position
- Adjust rotation angle in `base_calibration_task`

## Performance Parameters

- **Angle Control Precision**: ±0.5°
- **Fastest Rotation Speed**: 600 μs/step (about 200°/second)
- **Magnetic Switch Response Time**: < 50ms
- **Magnetic Switch Sampling Rate**: 200 Hz (BMM150) / 500 Hz (QMC6309)
- **UART Communication Rate**: 115200 bps
- **Memory Usage**: ~80KB RAM, ~200KB Flash

## Version History

### v1.0.0
- Stepper motor basic control
- Multiple predefined actions
- Magnetic slide switch event detection
- Auto calibration function
- NVS data persistence
- UART communication protocol
- Smart beat following algorithm
- Long press Boot button recalibration
- Calibration process UART communication
- Slow movement calibration optimization

## License

This project follows the GPL 3.0 license.

---

**Note**: This project is supporting firmware for Espressif's EchoEar smart device, for learning and reference only.

