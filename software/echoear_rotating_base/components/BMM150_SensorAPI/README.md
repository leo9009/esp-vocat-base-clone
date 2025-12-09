# BMM150 Magnetometer AUX Interface Adapter

## Overview

This component provides BMM150 magnetometer functionality through BMI270's AUX interface. It allows direct access to BMM150 registers via BMI270's auxiliary I2C interface, enabling magnetometer data collection in systems where BMM150 is connected through BMI270.

## Project Structure

```
LLM_camera/
├── examples/bmi270_bmm150/
│   ├── main/
│   │   ├── main.c                    # Main application entry point
│   │   ├── CMakeLists.txt           # Build configuration
│   │   └── idf_component.yml        # Component dependencies
│   └── components/
│       └── BMM150_SensorAPI/        # BMM150 component library
│           ├── bmm150_aux_adapter.h  # AUX adapter header file
│           ├── bmm150_aux_adapter.c  # AUX adapter implementation
│           ├── bmm150_aux_example.c  # Usage example
│           ├── bmm150.h              # Original BMM150 driver header
│           ├── bmm150.c              # Original BMM150 driver implementation
│           ├── bmm150_defs.h         # BMM150 definitions and constants
│           ├── test_apps/            # Test applications
│           ├── examples/             # Example code
│           ├── CMakeLists.txt        # Component build configuration
│           ├── idf_component.yml     # Component dependencies
│           ├── README.md             # This file
│           ├── CHANGELOG.md          # Version history
│           └── LICENSE               # License information
```

## Call Hierarchy

### 1. Main Application (main.c)
- **Entry Point**: `app_main()`
- **Initialization Flow**:
  1. `i2c_sensor_bmi270_init()` - Initialize I2C bus and BMI270 driver
  2. `bmi270_init_and_config()` - Initialize BMI270 and configure AUX interface
  3. `bmm150_init_and_config()` - Initialize BMM150 through AUX adapter
  4. `enable_sensors()` - Enable all sensors (accelerometer, gyroscope, magnetometer)

### 2. BMM150 AUX Adapter (bmm150_aux_adapter.c)
- **Core Functions**:
  - `bmm150_aux_adapter_init()` - Initialize BMM150 through AUX interface
  - `bmm150_aux_adapter_read_mag_data()` - Read magnetometer data
  - `bmm150_aux_adapter_configure()` - Configure BMM150 settings
  - `bmm150_aux_adapter_calculate_heading()` - Calculate heading from magnetometer data
  - `bmm150_aux_adapter_is_field_normal()` - Check magnetic field validity

### 3. BMI270 Component
- **AUX Interface**: Provides I2C bridge to BMM150
- **Data Collection**: Collects accelerometer, gyroscope, and magnetometer data
- **Sensor Fusion**: Enables combined sensor data processing

## Hardware Configuration

### Supported Hardware
- **ESP-ASTOM-S3**: I2C pins 0/45, INT pin 16

### I2C Configuration
- **Master Mode**: I2C_NUM_0
- **Frequency**: 100kHz
- **Pull-up**: Enabled on SDA and SCL
- **Software I2C**: Optional (UE_SW_I2C flag)

## Face Up/Down Orientation Correction

### Problem Description
When a device with magnetometer is flipped (face up vs face down), the magnetometer readings are inverted, but the heading calculation needs to remain consistent. This is especially important for applications where the device can be used in both orientations.

### Solution Implementation

#### 1. Z-Axis Orientation Detection
```c
/* Determine current Z-axis orientation using accelerometer */
float acc_z_mg = (float)sensor_data.acc.z / 16384.0f;
int current_z_sign = (acc_z_mg > 0) ? 1 : -1;
```

#### 2. Orientation Change Detection
```c
/* Detect Z-axis orientation change */
if (last_z_sign != 0 && current_z_sign != last_z_sign) {
    if (current_z_sign > 0) {
        ESP_LOGI(TAG, "Z axis flipped: Now facing UP (acc_z_mg=%.2f)", acc_z_mg);
    } else {
        ESP_LOGI(TAG, "Z axis flipped: Now facing DOWN (acc_z_mg=%.2f)", acc_z_mg);
    }
}
```

#### 3. Heading Correction Algorithm
```c
/* Apply face up/down orientation correction */
float corrected_heading = heading;
if (current_z_sign > 0) {
    /* Face UP: Apply 360 - heading correction */
    /* This is necessary because magnetometer readings are inverted when device is flipped */
    /* The correction ensures consistent heading regardless of device orientation */
    corrected_heading = 360.0f - heading;
    if (corrected_heading >= 360.0f) corrected_heading -= 360.0f;
} else {
    /* Face DOWN: Use original heading directly */
}
```

### Technical Details

#### Why Correction is Needed
1. **Magnetometer Inversion**: When the device is flipped, the X and Y magnetometer readings are inverted
2. **Heading Calculation**: `atan2(mag_y, mag_x)` produces different results for inverted readings
3. **Consistency Requirement**: The same physical direction should show the same heading regardless of device orientation

#### Correction Logic
- **Face DOWN (Z-axis pointing down)**: Use original heading directly
- **Face UP (Z-axis pointing up)**: Apply `360° - heading` correction
- **Result**: Same physical direction always shows the same heading

#### Mathematical Basis
- **Original heading**: `θ = atan2(mag_y, mag_x)`
- **Inverted readings**: `mag_x' = -mag_x`, `mag_y' = -mag_y`
- **Inverted heading**: `θ' = atan2(-mag_y, -mag_x) = atan2(mag_y, mag_x) + 180°`
- **Correction**: `θ_corrected = 360° - θ' = 360° - (θ + 180°) = 180° - θ`

### Usage Example
```c
/* Calculate heading with Z-axis orientation correction */
float heading = bmm150_aux_adapter_calculate_heading(mag_x, mag_y);
float acc_z_mg = (float)sensor_data.acc.z / 16384.0f;

/* Determine current Z-axis orientation */
int current_z_sign = (acc_z_mg > 0) ? 1 : -1;

/* Apply face up/down orientation correction */
float corrected_heading = heading;
if (current_z_sign > 0) {
    /* Face UP: Apply 360 - heading correction */
    corrected_heading = 360.0f - heading;
    if (corrected_heading >= 360.0f) corrected_heading -= 360.0f;
} else {
    /* Face DOWN: Use original heading directly */
}

/* Display direction and angle */
const char* direction = bmm150_aux_adapter_get_direction(corrected_heading);
ESP_LOGI(TAG, "Heading: %s (%.1f)", direction, corrected_heading);
```

## Data Processing Features

### Motion Detection
- **Threshold**: 50mg acceleration change
- **Axes**: X, Y, Z acceleration monitoring
- **Output**: Motion detection alerts

### Rotation Detection
- **Threshold**: 10 degrees/second angular velocity
- **Axes**: X, Y, Z rotation monitoring
- **Output**: Rotation direction identification

### Magnetic Field Monitoring
- **Strength Calculation**: `sqrt(mag_x² + mag_y² + mag_z²)`
- **Normal Range**: 20-100 μT
- **Change Detection**: 10 μT threshold
- **Output**: Field strength and validity status

### Heading Calculation
- **Method**: `atan2(mag_y, mag_x)`
- **Range**: 0-360 degrees
- **Direction Mapping**: North (315-45°), East (45-135°), South (135-225°), West (225-315°)
- **Orientation Correction**: Automatic face up/down adjustment

## Error Handling

### Initialization Errors
- **I2C Bus Creation**: Check for bus creation failure
- **BMI270 Initialization**: Verify chip ID and communication
- **BMM150 Initialization**: Verify chip ID and AUX interface
- **Sensor Configuration**: Validate settings application

### Runtime Errors
- **Data Read Failures**: Handle sensor communication errors
- **Invalid Data**: Check for abnormal magnetic field strength
- **Orientation Changes**: Detect and handle device flipping

## Performance Considerations

### Timing
- **Data Rate**: 10Hz magnetometer, 100Hz accelerometer/gyroscope
- **Processing Delay**: Minimal overhead for orientation correction
- **Memory Usage**: Static variables for orientation tracking

### Power Consumption
- **Normal Mode**: BMM150 in normal power mode
- **AUX Interface**: Minimal power overhead
- **Processing**: Efficient mathematical operations

## Troubleshooting

### Common Issues
1. **No Magnetometer Data**: Check AUX interface configuration
2. **Incorrect Heading**: Verify magnetic field strength and orientation
3. **Communication Errors**: Check I2C connections and addresses
4. **Orientation Issues**: Ensure accelerometer data is valid

### Debug Information
- **Raw Data**: Available when magnetic field changes
- **Chip ID**: Verified during initialization
- **Orientation Changes**: Logged when detected
- **Field Strength**: Monitored for validity

## License

This component is licensed under the Apache-2.0 License. See LICENSE file for details.

## Version History

See CHANGELOG.md for detailed version history and changes.