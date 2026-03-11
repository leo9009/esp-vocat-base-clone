/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file main.c
 * @brief BMI270 + BMM150 Joint Data Collection Example
 * 
 * This example demonstrates how to use BMI270 (accelerometer + gyroscope) 
 * and BMM150 (magnetometer) sensors together through BMI270's AUX interface.
 * 
 * @version 1.0.0
 * @date 2025-07-14
 */

#include <stdio.h>
#include <math.h>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmi270.h"
#include "bmm150.h"
#include "common/common.h"
#include "i2c_bus.h"
#include "bmm150_aux_adapter.h"
#include "sdkconfig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define I2C_MASTER_FREQ_HZ (100 * 1000)
#define UE_SW_I2C           1

/* Global device handles */
static bmi270_handle_t bmi_handle = NULL;
static i2c_bus_handle_t i2c_bus;
static struct bmi2_dev *bmi2_dev = NULL;
static bmm150_aux_handle_t bmm150_handle;

/* Log tag for this module */
static const char *TAG = "BMI270_BMM150";

/**
 * @brief Initialize I2C bus and BMI270 driver handle
 * 
 * This function creates the I2C bus and initializes the BMI270 driver handle.
 * It configures the I2C bus parameters and creates the BMI270 sensor handle.
 * 
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
static esp_err_t i2c_sensor_bmi270_init(void)
{
    /* Configure I2C bus parameters */
    const i2c_config_t i2c_bus_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = CONFIG_I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
#if UE_SW_I2C
    i2c_bus = i2c_bus_create(I2C_NUM_SW_1, &i2c_bus_conf);
#else
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &i2c_bus_conf);
#endif
    if (!i2c_bus) {
        ESP_LOGE(TAG, "I2C bus create failed");
        return ESP_FAIL;
    }
    
    /* Create BMI270 driver handle */
    bmi270_i2c_config_t i2c_bmi270_conf = {
        .i2c_handle = i2c_bus,
        .i2c_addr = BMI270_I2C_ADDRESS,
    };
    if (bmi270_sensor_create(&i2c_bmi270_conf, &bmi_handle) != ESP_OK || bmi_handle == NULL) {
        ESP_LOGE(TAG, "BMI270 create failed");
        return ESP_FAIL;
    }
    
    /* Get BMI270 underlying structure pointer for AUX adaptation and BMM150 initialization */
    bmi2_dev = (struct bmi2_dev *)bmi_handle;
    return ESP_OK;
}

/**
 * @brief Initialize BMI270 and configure AUX interface
 * 
 * This function initializes the BMI270 chip and configures its AUX interface
 * to communicate with the BMM150 magnetometer.
 * 
 * @return int8_t BMI2_OK on success, error code otherwise
 */
static int8_t bmi270_init_and_config(void)
{
    int8_t rslt;
    
    /* Initialize BMI270 chip */
    rslt = bmi270_init(bmi2_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 init failed: %d", rslt);
        return rslt;
    }
    ESP_LOGI(TAG, "BMI270 initialized successfully");
    
    /* Configure AUX interface parameters, enable AUX and set to manual mode, specify BMM150 I2C address */
    struct bmi2_sens_config sens_cfg = {0};
    sens_cfg.type = BMI2_AUX;
    sens_cfg.cfg.aux.aux_en = 1;              /* Enable AUX */
    sens_cfg.cfg.aux.manual_en = 1;           /* Manual mode */
    sens_cfg.cfg.aux.fcu_write_en = 0;
    sens_cfg.cfg.aux.man_rd_burst = 1;
    sens_cfg.cfg.aux.aux_rd_burst = 1;
    sens_cfg.cfg.aux.odr = 2;                 /* 2=100Hz */
    sens_cfg.cfg.aux.offset = 0;
    sens_cfg.cfg.aux.i2c_device_addr = 0x10;  /* BMM150 default I2C address */
    sens_cfg.cfg.aux.read_addr = 0x40;        /* BMM150 CHIP_ID */
    rslt = bmi2_set_sensor_config(&sens_cfg, 1, bmi2_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 AUX config failed: %d", rslt);
        return rslt;
    }
    ESP_LOGI(TAG, "BMI270 AUX interface configured");
    return BMI2_OK;
}

/**
 * @brief Initialize BMM150 using AUX adapter
 * 
 * This function initializes the BMM150 magnetometer through BMI270's AUX interface.
 * It configures the BMM150 AUX adapter and sets up the magnetometer settings.
 * 
 * @return int8_t BMM150_OK on success, error code otherwise
 */
static int8_t bmm150_init_and_config(void)
{
    int8_t rslt;
    
    /* Configure BMM150 AUX adapter */
    bmm150_aux_config_t config = {
        .bmi2_dev = bmi2_dev,
        .i2c_addr = 0x10,        /* BMM150 default address */
        .chip_id_reg = 0x40       /* BMM150 chip ID register */
    };
    
    /* Initialize BMM150 AUX adapter */
    rslt = bmm150_aux_adapter_init(&config, &bmm150_handle);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 AUX adapter init failed: %d", rslt);
        return rslt;
    }
    
    /* Configure BMM150 settings */
    struct bmm150_settings settings = {0};
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    settings.data_rate = BMM150_DATA_RATE_10HZ;
    settings.xy_rep = 9;
    settings.z_rep = 15;
    rslt = bmm150_aux_adapter_configure(&bmm150_handle, &settings);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 configure failed: %d", rslt);
        return rslt;
    }
    
    ESP_LOGI(TAG, "BMM150 configured successfully");
    return BMM150_OK;
}

/**
 * @brief Enable BMI270 three-axis sensors (accelerometer, gyroscope, AUX magnetometer)
 * 
 * This function enables all three sensors: accelerometer, gyroscope, and AUX magnetometer.
 * 
 * @return int8_t BMI2_OK on success, error code otherwise
 */
static int8_t enable_sensors(void)
{
    int8_t rslt;
    uint8_t sens_list[3] = { BMI2_ACCEL, BMI2_GYRO, BMI2_AUX };
    rslt = bmi2_sensor_enable(sens_list, 3, bmi2_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 sensor enable failed: %d", rslt);
        return rslt;
    }
    ESP_LOGI(TAG, "BMI270 sensors enabled");
    return BMI2_OK;
}

/**
 * @brief Collect and print three-axis data (accelerometer, gyroscope, magnetometer)
 * 
 * This function reads data from all three sensors and processes it.
 * It includes motion detection, rotation detection, and heading calculation
 * with automatic face up/down orientation correction.
 * 
 * Face Up/Down Orientation Correction:
 * - When the device is face DOWN (Z-axis pointing down): Use original heading directly
 * - When the device is face UP (Z-axis pointing up): Apply 360 - heading correction
 * - This correction is necessary because the magnetometer readings are inverted
 *   when the device is flipped, but the heading calculation needs to remain consistent
 * - The correction ensures that the same physical direction always shows the same heading
 *   regardless of device orientation
 */
static void print_sensor_data(void)
{
    int8_t rslt;
    /* Z-axis orientation tracking: 0=uninitialized, 1=positive, -1=negative */
    static int last_z_sign = 0;
    struct bmi2_sens_data sensor_data = {0};
    struct bmm150_mag_data mag_data = {0};
    
    /* Read BMI270 accelerometer and gyroscope data */
    rslt = bmi2_get_sensor_data(&sensor_data, bmi2_dev);
    if (rslt == BMI2_OK) {
        /* Parse accelerometer data */
        float acc_x_mg = (float)sensor_data.acc.x / 16384.0f;
        float acc_y_mg = (float)sensor_data.acc.y / 16384.0f;
        float acc_z_mg = (float)sensor_data.acc.z / 16384.0f;
        
        /* Parse gyroscope data */
        float gyro_x_dps = (float)sensor_data.gyr.x / 16.4f;
        float gyro_y_dps = (float)sensor_data.gyr.y / 16.4f;
        float gyro_z_dps = (float)sensor_data.gyr.z / 16.4f;
        
        /* Motion detection using accelerometer data */
        static float last_acc_x = 0, last_acc_y = 0, last_acc_z = 0;
        float acc_diff_x = fabs(acc_x_mg - last_acc_x);
        float acc_diff_y = fabs(acc_y_mg - last_acc_y);
        float acc_diff_z = fabs(acc_z_mg - last_acc_z);
        
        bool motion_detected = false;
        if (acc_diff_x > 50 || acc_diff_y > 50 || acc_diff_z > 50) {
            ESP_LOGI(TAG, "Motion detected!");
            motion_detected = true;
        }
        
        /* Rotation detection using gyroscope data */
        bool rotation_detected = false;
        if (fabs(gyro_x_dps) > 10) {
            ESP_LOGI(TAG, "X-axis rotation: %s", gyro_x_dps > 0 ? "right" : "left");
            rotation_detected = true;
        }
        if (fabs(gyro_y_dps) > 10) {
            ESP_LOGI(TAG, "Y-axis rotation: %s", gyro_y_dps > 0 ? "forward" : "backward");
            rotation_detected = true;
        }
        if (fabs(gyro_z_dps) > 10) {
            ESP_LOGI(TAG, "Z-axis rotation: %s", gyro_z_dps > 0 ? "clockwise" : "counterclockwise");
            rotation_detected = true;
        }
        
        /* Only print detailed data when there's motion or rotation */
        if (motion_detected || rotation_detected) {
            ESP_LOGI(TAG, "[DATA] Accel(mg): X=%.1f, Y=%.1f, Z=%.1f | Gyro(dps): X=%.1f, Y=%.1f, Z=%.1f", 
                     acc_x_mg, acc_y_mg, acc_z_mg, gyro_x_dps, gyro_y_dps, gyro_z_dps);
        }
        
        last_acc_x = acc_x_mg;
        last_acc_y = acc_y_mg;
        last_acc_z = acc_z_mg;
        
    } else {
        ESP_LOGE(TAG, "BMI270 get sensor data failed: %d", rslt);
    }
    
    /* Read BMM150 magnetometer data using AUX adapter */
    rslt = bmm150_aux_adapter_read_mag_data(&bmm150_handle, &mag_data);
    if (rslt == BMM150_OK) {
        float mag_x = (float)mag_data.x;
        float mag_y = (float)mag_data.y;
        float mag_z = (float)mag_data.z;
        
        /* Calculate magnetic field strength and heading using adapter functions */
        float mag_strength = bmm150_aux_adapter_calculate_strength(mag_x, mag_y, mag_z);
        
        /* Magnetic field change detection */
        static float last_mag_x = 0, last_mag_y = 0, last_mag_z = 0;
        float mag_diff_x = fabs(mag_x - last_mag_x);
        float mag_diff_y = fabs(mag_y - last_mag_y);
        float mag_diff_z = fabs(mag_z - last_mag_z);
        bool mag_change = false;
        if (mag_diff_x > 10 || mag_diff_y > 10 || mag_diff_z > 10) {
            ESP_LOGI(TAG, "Magnetic field changed: X=%.1f, Y=%.1f, Z=%.1f uT", mag_diff_x, mag_diff_y, mag_diff_z);
            mag_change = true;
        }
        
        /* Heading detection (only when magnetic field is normal) */
        if (bmm150_aux_adapter_is_field_normal(mag_strength)) {
            /* Calculate heading with Z-axis orientation correction */
            float heading = bmm150_aux_adapter_calculate_heading(mag_x, mag_y);
            float acc_z_mg = (float)sensor_data.acc.z / 16384.0f;
            
            /* Determine current Z-axis orientation */
            int current_z_sign = (acc_z_mg > 0) ? 1 : -1;
            
            /* Detect Z-axis orientation change */
            if (last_z_sign != 0 && current_z_sign != last_z_sign) {
                if (current_z_sign > 0) {
                    ESP_LOGI(TAG, "Z axis flipped: Now facing UP (acc_z_mg=%.2f)", acc_z_mg);
                } else {
                    ESP_LOGI(TAG, "Z axis flipped: Now facing DOWN (acc_z_mg=%.2f)", acc_z_mg);
                }
            }
            last_z_sign = current_z_sign;
            
            /* Apply face up/down orientation correction */
            float corrected_heading = heading;
            if (current_z_sign > 0) {
                /* Face UP: Apply 360 - heading correction */
                /* This is necessary because magnetometer readings are inverted when device is flipped */
                /* The correction ensures consistent heading regardless of device orientation */
                corrected_heading = 360.0f - heading;
                if (corrected_heading >= 360.0f) corrected_heading -= 360.0f;
                ESP_LOGI(TAG, "Face UP detected: corrected heading from %.1f to %.1f", heading, corrected_heading);
            } else {
                /* Face DOWN: Use original heading directly */
                ESP_LOGI(TAG, "Face DOWN detected: using original heading %.1f", heading);
            }
            
            /* Display direction and angle */
            const char* direction = bmm150_aux_adapter_get_direction(corrected_heading);
            ESP_LOGI(TAG, "Heading: %s (%.1f)", direction, corrected_heading);
        } else {
            ESP_LOGW(TAG, "Warning: Abnormal magnetic field (%.1f uT), unreliable heading", mag_strength);
        }
        
        /* Only print raw data when magnetic field changes */
        if (mag_change) {
            uint8_t raw_data[8] = {0};
            rslt = bmm150_aux_adapter_read_regs(&bmm150_handle, BMM150_REG_DATA_X_LSB, raw_data, 8);
            if (rslt == BMM150_OK) {
                ESP_LOGI(TAG, "Raw data: X=0x%02X%02X, Y=0x%02X%02X, Z=0x%02X%02X", 
                         raw_data[1], raw_data[0], raw_data[3], raw_data[2], raw_data[5], raw_data[4]);
            }
        }
        last_mag_x = mag_x;
        last_mag_y = mag_y;
        last_mag_z = mag_z;
    } else {
        ESP_LOGE(TAG, "BMM150 read mag data failed: %d", rslt);
    }
}

/**
 * @brief Main entry point
 * 
 * This function initializes all sensors, configures them, collects data,
 * and releases resources when done.
 */
void app_main(void) {
    ESP_LOGI(TAG, "BMI270 + BMM150 Joint Data Collection Example");
    ESP_LOGI(TAG, "=============================================");
    
    /* Initialize I2C and BMI270 driver handle */
    if (i2c_sensor_bmi270_init() != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 initialization failed");
        return;
    }
    
    /* Initialize BMI270 chip and AUX interface */
    int8_t rslt = bmi270_init_and_config();
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 init and config failed");
        goto cleanup;
    }
    
    /* Initialize BMM150 magnetometer (via AUX adapter) */
    rslt = bmm150_init_and_config();
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 initialization failed");
        goto cleanup;
    }
    
    /* Enable three-axis joint collection */
    rslt = enable_sensors();
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Enable sensors failed");
        goto cleanup;
    }
    
    ESP_LOGI(TAG, "All sensors initialized successfully!");
    ESP_LOGI(TAG, "Starting data collection...");
    
    /* Main loop for data collection and printing */
    int count = 0;
    while (count < 100) {
        print_sensor_data();
        vTaskDelay(pdMS_TO_TICKS(1000));
        count++;
    }
    
    ESP_LOGI(TAG, "Data collection completed!");
    
cleanup:
    /* Resource release, delete driver handle and I2C bus */
    if (bmi_handle) {
        bmi270_sensor_del(bmi_handle);
    }
    if (i2c_bus) {
        i2c_bus_delete(&i2c_bus);
    }
    /* Deinitialize BMM150 AUX adapter */
    bmm150_aux_adapter_deinit(&bmm150_handle);
    ESP_LOGI(TAG, "Example finished.");
} 