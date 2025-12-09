/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file bmm150_aux_example.c
 * @brief Example usage of BMM150 AUX adapter
 * 
 * This example demonstrates how to use the BMM150 AUX adapter
 * to read magnetometer data through BMI270's AUX interface.
 * 
 * @version 1.0.0
 * @date 2025-07-14
 */

#include "bmm150_aux_adapter.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Log tag for this module */
static const char *TAG = "BMM150_AUX_EXAMPLE";

/**
 * @brief Example function to demonstrate BMM150 AUX adapter usage
 * 
 * This function shows the complete workflow for initializing,
 * configuring, and reading data from BMM150 magnetometer
 * through BMI270's AUX interface.
 * 
 * @param bmi2_dev BMI270 device pointer (must be initialized)
 */
void bmm150_aux_example_usage(struct bmi2_dev *bmi2_dev)
{
    if (!bmi2_dev) {
        ESP_LOGE(TAG, "Invalid BMI270 device pointer");
        return;
    }

    /* Configure BMM150 AUX adapter */
    bmm150_aux_config_t config = {
        .bmi2_dev = bmi2_dev,
        .i2c_addr = 0x10,        /* BMM150 default address */
        .chip_id_reg = 0x40       /* BMM150 chip ID register */
    };

    /* Initialize BMM150 AUX adapter */
    bmm150_aux_handle_t bmm150_handle;
    int8_t rslt = bmm150_aux_adapter_init(&config, &bmm150_handle);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 AUX adapter init failed: %d", rslt);
        return;
    }

    /* Configure BMM150 settings */
    struct bmm150_settings settings = {0};
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;  /* Normal power mode */
    settings.data_rate = BMM150_DATA_RATE_10HZ;   /* 10Hz data rate */
    settings.xy_rep = 9;                          /* XY repetition */
    settings.z_rep = 15;                          /* Z repetition */

    rslt = bmm150_aux_adapter_configure(&bmm150_handle, &settings);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 configure failed: %d", rslt);
        bmm150_aux_adapter_deinit(&bmm150_handle);
        return;
    }

    /* Read magnetometer data */
    struct bmm150_mag_data mag_data = {0};
    rslt = bmm150_aux_adapter_read_mag_data(&bmm150_handle, &mag_data);
    if (rslt == BMM150_OK) {
        /* Calculate heading and strength */
        float heading = bmm150_aux_adapter_calculate_heading(mag_data.x, mag_data.y);
        float strength = bmm150_aux_adapter_calculate_strength(mag_data.x, mag_data.y, mag_data.z);
        const char* direction = bmm150_aux_adapter_get_direction(heading);

        ESP_LOGI(TAG, "Magnetometer Data:");
        ESP_LOGI(TAG, "  X: %.2f uT", (float)mag_data.x);
        ESP_LOGI(TAG, "  Y: %.2f uT", (float)mag_data.y);
        ESP_LOGI(TAG, "  Z: %.2f uT", (float)mag_data.z);
        ESP_LOGI(TAG, "  Heading: %.1f (%s)", heading, direction);
        ESP_LOGI(TAG, "  Strength: %.2f uT", strength);

        /* Check if magnetic field is normal */
        if (bmm150_aux_adapter_is_field_normal(strength)) {
            ESP_LOGI(TAG, "  Magnetic field is normal");
        } else {
            ESP_LOGW(TAG, "  Magnetic field is abnormal (%.1f uT)", strength);
        }
    } else {
        ESP_LOGE(TAG, "Failed to read magnetometer data: %d", rslt);
    }

    /* Cleanup */
    bmm150_aux_adapter_deinit(&bmm150_handle);
} 