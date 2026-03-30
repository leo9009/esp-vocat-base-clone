/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file bmm150_aux_adapter.c
 * @brief BMM150 magnetometer AUX interface adapter implementation
 * 
 * This adapter provides BMM150 magnetometer functionality through BMI270's AUX interface.
 * It allows direct access to BMM150 registers via BMI270's auxiliary I2C interface.
 * 
 * @version 1.0.0
 * @date 2025-07-14
 */

#include <string.h>
#include "bmm150_aux_adapter.h"
#include "bmm150.h"
#include "bmi270.h"
#include "esp_log.h"

/* Log tag for this module */
static const char *TAG = "BMM150_AUX_ADAPTER";

/**
 * @brief BMM150 AUX interface read callback function
 * 
 * This function is called by the BMM150 driver to read data from BMM150 registers
 * through BMI270's AUX interface.
 * 
 * @param reg_addr Register address to read from
 * @param data Data buffer to store read data
 * @param len Number of bytes to read
 * @param intf_ptr Interface pointer (BMI270 device)
 * @return int8_t BMI2_OK on success, error code otherwise
 */
static int8_t bmm150_aux_read(unsigned char reg_addr, unsigned char *data, uint32_t len, void *intf_ptr)
{
    struct bmi2_dev *bmi2 = (struct bmi2_dev *)intf_ptr;
    return bmi2_read_aux_man_mode(reg_addr, data, len, bmi2);
}

/**
 * @brief BMM150 AUX interface write callback function
 * 
 * This function is called by the BMM150 driver to write data to BMM150 registers
 * through BMI270's AUX interface.
 * 
 * @param reg_addr Register address to write to
 * @param data Data buffer to write
 * @param len Number of bytes to write
 * @param intf_ptr Interface pointer (BMI270 device)
 * @return int8_t BMI2_OK on success, error code otherwise
 */
static int8_t bmm150_aux_write(unsigned char reg_addr, const unsigned char *data, uint32_t len, void *intf_ptr)
{
    struct bmi2_dev *bmi2 = (struct bmi2_dev *)intf_ptr;
    return bmi2_write_aux_man_mode(reg_addr, data, len, bmi2);
}

/**
 * @brief Delay function required by BMM150 driver
 * 
 * This function provides microsecond delay functionality required by the BMM150 driver.
 * It uses ESP32 ROM delay function for accurate timing.
 * 
 * @param period Delay period in microseconds
 * @param intf_ptr Interface pointer (unused)
 */
static void bmm150_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    /* Use ESP32 ROM delay function */
    for (volatile uint32_t i = 0; i < period; i++) {
        __asm__ __volatile__("nop");
    }
}

/**
 * @brief Initialize BMM150 AUX adapter
 * 
 * This function initializes the BMM150 magnetometer through BMI270's AUX interface.
 * It configures the BMM150 device structure with AUX interface callbacks,
 * performs chip initialization, verifies chip ID, and sets up normal operation mode.
 * 
 * @param config Configuration structure containing BMI270 device pointer and BMM150 settings
 * @param handle Device handle pointer to be initialized
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_init(const bmm150_aux_config_t *config, bmm150_aux_handle_t *handle)
{
    if (!config || !handle) {
        ESP_LOGE(TAG, "Invalid parameters");
        return BMM150_E_NULL_PTR;
    }

    /* Initialize handle */
    memset(handle, 0, sizeof(bmm150_aux_handle_t));
    handle->bmi2_dev = config->bmi2_dev;

    /* Configure BMM150 device structure with AUX interface callbacks */
    handle->bmm150_dev.intf = BMM150_I2C_INTF;
    handle->bmm150_dev.read = bmm150_aux_read;
    handle->bmm150_dev.write = bmm150_aux_write;
    handle->bmm150_dev.delay_us = bmm150_delay_us;
    handle->bmm150_dev.intf_ptr = config->bmi2_dev;

    /* Initialize BMM150 chip */
    int8_t rslt = bmm150_init(&handle->bmm150_dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 init failed: %d", rslt);
        return rslt;
    }

    /* Verify chip ID to ensure correct device */
    uint8_t chip_id = 0;
    rslt = bmm150_get_regs(BMM150_REG_CHIP_ID, &chip_id, 1, &handle->bmm150_dev);
    if (rslt == BMM150_OK) {
        if (chip_id != BMM150_CHIP_ID) {
            ESP_LOGE(TAG, "BMM150 Chip ID mismatch! Expected: 0x%02X, Got: 0x%02X", BMM150_CHIP_ID, chip_id);
            return BMM150_E_DEV_NOT_FOUND;
        }
    } else {
        ESP_LOGE(TAG, "BMM150 Chip ID read failed: %d", rslt);
        return rslt;
    }

    /* Perform soft reset to ensure clean state */
    rslt = bmm150_soft_reset(&handle->bmm150_dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 soft reset failed: %d", rslt);
        return rslt;
    }
    /* Wait for reset completion */
    handle->bmm150_dev.delay_us(10000, handle->bmm150_dev.intf_ptr); /* 10ms */

    /* Set BMM150 to normal operation mode */
    struct bmm150_settings settings = {0};
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &handle->bmm150_dev);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 set op mode failed: %d", rslt);
        return rslt;
    }
    handle->bmm150_dev.delay_us(5000, handle->bmm150_dev.intf_ptr); /* 5ms */

    handle->is_initialized = true;
    ESP_LOGI(TAG, "BMM150 AUX adapter initialized successfully");
    return BMM150_OK;
}

/**
 * @brief Deinitialize BMM150 AUX adapter
 * 
 * This function deinitializes the BMM150 AUX adapter and marks it as not initialized.
 * 
 * @param handle Device handle to deinitialize
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_deinit(bmm150_aux_handle_t *handle)
{
    if (!handle) {
        return BMM150_E_NULL_PTR;
    }

    handle->is_initialized = false;
    ESP_LOGI(TAG, "BMM150 AUX adapter deinitialized");
    return BMM150_OK;
}

/**
 * @brief Read magnetometer data from BMM150
 * 
 * This function reads raw magnetometer data from the BMM150 sensor.
 * 
 * @param handle Device handle
 * @param mag_data Magnetometer data structure to store the readings
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_read_mag_data(bmm150_aux_handle_t *handle, struct bmm150_mag_data *mag_data)
{
    if (!handle || !mag_data || !handle->is_initialized) {
        return BMM150_E_NULL_PTR;
    }

    return bmm150_read_mag_data(mag_data, &handle->bmm150_dev);
}

/**
 * @brief Configure BMM150 sensor settings
 * 
 * This function configures various BMM150 settings such as data rate,
 * repetition settings, and power mode.
 * 
 * @param handle Device handle
 * @param settings BMM150 settings structure containing configuration parameters
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_configure(bmm150_aux_handle_t *handle, const struct bmm150_settings *settings)
{
    if (!handle || !settings || !handle->is_initialized) {
        return BMM150_E_NULL_PTR;
    }

    return bmm150_set_sensor_settings(
        BMM150_SEL_DATA_RATE | BMM150_SEL_XY_REP | BMM150_SEL_Z_REP | BMM150_SEL_CONTROL_MEASURE,
        settings,
        &handle->bmm150_dev
    );
}

/**
 * @brief Perform BMM150 soft reset
 * 
 * This function performs a soft reset of the BMM150 sensor to restore
 * it to a known good state.
 * 
 * @param handle Device handle
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_soft_reset(bmm150_aux_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return BMM150_E_NULL_PTR;
    }

    return bmm150_soft_reset(&handle->bmm150_dev);
}

/**
 * @brief Get BMM150 chip ID
 * 
 * This function reads the chip ID register of the BMM150 sensor.
 * 
 * @param handle Device handle
 * @param chip_id Pointer to store the chip ID
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_get_chip_id(bmm150_aux_handle_t *handle, uint8_t *chip_id)
{
    if (!handle || !chip_id || !handle->is_initialized) {
        return BMM150_E_NULL_PTR;
    }

    return bmm150_get_regs(BMM150_REG_CHIP_ID, chip_id, 1, &handle->bmm150_dev);
}

/**
 * @brief Read BMM150 registers
 * 
 * This function reads data from BMM150 registers through the AUX interface.
 * 
 * @param handle Device handle
 * @param reg_addr Register address to read from
 * @param data Data buffer to store read data
 * @param len Number of bytes to read
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_read_regs(bmm150_aux_handle_t *handle, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
    if (!handle || !data || !handle->is_initialized) {
        return BMM150_E_NULL_PTR;
    }

    return bmm150_get_regs(reg_addr, data, len, &handle->bmm150_dev);
}

/**
 * @brief Write BMM150 registers
 * 
 * This function writes data to BMM150 registers through the AUX interface.
 * 
 * @param handle Device handle
 * @param reg_addr Register address to write to
 * @param data Data buffer to write
 * @param len Number of bytes to write
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_write_regs(bmm150_aux_handle_t *handle, uint8_t reg_addr, const uint8_t *data, uint32_t len)
{
    if (!handle || !data || !handle->is_initialized) {
        return BMM150_E_NULL_PTR;
    }

    return bmm150_set_regs(reg_addr, data, len, &handle->bmm150_dev);
}

/**
 * @brief Calculate heading from magnetometer data
 * 
 * This function calculates the heading angle in degrees from X and Y magnetometer readings.
 * The heading is calculated using atan2 and converted to degrees (0-360).
 * 
 * @param mag_x X-axis magnetic field reading
 * @param mag_y Y-axis magnetic field reading
 * @return float Heading angle in degrees (0-360)
 */
float bmm150_aux_adapter_calculate_heading(float mag_x, float mag_y)
{
    float heading = atan2(mag_y, mag_x) * 180.0f / M_PI;
    if (heading < 0) {
        heading += 360.0f;
    }
    return heading;
}

/**
 * @brief Calculate magnetic field strength
 * 
 * This function calculates the total magnetic field strength from all three axes.
 * 
 * @param mag_x X-axis magnetic field reading
 * @param mag_y Y-axis magnetic field reading
 * @param mag_z Z-axis magnetic field reading
 * @return float Magnetic field strength in microTesla (uT)
 */
float bmm150_aux_adapter_calculate_strength(float mag_x, float mag_y, float mag_z)
{
    return sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
}

/**
 * @brief Check if magnetic field is normal
 * 
 * This function checks if the magnetic field strength is within normal range.
 * Normal magnetic field strength is typically between 20-100 uT.
 * 
 * @param strength Magnetic field strength in uT
 * @return true if normal, false otherwise
 */
bool bmm150_aux_adapter_is_field_normal(float strength)
{
    /* Normal magnetic field strength: 25-65 uT */
    return (strength >= 20.0f && strength <= 100.0f);
}

/**
 * @brief Get direction string from heading angle
 * 
 * This function converts a heading angle to a cardinal direction string.
 * 
 * @param heading Heading angle in degrees (0-360)
 * @return const char* Direction string (North/East/South/West)
 */
const char* bmm150_aux_adapter_get_direction(float heading)
{
    if (heading >= 315.0f || heading < 45.0f) {
        return "North";
    } else if (heading >= 45.0f && heading < 135.0f) {
        return "East";
    } else if (heading >= 135.0f && heading < 225.0f) {
        return "South";
    } else {
        return "West";
    }
} 