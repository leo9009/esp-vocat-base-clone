/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file bmm150_aux_adapter.h
 * @brief BMM150 magnetometer AUX interface adapter for BMI270
 * 
 * This adapter provides BMM150 magnetometer functionality through BMI270's AUX interface.
 * It allows direct access to BMM150 registers via BMI270's auxiliary I2C interface.
 * 
 * @version 1.0.0
 * @date 2025-07-14
 */

#ifndef BMM150_AUX_ADAPTER_H
#define BMM150_AUX_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "bmm150.h"

// Forward declarations
struct bmi2_dev;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BMM150 AUX adapter configuration
 */
typedef struct {
    struct bmi2_dev *bmi2_dev;    /**< BMI270 device pointer */
    uint8_t i2c_addr;             /**< BMM150 I2C address (default: 0x10) */
    uint8_t chip_id_reg;          /**< BMM150 chip ID register (default: 0x40) */
} bmm150_aux_config_t;

/**
 * @brief BMM150 AUX adapter handle
 */
typedef struct {
    struct bmm150_dev bmm150_dev; /**< BMM150 device structure */
    struct bmi2_dev *bmi2_dev;    /**< BMI270 device pointer */
    bool is_initialized;           /**< Initialization status */
} bmm150_aux_handle_t;

/**
 * @brief Initialize BMM150 AUX adapter
 * 
 * @param config Configuration structure
 * @param handle Device handle pointer
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_init(const bmm150_aux_config_t *config, bmm150_aux_handle_t *handle);

/**
 * @brief Deinitialize BMM150 AUX adapter
 * 
 * @param handle Device handle
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_deinit(bmm150_aux_handle_t *handle);

/**
 * @brief Read magnetometer data
 * 
 * @param handle Device handle
 * @param mag_data Magnetometer data structure
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_read_mag_data(bmm150_aux_handle_t *handle, struct bmm150_mag_data *mag_data);

/**
 * @brief Configure BMM150 settings
 * 
 * @param handle Device handle
 * @param settings BMM150 settings structure
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_configure(bmm150_aux_handle_t *handle, const struct bmm150_settings *settings);

/**
 * @brief Perform BMM150 soft reset
 * 
 * @param handle Device handle
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_soft_reset(bmm150_aux_handle_t *handle);

/**
 * @brief Get BMM150 chip ID
 * 
 * @param handle Device handle
 * @param chip_id Chip ID pointer
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_get_chip_id(bmm150_aux_handle_t *handle, uint8_t *chip_id);

/**
 * @brief Read BMM150 registers
 * 
 * @param handle Device handle
 * @param reg_addr Register address
 * @param data Data buffer
 * @param len Data length
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_read_regs(bmm150_aux_handle_t *handle, uint8_t reg_addr, uint8_t *data, uint32_t len);

/**
 * @brief Write BMM150 registers
 * 
 * @param handle Device handle
 * @param reg_addr Register address
 * @param data Data buffer
 * @param len Data length
 * @return int8_t BMM150_OK on success, error code otherwise
 */
int8_t bmm150_aux_adapter_write_regs(bmm150_aux_handle_t *handle, uint8_t reg_addr, const uint8_t *data, uint32_t len);

/**
 * @brief Calculate heading from magnetometer data
 * 
 * @param mag_x X-axis magnetic field
 * @param mag_y Y-axis magnetic field
 * @return float Heading angle in degrees (0-360)
 */
float bmm150_aux_adapter_calculate_heading(float mag_x, float mag_y);

/**
 * @brief Calculate magnetic field strength
 * 
 * @param mag_x X-axis magnetic field
 * @param mag_y Y-axis magnetic field
 * @param mag_z Z-axis magnetic field
 * @return float Magnetic field strength in uT
 */
float bmm150_aux_adapter_calculate_strength(float mag_x, float mag_y, float mag_z);

/**
 * @brief Check if magnetic field is normal
 * 
 * @param strength Magnetic field strength
 * @return true if normal, false otherwise
 */
bool bmm150_aux_adapter_is_field_normal(float strength);

/**
 * @brief Get direction string from heading angle
 * 
 * @param heading Heading angle in degrees
 * @return const char* Direction string (North/East/South/West)
 */
const char* bmm150_aux_adapter_get_direction(float heading);

#ifdef __cplusplus
}
#endif

#endif /* BMM150_AUX_ADAPTER_H */ 