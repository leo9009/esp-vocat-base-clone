/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "bmm150.h"
#include "bmm150_defs.h"
#include "i2c_bus.h"
#include "magnetic_slide_switch.h"
#include "control_serial.h"

static const char *TAG = "Magnetic Slide Switch";

/* ========== NVS Storage Configuration ========== */
#define NVS_NAMESPACE       "mag_calib"     /**< NVS namespace */
#define NVS_KEY_REMOVED     "removed"       /**< REMOVED state center value key name */
#define NVS_KEY_UP          "up"            /**< UP state center value key name */
#define NVS_KEY_DOWN        "down"          /**< DOWN state center value key name */
#define NVS_KEY_CALIBRATED  "calibrated"    /**< Calibration flag key name */

/* ========== Global Variables ========== */
static magnetic_slide_switch_event_t s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_INIT;

#if CONFIG_SENSOR_LINEAR_HALL
    static int adc_raw[2][10];
    static int voltage[2][10];
    static int s_last_hall_voltage = 0;
    static bool s_first_hall_reading = true;
#endif

#if CONFIG_SENSOR_BMM150
static struct bmm150_dev s_bmm150 = { 0 };
static uint8_t s_bmm150_addr = BMM150_DEFAULT_I2C_ADDRESS;  /**< BMM150 default I2C address */
#endif

#ifdef CONFIG_SENSOR_MAGNETOMETER
/** I2C bus/device handles */
static i2c_bus_handle_t s_i2c_bus = NULL;
static i2c_bus_device_handle_t s_i2c_dev = NULL;
static uint8_t dev_addr;
#endif

/* ========== Linear Hall Sensor Related Functions ========== */

#if CONFIG_SENSOR_LINEAR_HALL
/**
 * @brief ADC calibration initialization
 * 
 * @param unit ADC unit
 * @param channel ADC channel
 * @param atten ADC attenuation
 * @param out_handle Output calibration handle
 * @return true Calibration successful
 * @return false Calibration failed
 */
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

/**
 * @brief Hall sensor read task
 * 
 * @param arg Task parameter (unused)
 * 
 * @note Acquires Hall sensor voltage via ADC, determines slider movement events based on voltage changes
 * @note Sampling period defined by HALL_SENSOR_SAMPLE_PERIOD_MS
 */
static void hall_sensor_read_task(void *arg)
{
    ESP_LOGI(TAG, "Hall sensor read task started");
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = HALL_SENSOR_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, HALL_SENSOR_ADC_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, HALL_SENSOR_ADC_CHANNEL, HALL_SENSOR_ADC_ATTEN, &adc1_cali_chan0_handle);

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, HALL_SENSOR_ADC_CHANNEL, &adc_raw[0][0]));
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            
            // Check voltage changes to determine magnetic switch events
            if (!s_first_hall_reading) {  // Not first reading
                int voltage_change = voltage[0][0] - s_last_hall_voltage;
                
                // Voltage decreases and change exceeds threshold, slider moved down
                if (voltage_change < -HALL_SENSOR_VOLTAGE_THRESHOLD && s_slider_state != MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN) {
                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN;
                    ESP_LOGI(TAG, "Hall switch: SLIDE_DOWN - Voltage dropped by %d mV (from %d to %d)", 
                             -voltage_change, s_last_hall_voltage, voltage[0][0]);
                    control_serial_send_magnetic_switch_event(s_slider_state);
                }
                // Voltage increases and change exceeds threshold, slider moved up
                else if (voltage_change > HALL_SENSOR_VOLTAGE_THRESHOLD && s_slider_state == MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN) {
                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP;
                    ESP_LOGI(TAG, "Hall switch: SLIDE_UP - Voltage increased by %d mV (from %d to %d)", 
                             voltage_change, s_last_hall_voltage, voltage[0][0]);
                    control_serial_send_magnetic_switch_event(s_slider_state);
                }
            } else {
                s_first_hall_reading = false;
                
                // Determine initial state based on first reading voltage (initial state doesn't send event)
                if (voltage[0][0] < HALL_SENSOR_INITIAL_STATE_THRESHOLD) {
                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN;
                    ESP_LOGI(TAG, "First reading: Slider initially at DOWN position (voltage=%d < %d mV)", 
                             voltage[0][0], HALL_SENSOR_INITIAL_STATE_THRESHOLD);
                } else {
                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP;
                    ESP_LOGI(TAG, "First reading: Slider initially at UP position (voltage=%d >= %d mV)", 
                             voltage[0][0], HALL_SENSOR_INITIAL_STATE_THRESHOLD);
                }
            }
            
            // Update last voltage value
            s_last_hall_voltage = voltage[0][0];
            
            // ESP_LOGI(TAG, "Hall sensor voltage: %d mV", voltage[0][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(HALL_SENSOR_SAMPLE_PERIOD_MS));
    }
#endif  // CONFIG_SENSOR_LINEAR_HALL

/* ========== BMM150 Geomagnetic Sensor Related Functions ========== */

#ifdef CONFIG_SENSOR_BMM150
/**
 * @brief BMM150 I2C read function
 * 
 * @param reg_addr Register address
 * @param reg_data Read data buffer
 * @param length Read length
 * @param intf_ptr Interface pointer (device address)
 * @return BMM150_INTF_RET_TYPE Return status
 */
static BMM150_INTF_RET_TYPE bmm150_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // Check if I2C device handle is valid
    if (s_i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return BMM150_E_COM_FAIL;
    }

    esp_err_t ret = i2c_bus_read_bytes(s_i2c_dev, reg_addr, (uint16_t)length, reg_data);
    return (ret == ESP_OK) ? BMM150_INTF_RET_SUCCESS : BMM150_E_COM_FAIL;
}

/**
 * @brief BMM150 I2C write function
 * 
 * @param reg_addr Register address
 * @param reg_data Write data buffer
 * @param length Write length
 * @param intf_ptr Interface pointer (device address)
 * @return BMM150_INTF_RET_TYPE Return status
 */
static BMM150_INTF_RET_TYPE bmm150_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // Check if I2C device handle is valid
    if (s_i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return BMM150_E_COM_FAIL;
    }

    esp_err_t ret = i2c_bus_write_bytes(s_i2c_dev, reg_addr, (uint16_t)length, (uint8_t*)reg_data);
    return (ret == ESP_OK) ? BMM150_INTF_RET_SUCCESS : BMM150_E_COM_FAIL;
}

/**
 * @brief BMM150 delay function
 * 
 * @param period Delay time (microseconds)
 * @param intf_ptr Interface pointer (unused)
 */
static void bmm150_delay(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

/**
 * @brief BMM150 interface initialization
 * 
 * @param dev BMM150 device structure pointer
 * @param device_addr I2C device address
 * @return int8_t BMM150_OK indicates success
 * 
 * @note Creates both I2C bus and device handles
 */
int8_t bmm150_interface_init(struct bmm150_dev *dev, uint8_t device_addr)
{
    int8_t rslt = BMM150_OK;
    
    if (dev == NULL) {
        return BMM150_E_NULL_PTR;
    }
    
    /* I2C bus setup */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    s_i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &i2c_conf);
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus create failed");
        return BMM150_E_COM_FAIL;
    }
    
    /* Create I2C device handle */
    s_i2c_dev = i2c_bus_device_create(s_i2c_bus, device_addr, 0);
    if (s_i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C device create failed for address 0x%02X", device_addr);
        i2c_bus_delete(&s_i2c_bus);
        s_i2c_bus = NULL;
        return BMM150_E_COM_FAIL;
    }
    
    dev_addr = device_addr;
    ESP_LOGI(TAG, "I2C interface initialized (bus + device at 0x%02X)", device_addr);
    
    return BMM150_OK;
}

/**
 * @brief BMM150 interface deinitialization
 */
void bmm150_coines_deinit(void)
{
    if (s_i2c_dev) {
        i2c_bus_device_delete(&s_i2c_dev);
        s_i2c_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_bus_delete(&s_i2c_bus);
        s_i2c_bus = NULL;
    }
    ESP_LOGI(TAG, "Interface deinitialized");
}
/**
 * @brief BMM150 standalone initialization (try multiple addresses)
 * 
 * @return int8_t BMM150_OK indicates success
 * 
 * @note Will attempt to find BMM150 device from address list
 * @note After success, will be configured to regular mode (REGULAR mode)
 */
static int8_t bmm150_init_standalone(void)
{
    /* Try BMM150 default address first, then try other possible addresses */
    const uint8_t addr_candidates[] = { MAGNETOMETER_I2C_ADDR };
    int8_t rslt = BMM150_E_DEV_NOT_FOUND;

    for (size_t i = 0; i < sizeof(addr_candidates); ++i) {
        uint8_t addr = addr_candidates[i];
        
        /* Initialize I2C interface (bus + device) for this address */
        rslt = bmm150_interface_init(&s_bmm150, addr);
        if (rslt != BMM150_OK) {
            ESP_LOGE(TAG, "Interface init failed for address 0x%02X", addr);
            continue;
        }
        
        /* Configure BMM150 device structure */
        s_bmm150_addr = addr;
        s_bmm150.intf = BMM150_I2C_INTF;
        s_bmm150.read = bmm150_i2c_read;
        s_bmm150.write = bmm150_i2c_write;
        s_bmm150.delay_us = bmm150_delay;
        s_bmm150.intf_ptr = (void *)&s_bmm150_addr;

        /* Initialize BMM150 (reads CHIP_ID internally) */
        rslt = bmm150_init(&s_bmm150);
        ESP_LOGI(TAG, "Init at 0x%02X -> rslt=%d, chip_id=0x%02X (expect 0x32)", addr, rslt, s_bmm150.chip_id);

        if (s_bmm150.chip_id == MAGNETOMETER_CHIP_ID) {
            if (rslt != BMM150_OK) {
                /* Perform soft reset */
                (void)bmm150_soft_reset(&s_bmm150);
                s_bmm150.delay_us(BMM150_START_UP_TIME * 1000, s_bmm150.intf_ptr); /* Wait for start-up time */
            }

            /* Configure sensor settings: set preset mode to regular mode */
            struct bmm150_settings settings = { 0 };
            settings.pwr_mode = BMM150_POWERMODE_NORMAL;
            settings.preset_mode = BMM150_PRESETMODE_REGULAR;
            
            /* Set preset mode */
            rslt = bmm150_set_presetmode(&settings, &s_bmm150);
            if (rslt != BMM150_OK) {
                ESP_LOGE(TAG, "Set preset mode failed: %d", rslt);
                bmm150_coines_deinit();  // Clean up before trying next address
                continue;
            }
            
            /* Set power mode */
            rslt = bmm150_set_op_mode(&settings, &s_bmm150);
            if (rslt != BMM150_OK) {
                ESP_LOGE(TAG, "Set power mode failed: %d", rslt);
                bmm150_coines_deinit();  // Clean up before trying next address
                continue;
            }
            
            ESP_LOGI(TAG, "BMM150 initialized successfully at address 0x%02X", addr);
            return BMM150_OK;
        } else {
            // Wrong chip ID, clean up and try next address
            bmm150_coines_deinit();
        }
    }

    return rslt;
}

#elif defined(CONFIG_SENSOR_QMC6309)

/* ========== QMC6309 Geomagnetic Sensor Related Functions ========== */

/**
 * @brief QMC6309 interface initialization
 * 
 * @return esp_err_t ESP_OK indicates success
 */
static esp_err_t qmc6309_interface_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    s_i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &i2c_conf);

    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus create failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "I2C interface initialized");
    return ESP_OK;
}

/**
 * @brief QMC6309 write register
 * 
 * @param reg Register address
 * @param val Value to write
 * @return esp_err_t ESP_OK indicates success
 */
static esp_err_t qmc_write_reg(uint8_t reg, uint8_t val)
{
    return i2c_bus_write_bytes(s_i2c_dev, reg, 1, &val);
}

/**
 * @brief QMC6309 read register
 * 
 * @param reg Register address
 * @param data Read data buffer
 * @param len Read length
 * @return esp_err_t ESP_OK indicates success
 */
static esp_err_t qmc_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_bus_read_bytes(s_i2c_dev, reg, len, data);
}

/**
 * @brief QMC6309 initialization
 * 
 * @return esp_err_t ESP_OK indicates success
 * 
 * @note Configured as continuous mode, ±32G range, ODR=200Hz, OSR=8/8
 */
static esp_err_t qmc6309_init(void)
{
    // Create I2C device handle
    s_i2c_dev = i2c_bus_device_create(s_i2c_bus, MAGNETOMETER_I2C_ADDR, 0);
    if (s_i2c_dev == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C device for QMC6309");
        return ESP_FAIL;
    }

    /* Soft reset */
    ESP_ERROR_CHECK(qmc_write_reg(0x0B, 0x80));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(qmc_write_reg(0x0B, 0x00));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Continuous mode, ±32G, ODR=200Hz, OSR=8/8 */
    ESP_ERROR_CHECK(qmc_write_reg(0x0B, 0x00));  // RNG=00
    ESP_ERROR_CHECK(qmc_write_reg(0x0A, 0x63));  // MODE=11, OSR=8
    
    ESP_LOGI(TAG, "QMC6309 initialized successfully");
    return ESP_OK;
}

/**
 * @brief QMC6309 read 3-axis raw data
 * 
 * @param x X-axis data
 * @param y Y-axis data
 * @param z Z-axis data
 * @return esp_err_t ESP_OK indicates success
 * 
 * @note Reads data after waiting for data ready
 */
static esp_err_t qmc6309_read_raw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t status = 0;
    /* Wait for data ready */
    do {
        ESP_ERROR_CHECK(qmc_read_reg(0x09, &status, 1));
    } while ((status & 0x01) == 0);
    
    uint8_t data[6];
    ESP_ERROR_CHECK(qmc_read_reg(0x01, data, 6));
    *x = (int16_t)(data[0] | (data[1] << 8));
    *y = (int16_t)(data[2] | (data[3] << 8));
    *z = (int16_t)(data[4] | (data[5] << 8));
    return ESP_OK;
}

/**
 * @brief QMC6309 interface deinitialization
 * 
 * @return esp_err_t ESP_OK indicates success
 */
static esp_err_t qmc6309_interface_deinit(void)
{
    if (s_i2c_dev) {
        i2c_bus_device_delete(&s_i2c_dev);
        s_i2c_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_bus_delete(&s_i2c_bus);
        s_i2c_bus = NULL;
    }
    ESP_LOGI(TAG, "QMC6309 interface deinitialized");
    return ESP_OK;
}
#endif  // CONFIG_SENSOR_BMM150 / CONFIG_SENSOR_QMC6309

#if CONFIG_SENSOR_MAGNETOMETER

/* ========== Geomagnetic Sensor State and Calibration Related ========== */

/**
 * @brief Magnetic field position enumeration
 */
typedef enum {
    MAG_POSITION_UNKNOWN = 0,   /**< Unknown position */
    MAG_POSITION_REMOVED,       /**< Removed */
    MAG_POSITION_UP,            /**< Up position */
    MAG_POSITION_DOWN           /**< Down position */
} mag_position_t;

/**
 * @brief Calibration state enumeration
 */
typedef enum {
    CALIBRATION_IDLE = 0,           /**< Idle state */
    CALIBRATION_FIRST_POSITION,     /**< Calibrate first position (waiting for stable) */
    CALIBRATION_WAIT_SECOND,        /**< Waiting to move to second position */
    CALIBRATION_SECOND_POSITION,    /**< Calibrate second position (waiting for stable) */
    CALIBRATION_WAIT_THIRD,         /**< Waiting to move to third position */
    CALIBRATION_THIRD_POSITION,     /**< Calibrate third position (waiting for stable) */
    CALIBRATION_COMPLETED           /**< Calibration completed */
} calibration_state_t;

/** Calibrated center values (global variables) */
static int16_t s_calibrated_removed_center = MAG_STATE_REMOVED_CENTER;
static int16_t s_calibrated_up_center = MAG_STATE_UP_CENTER;
static int16_t s_calibrated_down_center = MAG_STATE_DOWN_CENTER;

/** Recalibration flag (global variable) */
static volatile bool s_request_recalibration = false;

/**
 * @brief Save calibration data to NVS
 * 
 * @param removed_center REMOVED state center value
 * @param up_center UP state center value
 * @param down_center DOWN state center value
 * @return esp_err_t ESP_OK indicates success
 * 
 * @note Calibration data saved in Flash, survives power loss
 */
static esp_err_t save_calibration_to_nvs(int16_t removed_center, int16_t up_center, int16_t down_center)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    // Open NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save three center values
    err = nvs_set_i16(nvs_handle, NVS_KEY_REMOVED, removed_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving removed_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_set_i16(nvs_handle, NVS_KEY_UP, up_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving up_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_set_i16(nvs_handle, NVS_KEY_DOWN, down_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving down_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Set calibration flag
    err = nvs_set_u8(nvs_handle, NVS_KEY_CALIBRATED, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving calibrated flag: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Calibration data saved to NVS successfully");
    }
    
    // Close NVS
    nvs_close(nvs_handle);
    
    return err;
}

/**
 * @brief Load calibration data from NVS
 * 
 * @param removed_center Output parameter: REMOVED state center value
 * @param up_center Output parameter: UP state center value
 * @param down_center Output parameter: DOWN state center value
 * @return esp_err_t ESP_OK indicates read success, ESP_ERR_NVS_NOT_FOUND indicates not calibrated
 */
static esp_err_t load_calibration_from_nvs(int16_t *removed_center, int16_t *up_center, int16_t *down_center)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    uint8_t calibrated = 0;
    
    // Open NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No calibration data found, need to calibrate");
        } else {
            ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        }
        return err;
    }
    
    // Check if already calibrated
    err = nvs_get_u8(nvs_handle, NVS_KEY_CALIBRATED, &calibrated);
    if (err != ESP_OK || calibrated != 1) {
        ESP_LOGI(TAG, "Device not calibrated yet");
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    
    // Read three center values
    err = nvs_get_i16(nvs_handle, NVS_KEY_REMOVED, removed_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading removed_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_get_i16(nvs_handle, NVS_KEY_UP, up_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading up_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_get_i16(nvs_handle, NVS_KEY_DOWN, down_center);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading down_center: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Close NVS
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Calibration data loaded from NVS:");
    ESP_LOGI(TAG, "  REMOVED center: %d", *removed_center);
    ESP_LOGI(TAG, "  UP center: %d", *up_center);
    ESP_LOGI(TAG, "  DOWN center: %d", *down_center);
    
    return ESP_OK;
}

/**
 * @brief Magnetometer read task
 * 
 * @param arg Task parameter (unused)
 * 
 * @note This function implements the following:
 *       1. Initialize sensor (BMM150 or QMC6309)
 *       2. Load calibration data from NVS or perform automatic calibration
 *       3. Use sliding window filter to smooth data
 *       4. Detect slider events based on state machine
 *       5. Single click detection (only in DOWN state)
 *       6. Send event notifications to serial port
 */
static void magnetometer_read_task(void *arg)
{
    ESP_LOGI(TAG, "Magnetometer read task started");
    
    // Sliding window buffer
    static int16_t s_window[MAG_WINDOW_SIZE] = {0};
    static uint8_t s_window_index = 0;
    static uint8_t s_window_filled = 0;  // Window fill count
    
    // State tracking variables
    static mag_position_t s_current_position = MAG_POSITION_UNKNOWN;       // Current stable position
    static mag_position_t s_motion_start_position = MAG_POSITION_UNKNOWN;  // Motion start position
    static mag_position_t s_candidate_position = MAG_POSITION_UNKNOWN;     // Candidate new position
    static bool s_is_in_motion = false;  // Whether in motion
    static uint8_t s_stable_count = 0;   // Stability counter
    
    // Single click detection variables - based on peak detection
    static int16_t s_down_baseline = 0;        // DOWN state baseline value
    static bool s_click_drop_detected = false;  // Drop detected
    static int16_t s_click_max_drop = 0;       // Record maximum drop
    static uint8_t s_click_duration = 0;       // Drop duration
    static uint8_t s_recover_count = 0;        // Recovery count
    
    // Calibration related variables
    static calibration_state_t s_calibration_state = CALIBRATION_IDLE;
    static uint8_t s_calibration_stable_count = 0;
    static int16_t s_calibration_value = 0;
    static int16_t s_calibration_last_average = 0;
    static bool s_calibration_positions[3] = {false, false, false};  // Track calibrated position indices (0=REMOVED, 1=UP, 2=DOWN)
    static int16_t s_calibration_temp_values[3] = {0};  // Temporarily store calibration values
    static uint8_t s_calibration_settle_count = 0;  // Settle counter after movement, ensure data truly stable

#ifdef CONFIG_SENSOR_BMM150
    /* bmm150_init_standalone will handle both interface and sensor initialization */
    int8_t rslt = bmm150_init_standalone();
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 init failed: %d", rslt);
        goto cleanup;
    }
#elif defined(CONFIG_SENSOR_QMC6309)
    if (qmc6309_interface_init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC6309 interface init failed");
        return;
    }

    if (qmc6309_init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC6309 init failed");
        goto cleanup;
    }
#endif
    
    // Try to load calibration data from NVS
    esp_err_t load_err = load_calibration_from_nvs(
        &s_calibrated_removed_center,
        &s_calibrated_up_center,
        &s_calibrated_down_center
    );
    
    if (load_err == ESP_OK) {
        // Calibration data loaded successfully, skip calibration flow
        ESP_LOGI(TAG, "========== Using Saved Calibration Data ==========");
        s_calibration_state = CALIBRATION_COMPLETED;
    } else {
        // No calibration data or load failed, start calibration flow
        ESP_LOGI(TAG, "========== Magnetometer Calibration Started ==========");
        ESP_LOGI(TAG, "Please keep the slider in current position and wait for calibration...");
        s_calibration_state = CALIBRATION_FIRST_POSITION;
    }
    
    while (1) {
        // Check if recalibration is needed
        if (s_request_recalibration && s_calibration_state == CALIBRATION_COMPLETED) {
            ESP_LOGI(TAG, "========== Re-calibration Requested ==========");
            ESP_LOGI(TAG, "Please keep the slider in current position and wait for calibration...");
            control_serial_send_magnetic_switch_calibration_step(MAG_SWITCH_CALIB_START);
            
            // Reset calibration state
            s_calibration_state = CALIBRATION_FIRST_POSITION;
            s_calibration_stable_count = 0;
            s_calibration_value = 0;
            s_calibration_last_average = 0;
            s_calibration_settle_count = 0;  // Reset settle counter
            s_calibration_positions[0] = false;
            s_calibration_positions[1] = false;
            s_calibration_positions[2] = false;
            s_calibration_temp_values[0] = 0;
            s_calibration_temp_values[1] = 0;
            s_calibration_temp_values[2] = 0;
            
            // Reset window
            s_window_filled = 0;
            s_window_index = 0;
            
            // Clear request flag
            s_request_recalibration = false;
        }
        
        int8_t ret = ESP_FAIL;
        int16_t s_mag_value = 0;
        
#ifdef CONFIG_SENSOR_BMM150
        struct bmm150_mag_data mag = { 0 };
        ret = bmm150_read_mag_data(&mag, &s_bmm150);

        s_mag_value = mag.z;
#elif defined(CONFIG_SENSOR_QMC6309)
        int16_t qmc_x, qmc_y, qmc_z;
        ret = qmc6309_read_raw(&qmc_x, &qmc_y, &qmc_z);
        s_mag_value = -qmc_z / 10;
#endif

        if (s_mag_value < 0) {
            s_mag_value = -s_mag_value;
        }

        if (ret == ESP_OK) {
            // 1. Update sliding window
            s_window[s_window_index] = s_mag_value;
            s_window_index = (s_window_index + 1) % MAG_WINDOW_SIZE;
            if (s_window_filled < MAG_WINDOW_SIZE) {
                s_window_filled++;
            }
            
            // 2. Calculate sliding window average (start checking after window is filled)
            if (s_window_filled >= MAG_WINDOW_SIZE) {
                int32_t sum = 0;
                for (uint8_t i = 0; i < MAG_WINDOW_SIZE; i++) {
                    sum += s_window[i];
                }
                int16_t average = sum / MAG_WINDOW_SIZE;
                
                // ========== Calibration State Machine ==========
                if (s_calibration_state != CALIBRATION_COMPLETED) {
                    // In calibration, do not execute normal motion recognition logic
                    
                    switch (s_calibration_state) {
                        case CALIBRATION_FIRST_POSITION:
                        {
                            // Check value stability (need to be more strict)
                            if (s_calibration_last_average == 0) {
                                // First reading, initialize
                                s_calibration_last_average = average;
                                s_calibration_stable_count = 0;
                            } else {
                                int16_t diff = abs(average - s_calibration_last_average);
                                
                                if (diff < 5) {  // Change less than 5, considered stable (more strict)
                                    s_calibration_stable_count++;
                                    
                                    if (s_calibration_stable_count >= MAG_STABLE_THRESHOLD * 3) {  // Need longer time stable
                                        // Stable long enough, record first position
                                        s_calibration_value = average;
                                        
                                        // Determine which preset value is closest
                                        int16_t diff_removed = abs(average - MAG_STATE_REMOVED_CENTER);
                                        int16_t diff_up = abs(average - MAG_STATE_UP_CENTER);
                                        int16_t diff_down = abs(average - MAG_STATE_DOWN_CENTER);
                                        
                                        if (diff_removed < diff_up && diff_removed < diff_down) {
                                            s_calibration_temp_values[0] = average;
                                            s_calibration_positions[0] = true;
                                            ESP_LOGI(TAG, "First position calibrated as REMOVED: %d", average);
                                        } else if (diff_up < diff_down) {
                                            s_calibration_temp_values[1] = average;
                                            s_calibration_positions[1] = true;
                                            ESP_LOGI(TAG, "First position calibrated as UP: %d", average);
                                        } else {
                                            s_calibration_temp_values[2] = average;
                                            s_calibration_positions[2] = true;
                                            ESP_LOGI(TAG, "First position calibrated as DOWN: %d", average);
                                        }
                                        
                                        // Enter next state
                                        s_calibration_state = CALIBRATION_WAIT_SECOND;
                                        s_calibration_stable_count = 0;
                                        s_calibration_settle_count = 0;  // Reset settle counter
                                        ESP_LOGI(TAG, "Please move the slider to another position...");
                                    }
                                } else {
                                    // Value still changing, reset stable count
                                    s_calibration_stable_count = 0;
                                }
                                
                                s_calibration_last_average = average;
                            }
                            break;
                        }
                        
                        case CALIBRATION_WAIT_SECOND:
                        {
                            // Wait for data to change (indicates user moved the slider)
                            int16_t change = abs(average - s_calibration_value);
                            
                            // After movement detected, settle for a period to ensure data is stable
                            if (s_calibration_settle_count > 0) {
                                // Movement detected, in settle period
                                int16_t settle_diff = abs(average - s_calibration_last_average);
                                
                                if (settle_diff < 10) {
                                    // Data change is small, considered stabilizing
                                    s_calibration_settle_count++;
                                    
                                    // Settle time enough (~500ms), enter stability check state
                                    if (s_calibration_settle_count >= MAG_STABLE_THRESHOLD * 2) {
                                        ESP_LOGI(TAG, "Data settled at %d, starting position recording...", average);
                                        s_calibration_state = CALIBRATION_SECOND_POSITION;
                                        s_calibration_stable_count = 0;
                                        s_calibration_last_average = 0;  // Reset, force first reading
                                        s_calibration_settle_count = 0;
                                    }
                                } else {
                                    // Data still changing significantly, reset settle count (slider still moving)
                                    s_calibration_settle_count = 1;
                                }
                                
                                s_calibration_last_average = average;
                            } else {
                                // Movement not detected yet, waiting for change
                                if (change > 100) {
                                    ESP_LOGI(TAG, "Movement detected (change=%d), waiting for data to settle...", change);
                                    s_calibration_settle_count = 1;  // Start settle period
                                    s_calibration_last_average = average;
                                }
                            }
                            break;
                        }
                        
                        case CALIBRATION_SECOND_POSITION:
                        {
                            // Check value stability (need to be more strict)
                            if (s_calibration_last_average == 0) {
                                // First reading, initialize
                                s_calibration_last_average = average;
                                s_calibration_stable_count = 0;
                            } else {
                                int16_t diff = abs(average - s_calibration_last_average);
                                
                                if (diff < 5) {  // Change less than 5, considered stable (more strict)
                                    s_calibration_stable_count++;
                                    
                                    if (s_calibration_stable_count >= MAG_STABLE_THRESHOLD * 3) {  // Need longer time stable
                                        // Stable long enough, record second position
                                        s_calibration_value = average;
                                        
                                        // Determine which uncalibrated preset value is closest
                                        int16_t min_diff = 32767;
                                        int best_index = -1;
                                        
                                        if (!s_calibration_positions[0]) {
                                            int16_t diff = abs(average - MAG_STATE_REMOVED_CENTER);
                                            if (diff < min_diff) {
                                                min_diff = diff;
                                                best_index = 0;
                                            }
                                        }
                                        if (!s_calibration_positions[1]) {
                                            int16_t diff = abs(average - MAG_STATE_UP_CENTER);
                                            if (diff < min_diff) {
                                                min_diff = diff;
                                                best_index = 1;
                                            }
                                        }
                                        if (!s_calibration_positions[2]) {
                                            int16_t diff = abs(average - MAG_STATE_DOWN_CENTER);
                                            if (diff < min_diff) {
                                                min_diff = diff;
                                                best_index = 2;
                                            }
                                        }
                                        
                                    if (best_index >= 0) {
                                        s_calibration_temp_values[best_index] = average;
                                        s_calibration_positions[best_index] = true;
                                        const char *pos_names[] = {"REMOVED", "UP", "DOWN"};
                                        ESP_LOGI(TAG, "Second position calibrated as %s: %d", 
                                                 pos_names[best_index], average);
                                    }
                                    
                                    // Enter next state
                                    s_calibration_state = CALIBRATION_WAIT_THIRD;
                                    s_calibration_stable_count = 0;
                                    s_calibration_settle_count = 0;  // Reset settle counter
                                    ESP_LOGI(TAG, "Please move the slider to the last position...");
                                    
                                    // Send 0x11 notification to head: second position completed, please prompt user for third action
                                    control_serial_send_magnetic_switch_calibration_step(MAG_SWITCH_CALIB_FIRST_DONE);
                                    }
                                } else {
                                    // Value still changing, reset stable count
                                    s_calibration_stable_count = 0;
                                }
                                
                                s_calibration_last_average = average;
                            }
                            break;
                        }
                        
                        case CALIBRATION_WAIT_THIRD:
                        {
                            // Wait for data to change
                            int16_t change = abs(average - s_calibration_value);
                            
                            // After movement detected, settle for a period to ensure data is stable
                            if (s_calibration_settle_count > 0) {
                                // Movement detected, in settle period
                                int16_t settle_diff = abs(average - s_calibration_last_average);
                                
                                if (settle_diff < 10) {
                                    // Data change is small, considered stabilizing
                                    s_calibration_settle_count++;
                                    
                                    // Settle time enough (~500ms), enter stability check state
                                    if (s_calibration_settle_count >= MAG_STABLE_THRESHOLD * 2) {
                                        ESP_LOGI(TAG, "Data settled at %d, starting position recording...", average);
                                        s_calibration_state = CALIBRATION_THIRD_POSITION;
                                        s_calibration_stable_count = 0;
                                        s_calibration_last_average = 0;  // Reset, force first reading
                                        s_calibration_settle_count = 0;
                                    }
                                } else {
                                    // Data still changing significantly, reset settle count (slider still moving)
                                    s_calibration_settle_count = 1;
                                }
                                
                                s_calibration_last_average = average;
                            } else {
                                // Movement not detected yet, waiting for change
                                if (change > 100) {
                                    ESP_LOGI(TAG, "Movement detected (change=%d), waiting for data to settle...", change);
                                    s_calibration_settle_count = 1;  // Start settle period
                                    s_calibration_last_average = average;
                                }
                            }
                            break;
                        }
                        
                        case CALIBRATION_THIRD_POSITION:
                        {
                            // Check value stability (need to be more strict)
                            if (s_calibration_last_average == 0) {
                                // First reading, initialize
                                s_calibration_last_average = average;
                                s_calibration_stable_count = 0;
                            } else {
                                int16_t diff = abs(average - s_calibration_last_average);
                                
                                if (diff < 5) {  // Change less than 5, considered stable (more strict)
                                    s_calibration_stable_count++;
                                    
                                    if (s_calibration_stable_count >= MAG_STABLE_THRESHOLD * 3) {  // Need longer time stable
                                        // Stable long enough, record third position
                                        
                                    // Find the last uncalibrated position
                                    for (int i = 0; i < 3; i++) {
                                        if (!s_calibration_positions[i]) {
                                            s_calibration_temp_values[i] = average;
                                            s_calibration_positions[i] = true;
                                            const char *pos_names[] = {"REMOVED", "UP", "DOWN"};
                                            ESP_LOGI(TAG, "Third position calibrated as %s: %d", 
                                                     pos_names[i], average);
                                            break;
                                        }
                                    }
                                        
                                        // Calibration complete, re-sort by magnitude relationship
                                        // REMOVED must be minimum value, UP is middle value, DOWN is maximum value
                                        int16_t sorted_values[3];
                                        sorted_values[0] = s_calibration_temp_values[0];
                                        sorted_values[1] = s_calibration_temp_values[1];
                                        sorted_values[2] = s_calibration_temp_values[2];
                                        
                                        // Simple bubble sort
                                        for (int i = 0; i < 2; i++) {
                                            for (int j = 0; j < 2 - i; j++) {
                                                if (sorted_values[j] > sorted_values[j + 1]) {
                                                    int16_t temp = sorted_values[j];
                                                    sorted_values[j] = sorted_values[j + 1];
                                                    sorted_values[j + 1] = temp;
                                                }
                                            }
                                        }
                                        
                                        // Assign by magnitude: minimum=REMOVED, middle=UP, maximum=DOWN
                                        s_calibrated_removed_center = sorted_values[0];
                                        s_calibrated_up_center = sorted_values[1];
                                        s_calibrated_down_center = sorted_values[2];
                                        
                                        s_calibration_state = CALIBRATION_COMPLETED;
                                        
                                        ESP_LOGI(TAG, "========== Calibration Completed ==========");
                                        ESP_LOGI(TAG, "Calibrated values (sorted by magnitude):");
                                        ESP_LOGI(TAG, "  REMOVED center: %d (default: %d)", 
                                                 s_calibrated_removed_center, MAG_STATE_REMOVED_CENTER);
                                        ESP_LOGI(TAG, "  UP center: %d (default: %d)", 
                                                 s_calibrated_up_center, MAG_STATE_UP_CENTER);
                                        ESP_LOGI(TAG, "  DOWN center: %d (default: %d)", 
                                                 s_calibrated_down_center, MAG_STATE_DOWN_CENTER);
                                        
                                        // Save calibration data to NVS
                                        esp_err_t save_err = save_calibration_to_nvs(
                                            s_calibrated_removed_center,
                                            s_calibrated_up_center,
                                            s_calibrated_down_center
                                        );
                                        
                                        if (save_err == ESP_OK) {
                                            ESP_LOGI(TAG, "Calibration data saved to Flash successfully");
                                        } else {
                                            ESP_LOGW(TAG, "Failed to save calibration data to Flash: %s", 
                                                     esp_err_to_name(save_err));
                                        }
                                        
                                        ESP_LOGI(TAG, "Normal operation started.");
                                        
                                        // Send 0x12 notification to head: calibration complete
                                        control_serial_send_magnetic_switch_calibration_step(MAG_SWITCH_CALIB_COMPLETE);
                                    }
                                } else {
                                    // Value still changing, reset stable count
                                    s_calibration_stable_count = 0;
                                }
                                
                                s_calibration_last_average = average;
                            }
                            break;
                        }
                        
                        default:
                            break;
                    }
                    
                    // Do not execute subsequent motion recognition logic during calibration
                    vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
                    continue;
                }
                
                // ========== Normal Operation Mode (after calibration completed) ==========
                
                // 3. Determine current position state based on average value (using calibrated values)
                mag_position_t detected_position = MAG_POSITION_UNKNOWN;
                
                // Calculate state ranges using calibrated center values
                int16_t calibrated_removed_min = s_calibrated_removed_center - MAG_STATE_REMOVED_OFFSET;
                int16_t calibrated_removed_max = s_calibrated_removed_center + MAG_STATE_REMOVED_OFFSET;
                int16_t calibrated_up_min = s_calibrated_up_center - MAG_STATE_UP_OFFSET;
                int16_t calibrated_up_max = s_calibrated_up_center + MAG_STATE_UP_OFFSET;
                int16_t calibrated_down_min = s_calibrated_down_center - MAG_STATE_DOWN_OFFSET;
                int16_t calibrated_down_max = s_calibrated_down_center + MAG_STATE_DOWN_OFFSET;

                // printf("average: %d\n", average);
                
                if (average >= calibrated_removed_min && average <= calibrated_removed_max) {
                    detected_position = MAG_POSITION_REMOVED;  // Removed
                } else if (average >= calibrated_up_min && average <= calibrated_up_max) {
                    detected_position = MAG_POSITION_UP;       // Up
                } else if (average >= calibrated_down_min && average <= calibrated_down_max) {
                    detected_position = MAG_POSITION_DOWN;     // Down
                }
                
                // 4. Single click detection - based on peak detection (not using average value)
                if (s_current_position == MAG_POSITION_DOWN) {
                    // In DOWN state, monitor rapid drop and recovery of magnetic field value
                    
                    // Update baseline value (use smooth update to avoid noise impact)
                    if (s_down_baseline == 0) {
                        s_down_baseline = s_mag_value;  // First initialization
                    } else if (!s_click_drop_detected) {
                        // When drop not detected, slowly track baseline value
                        s_down_baseline = (s_down_baseline * 9 + s_mag_value) / 10;
                    }
                    
                    // Detect drop (real-time value relative to baseline)
                    int16_t drop_value = s_down_baseline - s_mag_value;
                    
                    if (!s_click_drop_detected && drop_value >= MAG_CLICK_DROP_THRESHOLD) {
                        // First drop detected, enter single click detection mode
                        s_click_drop_detected = true;
                        s_click_max_drop = drop_value;
                        s_click_duration = 1;
                        s_recover_count = 0;
                    } else if (s_click_drop_detected) {
                        // Already in single click detection mode
                        s_click_duration++;
                        
                        // Update maximum drop
                        if (drop_value > s_click_max_drop) {
                            s_click_max_drop = drop_value;
                        }
                        
                        // Detect recovery: current drop value less than 40% of max drop, considered starting recovery
                        if (drop_value < (s_click_max_drop * 4 / 10)) {
                            s_recover_count++;
                            
                            // Recovery stable, trigger single click event
                            if (s_recover_count >= MAG_STABLE_THRESHOLD) {
                                s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SINGLE_CLICK;
                                ESP_LOGI(TAG, "SINGLE_CLICK detected (max_drop=%d, duration=%d)", 
                                         s_click_max_drop, s_click_duration);
                                
                                // Send event notification
                                control_serial_send_magnetic_switch_event(s_slider_state);
                                
                                // Reset single click detection
                                s_click_drop_detected = false;
                                s_click_max_drop = 0;
                                s_click_duration = 0;
                                s_recover_count = 0;
                                s_down_baseline = s_mag_value;
                            }
                        } else {
                            // Not recovered yet, reset recovery count
                            s_recover_count = 0;
                        }
                        
                        // Timeout detection: duration too long (>150ms), reset
                        if (s_click_duration > MAG_CLICK_TRANSITION_MAX_COUNT * 2) {
                            s_click_drop_detected = false;
                            s_click_max_drop = 0;
                            s_click_duration = 0;
                            s_recover_count = 0;
                            s_down_baseline = s_mag_value;
                        }
                    }
                } else {
                    // Not in DOWN state, reset single click detection
                    s_click_drop_detected = false;
                    s_click_max_drop = 0;
                    s_click_duration = 0;
                    s_recover_count = 0;
                    s_down_baseline = 0;
                }
                
                // 5. Stability detection and slide event triggering
                if (detected_position == MAG_POSITION_UNKNOWN) {
                    // In transition area (not in any defined state range)
                    if (!s_is_in_motion && s_current_position != MAG_POSITION_UNKNOWN) {
                        // Just left stable state, record motion start position
                        s_motion_start_position = s_current_position;
                        s_is_in_motion = true;
                    }
                    
                    // Reset candidate state and stable count
                    s_candidate_position = MAG_POSITION_UNKNOWN;
                    s_stable_count = 0;
                }
                else {
                    // Detected valid state (REMOVED/UP/DOWN)
                    if (detected_position == s_candidate_position) {
                        // Candidate state unchanged, accumulate stable count
                        s_stable_count++;
                        
                        // Check if reached stability threshold
                        if (s_stable_count >= MAG_STABLE_THRESHOLD && 
                            detected_position != s_current_position) {
                            // New state stabilized, can trigger event
                            
                            // Only trigger event during motion (avoid false trigger at initialization)
                            if (s_is_in_motion && s_motion_start_position != MAG_POSITION_UNKNOWN) {
                                magnetic_slide_switch_event_t event = MAGNETIC_SLIDE_SWITCH_EVENT_INIT;
                                
                                // Determine event based on start position and target position
                                if (s_motion_start_position == MAG_POSITION_UP && 
                                    detected_position == MAG_POSITION_DOWN) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN;
                                    ESP_LOGI(TAG, "SLIDE_DOWN");
                                }
                                else if (s_motion_start_position == MAG_POSITION_DOWN && 
                                         detected_position == MAG_POSITION_UP) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP;
                                    ESP_LOGI(TAG, "SLIDE_UP");
                                }
                                else if (s_motion_start_position == MAG_POSITION_UP && 
                                         detected_position == MAG_POSITION_REMOVED) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_UP;
                                    ESP_LOGI(TAG, "REMOVE_FROM_UP");
                                }
                                else if (s_motion_start_position == MAG_POSITION_DOWN && 
                                         detected_position == MAG_POSITION_REMOVED) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_DOWN;
                                    ESP_LOGI(TAG, "REMOVE_FROM_DOWN");
                                }
                                else if (s_motion_start_position == MAG_POSITION_REMOVED && 
                                         detected_position == MAG_POSITION_UP) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_UP;
                                    ESP_LOGI(TAG, "PLACE_FROM_UP");
                                }
                                else if (s_motion_start_position == MAG_POSITION_REMOVED && 
                                         detected_position == MAG_POSITION_DOWN) {
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_DOWN;
                                    ESP_LOGI(TAG, "PLACE_FROM_DOWN");
                                }
                                
                                // Update state and send event
                                if (event != MAGNETIC_SLIDE_SWITCH_EVENT_INIT) {
                                    s_slider_state = event;
                                    control_serial_send_magnetic_switch_event(event);
                                    
                                    // Only update motion start position after triggering event
                                    // This allows correct handling of continuous action sequences
                                    s_motion_start_position = detected_position;
                                }
                            } else {
                                // During initialization or no event triggered, also update start position
                                // This is first time position detected, or state unchanged
                                if (s_current_position == MAG_POSITION_UNKNOWN) {
                                    s_motion_start_position = detected_position;
                                }
                            }
                            
                            // Update current position, clear motion flag
                            s_current_position = detected_position;
                            s_is_in_motion = false;
                        }
                    }
                    else {
                        // Detected new candidate state, restart stability verification
                        s_candidate_position = detected_position;
                        s_stable_count = 1;
                    }
                }
            }
        } else {
            ESP_LOGE(TAG, "read_mag_data failed: %d", ret);
        }
        
        vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
#endif
    }

cleanup:
#ifdef CONFIG_SENSOR_BMM150
    bmm150_coines_deinit();
#elif defined(CONFIG_SENSOR_QMC6309)
    qmc6309_interface_deinit();
#endif
    ESP_LOGI(TAG, "Magnetometer read task finished.");
    vTaskDelete(NULL);
}

/* ========== Public Function Implementations ========== */

/**
 * @brief Start magnetic slide switch task
 * 
 * @note Creates Hall sensor or magnetometer read task based on configuration
 */
void magnetic_slide_switch_start(void)
{
#ifdef CONFIG_SENSOR_LINEAR_HALL
    xTaskCreate(hall_sensor_read_task, "hall_sensor_read_task", MAGNETIC_SLIDE_SWITCH_TASK_STACK_SIZE, NULL, 2, NULL);
#elif defined(CONFIG_SENSOR_MAGNETOMETER)
    xTaskCreate(magnetometer_read_task, "magnetometer_read_task", MAGNETIC_SLIDE_SWITCH_TASK_STACK_SIZE, NULL, 2, NULL);
#endif
}

/**
 * @brief Get current magnetic slide switch event
 * 
 * @return magnetic_slide_switch_event_t Current event type
 */
magnetic_slide_switch_event_t magnetic_slide_switch_get_event(void)
{
    return s_slider_state;
}

/**
 * @brief Trigger recalibration (no restart needed, takes effect immediately)
 * 
 * @note This function sets recalibration flag, magnetometer_read_task will automatically
 *       re-enter calibration mode in next loop, after calibration completes it will
 *       overwrite previously saved calibration data
 */
void magnetic_slide_switch_start_recalibration(void)
{
    ESP_LOGI(TAG, "Recalibration request received");
    s_request_recalibration = true;
}
