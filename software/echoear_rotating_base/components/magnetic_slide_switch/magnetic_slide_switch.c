/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

/** Shared magnetometer data structure */
typedef struct {
    int16_t x;  /**< X-axis magnetic field value */
    int16_t y;  /**< Y-axis magnetic field value */
    int16_t z;  /**< Z-axis magnetic field value */
    bool valid; /**< Data validity flag */
} magnetometer_data_t;

static magnetometer_data_t s_mag_data = {0};  /**< Shared magnetometer data */
static SemaphoreHandle_t s_mag_data_mutex = NULL;  /**< Mutex for protecting shared data */
static SemaphoreHandle_t s_calibration_complete_sem = NULL;  /**< Semaphore to signal calibration completion */
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
    CALIBRATION_IDLE = 0,               /**< Idle state */
    CALIBRATION_DETECTING_FIRST,        /**< Detecting first stable position (2s stable) */
    CALIBRATION_WAITING_CHANGE_1,      /**< Waiting for change from first position */
    CALIBRATION_DETECTING_SECOND,       /**< Detecting second stable position (2s stable, diff > threshold) */
    CALIBRATION_WAITING_CHANGE_2,       /**< Waiting for change from second position */
    CALIBRATION_DETECTING_THIRD,        /**< Detecting third stable position (2s stable, diff > threshold) */
    CALIBRATION_COMPLETED               /**< Calibration completed */
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
 * @brief Initialize magnetometer sensor
 * 
 * @return esp_err_t ESP_OK indicates success, ESP_FAIL indicates failure
 * 
 * @note Initializes BMM150 or QMC6309 sensor based on configuration
 */
static esp_err_t magnetometer_sensor_init(void)
{
#ifdef CONFIG_SENSOR_BMM150
    /* bmm150_init_standalone will handle both interface and sensor initialization */
    int8_t rslt = bmm150_init_standalone();
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "BMM150 init failed: %d", rslt);
        return ESP_FAIL;
    }
    return ESP_OK;
#elif defined(CONFIG_SENSOR_QMC6309)
    if (qmc6309_interface_init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC6309 interface init failed");
        return ESP_FAIL;
    }

    if (qmc6309_init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC6309 init failed");
        qmc6309_interface_deinit();
        return ESP_FAIL;
    }
    return ESP_OK;
#else
    return ESP_FAIL;
#endif
}

/**
 * @brief Magnetometer data read task (shared data provider)
 * 
 * @param arg Task parameter (unused)
 * 
 * @note This task continuously reads magnetometer data and updates shared data structure
 * @note Other tasks (slide switch and rotary knob) can read from shared data
 */
static void magnetometer_data_read_task(void *arg)
{
    ESP_LOGI(TAG, "Magnetometer data read task started");
    
    // Initialize magnetometer sensor
    if (magnetometer_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Magnetometer sensor initialization failed");
        goto cleanup;
    }
    
    // Create mutex for protecting shared data
    s_mag_data_mutex = xSemaphoreCreateMutex();
    if (s_mag_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex for magnetometer data");
        goto cleanup;
    }
    
    // Initialize shared data
    s_mag_data.x = 0;
    s_mag_data.y = 0;
    s_mag_data.z = 0;
    s_mag_data.valid = false;
    
    while (1) {
        int8_t ret = ESP_FAIL;
        int16_t mag_x = 0, mag_y = 0, mag_z = 0;
        
#ifdef CONFIG_SENSOR_BMM150
        struct bmm150_mag_data mag = { 0 };
        ret = bmm150_read_mag_data(&mag, &s_bmm150);
        if (ret == BMM150_OK) {
            mag_x = mag.x;
            mag_y = mag.y;
            mag_z = mag.z;
        }
#elif defined(CONFIG_SENSOR_QMC6309)
        int16_t qmc_x, qmc_y, qmc_z;
        ret = qmc6309_read_raw(&qmc_x, &qmc_y, &qmc_z);
        if (ret == ESP_OK) {
            // QMC6309 data processing: store raw values, z-axis will be processed in slide switch task
            mag_x = qmc_x;
            mag_y = qmc_y;
            mag_z = -qmc_z / 10;  // Same processing as before for z-axis
        }
#endif
        
        // Update shared data with mutex protection
        if (ret == ESP_OK || ret == BMM150_OK) {
            if (xSemaphoreTake(s_mag_data_mutex, portMAX_DELAY) == pdTRUE) {
                s_mag_data.x = mag_x;
                s_mag_data.y = mag_y;
                s_mag_data.z = mag_z;
                s_mag_data.valid = true;
                xSemaphoreGive(s_mag_data_mutex);
            }
        } else {
            ESP_LOGE(TAG, "Failed to read magnetometer data: %d", ret);
        }
        
        vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
    }

cleanup:
#ifdef CONFIG_SENSOR_BMM150
    bmm150_coines_deinit();
#elif defined(CONFIG_SENSOR_QMC6309)
    qmc6309_interface_deinit();
#endif
    if (s_mag_data_mutex != NULL) {
        vSemaphoreDelete(s_mag_data_mutex);
        s_mag_data_mutex = NULL;
    }
    ESP_LOGI(TAG, "Magnetometer data read task finished.");
    vTaskDelete(NULL);
}

/**
 * @brief Get magnetometer data from shared structure
 * 
 * @param x Output parameter: X-axis value
 * @param y Output parameter: Y-axis value
 * @param z Output parameter: Z-axis value
 * @return true Data is valid
 * @return false Data is invalid or mutex timeout
 */
static bool get_magnetometer_data(int16_t *x, int16_t *y, int16_t *z)
{
    if (s_mag_data_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(s_mag_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool valid = s_mag_data.valid;
        if (valid) {
            *x = s_mag_data.x;
            *y = s_mag_data.y;
            *z = s_mag_data.z;
        }
        xSemaphoreGive(s_mag_data_mutex);
        return valid;
    }
    
    return false;
}

/**
 * @brief Magnetometer calibration task
 * 
 * @param arg Task parameter (unused)
 * 
 * @note This task performs magnetometer calibration:
 *       1. Try to load calibration data from NVS
 *       2. If not found, perform automatic calibration (three positions)
 *       3. Save calibration data to NVS
 *       4. Signal completion via semaphore
 */
static void magnetometer_calibration_task(void *arg)
{
    ESP_LOGI(TAG, "Magnetometer calibration task started");
    
    // Wait for magnetometer data read task to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Sliding window buffer
    static int16_t s_window[MAG_WINDOW_SIZE] = {0};
    static uint8_t s_window_index = 0;
    static uint8_t s_window_filled = 0;  // Window fill count
    
    // Calibration related variables
    static calibration_state_t s_calibration_state = CALIBRATION_IDLE;
    static uint8_t s_calibration_stable_count = 0;
    static int16_t s_calibration_value = 0;
    static int16_t s_calibration_last_average = 0;
    static bool s_calibration_positions[3] = {false, false, false};  // Track calibrated position indices (0=REMOVED, 1=UP, 2=DOWN)
    static int16_t s_calibration_temp_values[3] = {0};  // Temporarily store calibration values
    static uint8_t s_calibration_settle_count = 0;  // Settle counter after movement, ensure data truly stable
    static bool s_first_completion_signaled = false;  // Track if calibration completion has been signaled
    
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
        
        // Immediately signal calibration completion (no need to wait for sliding window)
        if (!s_first_completion_signaled && s_calibration_complete_sem != NULL) {
            xSemaphoreGive(s_calibration_complete_sem);
            s_first_completion_signaled = true;
        }
    } else {
        // No calibration data or load failed, start automatic calibration flow
        ESP_LOGI(TAG, "========== Automatic Magnetometer Calibration Started ==========");
        ESP_LOGI(TAG, "Detecting stable positions automatically...");
        s_calibration_state = CALIBRATION_DETECTING_FIRST;
    }
    
    while (1) {
        // Check if recalibration is needed
        if (s_request_recalibration && s_calibration_state == CALIBRATION_COMPLETED) {
            ESP_LOGI(TAG, "========== Re-calibration Requested ==========");
            ESP_LOGI(TAG, "Starting automatic recalibration...");
            control_serial_send_magnetic_switch_calibration_step(MAG_SWITCH_CALIB_START);
            
            // Reset calibration state
            s_calibration_state = CALIBRATION_DETECTING_FIRST;
            s_calibration_stable_count = 0;
            s_calibration_value = 0;
            s_calibration_last_average = 0;
            s_calibration_settle_count = 0;
            s_calibration_positions[0] = false;
            s_calibration_positions[1] = false;
            s_calibration_positions[2] = false;
            s_calibration_temp_values[0] = 0;
            s_calibration_temp_values[1] = 0;
            s_calibration_temp_values[2] = 0;
            s_first_completion_signaled = false;  // Reset signal flag for recalibration
            
            // Reset window
            s_window_filled = 0;
            s_window_index = 0;
            
            // Clear request flag
            s_request_recalibration = false;
        }
        
        // Read z-axis data from shared structure
        int16_t mag_x, mag_y, mag_z;
        bool data_valid = get_magnetometer_data(&mag_x, &mag_y, &mag_z);
        
        if (!data_valid) {
            // Data not available yet, skip this iteration
            vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
            continue;
        }
        
        // Use z-axis data for calibration
        int16_t s_mag_value = mag_z;
        
        // Process absolute value
        if (s_mag_value < 0) {
            s_mag_value = -s_mag_value;
        }
        
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
            
            // ========== Automatic Calibration State Machine ==========
            if (s_calibration_state != CALIBRATION_COMPLETED) {
                // Calculate stability time threshold in ticks
                const TickType_t stability_ticks = pdMS_TO_TICKS(CALIBRATION_STABILITY_TIME_MS);
                static TickType_t s_stable_start_time = 0;  // Time when value became stable
                static int16_t s_stable_value = 0;  // The stable value being monitored
                static uint8_t s_calibrated_count = 0;  // Number of positions calibrated (0, 1, or 2)
                
                switch (s_calibration_state) {
                    case CALIBRATION_DETECTING_FIRST:
                    {
                        // Check if value is stable (change < threshold)
                        if (s_calibration_last_average == 0) {
                            // First reading, initialize
                            s_calibration_last_average = average;
                            s_stable_start_time = xTaskGetTickCount();
                            s_stable_value = average;
                        } else {
                            int16_t diff = abs(average - s_calibration_last_average);
                            
                            if (diff < 10) {  // Value is stable (within 10 units)
                                // Check if still the same stable value
                                if (abs(average - s_stable_value) < 10) {
                                    // Value is still stable, check if stable time >= 2 seconds
                                    TickType_t current_time = xTaskGetTickCount();
                                    TickType_t stable_duration = current_time - s_stable_start_time;
                                    
                                    if (stable_duration >= stability_ticks) {
                                        // Stable for required time, record first position
                                        s_calibration_temp_values[0] = average;
                                        s_calibrated_count = 1;
                                        ESP_LOGI(TAG, "First position calibrated: %d (stable for %lu ms)", 
                                                 average, (unsigned long)(stable_duration * portTICK_PERIOD_MS));
                                        
                                        // Enter next state: wait for change
                                        s_calibration_state = CALIBRATION_WAITING_CHANGE_1;
                                        s_calibration_last_average = 0;  // Reset for next detection
                                        s_stable_start_time = 0;
                                    }
                                } else {
                                    // Value changed but still stable, update stable value and time
                                    s_stable_value = average;
                                    s_stable_start_time = xTaskGetTickCount();
                                }
                            } else {
                                // Value changed significantly, reset stable tracking
                                s_stable_value = average;
                                s_stable_start_time = xTaskGetTickCount();
                            }
                            
                            s_calibration_last_average = average;
                        }
                        break;
                    }
                    
                    case CALIBRATION_WAITING_CHANGE_1:
                    {
                        // Wait for value to change significantly from first position
                        int16_t diff_from_first = abs(average - s_calibration_temp_values[0]);
                        
                        if (diff_from_first > CALIBRATION_VALUE_DIFF_THRESHOLD) {
                            // Value changed significantly, enter second position detection
                            ESP_LOGI(TAG, "Change detected from first position (diff=%d), detecting second position...", diff_from_first);
                            s_calibration_state = CALIBRATION_DETECTING_SECOND;
                            s_calibration_last_average = 0;  // Reset for next detection
                            s_stable_start_time = 0;
                        }
                        break;
                    }
                    
                    case CALIBRATION_DETECTING_SECOND:
                    {
                        // Check if value is stable and different from first position
                        int16_t diff_from_first = abs(average - s_calibration_temp_values[0]);
                        
                        if (diff_from_first <= CALIBRATION_VALUE_DIFF_THRESHOLD) {
                            // Too close to first position, reset
                            s_calibration_last_average = 0;
                            s_stable_start_time = 0;
                            break;
                        }
                        
                        // Check if value is stable
                        if (s_calibration_last_average == 0) {
                            // First reading, initialize
                            s_calibration_last_average = average;
                            s_stable_start_time = xTaskGetTickCount();
                            s_stable_value = average;
                        } else {
                            int16_t diff = abs(average - s_calibration_last_average);
                            
                            if (diff < 10) {  // Value is stable (within 10 units)
                                // Check if still the same stable value
                                if (abs(average - s_stable_value) < 10) {
                                    // Value is still stable, check if stable time >= 2 seconds
                                    TickType_t current_time = xTaskGetTickCount();
                                    TickType_t stable_duration = current_time - s_stable_start_time;
                                    
                                    if (stable_duration >= stability_ticks) {
                                        // Stable for required time and different from first, record second position
                                        s_calibration_temp_values[1] = average;
                                        s_calibrated_count = 2;
                                        ESP_LOGI(TAG, "Second position calibrated: %d (stable for %lu ms, diff from first=%d)", 
                                                 average, (unsigned long)(stable_duration * portTICK_PERIOD_MS), diff_from_first);
                                        
                                        // Send notification: second position calibration done (0x0012)
                                        control_serial_send_magnetic_switch_calibration_step(MAG_SWITCH_CALIB_FIRST_DONE);
                                        
                                        // Enter next state: wait for change
                                        s_calibration_state = CALIBRATION_WAITING_CHANGE_2;
                                        s_calibration_last_average = 0;  // Reset for next detection
                                        s_stable_start_time = 0;
                                    }
                                } else {
                                    // Value changed but still stable, update stable value and time
                                    s_stable_value = average;
                                    s_stable_start_time = xTaskGetTickCount();
                                }
                            } else {
                                // Value changed significantly, reset stable tracking
                                s_stable_value = average;
                                s_stable_start_time = xTaskGetTickCount();
                            }
                            
                            s_calibration_last_average = average;
                        }
                        break;
                    }
                    
                    case CALIBRATION_WAITING_CHANGE_2:
                    {
                        // Wait for value to change significantly from second position
                        int16_t diff_from_second = abs(average - s_calibration_temp_values[1]);
                        
                        if (diff_from_second > CALIBRATION_VALUE_DIFF_THRESHOLD) {
                            // Value changed significantly, enter third position detection
                            ESP_LOGI(TAG, "Change detected from second position (diff=%d), detecting third position...", diff_from_second);
                            s_calibration_state = CALIBRATION_DETECTING_THIRD;
                            s_calibration_last_average = 0;  // Reset for next detection
                            s_stable_start_time = 0;
                        }
                        break;
                    }
                    
                    case CALIBRATION_DETECTING_THIRD:
                    {
                        // Check if value is stable and different from both first and second positions
                        int16_t diff_from_first = abs(average - s_calibration_temp_values[0]);
                        int16_t diff_from_second = abs(average - s_calibration_temp_values[1]);
                        
                        if (diff_from_first <= CALIBRATION_VALUE_DIFF_THRESHOLD || 
                            diff_from_second <= CALIBRATION_VALUE_DIFF_THRESHOLD) {
                            // Too close to existing positions, reset
                            s_calibration_last_average = 0;
                            s_stable_start_time = 0;
                            break;
                        }
                        
                        // Check if value is stable
                        if (s_calibration_last_average == 0) {
                            // First reading, initialize
                            s_calibration_last_average = average;
                            s_stable_start_time = xTaskGetTickCount();
                            s_stable_value = average;
                        } else {
                            int16_t diff = abs(average - s_calibration_last_average);
                            
                            if (diff < 10) {  // Value is stable (within 10 units)
                                // Check if still the same stable value
                                if (abs(average - s_stable_value) < 10) {
                                    // Value is still stable, check if stable time >= 2 seconds
                                    TickType_t current_time = xTaskGetTickCount();
                                    TickType_t stable_duration = current_time - s_stable_start_time;
                                    
                                    if (stable_duration >= stability_ticks) {
                                        // Stable for required time and different from both positions, record third position
                                        s_calibration_temp_values[2] = average;
                                        s_calibrated_count = 3;
                                        ESP_LOGI(TAG, "Third position (REMOVED) calibrated: %d (stable for %lu ms, diff from first=%d, diff from second=%d)", 
                                                 average, (unsigned long)(stable_duration * portTICK_PERIOD_MS), 
                                                 diff_from_first, diff_from_second);
                                        
                                        // Now we have all three positions, re-sort by magnitude relationship
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
                                        
                                        ESP_LOGI(TAG, "========== Automatic Calibration Completed ==========");
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
                                        
                                        // Send notification: calibration complete
                                        control_serial_send_magnetic_switch_calibration_step(MAG_SWITCH_CALIB_COMPLETE);
                                        
                                        // Signal calibration completion will be handled in the loop check below
                                    }
                                } else {
                                    // Value changed but still stable, update stable value and time
                                    s_stable_value = average;
                                    s_stable_start_time = xTaskGetTickCount();
                                }
                            } else {
                                // Value changed significantly, reset stable tracking
                                s_stable_value = average;
                                s_stable_start_time = xTaskGetTickCount();
                            }
                            
                            s_calibration_last_average = average;
                        }
                        break;
                    }
                    
                    default:
                        break;
                }
                
                // If calibration is completed (either loaded from NVS or finished), signal and wait for recalibration
                if (s_calibration_state == CALIBRATION_COMPLETED) {
                    // Signal calibration completion (only once, on first completion)
                    if (!s_first_completion_signaled && s_calibration_complete_sem != NULL) {
                        xSemaphoreGive(s_calibration_complete_sem);
                        s_first_completion_signaled = true;
                    }
                    // Wait for recalibration request (task stays alive to handle recalibration)
                    // The recalibration check at the beginning of the loop will handle it
                }
            } else {
                // Calibration already completed, signal and wait for recalibration
                if (!s_first_completion_signaled && s_calibration_complete_sem != NULL) {
                    xSemaphoreGive(s_calibration_complete_sem);
                    s_first_completion_signaled = true;
                }
                // Wait for recalibration request (task stays alive to handle recalibration)
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
    }
}

/**
 * @brief Slide switch event detection task
 * 
 * @param arg Task parameter (unused)
 * 
 * @note This function implements the following:
 *       1. Use sliding window filter to smooth z-axis data
 *       2. Detect slider events based on state machine
 *       3. Single click detection (only in DOWN state)
 *       4. Send event notifications to serial port
 */
static void slide_switch_event_detect_task(void *arg)
{
    ESP_LOGI(TAG, "Slide switch event detection task started");
    
    // Wait for magnetometer data read task and calibration task to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    
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
    
    // Single click detection state machine (based on mag_x)
    typedef enum {
        CLICK_STATE_IDLE = 0,           // Idle state (waiting for click)
        CLICK_STATE_DROP_DETECTED,      // State 1: mag_x dropped significantly (drop detected, click half executed)
        CLICK_STATE_RECOVERING          // State 2: mag_x recovering (recovering)
    } click_state_t;
    
    static click_state_t s_click_state = CLICK_STATE_IDLE;
    static int16_t s_click_initial_mag_x = 0;  // Initial mag_x value before click
    static TickType_t s_click_start_time = 0;  // Click start time (when change detected)
    
    while (1) {
        
        // Read z-axis data from shared structure
        int16_t mag_x, mag_y, mag_z;
        bool data_valid = get_magnetometer_data(&mag_x, &mag_y, &mag_z);
        // control_serial_print_magnetometer_data(mag_x, mag_y, mag_z);
        
        if (!data_valid) {
            // Data not available yet, skip this iteration
            vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
            continue;
        }
        
        // Use z-axis data for slide switch detection
        int16_t s_mag_value = mag_z;
        
        // Process absolute value (same as before)
        if (s_mag_value < 0) {
            s_mag_value = -s_mag_value;
        }
        
        // Debug output (can be removed later)
        // ESP_LOGI(TAG, "Slide switch z-axis: %d", s_mag_value);
        
        {
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
                
                // ========== Normal Operation Mode ==========
                
                // 3. Determine which center value the average is closest to
                // Also calculate detected_position for pairing event detection
                mag_position_t detected_position = MAG_POSITION_UNKNOWN;
                
                // Calculate state ranges using calibrated center values (for pairing event detection)
                int16_t calibrated_removed_min = s_calibrated_removed_center - MAG_STATE_REMOVED_OFFSET;
                int16_t calibrated_removed_max = s_calibrated_removed_center + MAG_STATE_REMOVED_OFFSET;
                int16_t calibrated_up_min = s_calibrated_up_center - MAG_STATE_UP_OFFSET;
                int16_t calibrated_up_max = s_calibrated_up_center + MAG_STATE_UP_OFFSET;
                int16_t calibrated_down_min = s_calibrated_down_center - MAG_STATE_DOWN_OFFSET;
                int16_t calibrated_down_max = s_calibrated_down_center + MAG_STATE_DOWN_OFFSET;
                
                // ========== Initial Position Detection (Power-on) ==========
                // Detect initial slider position once after power-on
                // Only detect if average falls within UP or DOWN range (not REMOVED)
                static bool s_initial_position_detected = false;
                if (!s_initial_position_detected) {
                    // Check if average falls within UP range
                    if (average >= calibrated_up_min && average <= calibrated_up_max) {
                        // In UP position
                        s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP;
                        ESP_LOGI(TAG, "Initial position detected: SLIDE_UP (average: %d, UP range: [%d-%d])", 
                                 average, calibrated_up_min, calibrated_up_max);
                        control_serial_send_magnetic_switch_event(s_slider_state);
                        s_initial_position_detected = true;
                    }
                    // Check if average falls within DOWN range
                    else if (average >= calibrated_down_min && average <= calibrated_down_max) {
                        // In DOWN position
                        s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN;
                        ESP_LOGI(TAG, "Initial position detected: SLIDE_DOWN (average: %d, DOWN range: [%d-%d])", 
                                 average, calibrated_down_min, calibrated_down_max);
                        control_serial_send_magnetic_switch_event(s_slider_state);
                        s_initial_position_detected = true;
                    }
                    // If average is in REMOVED range or unknown, wait for next reading
                    // (Don't mark as detected, keep checking until we find UP or DOWN position)
                }

                // control_serial_print_magnetometer_data(0, 0, average);

                if (average >= calibrated_removed_min && average <= calibrated_removed_max) {
                    detected_position = MAG_POSITION_REMOVED;  // Removed
                } else if (average >= calibrated_up_min && average <= calibrated_up_max) {
                    detected_position = MAG_POSITION_UP;       // Up
                } else if (average >= calibrated_down_min && average <= calibrated_down_max) {
                    detected_position = MAG_POSITION_DOWN;     // Down
                }
                
                // Calculate distances to each center value (for event detection)
                int16_t dist_to_removed = abs(average - s_calibrated_removed_center);
                int16_t dist_to_up = abs(average - s_calibrated_up_center);
                int16_t dist_to_down = abs(average - s_calibrated_down_center);
                
                // Find the closest center value
                int16_t current_center = s_calibrated_removed_center;
                mag_position_t current_center_position = MAG_POSITION_REMOVED;
                int16_t min_dist = dist_to_removed;
                
                if (dist_to_up < min_dist) {
                    min_dist = dist_to_up;
                    current_center = s_calibrated_up_center;
                    current_center_position = MAG_POSITION_UP;
                }
                if (dist_to_down < min_dist) {
                    min_dist = dist_to_down;
                    current_center = s_calibrated_down_center;
                    current_center_position = MAG_POSITION_DOWN;
                }
                
                // Track last stable center value and position
                static int16_t s_last_stable_center = 0;
                static mag_position_t s_last_stable_position = MAG_POSITION_UNKNOWN;
                static int16_t s_candidate_center = 0;
                static mag_position_t s_candidate_position = MAG_POSITION_UNKNOWN;
                static uint8_t s_stable_count = 0;
                
                // Check if average is stable near current center (within threshold)
                const int16_t CENTER_STABILITY_THRESHOLD = MAG_STATE_REMOVED_OFFSET;  // Use same threshold as position detection
                bool is_near_center = (min_dist <= CENTER_STABILITY_THRESHOLD);
                
                if (is_near_center) {
                    // Average is near a center value
                    if (current_center == s_candidate_center && current_center_position == s_candidate_position) {
                        // Same candidate, accumulate stable count
                        s_stable_count++;
                        
                        // Check if stable enough to confirm position change
                        if (s_stable_count >= MAG_STABLE_THRESHOLD) {
                            // Position is stable, check if it's different from last stable position
                            if (s_last_stable_position != current_center_position && s_last_stable_position != MAG_POSITION_UNKNOWN) {
                                // Position changed, trigger event based on center value transition
                                magnetic_slide_switch_event_t event = MAGNETIC_SLIDE_SWITCH_EVENT_INIT;
                                
                                // Determine event based on center value transitions
                                if (s_last_stable_center == s_calibrated_down_center && 
                                    current_center == s_calibrated_up_center) {
                                    // Down -> Up: Slide up
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP;
                                    ESP_LOGI(TAG, "SLIDE_UP (average: %d -> %d)", s_last_stable_center, current_center);
                                }
                                else if (s_last_stable_center == s_calibrated_up_center && 
                                         current_center == s_calibrated_down_center) {
                                    // Up -> Down: Slide down
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN;
                                    ESP_LOGI(TAG, "SLIDE_DOWN (average: %d -> %d)", s_last_stable_center, current_center);
                                }
                                else if (s_last_stable_center == s_calibrated_up_center && 
                                         current_center == s_calibrated_removed_center) {
                                    // Up -> Removed: Remove from up
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_UP;
                                    ESP_LOGI(TAG, "REMOVE_FROM_UP (average: %d -> %d)", s_last_stable_center, current_center);
                                }
                                else if (s_last_stable_center == s_calibrated_removed_center && 
                                         current_center == s_calibrated_up_center) {
                                    // Removed -> Up: Place from up
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_UP;
                                    ESP_LOGI(TAG, "PLACE_FROM_UP (average: %d -> %d)", s_last_stable_center, current_center);
                                }
                                else if (s_last_stable_center == s_calibrated_down_center && 
                                         current_center == s_calibrated_removed_center) {
                                    // Down -> Removed: Remove from down
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_DOWN;
                                    ESP_LOGI(TAG, "REMOVE_FROM_DOWN (average: %d -> %d)", s_last_stable_center, current_center);
                                }
                                else if (s_last_stable_center == s_calibrated_removed_center && 
                                         current_center == s_calibrated_down_center) {
                                    // Removed -> Down: Place from down
                                    event = MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_DOWN;
                                    ESP_LOGI(TAG, "PLACE_FROM_DOWN (average: %d -> %d)", s_last_stable_center, current_center);
                                }
                                
                                // Send event if detected
                                if (event != MAGNETIC_SLIDE_SWITCH_EVENT_INIT) {
                                    s_slider_state = event;
                                    control_serial_send_magnetic_switch_event(event);
                                    
                                    // Enable "the fish is attached" detection after REMOVE_FROM_UP or REMOVE_FROM_DOWN event
                                    if (event == MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_UP || 
                                        event == MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_DOWN) {
                                        // Will be handled in fish detection section
                                    }
                                    // Disable "the fish is attached" detection after PLACE_FROM_UP or PLACE_FROM_DOWN event
                                    else if (event == MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_UP || 
                                             event == MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_DOWN) {
                                        // Will be handled in fish detection section
                                    }
                                }
                                
                                // Update last stable position
                                s_last_stable_center = current_center;
                                s_last_stable_position = current_center_position;
                            } else if (s_last_stable_position == MAG_POSITION_UNKNOWN) {
                                // First time detection, just record position
                                s_last_stable_center = current_center;
                                s_last_stable_position = current_center_position;
                            }
                            
                            // Reset stable count
                            s_stable_count = 0;
                        }
                    } else {
                        // New candidate, reset
                        s_candidate_center = current_center;
                        s_candidate_position = current_center_position;
                        s_stable_count = 1;
                    }
                } else {
                    // Not near any center, reset candidate
                    s_candidate_center = 0;
                    s_candidate_position = MAG_POSITION_UNKNOWN;
                    s_stable_count = 0;
                }
                
                // 4. Single click detection - state machine based on mag_x value
                // Click event: mag_x drops by MAG_CLICK_X_DROP_THRESHOLD, then recovers to initial value
                // Only detect click when last stable position is DOWN
                if (s_last_stable_position == MAG_POSITION_DOWN) {
                    // State machine for click detection
                    switch (s_click_state) {
                        case CLICK_STATE_IDLE:
                        {
                            // Update initial mag_x value when stable in DOWN position
                            if (s_click_initial_mag_x == 0) {
                                s_click_initial_mag_x = mag_x;
                            } else {
                                // Update initial value if mag_x is stable (within small range)
                                int16_t diff = abs(mag_x - s_click_initial_mag_x);
                                if (diff <= 50) {  // Value is stable (within 20 units)
                                    s_click_initial_mag_x = mag_x;
                                    // printf("s_click_initial_mag_x updated: %d\n", s_click_initial_mag_x);
                                }
                            }
                            
                            // Check if mag_x changes significantly (State 1: change detected)
                            // Support both drop and rise
                            int16_t change_amount = s_click_initial_mag_x - mag_x;  // Negative means rise
                            int16_t abs_change = abs(change_amount);
                            if (abs_change >= MAG_CLICK_X_DROP_THRESHOLD) {
                                // Before entering click detection, verify slider is in DOWN position
                                // During click, mag_x may briefly drop below mag_y at lowest point, but at start it should be mag_x > mag_y
                                if (mag_x > mag_y) {
                                    // printf("change_amount: %d\n", change_amount);
                                    s_click_state = CLICK_STATE_DROP_DETECTED;
                                    s_click_start_time = xTaskGetTickCount();  // Record start time
                                    // printf("s_click_start_time: %ld\n", s_click_start_time);
                                    if (change_amount > 0) {
                                        ESP_LOGD(TAG, "[CLICK] State 1: Drop detected (mag_x=%d > mag_y=%d, initial=%d, drop=%d >= %d)", 
                                                 mag_x, mag_y, s_click_initial_mag_x, change_amount, MAG_CLICK_X_DROP_THRESHOLD);
                                    } else {
                                        ESP_LOGD(TAG, "[CLICK] State 1: Rise detected (mag_x=%d > mag_y=%d, initial=%d, rise=%d >= %d)", 
                                                 mag_x, mag_y, s_click_initial_mag_x, -change_amount, MAG_CLICK_X_DROP_THRESHOLD);
                                    }
                                } else {
                                    // mag_x <= mag_y, this is likely a slide action, not a click
                                    ESP_LOGD(TAG, "[CLICK] State 0: Change detected but mag_x=%d <= mag_y=%d, skip (likely sliding)", 
                                             mag_x, mag_y);
                                }
                            }
                            break;
                        }
                        
                        case CLICK_STATE_DROP_DETECTED:
                        {
                            // State 1: mag_x changed significantly (drop or rise)
                            // Wait for mag_x to start recovering (State 2: recovering)
                            // Note: At the lowest point, mag_y may briefly exceed mag_x, this is normal for a click
                            //       So we don't check mag_x > mag_y here to avoid false resets
                            
                            int16_t change_amount = s_click_initial_mag_x - mag_x;  // Negative means rise
                            int16_t abs_change = abs(change_amount);
                            // printf("CLICK_STATE_DROP_DETECTED change_amount: %d\n", change_amount);
                            
                            if (abs_change < MAG_CLICK_X_DROP_THRESHOLD) {
                                // mag_x has recovered below threshold, enter recovering state
                                s_click_state = CLICK_STATE_RECOVERING;
                                ESP_LOGD(TAG, "[CLICK] State 2: Recovering (mag_x=%d, initial=%d, change=%d)", 
                                         mag_x, s_click_initial_mag_x, change_amount);
                            }
                            // Otherwise, keep waiting (mag_x still changed or in middle range)
                            else {
                                // Check timeout to prevent getting stuck
                                TickType_t current_time = xTaskGetTickCount();
                                TickType_t elapsed_ticks = current_time - s_click_start_time;
                                uint32_t elapsed_ms = elapsed_ticks * portTICK_PERIOD_MS;
                                
                                if (elapsed_ms >= MAG_CLICK_DURATION_MAX_MS) {
                                    // Timeout: not a click, reset to idle
                                    ESP_LOGW(TAG, "[CLICK] State 1: Timeout (mag_x=%d, change=%d, elapsed=%lums), reset to idle", 
                                             mag_x, change_amount, (unsigned long)elapsed_ms);
                                    s_click_state = CLICK_STATE_IDLE;
                                    s_click_initial_mag_x = mag_x;  // Update to new stable position
                                    s_click_start_time = 0;
                                } else {
                                    // Still within timeout, keep waiting
                                    if (change_amount > 0) {
                                        ESP_LOGD(TAG, "[CLICK] State 1: Still dropping (mag_x=%d, drop=%d)", mag_x, change_amount);
                                    } else if (change_amount < 0) {
                                        ESP_LOGD(TAG, "[CLICK] State 1: Still rising (mag_x=%d, rise=%d)", mag_x, -change_amount);
                                    }
                                }
                            }
                            break;
                        }
                        
                        case CLICK_STATE_RECOVERING:
                        {
                            // State 2: mag_x is recovering
                            // During recovery, slider must stay in DOWN position (mag_x > mag_y)
                            
                            // Check if slider is still in DOWN position (mag_x > mag_y)
                            if (mag_x <= mag_y) {
                                // Slider moved to UP position during recovery, this is a slide action, not a click
                                ESP_LOGW(TAG, "[CLICK] State 2: Slider moved to UP (mag_x=%d <= mag_y=%d), reset to idle", 
                                         mag_x, mag_y);
                                s_click_state = CLICK_STATE_IDLE;
                                s_click_initial_mag_x = 0;
                                s_click_start_time = 0;
                                break;
                            }
                            
                            // Check if mag_x recovers back to initial value (State 3: click completed)
                            int16_t diff_from_initial = abs(mag_x - s_click_initial_mag_x);
                            if (diff_from_initial <= 100) {  // Recovered to within 100 units of initial value
                                
                                // Calculate elapsed time from change detection to recovery
                                TickType_t current_time = xTaskGetTickCount();
                                TickType_t elapsed_ticks = current_time - s_click_start_time;
                                uint32_t elapsed_ms = elapsed_ticks * portTICK_PERIOD_MS;
                                
                                // Only trigger click if duration is less than threshold
                                if (elapsed_ms < MAG_CLICK_DURATION_MAX_MS) {
                                    // Final check: slider must still be in DOWN position (mag_x > mag_y)
                                    if (mag_x > mag_y) {
                                        // Click completed, trigger event
                                        s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_SINGLE_CLICK;
                                        ESP_LOGD(TAG, "[CLICK] State 3: Click completed (mag_x=%d > mag_y=%d, initial=%d, diff=%d, duration=%lums)", 
                                                 mag_x, mag_y, s_click_initial_mag_x, diff_from_initial, (unsigned long)elapsed_ms);
                                        ESP_LOGI(TAG, "SINGLE_CLICK detected (initial_mag_x=%d, final_mag_x=%d, duration=%lums)", 
                                                 s_click_initial_mag_x, mag_x, (unsigned long)elapsed_ms);
                                        
                                        // Send event notification
                                        control_serial_send_magnetic_switch_event(s_slider_state);
                                    } else {
                                        // mag_x <= mag_y at completion, this is not a valid click
                                        ESP_LOGD(TAG, "[CLICK] State 3: Rejected - mag_x=%d <= mag_y=%d at completion", 
                                                 mag_x, mag_y);
                                    }
                                } else {
                                    // Duration too long, not a valid click (likely a slide action)
                                    ESP_LOGD(TAG, "[CLICK] Rejected: Duration too long (mag_x=%d, initial=%d, duration=%lums >= %dms)", 
                                             mag_x, s_click_initial_mag_x, (unsigned long)elapsed_ms, MAG_CLICK_DURATION_MAX_MS);
                                }
                                
                                // Reset to idle state
                                s_click_state = CLICK_STATE_IDLE;
                                s_click_initial_mag_x = mag_x;  // Update initial value for next click
                                s_click_start_time = 0;  // Reset start time
                            }
                            // If mag_x changes significantly again, might be noise, reset to drop state
                            else {
                                int16_t change_amount = s_click_initial_mag_x - mag_x;  // Negative means rise
                                int16_t abs_change = abs(change_amount);
                                
                                if (abs_change >= MAG_CLICK_X_DROP_THRESHOLD) {
                                    ESP_LOGD(TAG, "[CLICK] State 2->1: Changed again (mag_x=%d, change=%d >= %d), reset to change detected state", 
                                             mag_x, abs_change, MAG_CLICK_X_DROP_THRESHOLD);
                                    s_click_state = CLICK_STATE_DROP_DETECTED;
                                    s_click_start_time = xTaskGetTickCount();  // Update start time for new change
                                } else {
                                    // Still recovering, check timeout
                                    TickType_t current_time = xTaskGetTickCount();
                                    TickType_t elapsed_ticks = current_time - s_click_start_time;
                                    uint32_t elapsed_ms = elapsed_ticks * portTICK_PERIOD_MS;
                                    
                                    if (elapsed_ms >= MAG_CLICK_DURATION_MAX_MS) {
                                        // Timeout, reset to idle state
                                        ESP_LOGD(TAG, "[CLICK] State 2: Timeout in recovering (mag_x=%d, diff=%d, elapsed=%lums), reset to idle", 
                                                 mag_x, diff_from_initial, (unsigned long)elapsed_ms);
                                        s_click_state = CLICK_STATE_IDLE;
                                        s_click_start_time = 0;
                                        s_click_initial_mag_x = mag_x;
                                    } else {
                                        // Still recovering
                                        ESP_LOGD(TAG, "[CLICK] State 2: Recovering (mag_x=%d, initial=%d, diff=%d)", 
                                                 mag_x, s_click_initial_mag_x, diff_from_initial);
                                    }
                                }
                            }
                            break;
                        }
                    }
                } else {
                    // Not in DOWN state, reset single click detection
                    if (s_click_state != CLICK_STATE_IDLE) {
                        ESP_LOGD(TAG, "[CLICK] Reset: Not in DOWN state (current=%d)", s_last_stable_position);
                        s_click_state = CLICK_STATE_IDLE;
                        s_click_initial_mag_x = 0;  // Reset initial value
                        s_click_start_time = 0;  // Reset start time
                    }
                }
                
                // 4.5. Pairing mode detection (pairing and pairing cancelled)
                // Pairing event: average drops more than MAG_PAIRING_DROP_THRESHOLD from s_calibrated_removed_center
                // Pairing cancelled event: average recovers from pairing state back to REMOVED center
                // Only valid when slider is in REMOVED position (after REMOVE_FROM_UP or REMOVE_FROM_DOWN event)
                static bool s_pairing_event_triggered = false;  // Track if pairing event was already triggered
                static int16_t s_pairing_last_average = 0;  // Last average value for pairing detection
                static uint8_t s_pairing_stable_count = 0;  // Stability counter for pairing detection
                static int16_t s_pairing_cancelled_last_average = 0;  // Last average value for pairing cancelled detection
                static uint8_t s_pairing_cancelled_stable_count = 0;  // Stability counter for pairing cancelled detection
                
                // Only detect pairing when slider is in REMOVED position
                if (s_last_stable_position == MAG_POSITION_REMOVED) {
                    // Calculate drop from REMOVED center
                    int16_t drop_from_removed = s_calibrated_removed_center - average;
                    
                    // 4.5.1. Pairing cancelled detection (check first, before pairing detection)
                    // Pairing cancelled event: average recovers from pairing state back to REMOVED center
                    // Only valid when pairing was previously triggered (s_pairing_event_triggered == true)
                    if (s_pairing_event_triggered) {
                        // Check if average has recovered back to REMOVED center (drop < MAG_PAIRING_DROP_THRESHOLD)
                        if (drop_from_removed < MAG_PAIRING_DROP_THRESHOLD) {
                            // Average has recovered, check stability
                            if (s_pairing_cancelled_last_average == 0) {
                                // First reading, initialize
                                s_pairing_cancelled_last_average = average;
                                s_pairing_cancelled_stable_count = 1;
                            } else {
                                int16_t cancelled_diff = abs(average - s_pairing_cancelled_last_average);
                                if (cancelled_diff <= 10) {  // Value is stable (within 10 units)
                                    s_pairing_cancelled_stable_count++;
                                    
                                    if (s_pairing_cancelled_stable_count >= MAG_STABLE_THRESHOLD) {
                                        // "Pairing cancelled" state is stable, trigger event
                                        ESP_LOGI(TAG, "PAIRING_CANCELLED detected (average: %d, REMOVED center: %d, drop: %d < %d)", 
                                                 average, s_calibrated_removed_center, drop_from_removed, MAG_PAIRING_DROP_THRESHOLD);
                                        
                                        // Send event notification
                                        control_serial_send_magnetic_switch_event(MAGNETIC_SLIDE_SWITCH_EVENT_PAIRING_CANCELLED);
                                        
                                        // Reset pairing state to allow new detection cycle
                                        s_pairing_event_triggered = false;
                                        
                                        // Reset counters for next detection cycle
                                        s_pairing_cancelled_last_average = 0;
                                        s_pairing_cancelled_stable_count = 0;
                                        s_pairing_last_average = 0;
                                        s_pairing_stable_count = 0;
                                    }
                                } else {
                                    // Value changed, reset
                                    s_pairing_cancelled_last_average = average;
                                    s_pairing_cancelled_stable_count = 1;
                                }
                            }
                        } else {
                            // Average is still in pairing range, reset cancelled detection
                            s_pairing_cancelled_last_average = 0;
                            s_pairing_cancelled_stable_count = 0;
                        }
                    } else {
                        // Pairing not triggered, reset cancelled detection
                        s_pairing_cancelled_last_average = 0;
                        s_pairing_cancelled_stable_count = 0;
                    }
                    
                    // 4.5.2. Pairing detection (check after cancelled detection)
                    // Check if average dropped more than threshold
                    if (drop_from_removed >= MAG_PAIRING_DROP_THRESHOLD && !s_pairing_event_triggered) {
                        // Pairing condition met, check stability
                        if (s_pairing_last_average == 0) {
                            // First reading, initialize
                            s_pairing_last_average = average;
                            s_pairing_stable_count = 1;
                        } else {
                            int16_t pairing_diff = abs(average - s_pairing_last_average);
                            if (pairing_diff <= 10) {  // Value is stable (within 10 units)
                                s_pairing_stable_count++;
                                
                                if (s_pairing_stable_count >= MAG_STABLE_THRESHOLD) {
                                    // Pairing state is stable, trigger event
                                    s_slider_state = MAGNETIC_SLIDE_SWITCH_EVENT_PAIRING;
                                    ESP_LOGI(TAG, "PAIRING mode detected (average: %d, REMOVED center: %d, drop: %d >= %d)", 
                                             average, s_calibrated_removed_center, drop_from_removed, MAG_PAIRING_DROP_THRESHOLD);
                                    
                                    // Send event notification
                                    control_serial_send_magnetic_switch_event(s_slider_state);
                                    
                                    // Mark as triggered to avoid repeated triggers
                                    s_pairing_event_triggered = true;
                                    
                                    // Reset counters for next detection cycle
                                    s_pairing_last_average = 0;
                                    s_pairing_stable_count = 0;
                                }
                            } else {
                                // Value changed, reset
                                s_pairing_last_average = average;
                                s_pairing_stable_count = 1;
                            }
                        }
                    } else if (drop_from_removed < MAG_PAIRING_DROP_THRESHOLD && !s_pairing_event_triggered) {
                        // Average is not low enough and pairing not triggered, reset detection
                        s_pairing_last_average = 0;
                        s_pairing_stable_count = 0;
                    }
                } else {
                    // Not in REMOVED position, reset pairing detection
                    if (s_pairing_event_triggered) {
                        s_pairing_event_triggered = false;
                    }
                    s_pairing_last_average = 0;
                    s_pairing_stable_count = 0;
                    s_pairing_cancelled_last_average = 0;
                    s_pairing_cancelled_stable_count = 0;
                }
                
                // 5. Fish detection (attached and detached)
                // Fish event: only trigger when fish is placed from UP position
                // When fish is placed from UP: average increases 150-200 from s_calibrated_removed_center
                // When fish is detached: average recovers back to s_calibrated_removed_center
                // Only valid after REMOVE_FROM_UP or REMOVE_FROM_DOWN event
                static bool s_fish_state_detected = false;  // Track if "the fish is attached" state was already detected
                static bool s_fish_detection_enabled = false;  // Enable detection only after remove event
                static int16_t s_fish_candidate_average = 0;
                static uint8_t s_fish_stable_count = 0;
                static int16_t s_fish_detached_candidate_average = 0;
                static uint8_t s_fish_detached_stable_count = 0;
                
                // Check if average indicates fish placed from UP position
                // Average should increase 150-200 from REMOVED center
                int16_t diff_from_removed = average - s_calibrated_removed_center;
                bool is_fish_state = (diff_from_removed >= FISH_FROM_UP_MIN_DIFF && 
                                      diff_from_removed <= FISH_FROM_UP_MAX_DIFF);
                
                // 5.1. Fish detached detection (check first, before fish attached detection)
                // Fish detached event: average recovers from fish attached state back to REMOVED center
                // Only valid when fish was previously attached (s_fish_state_detected == true)
                if (s_fish_state_detected && s_fish_detection_enabled && s_last_stable_position == MAG_POSITION_REMOVED) {
                    // Check if average has recovered back to REMOVED center (diff < FISH_FROM_UP_MIN_DIFF)
                    if (diff_from_removed < FISH_FROM_UP_MIN_DIFF) {
                        // Average has recovered, check stability
                        if (s_fish_detached_candidate_average == 0) {
                            // First reading, initialize
                            s_fish_detached_candidate_average = average;
                            s_fish_detached_stable_count = 1;
                        } else {
                            int16_t detached_diff = abs(average - s_fish_detached_candidate_average);
                            if (detached_diff <= 10) {  // Value is stable (within 10 units)
                                s_fish_detached_stable_count++;
                                
                                if (s_fish_detached_stable_count >= MAG_STABLE_THRESHOLD * 2) {
                                    // "Fish detached" state is stable, trigger event
                                    int16_t log_diff_from_removed = average - s_calibrated_removed_center;
                                    ESP_LOGI(TAG, "Fish detached detected (average: %d, REMOVED: %d, diff_from_removed: %d < %d)", 
                                             average, s_calibrated_removed_center, 
                                             log_diff_from_removed, FISH_FROM_UP_MIN_DIFF);
                                    
                                    // Send event notification
                                    control_serial_send_magnetic_switch_event(MAGNETIC_SLIDE_SWITCH_EVENT_FISH_DETACHED);
                                    
                                    // Reset fish attached state to allow new detection cycle
                                    s_fish_state_detected = false;
                                    
                                    // Reset counters for next detection cycle
                                    s_fish_detached_candidate_average = 0;
                                    s_fish_detached_stable_count = 0;
                                    s_fish_candidate_average = 0;
                                    s_fish_stable_count = 0;
                                }
                            } else {
                                // Value changed, reset
                                s_fish_detached_candidate_average = average;
                                s_fish_detached_stable_count = 1;
                            }
                        }
                    } else {
                        // Average is still in fish attached range, reset detached detection
                        s_fish_detached_candidate_average = 0;
                        s_fish_detached_stable_count = 0;
                    }
                } else {
                    // Fish not attached or detection disabled, reset detached detection
                    s_fish_detached_candidate_average = 0;
                    s_fish_detached_stable_count = 0;
                }
                
                // 5.2. Fish attached detection (check after detached detection)
                // Only detect "the fish is attached" state if:
                // 1. Detection is enabled (after remove event)
                // 2. Average is above pairing threshold (disable fish detection during pairing mode)
                // 3. Last stable position is REMOVED (slider was removed)
                // 4. Fish is not already detected (s_fish_state_detected == false)
                // When average < MAG_PAIRING_THRESHOLD, pairing mode is active, so fish detection is disabled
                if (is_fish_state && s_fish_detection_enabled && s_last_stable_position == MAG_POSITION_REMOVED && !s_fish_state_detected) {
                    // Check if value is stable (using same stability mechanism)
                    if (s_fish_candidate_average == 0) {
                        // First reading, initialize
                        s_fish_candidate_average = average;
                        s_fish_stable_count = 1;
                    } else {
                        int16_t fish_diff = abs(average - s_fish_candidate_average);
                        if (fish_diff <= 10) {  // Value is stable (within 10 units)
                            s_fish_stable_count++;
                            
                            if (s_fish_stable_count >= MAG_STABLE_THRESHOLD * 2) {
                                // "the fish is attached" state is stable, trigger event
                                // Recalculate difference for logging
                                int16_t log_diff_from_removed = average - s_calibrated_removed_center;
                                ESP_LOGI(TAG, "Fish attached (from UP position) detected (average: %d, REMOVED: %d, diff_from_removed: %d [%d-%d])", 
                                         average, s_calibrated_removed_center, 
                                         log_diff_from_removed, FISH_FROM_UP_MIN_DIFF, FISH_FROM_UP_MAX_DIFF);
                                control_serial_send_magnetic_switch_event(MAGNETIC_SLIDE_SWITCH_EVENT_FISH_ATTACHED);
                                s_fish_state_detected = true;
                                
                                // Keep detection enabled, but reset counters for next detection cycle
                                s_fish_candidate_average = 0;
                                s_fish_stable_count = 0;
                            }
                        } else {
                            // Value changed, reset
                            s_fish_candidate_average = average;
                            s_fish_stable_count = 1;
                        }
                    }
                } else if (!is_fish_state && !s_fish_state_detected) {
                    // Not in "the fish is attached" state and fish not detected, reset detection counters
                    // But keep s_fish_detection_enabled flag (only reset when detection succeeds or on new remove event)
                    s_fish_candidate_average = 0;
                    s_fish_stable_count = 0;
                }
                
                // Enable/disable fish detection based on remove/place events
                // This is handled in the event detection section above
                // Update fish detection enabled flag based on last event
                static magnetic_slide_switch_event_t s_last_event = MAGNETIC_SLIDE_SWITCH_EVENT_INIT;
                if (s_slider_state != s_last_event && s_slider_state != MAGNETIC_SLIDE_SWITCH_EVENT_INIT) {
                    if (s_slider_state == MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_UP || 
                        s_slider_state == MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_DOWN) {
                        s_fish_detection_enabled = true;
                        s_fish_state_detected = false;  // Reset detection flag to allow new detection
                        s_fish_candidate_average = 0;
                        s_fish_stable_count = 0;
                        ESP_LOGI(TAG, "Fish detection enabled after remove event");
                    }
                    else if (s_slider_state == MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_UP || 
                             s_slider_state == MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_DOWN) {
                        s_fish_detection_enabled = false;
                        s_fish_state_detected = false;
                        s_fish_candidate_average = 0;
                        s_fish_stable_count = 0;
                        ESP_LOGI(TAG, "Fish detection disabled after place event");
                    }
                    s_last_event = s_slider_state;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
    }
    
    ESP_LOGI(TAG, "Slide switch event detection task finished.");
    vTaskDelete(NULL);
}

/**
 * @brief Rotary knob event detection task
 * 
 * @param arg Task parameter (unused)
 * 
 * @note This task monitors y-axis magnetometer data to detect rotary knob events
 * @note Uses sliding window filter to smooth y-axis data
 */
static void rotary_knob_event_detect_task(void *arg)
{
    ESP_LOGI(TAG, "Rotary knob event detection task started");
    
    // Wait for magnetometer data read task to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Sliding window buffer for y-axis data
    static int16_t s_knob_window[MAG_WINDOW_SIZE] = {0};
    static uint8_t s_knob_window_index = 0;
    static uint8_t s_knob_window_filled = 0;  // Window fill count
    
    // Sliding window buffer for z-axis data (to detect slide switch movement)
    static int16_t s_z_axis_window[MAG_WINDOW_SIZE] = {0};
    static uint8_t s_z_axis_window_index = 0;
    static uint8_t s_z_axis_window_filled = 0;  // Window fill count
    
    // Rotary knob state tracking variables
    static int16_t s_last_knob_average = 0;
    static int16_t s_last_z_axis_average = 0;
    static bool s_first_knob_reading = true;
    
    // Stability detection variables (to filter intermediate values during rotation)
    static int16_t s_candidate_y_average = 0;  // Candidate y-axis average value
    static int8_t s_candidate_direction = 0;   // Candidate rotation direction: 1=right, -1=left, 0=unknown
    static uint8_t s_stable_count = 0;         // Stability counter
    
    // Rotation detection threshold (adjust based on actual sensitivity)
    const int16_t ROTATION_THRESHOLD = 20;  // Minimum change to detect rotation
    const int16_t Z_AXIS_CHANGE_THRESHOLD = 20;  // Maximum z-axis change to consider rotation valid
    const uint8_t ROTATION_STABLE_THRESHOLD = 5;  // Consecutive stable readings required to confirm rotation
    
    while (1) {
        // Read y-axis data from shared structure
        int16_t mag_x, mag_y, mag_z;
        bool data_valid = get_magnetometer_data(&mag_x, &mag_y, &mag_z);
        
        if (!data_valid) {
            // Data not available yet, skip this iteration
            vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
            continue;
        }
        
        // Use y-axis data for rotary knob detection (keep original sign for direction detection)
        int16_t s_knob_value = mag_y;
        // Use z-axis data to detect slide switch movement (which invalidates rotation)
        int16_t s_z_axis_value = mag_z;
        
        // Process z-axis absolute value for comparison
        if (s_z_axis_value < 0) {
            s_z_axis_value = -s_z_axis_value;
        }
        
        // 1. Update sliding window for y-axis
        s_knob_window[s_knob_window_index] = s_knob_value;
        s_knob_window_index = (s_knob_window_index + 1) % MAG_WINDOW_SIZE;
        if (s_knob_window_filled < MAG_WINDOW_SIZE) {
            s_knob_window_filled++;
        }
        
        // 2. Update sliding window for z-axis
        s_z_axis_window[s_z_axis_window_index] = s_z_axis_value;
        s_z_axis_window_index = (s_z_axis_window_index + 1) % MAG_WINDOW_SIZE;
        if (s_z_axis_window_filled < MAG_WINDOW_SIZE) {
            s_z_axis_window_filled++;
        }
        
        // 3. Calculate sliding window averages (start checking after windows are filled)
        if (s_knob_window_filled >= MAG_WINDOW_SIZE && s_z_axis_window_filled >= MAG_WINDOW_SIZE) {
            // First, calculate z-axis average and check if slide switch is stable
            int32_t z_sum = 0;
            for (uint8_t i = 0; i < MAG_WINDOW_SIZE; i++) {
                z_sum += s_z_axis_window[i];
            }
            int16_t z_average = z_sum / MAG_WINDOW_SIZE;
            
            // Check z-axis change first (only proceed with y-axis detection if z-axis is stable)
            if (!s_first_knob_reading) {
                int16_t z_change = abs(z_average - s_last_z_axis_average);
                
                // Only proceed with y-axis rotation detection if z-axis change is within threshold
                if (z_change <= Z_AXIS_CHANGE_THRESHOLD) {
                    // Z-axis is stable, now check y-axis for rotation
                    int32_t y_sum = 0;
                    for (uint8_t i = 0; i < MAG_WINDOW_SIZE; i++) {
                        y_sum += s_knob_window[i];
                    }
                    int16_t y_average = y_sum / MAG_WINDOW_SIZE;
                    int16_t y_change = y_average - s_last_knob_average;
                    
                    // Check if y-axis change exceeds threshold (potential rotation detected)
                    if (abs(y_change) > ROTATION_THRESHOLD) {
                        // Determine rotation direction
                        int8_t current_direction = (y_change > 0) ? 1 : -1;  // 1=right, -1=left
                        
                        // Check if this matches the candidate direction
                        if (s_candidate_direction == current_direction) {
                            // Same direction, check if value is stable (within small range)
                            int16_t candidate_diff = abs(y_average - s_candidate_y_average);
                            if (candidate_diff <= 10) {  // Value is stable (within 10 units)
                                s_stable_count++;
                                
                                // Check if stable enough to confirm rotation
                                if (s_stable_count >= ROTATION_STABLE_THRESHOLD) {
                                    // Rotation event is confirmed (z-axis was already confirmed stable)
                                    if (current_direction > 0) {
                                        // Y-axis data increased -> Right rotation (clockwise)
                                        ESP_LOGI(TAG, "Rotary knob: RIGHT ROTATION (y-axis: %d -> %d, change: +%d, z-axis change: %d)", 
                                                 s_last_knob_average, y_average, y_change, z_change);
                                    } else {
                                        // Y-axis data decreased -> Left rotation (counter-clockwise)
                                        ESP_LOGI(TAG, "Rotary knob: LEFT ROTATION (y-axis: %d -> %d, change: %d, z-axis change: %d)", 
                                                 s_last_knob_average, y_average, y_change, z_change);
                                    }
                                    
                                    // Update y-axis average and reset stability tracking
                                    s_last_knob_average = y_average;
                                    s_candidate_direction = 0;
                                    s_stable_count = 0;
                                    s_candidate_y_average = 0;
                                }
                            } else {
                                // Value changed significantly, reset stability counter but keep direction
                                s_stable_count = 1;
                                s_candidate_y_average = y_average;
                            }
                        } else {
                            // New direction detected, reset candidate
                            s_candidate_direction = current_direction;
                            s_candidate_y_average = y_average;
                            s_stable_count = 1;
                        }
                    } else {
                        // Y-axis change is too small, reset stability tracking
                        s_candidate_direction = 0;
                        s_stable_count = 0;
                        s_candidate_y_average = 0;
                    }
                } else {
                    // Z-axis changed too much, skip y-axis detection and reset stability tracking
                    s_candidate_direction = 0;
                    s_stable_count = 0;
                    s_candidate_y_average = 0;
                    ESP_LOGD(TAG, "Rotary knob: Skipped y-axis detection (z-axis change: %d > %d, slide switch moving)", 
                             z_change, Z_AXIS_CHANGE_THRESHOLD);
                }
                
                // Always update z-axis average
                s_last_z_axis_average = z_average;
            } else {
                // First reading, initialize both axes
                int32_t y_sum = 0;
                for (uint8_t i = 0; i < MAG_WINDOW_SIZE; i++) {
                    y_sum += s_knob_window[i];
                }
                int16_t y_average = y_sum / MAG_WINDOW_SIZE;
                
                s_first_knob_reading = false;
                s_last_knob_average = y_average;
                s_last_z_axis_average = z_average;
                ESP_LOGI(TAG, "Rotary knob initialized, initial y-axis: %d, z-axis: %d", y_average, z_average);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(MAG_SAMPLE_PERIOD_MS));
    }
    
    ESP_LOGI(TAG, "Rotary knob event detection task finished.");
    vTaskDelete(NULL);
}

#endif  // CONFIG_SENSOR_MAGNETOMETER

/* ========== Public Function Implementations ========== */

/**
 * @brief Start magnetic slide switch task
 * 
 * @note Creates Hall sensor or magnetometer tasks based on configuration
 * @note For magnetometer: 
 *       1. Creates data read task
 *       2. Creates calibration task and waits for completion
 *       3. Creates slide switch detection task after calibration completes
 *       4. Optionally creates rotary knob detection task
 */
void magnetic_slide_switch_start(void)
{
#ifdef CONFIG_SENSOR_LINEAR_HALL
    xTaskCreate(hall_sensor_read_task, "hall_sensor_read_task", MAGNETIC_SENSOR_TASK_STACK_SIZE, NULL, 2, NULL);
#elif defined(CONFIG_SENSOR_MAGNETOMETER)
    // Create magnetometer data read task (shared data provider)
    xTaskCreate(magnetometer_data_read_task, "mag_data_read_task", MAGNETIC_SENSOR_TASK_STACK_SIZE, NULL, 3, NULL);
    
    // Create semaphore for calibration completion synchronization
    s_calibration_complete_sem = xSemaphoreCreateBinary();
    if (s_calibration_complete_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create calibration completion semaphore");
        return;
    }
    
    // Create calibration task (will signal completion via semaphore)
    xTaskCreate(magnetometer_calibration_task, "mag_calibration_task", MAGNETIC_SENSOR_TASK_STACK_SIZE, NULL, 3, NULL);
    
    // Wait for calibration to complete (blocking wait)
    if (xSemaphoreTake(s_calibration_complete_sem, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "Calibration completed, starting slide switch detection task");
        
        // Create slide switch event detection task (uses z-axis data)
        xTaskCreate(slide_switch_event_detect_task, "slide_switch_detect_task", MAGNETIC_SENSOR_TASK_STACK_SIZE, NULL, 2, NULL);
        
        // Create rotary knob event detection task (uses y-axis data)
        // xTaskCreate(rotary_knob_event_detect_task, "rotary_knob_detect_task", MAGNETIC_SENSOR_TASK_STACK_SIZE, NULL, 2, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to wait for calibration completion");
    }
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
