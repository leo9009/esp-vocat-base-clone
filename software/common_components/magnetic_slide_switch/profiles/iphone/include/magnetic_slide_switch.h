/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sdkconfig.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== General Task Configuration ========== */
#define MAGNETIC_SENSOR_TASK_STACK_SIZE     (1024 * 3)  /**< Task stack size */

/* ========== Conditional Compilation by Sensor Type ========== */

#ifdef CONFIG_SENSOR_LINEAR_HALL
/* ---------- Linear Hall Sensor Configuration ---------- */
#define HALL_SENSOR_ADC_CHANNEL             (ADC_CHANNEL_3)         /**< ADC channel */
#define HALL_SENSOR_ADC_ATTEN               ADC_ATTEN_DB_12         /**< ADC attenuation */
#define HALL_SENSOR_VOLTAGE_THRESHOLD       (25)                    /**< Voltage change threshold (mV) */
#define HALL_SENSOR_INITIAL_STATE_THRESHOLD (1400)                  /**< Initial state determination threshold (mV) */
#define HALL_SENSOR_SAMPLE_PERIOD_MS        (50)                    /**< Sampling period (ms) */

#elif defined(CONFIG_SENSOR_MAGNETOMETER)
/* ---------- Geomagnetic Sensor I2C Configuration ---------- */
#define I2C_MASTER_SCL_IO   (GPIO_NUM_3)            /**< I2C SCL pin */
#define I2C_MASTER_SDA_IO   (GPIO_NUM_2)            /**< I2C SDA pin */
#define I2C_MASTER_NUM      I2C_NUM_0               /**< I2C port number */
#define I2C_MASTER_FREQ_HZ  400000                  /**< I2C clock frequency (Hz) */

#ifdef CONFIG_SENSOR_BMM150
    /* BMM150 magnetometer specific configuration */
    #define MAGNETOMETER_I2C_ADDR           (0x10)      /**< BMM150 I2C address */
    #define MAGNETOMETER_CHIP_ID            (0x32)      /**< BMM150 chip ID */

    /* Sliding window configuration */
    #define MAG_WINDOW_SIZE                     (10)         /**< Sliding window size */
    #define MAG_SAMPLE_PERIOD_MS                (5)        /**< Sampling period (ms) - smaller for faster response */
    #define MAG_STABLE_THRESHOLD                (5)         /**< Consecutive N same states to be considered stable */

    /* State threshold configuration (used by calibration only) */
    #define MAG_STATE_REMOVED_CENTER        (709)          /**< REMOVED state center value */
    #define MAG_STATE_REMOVED_OFFSET        (50)           /**< REMOVED state offset */
    #define MAG_STATE_UP_CENTER             (1383)          /**< UP state center value */
    #define MAG_STATE_UP_OFFSET             (50)           /**< UP state offset */
    #define MAG_STATE_DOWN_CENTER           (2047)          /**< DOWN state center value */
    #define MAG_STATE_DOWN_OFFSET           (50)           /**< DOWN state offset */

    /* iPhone detection: Z increases from magnetic_accessory_detection baseline when phone approaches */
    #define MAGNETIC_ACCESSORY_DETECTION_Z_INCREASE  (450)  /**< Z increase threshold from baseline; above this = phone lean front */
    #define MAGNETIC_ACCESSORY_DETECTION_Z_DROP_UNDER_BASE (600)  /**< Z drop threshold from baseline; above this = phone under base */

    /* Automatic calibration configuration */
    #define CALIBRATION_STABILITY_TIME_MS    (500)          /**< Time (ms) for value to be stable before recording calibration point */
    #define CALIBRATION_VALUE_DIFF_THRESHOLD (100)          /**< Minimum difference between calibration values to consider them different */
    
#elif defined(CONFIG_SENSOR_QMC6309)
    /* QMC6309 magnetometer specific configuration */
    #define MAGNETOMETER_I2C_ADDR           (0x7C)      /**< QMC6309 I2C address */
    #define MAGNETOMETER_CHIP_ID            (0x90)      /**< QMC6309 chip ID */

    /* Sliding window configuration */
    #define MAG_WINDOW_SIZE                     (5)         /**< Sliding window size */
    #define MAG_SAMPLE_PERIOD_MS                (2)         /**< Sampling period (ms) - smaller for faster response */
    #define MAG_STABLE_THRESHOLD                (10)        /**< Consecutive N same states to be considered stable */

    /* State threshold configuration (used by calibration only) */
    #define MAG_STATE_REMOVED_CENTER        (1096)          /**< REMOVED state center value */
    #define MAG_STATE_REMOVED_OFFSET        (100)           /**< REMOVED state offset */
    #define MAG_STATE_UP_CENTER             (1275)          /**< UP state center value */
    #define MAG_STATE_UP_OFFSET             (100)           /**< UP state offset */
    #define MAG_STATE_DOWN_CENTER           (2598)          /**< DOWN state center value */
    #define MAG_STATE_DOWN_OFFSET           (100)           /**< DOWN state offset */

    /* iPhone detection: Z increases from magnetic_accessory_detection baseline when phone approaches */
    #define MAGNETIC_ACCESSORY_DETECTION_Z_INCREASE  (450)  /**< Z increase threshold from baseline; above this = phone lean front */
    #define MAGNETIC_ACCESSORY_DETECTION_Z_DROP_UNDER_BASE (800)  /**< Z drop threshold from baseline; above this = phone under base */

    /* Automatic calibration configuration */
    #define CALIBRATION_STABILITY_TIME_MS    (500)         /**< Time (ms) for value to be stable before recording calibration point */
    #define CALIBRATION_VALUE_DIFF_THRESHOLD (100)          /**< Minimum difference between calibration values to consider them different */
    
#endif  // CONFIG_SENSOR_BMM150 / CONFIG_SENSOR_QMC6309

#endif  // CONFIG_SENSOR_LINEAR_HALL / CONFIG_SENSOR_MAGNETOMETER

/* ========== Magnetic Slide Switch Event Types ========== */
/**
 * @brief Magnetic slide switch event enumeration
 */
typedef enum {
    MAGNETIC_SLIDE_SWITCH_EVENT_INIT = 0,                       /**< Power-on initial state, no action determined */
    MAGNETIC_SLIDE_SWITCH_EVENT_IPHONE_LEAN_FRONT = 16,          /**< 手机从底座正面倚靠上去 (Z increase >= threshold) */
    MAGNETIC_SLIDE_SWITCH_EVENT_IPHONE_LEAN_FRONT_DETACHED = 17, /**< 手机从正面离开 */
    MAGNETIC_SLIDE_SWITCH_EVENT_IPHONE_UNDER_BASE = 18,          /**< 手机被压在底座底下 (Z < 0) */
    MAGNETIC_SLIDE_SWITCH_EVENT_IPHONE_UNDER_BASE_DETACHED = 19, /**< 手机从底座下离开 */
} magnetic_slide_switch_event_t;

/* ========== Callback Function Type ========== */
/**
 * @brief Event callback function type
 * 
 * @param event Detected event type
 */
typedef void (*magnetic_slide_switch_event_callback_t)(magnetic_slide_switch_event_t event);

/* ========== Function Declarations ========== */

/**
 * @brief Start magnetic slide switch task
 * 
 * @note Automatically selects Hall sensor or geomagnetic sensor based on configuration
 * @note After starting, will automatically calibrate (if no saved calibration data exists)
 */
void magnetic_slide_switch_start(void);

/**
 * @brief Get current magnetic slide switch event
 * 
 * @return magnetic_slide_switch_event_t Current event type
 */
magnetic_slide_switch_event_t magnetic_slide_switch_get_event(void);

/**
 * @brief Register event callback function
 * 
 * @param callback Callback function pointer
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if callback is NULL, ESP_ERR_NO_MEM if out of memory
 * 
 * @note Multiple callbacks can be registered, all will be called when event occurs
 */
esp_err_t magnetic_slide_switch_register_callback(magnetic_slide_switch_event_callback_t callback);

/**
 * @brief Unregister event callback function
 * 
 * @param callback Callback function pointer to unregister
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if callback not found
 */
esp_err_t magnetic_slide_switch_unregister_callback(magnetic_slide_switch_event_callback_t callback);

/**
 * @brief Trigger recalibration (no restart needed, takes effect immediately)
 * 
 * @note This function sets recalibration flag, sensor task will automatically re-enter calibration mode in next loop
 * @note After calibration completes, will overwrite previously saved calibration data
 */
void magnetic_slide_switch_start_recalibration(void);

#ifdef __cplusplus
}
#endif
