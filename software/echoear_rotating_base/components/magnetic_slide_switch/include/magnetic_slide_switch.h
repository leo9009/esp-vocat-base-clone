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
#define MAGNETIC_SLIDE_SWITCH_TASK_STACK_SIZE    (1024 * 3)  /**< Task stack size */

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
    #define MAG_WINDOW_SIZE                     (5)         /**< Sliding window size */
    #define MAG_SAMPLE_PERIOD_MS                (5)         /**< Sampling period (ms) - smaller for faster response */
    #define MAG_STABLE_THRESHOLD                (5)         /**< Consecutive N same states to be considered stable */
    
    /* Single click detection configuration */
    #define MAG_CLICK_TRANSITION_MAX_COUNT      (50)        /**< Maximum click drop duration count (30×5ms=150ms) */
    #define MAG_CLICK_DROP_THRESHOLD            (30)        /**< Click drop threshold (magnetic field value change) */

    /* State threshold configuration: center value ± offset */
    #define MAG_STATE_REMOVED_CENTER        (1230)          /**< REMOVED state center value */
    #define MAG_STATE_REMOVED_OFFSET        (100)           /**< REMOVED state offset */
    #define MAG_STATE_REMOVED_MIN           (MAG_STATE_REMOVED_CENTER - MAG_STATE_REMOVED_OFFSET)
    #define MAG_STATE_REMOVED_MAX           (MAG_STATE_REMOVED_CENTER + MAG_STATE_REMOVED_OFFSET)
    
    #define MAG_STATE_UP_CENTER             (1619)          /**< UP state center value */
    #define MAG_STATE_UP_OFFSET             (100)           /**< UP state offset */
    #define MAG_STATE_UP_MIN                (MAG_STATE_UP_CENTER - MAG_STATE_UP_OFFSET)
    #define MAG_STATE_UP_MAX                (MAG_STATE_UP_CENTER + MAG_STATE_UP_OFFSET)
    
    #define MAG_STATE_DOWN_CENTER           (1894)          /**< DOWN state center value */
    #define MAG_STATE_DOWN_OFFSET           (100)           /**< DOWN state offset */
    #define MAG_STATE_DOWN_MIN              (MAG_STATE_DOWN_CENTER - MAG_STATE_DOWN_OFFSET)
    #define MAG_STATE_DOWN_MAX              (MAG_STATE_DOWN_CENTER + MAG_STATE_DOWN_OFFSET)
    
#elif defined(CONFIG_SENSOR_QMC6309)
    /* QMC6309 magnetometer specific configuration */
    #define MAGNETOMETER_I2C_ADDR           (0x7C)      /**< QMC6309 I2C address */
    #define MAGNETOMETER_CHIP_ID            (0x90)      /**< QMC6309 chip ID */

    /* Sliding window configuration */
    #define MAG_WINDOW_SIZE                     (5)         /**< Sliding window size */
    #define MAG_SAMPLE_PERIOD_MS                (2)         /**< Sampling period (ms) - smaller for faster response */
    #define MAG_STABLE_THRESHOLD                (10)        /**< Consecutive N same states to be considered stable */
    
    /* Single click detection configuration */
    #define MAG_CLICK_TRANSITION_MAX_COUNT      (20)        /**< Maximum click drop duration count (20×2ms=40ms) */
    #define MAG_CLICK_DROP_THRESHOLD            (60)        /**< Click drop threshold (magnetic field value change) */

    /* State threshold configuration: center value ± offset */
    #define MAG_STATE_REMOVED_CENTER        (1096)          /**< REMOVED state center value */
    #define MAG_STATE_REMOVED_OFFSET        (100)           /**< REMOVED state offset */
    #define MAG_STATE_REMOVED_MIN           (MAG_STATE_REMOVED_CENTER - MAG_STATE_REMOVED_OFFSET)
    #define MAG_STATE_REMOVED_MAX           (MAG_STATE_REMOVED_CENTER + MAG_STATE_REMOVED_OFFSET)
    
    #define MAG_STATE_UP_CENTER             (1422)          /**< UP state center value */
    #define MAG_STATE_UP_OFFSET             (100)           /**< UP state offset */
    #define MAG_STATE_UP_MIN                (MAG_STATE_UP_CENTER - MAG_STATE_UP_OFFSET)
    #define MAG_STATE_UP_MAX                (MAG_STATE_UP_CENTER + MAG_STATE_UP_OFFSET)
    
    #define MAG_STATE_DOWN_CENTER           (1813)          /**< DOWN state center value */
    #define MAG_STATE_DOWN_OFFSET           (100)           /**< DOWN state offset */
    #define MAG_STATE_DOWN_MIN              (MAG_STATE_DOWN_CENTER - MAG_STATE_DOWN_OFFSET)
    #define MAG_STATE_DOWN_MAX              (MAG_STATE_DOWN_CENTER + MAG_STATE_DOWN_OFFSET)

#endif  // CONFIG_SENSOR_BMM150 / CONFIG_SENSOR_QMC6309

#endif  // CONFIG_SENSOR_LINEAR_HALL / CONFIG_SENSOR_MAGNETOMETER

/* ========== Magnetic Slide Switch Event Types ========== */
/**
 * @brief Magnetic slide switch event enumeration
 */
typedef enum {
    MAGNETIC_SLIDE_SWITCH_EVENT_INIT = 0,               /**< Power-on initial state, no action determined */
    MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_DOWN = 1,         /**< Slider moved from up to down */
    MAGNETIC_SLIDE_SWITCH_EVENT_SLIDE_UP = 2,           /**< Slider moved from down to up */
    MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_UP = 3,     /**< Slider removed from up position */
    MAGNETIC_SLIDE_SWITCH_EVENT_REMOVE_FROM_DOWN = 4,   /**< Slider removed from down position */
    MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_UP = 5,      /**< Slider placed from up position */
    MAGNETIC_SLIDE_SWITCH_EVENT_PLACE_FROM_DOWN = 6,    /**< Slider placed from down position */
    MAGNETIC_SLIDE_SWITCH_EVENT_SINGLE_CLICK = 7,       /**< Single click event */
} magnetic_slide_switch_event_t;

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
 * @brief Trigger recalibration (no restart needed, takes effect immediately)
 * 
 * @note This function sets recalibration flag, sensor task will automatically re-enter calibration mode in next loop
 * @note After calibration completes, will overwrite previously saved calibration data
 */
void magnetic_slide_switch_start_recalibration(void);

#ifdef __cplusplus
}
#endif
