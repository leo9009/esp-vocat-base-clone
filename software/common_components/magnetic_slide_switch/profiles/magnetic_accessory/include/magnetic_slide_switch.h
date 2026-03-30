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

    /* State threshold configuration: center value ± offset */
    #define MAG_STATE_REMOVED_CENTER        (709)          /**< REMOVED state center value */
    #define MAG_STATE_REMOVED_OFFSET        (50)           /**< REMOVED state offset */
    
    #define MAG_STATE_UP_CENTER             (1383)          /**< UP state center value */
    #define MAG_STATE_UP_OFFSET             (50)           /**< UP state offset */
    
    #define MAG_STATE_DOWN_CENTER           (2047)          /**< DOWN state center value */
    #define MAG_STATE_DOWN_OFFSET           (50)           /**< DOWN state offset */
    
    /* Pairing mode detection configuration */
    #define MAG_PAIRING_DROP_THRESHOLD      (100)           /**< Pairing mode drop threshold: when average drops more than this from REMOVED center */
    
    /* Fish event detection (Z-axis only): Z increases 120~160 from REMOVED center = fish attached; Z back to REMOVED = fish detached */
    #define FISH_ATTACHED_Z_MIN             (50)           /**< Z increase from REMOVED center, min for fish attached */
    #define FISH_ATTACHED_Z_MAX             (130)           /**< Z increase from REMOVED center, max for fish attached */
    #define FISH_RISE_RESET_THRESHOLD       (4)             /**< If Z rises by more than this in fish range, treat as passing through to ice cream, reset fish candidate */
    /* Ice cream event (吃雪糕): Z increases 205~245 from REMOVED center = ice cream attached; Z back to REMOVED = ice cream detached */
    #define ICE_CREAM_ATTACHED_Z_MIN        (140)           /**< Z increase from REMOVED center, min for ice cream attached (225-20) */
    #define ICE_CREAM_ATTACHED_Z_MAX        (200)           /**< Z increase from REMOVED center, max for ice cream attached (225+20) */
    #define ICE_CREAM_RISE_RESET_THRESHOLD  (4)             /**< If Z rises by more than this in ice cream range, treat as passing through to donut, reset ice cream candidate */
    /* Donut event (吃甜甜圈): Z increases 314~334 from REMOVED center = donut attached; Z back to REMOVED = donut detached */
    #define DONUT_ATTACHED_Z_MIN            (270)           /**< Z increase from REMOVED center, min for donut attached (324-10) */
    #define DONUT_ATTACHED_Z_MAX            (310)           /**< Z increase from REMOVED center, max for donut attached (324+10) */
    /** 喂鱼/雪糕 延迟确认时间(ms)：先触发的候选需等待此时间，若无雪糕/甜甜圈则确认 */
    #define ACCESSORY_CONFIRM_MS            (200)
    /** 动态基线 EMA：无配件时基线跟随当前 average，1/8 平滑，使 diff_z 随环境自适应 */
    #define DYNAMIC_BASELINE_EMA_SHIFT      (3)
    /** 基线锁定：diff_z 超过此阈值视为“变化大”，锁定基线不再更新，防止吸上配件后基线被拉高导致 diff_z 回落为 0 */
    #define DIFF_Z_BASELINE_LOCK_THRESHOLD   (15)           /**< diff_z > 此值则锁定基线（宜小于鱼阈值，避免上升过程中基线被跟高） */
    #define DIFF_Z_BASELINE_UNLOCK_THRESHOLD (15)           /**< diff_z < 此值则解锁基线 */
    
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

    /* State threshold configuration: center value ± offset */
    #define MAG_STATE_REMOVED_CENTER        (1096)          /**< REMOVED state center value */
    #define MAG_STATE_REMOVED_OFFSET        (100)           /**< REMOVED state offset */
    
    #define MAG_STATE_UP_CENTER             (1275)          /**< UP state center value */
    #define MAG_STATE_UP_OFFSET             (100)           /**< UP state offset */
    
    #define MAG_STATE_DOWN_CENTER           (2598)          /**< DOWN state center value */
    #define MAG_STATE_DOWN_OFFSET           (100)           /**< DOWN state offset */
    
    /* Pairing mode detection configuration */
    #define MAG_PAIRING_DROP_THRESHOLD      (150)           /**< Pairing mode drop threshold: when average drops more than this from REMOVED center */
    
    /* Fish event detection (Z-axis only): Z increases 120~160 from REMOVED center = fish attached; Z back to REMOVED = fish detached */
    #define FISH_ATTACHED_Z_MIN             (120)           /**< Z increase from REMOVED center, min for fish attached */
    #define FISH_ATTACHED_Z_MAX             (160)           /**< Z increase from REMOVED center, max for fish attached */
    #define FISH_RISE_RESET_THRESHOLD       (8)             /**< If Z rises by more than this in fish range, treat as passing through to ice cream, reset fish candidate */
    /* Ice cream event (吃雪糕): Z increases 205~245 from REMOVED center = ice cream attached; Z back to REMOVED = ice cream detached */
    #define ICE_CREAM_ATTACHED_Z_MIN        (205)           /**< Z increase from REMOVED center, min for ice cream attached (225-20) */
    #define ICE_CREAM_ATTACHED_Z_MAX        (245)           /**< Z increase from REMOVED center, max for ice cream attached (225+20) */
    #define ICE_CREAM_RISE_RESET_THRESHOLD  (8)             /**< If Z rises by more than this in ice cream range, treat as passing through to donut, reset ice cream candidate */
    /* Donut event (吃甜甜圈): Z increases 314~334 from REMOVED center = donut attached; Z back to REMOVED = donut detached */
    #define DONUT_ATTACHED_Z_MIN            (314)           /**< Z increase from REMOVED center, min for donut attached (324-10) */
    #define DONUT_ATTACHED_Z_MAX            (334)           /**< Z increase from REMOVED center, max for donut attached (324+10) */
    /** 喂鱼/雪糕 延迟确认时间(ms)：先触发的候选需等待此时间，若无雪糕/甜甜圈则确认 */
    #define ACCESSORY_CONFIRM_MS            (200)
    /** 动态基线 EMA：无配件时基线跟随当前 average，1/8 平滑 */
    #define DYNAMIC_BASELINE_EMA_SHIFT      (3)
    /** 基线锁定：diff_z 超过此阈值时锁定基线，低于解锁阈值时解锁 */
    #define DIFF_Z_BASELINE_LOCK_THRESHOLD   (25)
    #define DIFF_Z_BASELINE_UNLOCK_THRESHOLD (45)
    
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
    MAGNETIC_SLIDE_SWITCH_EVENT_INIT = 0,                   /**< Power-on initial state, no action determined */
    MAGNETIC_SLIDE_SWITCH_EVENT_FISH_ATTACHED = 8,          /**< Fish attached event */
    MAGNETIC_SLIDE_SWITCH_EVENT_FISH_DETACHED = 9,          /**< Fish detached event */
    MAGNETIC_SLIDE_SWITCH_EVENT_PAIRING = 10,               /**< Pairing mode event */
    MAGNETIC_SLIDE_SWITCH_EVENT_PAIRING_CANCELLED = 11,     /**< Pairing cancelled event */
    MAGNETIC_SLIDE_SWITCH_EVENT_ICE_CREAM_ATTACHED = 12,    /**< Ice cream attached (吃雪糕) */
    MAGNETIC_SLIDE_SWITCH_EVENT_ICE_CREAM_DETACHED = 13,   /**< Ice cream detached (不吃雪糕) */
    MAGNETIC_SLIDE_SWITCH_EVENT_DONUT_ATTACHED = 14,       /**< Donut attached (吃甜甜圈) */
    MAGNETIC_SLIDE_SWITCH_EVENT_DONUT_DETACHED = 15,       /**< Donut detached (不吃甜甜圈) */
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
