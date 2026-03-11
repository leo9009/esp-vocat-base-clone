/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_app_format.h"
#include "esp_ota_ops.h"
#include "iot_button.h"
#include "nvs_flash.h"
#include "control_serial.h"
#include "stepper_motor.h"
#include "magnetic_slide_switch.h"

static const char *TAG = "VoCat Rotating Base";

#define BASE_ANGLE_LIMIT_SWITCH_GPIO (GPIO_NUM_1)
#define BOOT_BUTTON_GPIO             (GPIO_NUM_9)
#define BASE_CALIBRATION_TASK_STACK_SIZE    (1024 * 3)

static SemaphoreHandle_t s_limit_switch_semaphore = NULL;

/**
 * @brief Base angle limit switch event callback function
 * 
 * @param arg Button handle
 * @param data Button event data
 */
static void base_angle_limit_switch_event_cb(void *arg, void *data)
{
    button_event_t event = iot_button_get_event(arg);
    if (event == BUTTON_PRESS_DOWN) {
        // Release semaphore to notify calibration task
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_limit_switch_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Boot button event callback function
 * 
 * @param arg Button handle
 * @param data Button event data
 */
static void boot_button_event_cb(void *arg, void *data)
{
    button_event_t event = iot_button_get_event(arg);
    if (event == BUTTON_LONG_PRESS_START) {
        ESP_LOGI(TAG, "Boot button long press detected, starting recalibration...");
        magnetic_slide_switch_start_recalibration();
    }
}

/**
 * @brief Initialize base angle limit switch
 */
static void base_angle_limit_switch_init(void)
{
    button_handle_t btn = NULL;

    button_config_t cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = BASE_ANGLE_LIMIT_SWITCH_GPIO,
            .active_level = 0,
        },
    };
    btn = iot_button_create(&cfg);
    assert(btn != NULL);
    iot_button_register_cb(btn, BUTTON_PRESS_DOWN, base_angle_limit_switch_event_cb, NULL);
    iot_button_register_cb(btn, BUTTON_PRESS_UP, base_angle_limit_switch_event_cb, NULL);
}

/**
 * @brief Initialize boot button
 */
static void boot_button_init(void)
{
    button_handle_t boot_btn = NULL;

    button_config_t boot_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,  // Use default long press time (usually 5000ms)
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = BOOT_BUTTON_GPIO,
            .active_level = 0,  // Active low
        },
    };
    boot_btn = iot_button_create(&boot_cfg);
    assert(boot_btn != NULL);
    iot_button_register_cb(boot_btn, BUTTON_LONG_PRESS_START, boot_button_event_cb, NULL);
    
    ESP_LOGI(TAG, "Boot button initialized on GPIO%d (long press to recalibrate)", BOOT_BUTTON_GPIO);
}

/**
 * @brief Base calibration task
 * 
 * @param arg Task parameter (unused)
 * 
 * @note Rotates motor until limit switch is pressed, then moves to home position
 * @note If limit switch is not pressed within 2 seconds (mechanical fault protection),
 *       directly rotates 95 degrees to the right to reach home position
 */
static void base_calibration_task(void *arg)
{
    ESP_LOGI(TAG, "Base calibration task started");
    base_angle_limit_switch_init();

    // Record start time for timeout detection
    TickType_t start_time = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(2000);  // 2 seconds timeout

    while (1) {
        // Check if timeout has occurred
        TickType_t elapsed_time = xTaskGetTickCount() - start_time;
        if (elapsed_time >= timeout_ticks) {
            ESP_LOGW(TAG, "Calibration timeout! Limit switch not pressed within 2 seconds");
            ESP_LOGW(TAG, "Assuming mechanical fault, moving directly to home position");
            vTaskDelay(pdMS_TO_TICKS(200));
            stepper_rotate_angle_with_accel(95.0, STEPPER_SPEED_FAST);
            stepper_motor_power_off();
            ESP_LOGI(TAG, "Base calibration completed (timeout fallback mode)");
            vTaskDelete(NULL);
        }

        // Try to take semaphore with no wait
        if (xSemaphoreTake(s_limit_switch_semaphore, 0) == pdTRUE) {
            // Limit switch pressed, move to home position
            ESP_LOGI(TAG, "Limit switch pressed, moving to home position");
            vTaskDelay(pdMS_TO_TICKS(200));
            stepper_rotate_angle_with_accel(95.0, STEPPER_SPEED_FAST);
            stepper_motor_power_off();
            ESP_LOGI(TAG, "Base calibration completed");
            vTaskDelete(NULL);
        } else {
            // Continue rotating
            stepper_rotate_angle_with_accel(-5.0, STEPPER_SPEED_FAST);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

void app_main(void)
{
    // Initialize NVS (for saving calibration data)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition is full or version mismatch, erase and reinitialize
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    
    // Create binary semaphore for limit switch notification
    s_limit_switch_semaphore = xSemaphoreCreateBinary();
    if (s_limit_switch_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create limit switch semaphore");
        return;
    }
    
    stepper_motor_gpio_init();
    base_angle_limit_switch_init();
    boot_button_init();  // Initialize Boot button
    control_serial_init();
    control_serial_start_magnetic_detect_task();

    xTaskCreate(base_calibration_task, "base_calibration_task", BASE_CALIBRATION_TASK_STACK_SIZE, NULL, 10, NULL);
    magnetic_slide_switch_start();
}
