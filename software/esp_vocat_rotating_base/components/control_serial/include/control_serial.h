/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* ========== UART Configuration ========== */
#define UART_ECHO_TXD               (GPIO_NUM_29)   /**< UART TX pin */
#define UART_ECHO_RXD               (GPIO_NUM_8)    /**< UART RX pin */
#define UART_ECHO_RTS               (-1)            /**< UART RTS pin (unused) */
#define UART_ECHO_CTS               (-1)            /**< UART CTS pin (unused) */

#define ECHO_UART_PORT_NUM          (1)             /**< UART port number */
#define ECHO_UART_BAUD_RATE         (115200)        /**< Baud rate */
#define UART_CMD_TASK_STACK_SIZE    (1024 * 3)      /**< Command receive task stack size */
#define BUF_SIZE                    (1024)          /**< Receive buffer size */

/* ========== Frame Format Definitions ========== */
#define UART_FRAME_HEAD_H           (0xAA)          /**< Frame header high byte */
#define UART_FRAME_HEAD_L           (0x55)          /**< Frame header low byte */
#define UART_FRAME_HEADER_LEN       (2)             /**< Frame header length */
#define UART_FRAME_MIN_LEN          (5)             /**< Minimum frame length: header+length+command+checksum */

/* ========== Command Code Definitions ========== */
#define CMD_MAGNETIC_ATTACH_STATUS  (0x00)          /**< Magnetic attachment status notification */
#define CMD_BASE_ANGLE_CONTROL      (0x01)          /**< Base angle control */
#define CMD_BASE_ACTION_CONTROL     (0x02)          /**< Base action control */
#define CMD_MAGNETIC_SWITCH_EVENT   (0x03)          /**< Magnetic slide switch event */
#define CMD_ACTION_COMPLETE         (0x02)          /**< Specific action execution complete */

/* ========== Magnetic Switch Special Command Codes ========== */
#define MAG_SWITCH_CMD_RECALIBRATE  (0x0010)        /**< Recalibrate magnetic switch */
#define MAG_SWITCH_CALIB_START      (0x0011)        /**< Start calibration */
#define MAG_SWITCH_CALIB_FIRST_DONE (0x0012)        /**< Second position calibration done, proceed to third action */
#define MAG_SWITCH_CALIB_COMPLETE   (0x0013)        /**< Calibration complete */

/* ========== Action Complete Status Code ========== */
#define ACTION_COMPLETE_CODE        (0x0010)        /**< Action complete code (16) */

/* ========== Magnetic Attachment Status Codes ========== */
#define MAGNETIC_ATTACH_STATUS_ATTACHED     (0x0001)    /**< ESP-VoCat attached to base */
#define MAGNETIC_ATTACH_STATUS_DETACHED     (0x0000)    /**< ESP-VoCat detached from base */

/* ========== Task Configuration ========== */
#define MAGNETIC_DETECT_TASK_STACK_SIZE     (1024 * 2)  /**< Magnetic detection task stack size */
#define MAGNETIC_DETECT_INTERVAL_MS         (500)      /**< Detection interval in milliseconds */

/* ========== Function Declarations ========== */

/**
 * @brief Initialize serial control module
 * 
 * @return esp_err_t ESP_OK indicates success
 * 
 * @note Initializes UART and creates command receive task
 */
esp_err_t control_serial_init(void);

/**
 * @brief Send magnetic slide switch event notification
 * 
 * @param event Event type (magnetic_slide_switch_event_t)
 * @return esp_err_t ESP_OK indicates success
 * 
 * Packet format:
 * AA 55 00 03 03 00 01 04
 * [0-1]: Header (0xAA 0x55)
 * [2-3]: Data length (0x00 0x03 = 3 bytes)
 * [4]:   Command code (0x03 = magnetic slide switch event)
 * [5-6]: Event data (0x00 0x01 = SLIDE_DOWN)
 * [7]:   Checksum (command code + data bytes sum)
 */
esp_err_t control_serial_send_magnetic_switch_event(uint16_t event);

/**
 * @brief Send specific action execution complete notification
 * 
 * @return esp_err_t ESP_OK indicates success
 * 
 * Packet format:
 * AA 55 00 03 02 00 10 12
 * [0-1]: Header (0xAA 0x55)
 * [2-3]: Data length (0x00 0x03 = 3 bytes)
 * [4]:   Command code (0x02 = action execution complete)
 * [5-6]: Complete code (0x00 0x10 = 16)
 * [7]:   Checksum (command code + data bytes sum)
 */
esp_err_t control_serial_send_action_complete(void);

/**
 * @brief Send calibration step notification
 * 
 * @param step_code Step code (MAG_SWITCH_CALIB_FIRST_DONE or MAG_SWITCH_CALIB_COMPLETE)
 * @return esp_err_t ESP_OK indicates success
 * 
 * Packet format example (second position complete):
 * AA 55 00 03 03 00 11 14
 * [0-1]: Header (0xAA 0x55)
 * [2-3]: Data length (0x00 0x03 = 3 bytes)
 * [4]:   Command code (0x03 = magnetic switch event)
 * [5-6]: Step code (0x00 0x11 = second position complete)
 * [7]:   Checksum (0x03 + 0x00 + 0x11 = 0x14)
 */
esp_err_t control_serial_send_magnetic_switch_calibration_step(uint16_t step_code);

/**
 * @brief Start magnetic attachment detection task
 * 
 * @note This task periodically checks if ESP-VoCat is magnetically attached to the base
 *       and sends status notifications via UART
 */
void control_serial_start_magnetic_detect_task(void);

/**
 * @brief Print magnetometer sensor data via UART
 * 
 * @param x X-axis magnetic field value
 * @param y Y-axis magnetic field value
 * @param z Z-axis magnetic field value
 * @return esp_err_t ESP_OK indicates success
 * 
 * @note Output format: "X: 100, Y: 100, Z: 100\r\n"
 * @note Sends data directly through UART without frame header
 */
esp_err_t control_serial_print_magnetometer_data(int16_t x, int16_t y, int16_t z);

#ifdef __cplusplus
}
#endif
