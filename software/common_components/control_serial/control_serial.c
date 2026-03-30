/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "stepper_motor.h"
#include "control_serial.h"
#include "magnetic_slide_switch.h"

static const char *TAG = "Control Serial";

/**
 * @brief UART command receive task
 * 
 * @param arg Task parameter (unused)
 * 
 * @note This task loops to receive UART data, parse commands and execute corresponding operations:
 *       - CMD_BASE_ANGLE_CONTROL: Control base rotation to specified angle
 *       - CMD_BASE_ACTION_CONTROL: Execute predefined actions
 *       - CMD_MAGNETIC_SWITCH_EVENT: Magnetic switch calibration command
 * @note Implements frame header synchronization to handle data misalignment
 */
static void uart_cmd_receive_task(void *arg)
{
    // Track absolute angle of the base (0-180 degrees, initial: 90 degrees)
    static float s_base_absolute_angle = 90.0f;
    
    // Configure temporary buffer for incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        // Read data from UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            // Frame header synchronization: search for 0xAA 0x55
            int frame_start = -1;
            for (int i = 0; i < len - 1; i++) {
                if (data[i] == UART_FRAME_HEAD_H && data[i + 1] == UART_FRAME_HEAD_L) {
                    frame_start = i;
                    break;
                }
            }
            
            // If frame header not found, discard data and continue
            if (frame_start == -1) {
                ESP_LOGD(TAG, "Frame header not found in %d bytes, discarding", len);
                continue;
            }
            
            // If frame header not at beginning, log warning
            if (frame_start > 0) {
                ESP_LOGW(TAG, "Frame header found at offset %d, discarded %d bytes", frame_start, frame_start);
            }
            
            // Check if we have enough data for minimum frame
            int remaining_len = len - frame_start;
            if (remaining_len < UART_FRAME_MIN_LEN) {
                ESP_LOGW(TAG, "Incomplete frame: only %d bytes after header", remaining_len);
                continue;
            }
            
            // Point to the start of the frame
            uint8_t *frame = data + frame_start;
            
            // Get data length from frame
            uint16_t data_len = (frame[2] << 8) | frame[3];
            
            // Validate data length
            if (data_len == 0 || data_len > BUF_SIZE - UART_FRAME_HEADER_LEN - 1) {
                ESP_LOGW(TAG, "Invalid data length: %d", data_len);
                continue;
            }
            
            // Check if we have complete frame
            int expected_frame_len = UART_FRAME_HEADER_LEN + data_len + 1; // header + data + checksum
            if (remaining_len < expected_frame_len) {
                ESP_LOGW(TAG, "Incomplete frame: expected %d bytes, got %d", expected_frame_len, remaining_len);
                continue;
            }
            
            // Get command code and data length
            uint8_t cmd = frame[4];
            
            // Calculate checksum for the entire data payload (command code + data bytes)
            uint8_t calc_checksum = 0;
            for (int i = 4; i < 4 + data_len; i++) {
                calc_checksum += frame[i];
            }
            
            // Get received checksum (last byte of frame)
            uint8_t received_checksum = frame[4 + data_len];
            
            // Verify checksum FIRST before parsing any data
            if (calc_checksum != received_checksum) {
                ESP_LOGW(TAG, "Checksum verification failed: calc=0x%02X, recv=0x%02X, cmd=0x%02X", 
                         calc_checksum, received_checksum, cmd);
                continue;  // Discard this frame and continue
            }
            
            // Checksum verified, now safe to parse and execute commands
            
            // Parse data according to command type
            if (cmd == CMD_BASE_ANGLE_CONTROL) {  // Base angle control
                if (data_len != 3) {  // Should be: cmd(1) + angle(2) = 3 bytes
                    ESP_LOGW(TAG, "Invalid data length for angle control: %d (expected 3)", data_len);
                    continue;
                }
                
                // Get data value (combine two bytes) - sound source angle from head (0-180 degrees)
                // Value is relative to initial position (90 degrees = facing head directly)
                uint16_t value = (frame[5] << 8) | frame[6];
                
                // Check if received angle is within valid range (0-180 degrees)
                if (value > 180) {
                    ESP_LOGW(TAG, "Received angle %d out of range [0, 180], ignoring rotation. Current absolute angle: %.1f", 
                             value, s_base_absolute_angle);
                    continue;
                }
                
                // Calculate rotation angle relative to initial position (90 degrees)
                // diff_angle = value - 90 means:
                //   - value = 90: diff_angle = 0 (no rotation, facing head)
                //   - value = 120: diff_angle = 30 (rotate right 30 degrees)
                //   - value = 60: diff_angle = -30 (rotate left 30 degrees)
                float diff_angle = (float)value - 90.0f;
                
                // Calculate new absolute angle after rotation
                float new_absolute_angle = s_base_absolute_angle + diff_angle;
                
                // Check if new absolute angle would be out of range (0-180 degrees)
                if (new_absolute_angle < 0.0f || new_absolute_angle > 180.0f) {
                    ESP_LOGW(TAG, "Rotation would result in angle %.1f out of range [0, 180], ignoring. Current: %.1f, received: %d, diff: %.1f", 
                             new_absolute_angle, s_base_absolute_angle, value, diff_angle);
                    continue;
                }
                
                ESP_LOGI(TAG, "Received sound source angle: %d, current absolute angle: %.1f, diff_angle: %.1f, new absolute angle: %.1f", 
                         value, s_base_absolute_angle, diff_angle, new_absolute_angle);
                
                // Execute angle control (rotate by diff_angle degrees)
                stepper_rotate_angle_with_accel((int16_t)diff_angle, STEPPER_SPEED_ULTRA_FAST);
                vTaskDelay(pdMS_TO_TICKS(100));
                stepper_motor_power_off();
                
                // Update absolute angle after successful rotation
                s_base_absolute_angle = new_absolute_angle;
                ESP_LOGI(TAG, "Base rotated to absolute angle: %.1f degrees", s_base_absolute_angle);
            }
            else if (cmd == CMD_BASE_ACTION_CONTROL) {  // Base action control
                if (data_len != 3) {  // Should be: cmd(1) + action(2) = 3 bytes
                    ESP_LOGW(TAG, "Invalid data length for action control: %d (expected 3)", data_len);
                    continue;
                }
                
                // Get action value (combine two bytes)
                uint16_t action = (frame[5] << 8) | frame[6];
                ESP_LOGI(TAG, "Received action value: %d", action);
                
                // Execute different predefined actions based on action value
                switch (action) {
                    case STEPPER_ACTION_SHAKE_HEAD:
                        ESP_LOGI(TAG, "Execute action: SHAKE_HEAD");
                        stepper_shake_head(6.0, 2, 600);
                        stepper_motor_power_off();
                        control_serial_send_action_complete();  // Send action complete notification
                        break;
                        
                    case STEPPER_ACTION_SHAKE_HEAD_DECAY:
                        ESP_LOGI(TAG, "Execute action: SHAKE_HEAD_DECAY");
                        stepper_shake_head_decay(20.0, 0.8, 800);
                        stepper_motor_power_off();
                        control_serial_send_action_complete();  // Send action complete notification
                        break;
                        
                    case STEPPER_ACTION_LOOK_AROUND:
                        ESP_LOGI(TAG, "Execute action: LOOK_AROUND");
                        stepper_look_around(35.0, 35.0, 10.0, 600, 600, 800);
                        stepper_motor_power_off();
                        control_serial_send_action_complete();  // Send action complete notification
                        break;
                        
                    case STEPPER_ACTION_BEAT_SWING:  // action = 3: drum beat
                        ESP_LOGI(TAG, "Execute action: BEAT_SWING (drum beat)");
                        stepper_beat_swing(20.0, 800);
                        stepper_motor_power_off();
                        // control_serial_send_action_complete();  // Send action complete notification
                        break;
                        
                    case STEPPER_ACTION_CAT_NUZZLE:
                        ESP_LOGI(TAG, "Execute action: CAT_NUZZLE");
                        stepper_cat_nuzzle(20, 3, 1500);
                        stepper_motor_power_off();
                        control_serial_send_action_complete();  // Send action complete notification
                        break;
                        
                    default:
                        ESP_LOGW(TAG, "Unknown action: %d", action);
                        break;
                }
            }
            else if (cmd == CMD_MAGNETIC_SWITCH_EVENT) {  // Magnetic switch command
                if (data_len != 3) {  // Should be: cmd(1) + mag_cmd(2) = 3 bytes
                    ESP_LOGW(TAG, "Invalid data length for magnetic switch command: %d (expected 3)", data_len);
                    continue;
                }
                
                // Get command data (combine two bytes)
                uint16_t mag_cmd = (frame[5] << 8) | frame[6];
                
                // Check if it's recalibration command
                if (mag_cmd == MAG_SWITCH_CMD_RECALIBRATE) {
                    ESP_LOGI(TAG, "Magnetic switch recalibration command received");
                    magnetic_slide_switch_start_recalibration();
                    ESP_LOGI(TAG, "Recalibration started, please follow the prompts");
                } else {
                    ESP_LOGW(TAG, "Unknown magnetic switch command: 0x%04X", mag_cmd);
                }
            }
            else {
                ESP_LOGW(TAG, "Unknown command code: 0x%02X, data_len: %d", cmd, data_len);
            }
        }
    }
}

/**
 * @brief Initialize serial control module
 * 
 * @return esp_err_t ESP_OK indicates success
 * 
 * @note Configures UART parameters and creates command receive task
 */
esp_err_t control_serial_init(void)
{
    /* Configure UART driver parameters, communication pins and install driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, UART_ECHO_TXD, UART_ECHO_RXD, UART_ECHO_RTS, UART_ECHO_CTS));

    xTaskCreate(uart_cmd_receive_task, "uart_cmd_receive_task", UART_CMD_TASK_STACK_SIZE, NULL, 12, NULL);
    return ESP_OK;
}

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
esp_err_t control_serial_send_magnetic_switch_event(uint16_t event)
{
    uint8_t tx_buffer[8];
    
    // Frame header
    tx_buffer[0] = UART_FRAME_HEAD_H;  // 0xAA
    tx_buffer[1] = UART_FRAME_HEAD_L;  // 0x55
    
    // Data length (command code 1 byte + data 2 bytes = 3 bytes)
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x03;
    
    // Command code
    tx_buffer[4] = CMD_MAGNETIC_SWITCH_EVENT;  // 0x03
    
    // Event data (big-endian, high byte first)
    tx_buffer[5] = (event >> 8) & 0xFF;  // High byte
    tx_buffer[6] = event & 0xFF;          // Low byte
    
    // Checksum (command code + data high byte + data low byte)
    tx_buffer[7] = tx_buffer[4] + tx_buffer[5] + tx_buffer[6];
    
    // Send through UART
    int sent = uart_write_bytes(ECHO_UART_PORT_NUM, tx_buffer, sizeof(tx_buffer));
    
    if (sent == sizeof(tx_buffer)) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send magnetic switch event: sent=%d", sent);
        return ESP_FAIL;
    }
}

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
 * [7]:   Checksum (command code + data bytes sum = 0x02 + 0x00 + 0x10 = 0x12)
 */
esp_err_t control_serial_send_action_complete(void)
{
    uint8_t tx_buffer[8];
    
    // Frame header
    tx_buffer[0] = UART_FRAME_HEAD_H;  // 0xAA
    tx_buffer[1] = UART_FRAME_HEAD_L;  // 0x55
    
    // Data length (command code 1 byte + data 2 bytes = 3 bytes)
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x03;
    
    // Command code
    tx_buffer[4] = CMD_ACTION_COMPLETE;  // 0x02
    
    // Complete code (big-endian, high byte first)
    tx_buffer[5] = (ACTION_COMPLETE_CODE >> 8) & 0xFF;  // 0x00
    tx_buffer[6] = ACTION_COMPLETE_CODE & 0xFF;          // 0x10
    
    // Checksum (command code + data high byte + data low byte)
    tx_buffer[7] = tx_buffer[4] + tx_buffer[5] + tx_buffer[6];
    
    // Send through UART
    int sent = uart_write_bytes(ECHO_UART_PORT_NUM, tx_buffer, sizeof(tx_buffer));
    
    if (sent == sizeof(tx_buffer)) {
        ESP_LOGI(TAG, "Action complete notification sent");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send action complete: sent=%d", sent);
        return ESP_FAIL;
    }
}

/**
 * @brief Send magnetic attachment status notification
 * 
 * @param status Attachment status (MAGNETIC_ATTACH_STATUS_ATTACHED or MAGNETIC_ATTACH_STATUS_DETACHED)
 * @return esp_err_t ESP_OK indicates success
 * 
 * Packet format:
 * AA 55 00 03 00 00 01 01
 * [0-1]: Header (0xAA 0x55)
 * [2-3]: Data length (0x00 0x03 = 3 bytes)
 * [4]:   Command code (0x00 = magnetic attachment status)
 * [5-6]: Status data (0x00 0x01 = ATTACHED, 0x00 0x00 = DETACHED)
 * [7]:   Checksum (command code + data bytes sum)
 */
static esp_err_t control_serial_send_magnetic_attach_status(uint16_t status)
{
    uint8_t tx_buffer[8];
    
    // Frame header
    tx_buffer[0] = UART_FRAME_HEAD_H;  // 0xAA
    tx_buffer[1] = UART_FRAME_HEAD_L;  // 0x55
    
    // Data length (command code 1 byte + data 2 bytes = 3 bytes)
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x03;
    
    // Command code
    tx_buffer[4] = CMD_MAGNETIC_ATTACH_STATUS;  // 0x00
    
    // Status data (big-endian, high byte first)
    tx_buffer[5] = (status >> 8) & 0xFF;  // High byte
    tx_buffer[6] = status & 0xFF;          // Low byte
    
    // Checksum (command code + data high byte + data low byte)
    tx_buffer[7] = tx_buffer[4] + tx_buffer[5] + tx_buffer[6];
    
    // Send through UART
    int sent = uart_write_bytes(ECHO_UART_PORT_NUM, tx_buffer, sizeof(tx_buffer));
    
    if (sent == sizeof(tx_buffer)) {
        ESP_LOGD(TAG, "Magnetic attach status sent: 0x%04X", status);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send magnetic attach status: sent=%d", sent);
        return ESP_FAIL;
    }
}

/**
 * @brief Magnetic attachment detection task
 * 
 * @param arg Task parameter (unused)
 * 
 * @note This task periodically sends magnetic attachment status notification to ESP-VoCat
 *       Currently always reports ATTACHED status at fixed intervals
 *       TODO: Implement actual magnetic detection logic using Hall sensor or other method
 */
static void magnetic_detect_task(void *arg)
{
    ESP_LOGI(TAG, "Magnetic detection task started");
    
    while (1) {
        // Send attached status notification
        control_serial_send_magnetic_attach_status(MAGNETIC_ATTACH_STATUS_ATTACHED);
        
        // Wait for next detection interval
        vTaskDelay(pdMS_TO_TICKS(MAGNETIC_DETECT_INTERVAL_MS));
    }
}

/**
 * @brief Start magnetic attachment detection task
 */
void control_serial_start_magnetic_detect_task(void)
{
    xTaskCreate(magnetic_detect_task, "magnetic_detect_task", 
                MAGNETIC_DETECT_TASK_STACK_SIZE, NULL, 5, NULL);
    ESP_LOGI(TAG, "Magnetic detection task created");
}

/**
 * @brief Send magnetic switch calibration step notification
 * 
 * @param step_code Step code (0x0011 or 0x0012)
 * @return esp_err_t ESP_OK indicates success
 * 
 * Packet format example (second position complete):
 * AA 55 00 03 03 00 11 14
 * [0-1]: Header (0xAA 0x55)
 * [2-3]: Data length (0x00 0x03 = 3 bytes)
 * [4]:   Command code (0x03 = magnetic switch event)
 * [5-6]: Step code (0x00 0x11 = second position complete)
 * [7]:   Checksum (0x03 + 0x00 + 0x11 = 0x14)
 * 
 * Packet format example (calibration complete):
 * AA 55 00 03 03 00 12 15
 */
esp_err_t control_serial_send_magnetic_switch_calibration_step(uint16_t step_code)
{
    uint8_t tx_buffer[8];
    
    // Frame header
    tx_buffer[0] = UART_FRAME_HEAD_H;  // 0xAA
    tx_buffer[1] = UART_FRAME_HEAD_L;  // 0x55
    
    // Data length (command code 1 byte + data 2 bytes = 3 bytes)
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x03;
    
    // Command code
    tx_buffer[4] = CMD_MAGNETIC_SWITCH_EVENT;  // 0x03
    
    // Step code (big-endian, high byte first)
    tx_buffer[5] = (step_code >> 8) & 0xFF;
    tx_buffer[6] = step_code & 0xFF;
    
    // Checksum (command code + data high byte + data low byte)
    tx_buffer[7] = tx_buffer[4] + tx_buffer[5] + tx_buffer[6];
    
    // Send through UART
    int sent = uart_write_bytes(ECHO_UART_PORT_NUM, tx_buffer, sizeof(tx_buffer));
    
    if (sent == sizeof(tx_buffer)) {
        ESP_LOGI(TAG, "Calibration step notification sent: 0x%04X", step_code);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send calibration step: sent=%d", sent);
        return ESP_FAIL;
    }
}

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
esp_err_t control_serial_print_magnetometer_data(int16_t x, int16_t y, int16_t z)
{
    // Format string: "X: 100, Y: 100, Z: 100\r\n"
    // Maximum length: "X: -32768, Y: -32768, Z: -32768\r\n" = 35 bytes
    char tx_buffer[50];
    int len = snprintf(tx_buffer, sizeof(tx_buffer), "%d,%d,%d\r\n", x, y, z);
    
    if (len < 0 || len >= sizeof(tx_buffer)) {
        ESP_LOGE(TAG, "Failed to format magnetometer data string");
        return ESP_FAIL;
    }
    
    // Send through UART
    int sent = uart_write_bytes(ECHO_UART_PORT_NUM, tx_buffer, len);
    
    if (sent == len) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send magnetometer data: sent=%d, expected=%d", sent, len);
        return ESP_FAIL;
    }
}
