/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_now.h"
#include "esp_event.h"

#include "espnow.h"

#ifdef __cplusplus
extern "C" {
#endif /**< _cplusplus */

/**
 * @brief Enumerated list of control event id
 */
#define ESP_EVENT_ESPNOW_CTRL_BIND      (ESP_EVENT_ESPNOW_CTRL_BASE + 0)
#define ESP_EVENT_ESPNOW_CTRL_UNBIND    (ESP_EVENT_ESPNOW_CTRL_BASE + 1)

/**
 * @brief Control attribute.
 * The high byte is device type, the low byte is device attribute.
 */
typedef enum {
    ESPNOW_ATTRIBUTE_BASE           = 0x0000,
    ESPNOW_ATTRIBUTE_POWER          = 0x0001,
    ESPNOW_ATTRIBUTE_POWER_ADD      = 0x0002,

    ESPNOW_ATTRIBUTE_ATTRIBUTE      = 0x0003,

    /**< light */
    ESPNOW_ATTRIBUTE_LIGHT_BASE     = 0x0100,
    ESPNOW_ATTRIBUTE_BRIGHTNESS     = 0x0101,
    ESPNOW_ATTRIBUTE_BRIGHTNESS_ADD = 0x0102,
    ESPNOW_ATTRIBUTE_HUE            = 0x0103,
    ESPNOW_ATTRIBUTE_HUE_ADD        = 0x0104,
    ESPNOW_ATTRIBUTE_SATURATION     = 0x0105,
    ESPNOW_ATTRIBUTE_SATURATION_ADD = 0x0106,
    ESPNOW_ATTRIBUTE_WARM           = 0x0107,
    ESPNOW_ATTRIBUTE_WARM_ADD       = 0x0108,
    ESPNOW_ATTRIBUTE_COLD           = 0x0109,
    ESPNOW_ATTRIBUTE_COLD_ADD       = 0x010a,
    ESPNOW_ATTRIBUTE_RED            = 0x010b,
    ESPNOW_ATTRIBUTE_RED_ADD        = 0x010c,
    ESPNOW_ATTRIBUTE_GREEN          = 0x010d,
    ESPNOW_ATTRIBUTE_GREEN_ADD      = 0x010e,
    ESPNOW_ATTRIBUTE_BLUE           = 0x010f,
    ESPNOW_ATTRIBUTE_BLUE_ADD       = 0x0110,
    ESPNOW_ATTRIBUTE_MODE           = 0x0111,
    ESPNOW_ATTRIBUTE_MODE_ADD       = 0x0112,

    /**< button */
    ESPNOW_ATTRIBUTE_BUTTON_BASE    = 0x0200,
    ESPNOW_ATTRIBUTE_KEY_1          = 0x0201,
    ESPNOW_ATTRIBUTE_KEY_2          = 0x0202,
    ESPNOW_ATTRIBUTE_KEY_3          = 0x0203,
    ESPNOW_ATTRIBUTE_KEY_4          = 0x0204,
    ESPNOW_ATTRIBUTE_KEY_5          = 0x0205,
    ESPNOW_ATTRIBUTE_KEY_6          = 0x0206,
    ESPNOW_ATTRIBUTE_KEY_7          = 0x0207,
    ESPNOW_ATTRIBUTE_KEY_8          = 0x0208,
    ESPNOW_ATTRIBUTE_KEY_9          = 0x0209,
    ESPNOW_ATTRIBUTE_KEY_10         = 0x0210,
} espnow_attribute_t;

/**
 * @brief Bind information from initiator
 */
typedef struct {
    uint8_t mac[6];                         /**< Initiator's MAC address */
    espnow_attribute_t initiator_attribute; /**< Initiator's attribute */
} espnow_ctrl_bind_info_t;

/**
 * @brief Control data from initiator
 */
typedef struct {
#ifdef CONFIG_ESPNOW_CONTROL_AUTO_CHANNEL_SENDING
    espnow_frame_head_t frame_head;
#endif
    espnow_attribute_t initiator_attribute;         /**< Initiator's attribute */
    espnow_attribute_t responder_attribute;         /**< Responder's attribute */
    union {
        bool responder_value_b;   /**< Boolean */
        struct {
            int responder_value_i;
            int status_value_i;
            int left_x_value_f;
            int left_y_value_f;
            int right_x_value_f;
            int right_y_value_f;
            int channel_one_value_i;
            int channel_two_value_i;
        };

        struct {
            uint32_t responder_value_s_flag : 24; /**< String flag, the value is 0x00, 0x00, 0x00 */
            uint8_t responder_value_s_size;       /**< String length */
        };
    };

    char responder_value_s[0];   /**< NULL terminated string */
} espnow_ctrl_data_t;

/**
 * @brief  The bind callback function
 *
 * @attention Each time a bind frame is received, the callback function will be called.
 *
 * @param[in]  initiator_attribute  initiator attribute
 * @param[in]  mac  initiator mac address
 * @param[in]  rssi  rssi of bind information
 *
 * @return
 *    - TRUE
 *    - FALSE
 */
typedef bool (* espnow_ctrl_bind_cb_t)(espnow_attribute_t initiator_attribute, uint8_t mac[6], int8_t rssi);

/**
 * @brief  The control data callback function
 *
 * @param[in]  initiator_attribute  the received initiator's attribute
 * @param[in]  responder_attribute  the received responder's attribute
 * @param[in]  responder_value  the received responder value
 *
 */
typedef void (* espnow_ctrl_data_cb_t)(espnow_attribute_t initiator_attribute,
                                       espnow_attribute_t responder_attribute,
                                       uint32_t responder_value,
                                       int status_value_i,
                                       int lx_value,
                                       int ly_value,
                                       int rx_value,
                                       int ry_value,
                                       int channel_one_value,
                                       int channel_two_value);

/**
 * @brief  The raw control data callback function
 *
 * @param[in]  src_addr  MAC address of sender
 * @param[in]  data  control data from sender
 * @param[in]  rx_ctrl  received packet radio metadata header
 *
 */
typedef void (* espnow_ctrl_data_raw_cb_t)(espnow_addr_t src_addr, espnow_ctrl_data_t *data, wifi_pkt_rx_ctrl_t *rx_ctrl);

/**
 * @brief  The initiator sends a broadcast bind frame
 *
 * @param[in]  initiator_attribute  initiator attribute
 * @param[in]  enable  bind or unbind
 *
 * @return
 *    - TRUE
 *    - FALSE
 */
esp_err_t espnow_ctrl_initiator_bind(espnow_attribute_t initiator_attribute, bool enable);

/**
 * @brief  The initiator sends a broadcast control data frame
 *
 * @param[in]  initiator_attribute  the sending initiator's attribute
 * @param[in]  responder_attribute  the sending responder's attribute
 * @param[in]  responder_value  the sending responder value
 *
 * @return
 *    - ESP_OK: succeed
 *    - others: fail
 */
esp_err_t espnow_ctrl_initiator_send(espnow_attribute_t initiator_attribute, espnow_attribute_t responder_attribute, uint32_t responder_value, int status,
                                    int x_value, int y_value, int rx_value, int ry_value, int channel_one_value, int channel_two_value);
/**
 * @brief  The responder creates a bind task to process the received bind frame
 *
 * @attention  The bind frame will be processed if the callback function returns true
 *             and wait time is not timeout, and bind frame's RSSI is higher than the set RSSI.
 *
 * @attention  The responder will bind or unbind with the sender according to the value in bind frame.
 *
 * @param[in]  wait_ms  maximum waiting bind time in millisecond
 * @param[in]  rssi  the minimum bind frame RSSI
 * @param[in]  cb  the bind callback function
 *
 * @return
 *    - ESP_OK: succeed
 *    - others: fail
 */
esp_err_t espnow_ctrl_responder_bind(uint32_t wait_ms, int8_t rssi, espnow_ctrl_bind_cb_t cb);

/**
 * @brief  The responder registers control data callback function
 *
 * @param[in]  cb  the control data callback function
 *
 * @return
 *    - ESP_OK: succeed
 *    - others: fail
 */
esp_err_t espnow_ctrl_responder_data(espnow_ctrl_data_cb_t cb);

/**
 * @brief  The responder gets bound list
 *
 * @param[out]  list  the buffer that stores the bound list
 * @param[inout]  size  input maximum bound list size, output the real bound list size
 *
 * @return
 *    - ESP_OK: succeed
 *    - others: fail
 */
esp_err_t espnow_ctrl_responder_get_bindlist(espnow_ctrl_bind_info_t *list, size_t *size);

/**
 * @brief  The responder sets bound list
 *
 * @attention  The bound information will be stored to flash
 *
 * @param[in]  info  the bound information to be set
 *
 * @return
 *    - ESP_OK: succeed
 *    - others: fail
 */
esp_err_t espnow_ctrl_responder_set_bindlist(const espnow_ctrl_bind_info_t *info);

/**
 * @brief  The responder removes bound list
 *
 * @attention  The bound information will be removed from flash
 *
 * @param[in]  info  the bound information to be removed
 *
 * @return
 *    - ESP_OK: succeed
 *    - others: fail
 */
esp_err_t espnow_ctrl_responder_remove_bindlist(const espnow_ctrl_bind_info_t *info);

/**
 * @brief  Send control data frame
 *
 * @param[in]  dest_addr  dest_addr is not used
 * @param[in]  data  control data
 * @param[in]  frame_head  frame header must not be NULL
 * @param[in]  wait_ticks  the maximum waiting time in ticks
 *
 * @return
 *    - ESP_OK: succeed
 *    - others: fail
 */
esp_err_t espnow_ctrl_send(const espnow_addr_t dest_addr, const espnow_ctrl_data_t *data, const espnow_frame_head_t *frame_head, TickType_t wait_ticks);

/**
 * @brief  The responder registers raw control data callback function
 *
 * @param[in]  cb  the raw control data callback function
 *
 * @return
 *    - ESP_OK: succeed
 *    - others: fail
 */
esp_err_t espnow_ctrl_recv(espnow_ctrl_data_raw_cb_t cb);

#ifdef __cplusplus
}
#endif /**< _cplusplus */