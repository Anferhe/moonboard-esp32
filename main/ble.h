/*
 * ble.h
 *
 *  Created on: Aug 16, 2023
 *      Author: yaroslav
 */

#ifndef MAIN_BLE_H_
#define MAIN_BLE_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include <string.h>
#include <stdint.h>


#define LOG_TAG_BLE             "BLE"

#define MAX_CONNECTION_COUNT    CONFIG_BT_ACL_CONNECTIONS
#define DEVICE_NAME             "Moonboard ElCap"


// Implemented in main.c
void on_data_receiving(const void *data, size_t data_len);

void setup_ble();


#endif /* MAIN_BLE_H_ */
