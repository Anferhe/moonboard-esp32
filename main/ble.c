/*
 * ble.c
 *
 *  Created on: Aug 16, 2023
 *      Author: yaroslav
 */


#include "ble.h"
#include "ble_util.h"


static void gatt_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatt_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


static const esp_ble_gap_ext_adv_params_t adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,
    .interval_min = 0x50,
    .interval_max = 0x50,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
};

static struct gatts_profile_inst gatt_profile = {
	.gatts_cb = gatt_profile_event_handler,
	.gatts_if = ESP_GATT_IF_NONE
};

static uint8_t connectedDevCount = 0;


static void gatt_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
		case ESP_GATTS_REG_EVT:
			ESP_LOGI(LOG_TAG_BLE, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
			gatt_profile.service_id.is_primary = true;
			gatt_profile.service_id.id.inst_id = 0x00;
			gatt_profile.service_id.id.uuid.len = ESP_UUID_LEN_128;
			memcpy(&gatt_profile.service_id.id.uuid.uuid.uuid128[0], uartServiceUuid, ESP_UUID_LEN_128);

			esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
			if (set_dev_name_ret) {
				ESP_LOGE(LOG_TAG_BLE, "set device name failed, error code = %x", set_dev_name_ret);
			}

			esp_ble_gatts_create_service(gatts_if, &gatt_profile.service_id, GATT_HANDLE_COUNT);
			break;

		case ESP_GATTS_WRITE_EVT:
			ESP_LOGI(LOG_TAG_BLE, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 \
					 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
			ESP_LOGI(LOG_TAG_BLE, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
			esp_log_buffer_hex(LOG_TAG_BLE, param->write.value, param->write.len);

			on_data_receiving(param->write.value, param->write.len);
			break;

		case ESP_GATTS_CREATE_EVT:
			ESP_LOGI(LOG_TAG_BLE, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
			gatt_profile.service_handle = param->create.service_handle;
			gatt_profile.char_uuid.len = ESP_UUID_LEN_128;
			memcpy(&gatt_profile.char_uuid.uuid.uuid128[0], uartRxCharUuid, ESP_UUID_LEN_128);

			esp_ble_gatts_start_service(gatt_profile.service_handle);
			esp_err_t add_char_ret = esp_ble_gatts_add_char(gatt_profile.service_handle,
			                                                &gatt_profile.char_uuid,
															ESP_GATT_PERM_WRITE,
															ESP_GATT_CHAR_PROP_BIT_WRITE,
															NULL, NULL);
			if (add_char_ret) {
				ESP_LOGE(LOG_TAG_BLE, "add char failed, error code =%x",add_char_ret);
			}
			break;

		case ESP_GATTS_START_EVT:
			ESP_LOGI(LOG_TAG_BLE, "SERVICE_START_EVT, status %d, service_handle %d",
					 param->start.status, param->start.service_handle);
			break;

		case ESP_GATTS_CONNECT_EVT:
			if (++connectedDevCount < MAX_CONNECTION_COUNT) {
				esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &ext_adv);
			}
			break;

		case ESP_GATTS_DISCONNECT_EVT:
			connectedDevCount--;
			ESP_LOGI(LOG_TAG_BLE, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
			esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &ext_adv);
			break;

		default:
			break;
    }
}


static void gatt_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	printf("GATT event: %d\n", event);
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
			gatt_profile.gatts_if = gatts_if;
        } else {
            ESP_LOGI(LOG_TAG_BLE, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id, param->reg.status);
            return;
        }
    }

    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gatt_profile.gatts_if) {
		gatt_profile.gatts_cb(event, gatts_if, param);
    }
}

void setup_ble()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatt_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATT_APP_ID));

    vTaskDelay(200 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &adv_params));
    ESP_ERROR_CHECK(esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(raw_adv_data), &raw_adv_data[0]));
    ESP_ERROR_CHECK(esp_ble_gap_config_ext_scan_rsp_data_raw(EXT_ADV_HANDLE, sizeof(raw_scan_rsp_data), &raw_scan_rsp_data[0]));

    // Start advertising
    esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &ext_adv);
}
