/*
 * ble.c
 *
 *  Created on: Aug 16, 2023
 *      Author: yaroslav
 */


#include <ble/ble.h>
#include <ble/ble_util.h>


static void gatt_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatt_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static int host_rcv_pkt(uint8_t *data, uint16_t len);
static void controller_rcv_pkt_ready(void);

static const esp_ble_gap_ext_adv_params_t adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,
    .interval_min = 0x080,
    .interval_max = 0x0c0,
    .channel_map = ADV_CHNL_ALL,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0, 0, 0, 0, 0, 0},
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
    .primary_phy = ESP_BLE_GAP_PRI_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_CODED,
    .sid = 0,
    .scan_req_notif = false,
};

static struct gatts_profile_inst gatt_profile = {
	.gatts_cb = gatt_profile_event_handler,
	.gatts_if = ESP_GATT_IF_NONE
};

static esp_vhci_host_callback_t vhci_host_cb = {
    .notify_host_send_available = controller_rcv_pkt_ready,
    .notify_host_recv = host_rcv_pkt
};

static uint8_t connectedDevCount = 0;


static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   char *key_str = NULL;
   switch(key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

   }

   return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
   char *auth_str = NULL;
   switch(auth_req) {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
   }

   return auth_str;
}

static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(LOG_TAG_BLE, "Bonded devices number : %d", dev_num);
    for (int i = 0; i < dev_num; i++) {
        esp_log_buffer_hex(LOG_TAG_BLE, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

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

			onDataReceiving(param->write.value, param->write.len);
			break;
		case ESP_GATTS_MTU_EVT:
			ESP_LOGI(LOG_TAG_BLE, "ESP_GATTS_MTU_EVT, mtu %d, conn_id %d", param->mtu.mtu, param->mtu.conn_id);
			esp_err_t set_mtu_ret = esp_ble_gatt_set_local_mtu(param->mtu.mtu);
			ESP_LOGI(LOG_TAG_BLE, "esp_ble_gatt_set_local_mtu result %d", set_mtu_ret);
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

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	printf("GAP event: %d\n", event);
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        ESP_LOGI(LOG_TAG_GAP,"ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT status %d",  param->ext_adv_set_params.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
         ESP_LOGI(LOG_TAG_GAP,"ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT status %d",  param->ext_adv_data_set.status);
         break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
         ESP_LOGI(LOG_TAG_GAP, "ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT, status = %d", param->ext_adv_data_set.status);
        break;
    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
        ESP_LOGI(LOG_TAG_GAP, "ESP_GAP_BLE_ADV_TERMINATED_EVT, status = %d", param->adv_terminate.status);
        if(param->adv_terminate.status == 0x00) {
            ESP_LOGI(LOG_TAG_GAP, "ADV successfully ended with a connection being created");
        }
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
        /* Call the following function to input the passkey which is displayed on the remote device */
        //esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT: {
        ESP_LOGI(LOG_TAG_GAP, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;
    }
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        ESP_LOGI(LOG_TAG_GAP, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        ESP_LOGI(LOG_TAG_GAP, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(LOG_TAG_GAP, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32, param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer device.
        ESP_LOGI(LOG_TAG_GAP, "The passkey Notify number:%06" PRIu32, param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(LOG_TAG_GAP, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(LOG_TAG_GAP, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(LOG_TAG_GAP, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(LOG_TAG_GAP, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(LOG_TAG_GAP, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(LOG_TAG_GAP, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
        }
        show_bonded_devices();
        break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
        ESP_LOGD(LOG_TAG_GAP, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(LOG_TAG_GAP, "ESP_GAP_BLE_REMOVE_BOND_DEV");
        ESP_LOGI(LOG_TAG_GAP, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        esp_log_buffer_hex(LOG_TAG_GAP, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(LOG_TAG_GAP, "------------------------------------");
        break;
    }
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        ESP_LOGI(LOG_TAG_GAP, "ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT, tatus = %x", param->local_privacy_cmpl.status);
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(LOG_TAG_GAP, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
        ESP_LOGI(LOG_TAG_GAP, "update phy params status = %d, tx_phy = %d, rx_phy = %d",
                 param->phy_update.status,
                 param->phy_update.tx_phy,
                 param->phy_update.rx_phy);

        ESP_LOG_BUFFER_HEX(LOG_TAG_GAP, param->phy_update.bda, ESP_BD_ADDR_LEN);
        break;
    default:
        break;
    }
}

static void controller_rcv_pkt_ready(void)
{
    ESP_LOGI(LOG_TAG_HCI, "Controller rcv pkt ready");
}

static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    /* Check second byte for HCI event. If event opcode is 0x0e, the event is
     * HCI Command Complete event. Sice we have recieved "0x0e" event, we can
     * check for byte 4 for command opcode and byte 6 for it's return status. */
    if (data[1] == 0x0e) {
        if (data[6] == 0) {
            ESP_LOGI(LOG_TAG_HCI, "Event opcode 0x%02x success.", data[4]);
        } else {
            ESP_LOGE(LOG_TAG_HCI, "Event opcode 0x%02x fail with reason: 0x%02x.", data[4], data[6]);
            return ESP_FAIL;
        }
    } else {
		ESP_LOG_BUFFER_HEX(LOG_TAG_HCI, data, len);
    }

    return ESP_OK;
}

void setupBle()
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

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatt_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATT_APP_ID));

    vTaskDelay(200 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &adv_params));
    ESP_ERROR_CHECK(esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(raw_adv_data), &raw_adv_data[0]));
    ESP_ERROR_CHECK(esp_ble_gap_config_ext_scan_rsp_data_raw(EXT_ADV_HANDLE, sizeof(raw_scan_rsp_data), &raw_scan_rsp_data[0]));

//    ESP_ERROR_CHECK(esp_vhci_host_register_callback(&vhci_host_cb));

    // Start advertising
    esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &ext_adv);
}
