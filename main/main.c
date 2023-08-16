/*
 * The most part of BLE code was copied from ESP examples ("gat_server")
 *
 */



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"


#include "ble.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


void on_data_receiving(const void *data, size_t data_len)
{

}


void app_main(void)
{
	setup_ble();
}

