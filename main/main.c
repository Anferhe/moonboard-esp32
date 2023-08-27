/*
 * The most part of BLE code was copied from ESP examples ("gat_server")
 *
 *    Author: yaroslav
 */

#include "ble/ble.h"
#include "led/led_structures.h"
#include "spi/spi.h"
#include "moonboard/moonboard.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"

#include "sdkconfig.h"

Moonboard moonboard;
spi_device_handle_t spi;
LedStrip leds;


void onDataReceiving(const void *data, size_t dataLen)
{
	processData(&moonboard, data, dataLen);
}

void app_main(void)
{
	configureSpi(&spi);
	initMoonboard(&moonboard, &leds, spi);
	setupBle();
}
