/*
 * moonboard.c
 *
 *  Created on: Aug 27, 2023
 *      Author: yaroslav
 */

#include "moonboard.h"

#include "map.h"
#include "esp_log.h"


void initMoonboard(Moonboard *moonboard, LedStrip *strip, spi_device_handle_t spi)
{
	moonboard->strip = strip;
	moonboard->spi = spi;
	resetParser(&moonboard->parser);

	clearAllLeds(moonboard);
	setAllLedsColor(moonboard, INIT_COLOR);

	sendData(moonboard->spi, (void *)(&moonboard->strip->leds[0]), SPI_TRANSACTION_LEN);
}

void clearAllLeds(Moonboard *moonboard)
{
	setAllLedsColor(moonboard, RESET_COLOR);
}

void setAllLedsColor(Moonboard *moonboard, uint8_t red, uint8_t green, uint8_t blue)
{
	for (size_t i = 0; i < MOONBOARD_LED_COUNT; ++i) {
		prepareLed(&moonboard->strip->leds[i], red, green, blue);
	}
}

void processData(Moonboard *moonboard, const void *data, size_t dataLen)
{
	Parser *parser = &moonboard->parser;

	if (processSequence(parser, data, dataLen) != PARSER_DONE) {
		return;
	}

	esp_log_buffer_char("PROBLEM", parser->sequence, parser->sequenceLen);

	parseSequence(parser);

	clearAllLeds(moonboard);

	for (size_t i = 0; i < parser->startHoldsLen; ++i) {
		prepareLed(&moonboard->strip->leds[mapLed(parser->startHolds[i])], START_COLOR);
	}

	for (size_t i = 0; i < parser->moveHoldsLen; ++i) {
		prepareLed(&moonboard->strip->leds[mapLed(parser->moveHolds[i])], MOVE_COLOR);
	}

	for (size_t i = 0; i < parser->finishHoldsLen; ++i) {
		prepareLed(&moonboard->strip->leds[mapLed(parser->finishHolds[i])], FINISH_COLOR);
	}

	sendData(moonboard->spi, &moonboard->strip->leds[0], SPI_TRANSACTION_LEN);
}
