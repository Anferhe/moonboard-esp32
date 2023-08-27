/*
 * moonboard.h
 *
 *  Created on: Aug 27, 2023
 *      Author: yaroslav
 */

#ifndef MOONBOARD_MOONBOARD_H_
#define MOONBOARD_MOONBOARD_H_

#include "defines.h"
#include "parser.h"
#include "map.h"

#include "led/led_structures.h"
#include "spi/spi.h"

#include <stddef.h>


struct Moonboard {
	LedStrip *strip;
	spi_device_handle_t spi;
	Parser parser;
};

typedef struct Moonboard Moonboard;

void initMoonboard(Moonboard *moonboard, LedStrip *strip, spi_device_handle_t spi);
void clearAllLeds(Moonboard *moonboard);
void setAllLedsColor(Moonboard *moonboard, uint8_t red, uint8_t green, uint8_t blue);
void processData(Moonboard *moonboard, const void *data, size_t dataLen);

#endif /* MOONBOARD_MOONBOARD_H_ */
