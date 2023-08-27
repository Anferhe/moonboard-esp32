/*
 * led_structures.c
 *
 *  Created on: Aug 26, 2023
 *      Author: yaroslav
 */


#include "led_structures.h"

#include <stdbool.h>
#include <string.h>


static inline void prepareDataBit(DataBit *bit, bool value)
{

#if BITS_PER_DATA_BIT == 24
	static const uint8_t logicalOne[3] =  {0b11111111, 0b11110000, 0b00000000};
	static const uint8_t logicalZero[3] = {0b11111000, 0b00000000, 0b00000000};
#elif BITS_PER_DATA_BIT == 16
	static const uint8_t logicalOne[2]  = {0b11111111, 0b00000000};
	static const uint8_t logicalZero[2] = {0b11100000, 0b00000000};
#else
#error "Supported only 16 and 24 values for BITS_PER_DATA_BIT variable"
#endif

	if (value) {
		memcpy(bit->data, logicalOne, BYTES_PER_DATA_BIT);
	} else {
		memcpy(bit->data, logicalZero, BYTES_PER_DATA_BIT);
	}
}

static inline void prepareChannel(Channel *color, uint8_t value)
{
	for(size_t i = 0; i < DATA_BITS_PER_COLOR; ++i) {
		prepareDataBit(&color->bits[i], value & (1 << (7 - i)));
	}
}

inline void prepareLed(Led *led, uint8_t red, uint8_t green, uint8_t blue)
{
	prepareChannel(&led->channels[0], red);
	prepareChannel(&led->channels[1], green);
	prepareChannel(&led->channels[2], blue);
}
