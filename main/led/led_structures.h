/*
 * led_structures.h
 *
 *  Created on: Aug 26, 2023
 *      Author: yaroslav
 */

#ifndef LED_LED_STRUCTURES_HPP_
#define LED_LED_STRUCTURES_HPP_

#include <stdint.h>
#include <stddef.h>

#include "defines.h"


// Every information bit is coded using 16 or 24 bits
struct DataBit {
	uint8_t data[BYTES_PER_DATA_BIT];
} __attribute__((packed));

typedef struct DataBit DataBit;


struct Channel {
	DataBit bits[8];
} __attribute__((packed));

typedef struct Channel Channel;


struct Led {
	Channel channels[3];
} __attribute__((packed));

typedef struct Led Led;


struct LedStrip {
	Led leds[MOONBOARD_LED_COUNT];
} __attribute__((packed));

typedef struct LedStrip LedStrip;


extern void prepareLed(Led *led, uint8_t red, uint8_t green, uint8_t blue);


#endif /* LED_LED_STRUCTURES_HPP_ */
