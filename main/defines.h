/*
 * defines.h
 *
 *  Created on: Aug 26, 2023
 *      Author: yaroslav
 */

#ifndef MAIN_DEFINES_H_
#define MAIN_DEFINES_H_

#include "driver/spi_master.h"


#define LED_SPEED  (800 * 1000)

#define DATA_BITS_PER_COLOR  (8)
#define BITS_PER_DATA_BIT    (16)
#define BYTES_PER_DATA_BIT   (BITS_PER_DATA_BIT / 8)

#define SPI_DEVICE       SPI2_HOST
#define SPI_MOSI_PIN     7
#define SPI_CLOCK_SPEED (LED_SPEED * BITS_PER_DATA_BIT)

#define MOONBOARD_MAP_TYPE      0

#define MOONBOARD_ROW_COUNT     18
#define MOONBOARD_COLUMN_COUNT  11
#define MOONBOARD_LED_COUNT     (MOONBOARD_ROW_COUNT * MOONBOARD_COLUMN_COUNT)

#define COLOR_COUNT 3

#define INIT_COLOR     255u, 32u,  32u
#define RESET_COLOR    0u,   0u,   0u
#define START_COLOR    0u,   255u, 0u
#define MOVE_COLOR     0u,   0u,   255u
#define FINISH_COLOR   255u, 0u,   0u

#define SPI_TRANSACTION_LEN (MOONBOARD_LED_COUNT * COLOR_COUNT * DATA_BITS_PER_COLOR * BYTES_PER_DATA_BIT)

#endif /* MAIN_DEFINES_H_ */
