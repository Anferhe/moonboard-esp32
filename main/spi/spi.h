/*
 * spi.h
 *
 *  Created on: Aug 26, 2023
 *      Author: yaroslav
 */

#ifndef SPI_SPI_H_
#define SPI_SPI_H_

#include "driver/spi_master.h"

void sendData(spi_device_handle_t spi, const void *data, size_t dataLen);
void configureSpi(spi_device_handle_t *spi);


#endif /* SPI_SPI_H_ */
