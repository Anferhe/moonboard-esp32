/*
 * spi.c
 *
 *  Created on: Aug 26, 2023
 *      Author: yaroslav
 */

#include "spi.h"
#include "defines.h"

void sendData(spi_device_handle_t spi, const void *data, size_t dataLen)
{
	spi_transaction_t transaction;
	transaction.flags = 0;
	transaction.length = dataLen * 8; // Length field in transaction is given in bits
	transaction.tx_buffer = data;
	transaction.rx_buffer = NULL;
	transaction.rxlength = 1;

	spi_device_transmit(spi, &transaction);
}

void configureSpi(spi_device_handle_t *spi)
{
    spi_bus_config_t buscfg={
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = SPI_TRANSACTION_LEN,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_MOSI
    };

    spi_device_interface_config_t devcfg={
    	.command_bits = 0,
    	.address_bits = 0,
    	.dummy_bits = 0,
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .clock_speed_hz = SPI_CLOCK_SPEED,
        .spics_io_num = -1,
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 1
    };

    spi_bus_initialize(SPI_DEVICE, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI_DEVICE, &devcfg, spi);
}
