#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

// Header file
#include "NRF24L01.h"

static const char* TAG = "NRF24L01.h";

void Pin_CSN(NRF24_t * dev, int x){
    gpio_set_direction(dev->csnPin, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->csnPin, x);
}

void Pin_CE(NRF24_t * dev, int x){
    gpio_set_direction(CE, GPIO_MODE_OUTPUT);
    gpio_set_level(CE, x);
}

void SPI_Config(NRF24_t * dev){
    esp_err_t ret;

	gpio_reset_pin(CSN);
    Pin_CSN(dev, 1);

	spi_bus_config_t spi_bus_config = {
		.sclk_io_num = SCLK,
		.mosi_io_num = MOSI,
		.miso_io_num = MISO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize( SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO );
	ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);

	spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof( spi_device_interface_config_t));
	devcfg.clock_speed_hz = 4000000 ;
	devcfg.spics_io_num = -1;
	devcfg.queue_size = 7;
	devcfg.mode = 0;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

	spi_device_handle_t handle;
	ret = spi_bus_add_device( SPI2_HOST, &devcfg, &handle);
	ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);
    ESP_LOGI(TAG, "SPI_CONFIG Done");

	dev->_SPIHandle = handle;
    
}

////////////////////////////////////////////

uint8_t spi_transfer(NRF24_t * dev, uint8_t address) {
	uint8_t datain[1];
	uint8_t dataout[1];
	dataout[0] = address;
	spi_read_byte(dev, datain, dataout, 1 );
	return datain[0];
}

bool spi_read_byte(NRF24_t * dev, uint8_t* Datain, uint8_t* Dataout, size_t DataLength )
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Dataout;
		SPITransaction.rx_buffer = Datain;
		spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	}
	return true;
}

bool spi_write_byte(NRF24_t * dev, uint8_t* Dataout, size_t DataLength )
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Dataout;
		SPITransaction.rx_buffer = NULL;
		spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	}

	return true;
}


void Write_Register(NRF24_t * dev, uint8_t reg, uint8_t * value, uint8_t len)
{
	Pin_CSN(dev, 0);
	spi_transfer(dev, W_REGISTER | (REGISTER_MASK & reg));
	spi_write_byte(dev, value, len);
	Pin_CSN(dev, 1);

}

void Read_Register(NRF24_t *dev, uint8_t reg, uint8_t *value, uint8_t len)
{
	Pin_CSN(dev, 0);
	spi_transfer(dev, R_REGISTER | (REGISTER_MASK & reg));
	spi_read_byte(dev, value, value, len);
	Pin_CSN(dev, 1);
}


void Register_Config(NRF24_t * dev ,uint8_t channel, uint8_t payload){
    Write_Register(dev, RF_CH, &channel, 1); // Set RF channel

	uint8_t temp[5];
	Read_Register(dev, RF_CH, temp, sizeof(temp));
	for (int i=0;i<5;i++) {
		ESP_LOGI(TAG, "buffer=0x%x", temp[i]);
	}	// Write_Register(dev, RX_PW_P0, payload);

	uint8_t * adr = (uint8_t *)"ARYAN";
	Write_Register(dev, RX_ADDR_P1, adr, mirf_ADDR_LEN);

	// this to verity whether address is properly set or not
	uint8_t buffer[5];
	Read_Register(dev, RX_ADDR_P1, buffer, sizeof(buffer));
	//ESP_LOGI(TAG, "Buffer = %x", buffer);
	ESP_LOGI(TAG, "Buffer = %s", buffer);
    for (int i=0;i<5;i++) {
		ESP_LOGI(TAG, "adr[%d]=0x%x RX_ADDR_P1 buffer[%d]=0x%x", i, adr[i], i, buffer[i]);
	}

	// Write_Register(dev, RX_PW_P1, payload); // Set length of incoming payload
}