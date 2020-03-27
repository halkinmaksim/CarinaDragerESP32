/*
 *
 *
 *
 *
 **/
#include "st7789.h"


st7789::st7789(gpio_num_t dc, gpio_num_t rst, gpio_num_t cs)
{
	csPin = cs;
	dcPin = dc;
	rstPin = rst;
}


void st7789::init(uint16_t width, uint16_t height) 
{
	commonST7789Init(NULL);

	_colstart = ST7789_240x240_XSTART;
	_rowstart = ST7789_240x240_YSTART;
//	_width  = width;
//	_height = height;
//	displayInit(init_240x240);
//	setRotation(2);
}


void st7789::writeCmd(uint8_t c)
{
	gpio_set_level(dcPin, (uint32_t)SPI_Command_Mode);
	spi_master_write_bytes(&c, 1);
}
void st7789::writeData(uint8_t d)
{
	gpio_set_level(dcPin, (uint32_t)SPI_Data_Mode);
	spi_master_write_bytes(&d, 1);
}
void st7789::writeSPI(uint8_t c)
{
	spi_master_write_bytes(&c, 1);
}

bool st7789::spi_master_write_bytes(const uint8_t* Data, size_t DataLength)
{
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	if (DataLength > 0) {
		memset(&SPITransaction, 0, sizeof(spi_transaction_t));
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;
#if 1
		ret = spi_device_transmit(m_spiDevice, &SPITransaction);
#endif
#if 0
		ret = spi_device_polling_transmit(m_spiDevice, &SPITransaction);
#endif
		assert(ret == ESP_OK); 
	}

	return true;
}