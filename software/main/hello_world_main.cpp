/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   
   https://www.esp32.com/viewtopic.php?t=7400
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "pump.h"
#include "bq2060a.h"
//#include  "TFT_eSPI/TFT_eSPI.h"
//#include "../../esp32VisualGDPcpp/TFT_eSPI/../../"


#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        21
#define I2C_MASTER_SCL_IO        22

extern "C"
{
	void app_main();	
}
void app_main()
{
    printf("Hello world!\n");
	pump ppp;
	ppp.SetPin();
	
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    for (int i = 2; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
	
	bq2060a battery_cntrl;
	//battery_cntrl.ResetControll();
	//battery_cntrl.ChargeSynchronization();
	//battery_cntrl.EnableVFCCalibration();
	battery_cntrl.StopVFCCalibration();
	for(int i = 5000 ; i >= 0 ; i--) {
		battery_cntrl.ReadInfo();
		battery_cntrl.PrintInfo();		
		vTaskDelay(4000 / portTICK_PERIOD_MS);
	}
	
	//battery_cntrl.ReadEEPROM();
	
	/*
	int i2c_master_port = I2C_NUM_1;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;   // GY-2561 provides 10k? pullups
	conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;   // GY-2561 provides 10k? pullups
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	i2c_param_config((i2c_port_t)i2c_master_port, &conf);
	i2c_driver_install((i2c_port_t)i2c_master_port,
		conf.mode,
		I2C_MASTER_RX_BUF_LEN,
		I2C_MASTER_TX_BUF_LEN,
		0);
	
	smbus_info_t* smbus_info = smbus_malloc();
	
	smbus_init(smbus_info, I2C_NUM_1, 0x0b);
	smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);
	
	//smbus_quick(smbus_info, true);
	
	uint16_t  data;
	//smbus_write_word(smbus_info, 0x4f, 0xff5a);
	//smbus_write_word(smbus_info, 0x7d, 0x0000);
	//smbus_write_word(smbus_info, 0x7d, 0x0080);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	for (int i = 0; i < 64; i++)
	{
		smbus_read_word(smbus_info, i, &data);
		printf("reg 0x%.2x =%d \n", i, data);
	}
	*/
	/*
	smbus_read_word(smbus_info, 0x00, &data);
	printf("data =%d \n",data);
	smbus_read_word(smbus_info, 0x01, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x02, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x03, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x04, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x05, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x06, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x07, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x08, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x09, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x0a, &data);
	printf("data =%d \n", data);
	smbus_read_word(smbus_info, 0x0b, &data);
	printf("data =%d \n", data);
	*/
	//smbus_send_byte(smbus_info, 0x59);
	//_is_init(&smbus_info);
    //printf("Restarting now.\n");
    //fflush(stdout);
    //esp_restart();
}
