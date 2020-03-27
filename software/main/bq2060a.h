#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_spi_flash.h"
#include <driver/spi_master.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "smbus.h"
#include "bq2060_eeprom.h"

#include "esp_system.h"
#include "esp_log.h"

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


#define INTERNAL_CHARGE_CONTROLLER		(1<<0)
#define PRIMARY_BATTERY_SUPPORT		(1<<1)
#define RELEARN_FLAG		(1<<7)
#define CHARGE_CONTROLLER_ENABLED		(1<<8)
#define PRIMARY_BATTERY		(1<<9)
#define ALARM_MODE		(1<<13)
#define CHARGER_MODE		(1<<14)
#define CAPACITY_MODE		(1<<15)

#define OVER_CHARGED_ALARM			0x8000
#define TERMINATE_CHARGE_ALARM		0x4000
#define OVER_TEMP_ALARM				0x1000
#define TERMINATE_DISCHARGE_ALARM	0x0800
#define REMAINING_CAPACITY_ALARM	0x0200
#define REMAINING_TIME_ALARM		0x0100
#define INITIALIZED					0x0080
#define DISCHARGING					0x0040
#define FULLY_CHARGED				0x0020
#define FULLY_DISCHARGED			0x0010
#define UnknownError				0x0007
#define BadSize						0x0006
#define Overflow_Underflow			0x0005
#define AccessDenied				0x0004
#define UnsupportedCommand			0x0003
#define ReservedCommand				0x0002
#define Busy						0x0001
#define Ok							0x0000




class bq2060a
{
public:
	bq2060a();
	~bq2060a();

	void ReadInfo();
	void PrintInfo();
	
	
	void ResetControll();
	void ChargeSynchronization();
	void EnableVFCCalibration();
	void StopVFCCalibration();
	void SelectEEPROM();
	void ReadEEPROM();
	esp_err_t i2c_read_eeprom(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);
	esp_err_t i2c_write_eeprom(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);
	esp_err_t i2c_write_eeprom(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data_wr, size_t size);
private:
	smbus_info_t* smbus_info;
	
public:
	uint16_t ManufacturerAccess;
	uint16_t RemainingCapacityAlarm;
	uint16_t RemainingTimeAlarm;
	uint16_t BatteryMode;
	uint16_t AtRate;
	uint16_t AtRateTimeToFull;
	uint16_t AtRateTimeToEmpty;
	uint16_t AtRateOK;
	uint16_t Temperature;
	uint16_t Voltage;
	uint16_t Current;
	uint16_t AverageCurrent;
	uint16_t MaxError;
	uint16_t RelativeStateOfCharge;
	uint16_t AbsoluteStateOfCharge;
	uint16_t RemainingCapacity;
	uint16_t FullChargeCapacity;
	
	uint16_t RunTimeToEmpty;
	uint16_t AverageTimeToEmpty;
	uint16_t AverageTimeToFull;
	uint16_t ChargingCurrent;
	uint16_t ChargingVoltage;
	uint16_t BatteryStatus;
	uint16_t CycleCount;
	uint16_t DesignCapacity;
	uint16_t DesignVoltage;
	uint16_t SpecificationInfo;
	uint16_t ManufactureDate;
	uint16_t SerialNumber;
	
	char ManufacturerName[10];
	char DeviceName[10];
	char DeviceChemistry[4];
	
	uint16_t ManufacturerData;
	uint16_t PackStatus;
	uint16_t PackConfiguration;
	uint16_t VCELL4;
	uint16_t VCELL3;
	uint16_t VCELL2;
	uint16_t VCELL1;		
	
};

