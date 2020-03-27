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


enum EREG_ADDR_bq2060_eeprom
{
	Check_Byte_1				= 0x00,
	RemainingTimeAlarm			= 0x02,
	RemainingCapacityAlarm		= 0x04,
	EDVA0ImpedanceAgeFactor		= 0x06,
	EDVTCColdImpedanceFactor	= 0x07,	
	MiscOptions					= 0x08,
	SafetyOvertemperature		= 0x09,
	ChargingVoltage				= 0x0a,
	CycleCount					= 0x0e,
	DesignVoltage				= 0x12,	
	SpecificationInformation	= 0x14,
	ManufactureDate				= 0x16,
	SerialNumber				= 0x18,
	FastChargingCurrent			= 0x1a,
	MaintenanceChargingCurrent	= 0x1c,
	PreChargeCurrent			= 0x1e,
	ManufacturerNameLength		= 0x20,	
	LightDischargeCurrent		= 0x2b,
	MaximumOvercharge			= 0x2e,
	DeviceNameLength			= 0x30,
	LastMeasuredDischarge		= 0x38,
	PackCapacity				= 0x3a,
	CycleCountThreshold			= 0x3c,
	PackConfiguration			= 0x3f,
	DeviceChemistryLength		= 0x40,	
	MaxTDeltaT					= 0x45,
	OverloadCurrent				= 0x46,
	OvervoltageMargin			= 0x48,
	OvercurrentMargin			= 0x49,
	CellUnderOverVoltage		= 0x4a,
	FastChargeTermination		= 0x4b,	
	FullyChargedClear			= 0x4c,
	ChargeEfficiency			= 0x4d,
	DeltaTTime					= 0x4e,
	HoldoffTime					= 0x4f,	
	ManufacturersDataLength		= 0x50,
	ControlMode					= 0x51,
	DigitalFilter				= 0x52,
	SelfDischargeRate			= 0x53,
	BatteryLow					= 0x54,
	NearFull					= 0x55,	
	VFCOffset1					= 0x5e,
	VFCOffset2					= 0x60,	
	TemperatureOffset			= 0x61,
	ADCOffset					= 0x62,
	EfficiencyTemperatureCompensation = 0x63,
	EfficiencyDropOffPercentage	= 0x64,
	EfficiencyReductionRate		= 0x65,
	ADCVoltageGain				= 0x66,
	ADCSenseResistorGain		= 0x68,
	VFCSenseResistor			= 0x6a,
	VOC25						= 0x6c,
	VOC50						= 0x6e,
	VOC75						= 0x70,
	EDVFEDV0					= 0x72,
	EMFEDV1						= 0x74,
	EDVT0Factor					= 0x76,
	EDVC1C0FactorEDV2			= 0x78,
	EDVR0Factor					= 0x7A,
	EDVR1Factor					= 0x7C,
	CheckByte2					= 0x7E	
};



class bq2060_eeprom
{
public:
	bq2060_eeprom();
	~bq2060_eeprom();

	void PrintHexValue();
	void PrintStrValue();
private:
	uint8_t eepom_dump[128];
	
	uint16_t Check_Byte_1;	
	uint16_t RemainingTimeAlarm;
	uint16_t RemainingCapacityAlarm;
	uint16_t EDVA0ImpedanceAgeFactor;
	uint16_t EDVTCColdImpedanceFactor;
	uint16_t MiscOptions;
	uint16_t SafetyOvertemperature;
	uint16_t ChargingVoltage;	
	uint16_t CycleCount;
	uint16_t DesignVoltage;
	uint16_t SpecificationInformation;
	uint16_t ManufactureDate;
	uint16_t SerialNumber;
	uint16_t FastChargingCurrent;
	uint16_t MaintenanceChargingCurrent;
	uint16_t PreChargeCurrent;
	uint16_t ManufacturerNameLength;
	char ManufacturerName[64];	
	uint16_t LightDischargeCurrent;
	uint16_t MaximumOvercharge;
	uint16_t DeviceNameLength;
	char DeviceName[64];
	uint16_t LastMeasuredDischarge;
	uint16_t PackCapacity;
	uint16_t CycleCountThreshold;
	uint16_t PackConfiguration;
	uint16_t DeviceChemistryLength;
	char DeviceChemistry[64];
	uint16_t MaxTDeltaT;
	uint16_t OverloadCurrent;
	uint16_t OvervoltageMargin;
	uint16_t OvercurrentMargin;
	uint16_t CellUnderOverVoltage;
	uint16_t FastChargeTermination;
	uint16_t FullyChargedClear;
	uint16_t ChargeEfficiency;
	uint16_t DeltaTTime;
	uint16_t HoldoffTime;
	uint16_t ManufacturersDataLength;
	char ManufacturersData[64];
	uint16_t ControlMode;
	uint16_t DigitalFilter;
	uint16_t SelfDischargeRate;
	uint16_t BatteryLow;
	uint16_t NearFull;
	uint16_t VFCOffset1;
	uint16_t VFCOffset2;
	uint16_t TemperatureOffset;
	uint16_t ADCOffset;
	uint16_t EfficiencyTemperatureCompensation;
	uint16_t EfficiencyDropOffPercentage;
	uint16_t EfficiencyReductionRate;
	uint16_t ADCVoltageGain;
	uint16_t ADCSenseResistorGain;
	uint16_t VFCSenseResistor;
	uint16_t VOC25;
	uint16_t VOC50;
	uint16_t VOC75;
	uint16_t EDVFEDV0;
	uint16_t EMFEDV1;
	uint16_t EDVT0Factor;
	uint16_t EDVC1C0FactorEDV2;
	uint16_t EDVR0Factor;
	uint16_t EDVR1Factor;
	uint16_t CheckByte2;
};

