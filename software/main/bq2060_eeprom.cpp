
#include "bq2060_eeprom.h"

bq2060_eeprom::bq2060_eeprom()
{
}

bq2060_eeprom::~bq2060_eeprom()
{
}


void bq2060_eeprom::PrintHexValue()
{
	
}

void bq2060_eeprom::PrintStrValue()
{
	printf("Check_Byte_1 \t\t = %d\n", Check_Byte_1);
	
	printf("RemainingTimeAlarm \t\t = %d\n", RemainingTimeAlarm);
	printf("RemainingCapacityAlarm \t\t = %d\n", RemainingCapacityAlarm);
	printf("EDVA0ImpedanceAgeFactor \t\t = %d\n", EDVA0ImpedanceAgeFactor);
	printf("EDVTCColdImpedanceFactor \t\t = %d\n", EDVTCColdImpedanceFactor);
	printf("MiscOptions \t\t = %d\n", MiscOptions);
	printf("SafetyOvertemperature \t\t = %d\n", SafetyOvertemperature);
	printf("ChargingVoltage \t\t = %d\n", ChargingVoltage);
	printf("CycleCount \t\t = %d\n", CycleCount);
	printf("DesignVoltage \t\t = %d\n", DesignVoltage);
	printf("SpecificationInformation \t\t = %d\n", SpecificationInformation);
	printf("ManufactureDate \t\t = %d\n", ManufactureDate);
	printf("SerialNumber \t\t = %d\n", SerialNumber);	
	printf("FastChargingCurrent \t\t = %d\n", FastChargingCurrent);
	printf("MaintenanceChargingCurrent \t\t = %d\n", MaintenanceChargingCurrent);
	printf("PreChargeCurrent \t\t = %d\n", PreChargeCurrent);
	printf("ManufacturerName \t\t = %s\n", ManufacturerName);
	printf("LightDischargeCurrent \t\t = %d\n", LightDischargeCurrent);
	printf("MaximumOvercharge \t\t = %d\n", MaximumOvercharge);
	
	
	
}
