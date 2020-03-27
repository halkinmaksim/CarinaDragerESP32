
#include "bq2060a.h"

bq2060a::bq2060a()
{
	int i2c_master_port = I2C_NUM_1;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)21;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;    // GY-2561 provides 10k? pullups
	conf.scl_io_num = (gpio_num_t)22;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;    // GY-2561 provides 10k? pullups
	conf.master.clk_speed = 100000;
	i2c_param_config((i2c_port_t)i2c_master_port, &conf);
	i2c_driver_install((i2c_port_t)i2c_master_port,
		conf.mode,
		0,
		0,
		0);
	
	smbus_info = smbus_malloc();
	
	smbus_init(smbus_info, I2C_NUM_1, 0x0b);
	smbus_set_timeout(smbus_info, 2000 / portTICK_RATE_MS);
}

bq2060a::~bq2060a()
{
}


void bq2060a::ReadInfo()
{
	if (smbus_read_word(smbus_info, 0x00, &ManufacturerAccess) != ESP_OK)
		smbus_read_word(smbus_info, 0x00, &ManufacturerAccess);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x01, &RemainingCapacityAlarm) != ESP_OK)
		smbus_read_word(smbus_info, 0x01, &RemainingCapacityAlarm);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x02, &RemainingTimeAlarm) != ESP_OK)
		smbus_read_word(smbus_info, 0x02, &RemainingTimeAlarm);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x03, &BatteryMode) != ESP_OK)
		smbus_read_word(smbus_info, 0x03, &BatteryMode);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x04, &AtRate) != ESP_OK)
		smbus_read_word(smbus_info, 0x04, &AtRate);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x05, &AtRateTimeToFull) != ESP_OK)
		smbus_read_word(smbus_info, 0x05, &AtRateTimeToFull);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x06, &AtRateTimeToEmpty) != ESP_OK)
		smbus_read_word(smbus_info, 0x06, &AtRateTimeToEmpty);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x07, &AtRateOK) != ESP_OK)
		smbus_read_word(smbus_info, 0x07, &AtRateOK);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x08, &Temperature) != ESP_OK)
		smbus_read_word(smbus_info, 0x08, &Temperature);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x09, &Voltage) != ESP_OK)
		smbus_read_word(smbus_info, 0x09, &Voltage);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x0a, &Current) != ESP_OK)
		smbus_read_word(smbus_info, 0x0a, &Current);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	
	if (smbus_read_word(smbus_info, 0x0b, &AverageCurrent) != ESP_OK)
		smbus_read_word(smbus_info, 0x0b, &AverageCurrent);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x0c, &MaxError) != ESP_OK)
		smbus_read_word(smbus_info, 0x0c, &MaxError);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x0d, &RelativeStateOfCharge) != ESP_OK)
		smbus_read_word(smbus_info, 0x0d, &RelativeStateOfCharge);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x0e, &AbsoluteStateOfCharge) != ESP_OK)
		smbus_read_word(smbus_info, 0x0e, &AbsoluteStateOfCharge);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x0f, &RemainingCapacity) != ESP_OK)
		smbus_read_word(smbus_info, 0x0f, &RemainingCapacity);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	
	if (smbus_read_word(smbus_info, 0x10, &FullChargeCapacity) != ESP_OK)
		smbus_read_word(smbus_info, 0x10, &FullChargeCapacity);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x11, &RunTimeToEmpty) != ESP_OK)
		smbus_read_word(smbus_info, 0x11, &RunTimeToEmpty);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x12, &AverageTimeToEmpty) != ESP_OK)
		smbus_read_word(smbus_info, 0x12, &AverageTimeToEmpty);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x13, &AverageTimeToFull) != ESP_OK)
		smbus_read_word(smbus_info, 0x13, &AverageTimeToFull);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x14, &ChargingCurrent) != ESP_OK)
		smbus_read_word(smbus_info, 0x14, &ChargingCurrent);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x15, &ChargingVoltage) != ESP_OK)
		smbus_read_word(smbus_info, 0x15, &ChargingVoltage);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x16, &BatteryStatus) != ESP_OK)
		smbus_read_word(smbus_info, 0x16, &BatteryStatus);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x17, &CycleCount) != ESP_OK)
		smbus_read_word(smbus_info, 0x17, &CycleCount);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x18, &DesignCapacity) != ESP_OK)
		smbus_read_word(smbus_info, 0x18, &DesignCapacity);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x19, &DesignVoltage) != ESP_OK)
		smbus_read_word(smbus_info, 0x19, &DesignVoltage);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x1a, &SpecificationInfo) != ESP_OK)
		smbus_read_word(smbus_info, 0x1a, &SpecificationInfo);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x1b, &ManufactureDate) != ESP_OK)
		smbus_read_word(smbus_info, 0x1b, &ManufactureDate);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x1c, &SerialNumber) != ESP_OK)
		smbus_read_word(smbus_info, 0x1c, &SerialNumber);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	
	//smbus_read_word(smbus_info, 0x20, &ManufacturerName);
	smbus_i2c_read_block(smbus_info, 0x20, (uint8_t*)ManufacturerName, 10);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	smbus_i2c_read_block(smbus_info, 0x21, (uint8_t*)DeviceName, 10);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	//smbus_read_word(smbus_info, 0x21, &DeviceName);
	
	//smbus_read_word(smbus_info, 0x22, &DeviceChemistry);
	smbus_i2c_read_block(smbus_info, 0x22, (uint8_t*)DeviceChemistry, 4);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	smbus_read_word(smbus_info, 0x23, &ManufacturerData);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	if (smbus_read_word(smbus_info, 0x2f, &PackStatus) != ESP_OK)
		smbus_read_word(smbus_info, 0x2f, &PackStatus);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	smbus_read_word(smbus_info, 0x2f, &PackConfiguration);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	smbus_read_word(smbus_info, 0x3c, &VCELL4);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	smbus_read_word(smbus_info, 0x3d, &VCELL3);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	smbus_read_word(smbus_info, 0x3e, &VCELL2);
	vTaskDelay(40 / portTICK_PERIOD_MS);
	smbus_read_word(smbus_info, 0x3f, &VCELL1);
	vTaskDelay(40 / portTICK_PERIOD_MS);


}

void bq2060a::PrintInfo()
{
	
	
	printf("ManufacturerAccess \t\t = %d\n", ManufacturerAccess);
	printf("RemainingCapacityAlarm \t\t = %d mAh\n", RemainingCapacityAlarm);	
	printf("RemainingTimeAlarm \t\t = %d min\n", RemainingTimeAlarm);
	
	printf("BatteryMode \t\t\t = %d\n", BatteryMode);
	if ((BatteryMode & INTERNAL_CHARGE_CONTROLLER) != 0)
		printf("\t\t\t INTERNAL_CHARGE_CONTROLLER\n");
	if ((BatteryMode & PRIMARY_BATTERY_SUPPORT) != 0)
		printf("\t\t\t PRIMARY_BATTERY_SUPPORT\n");
	if ((BatteryMode & RELEARN_FLAG) != 0)
		printf("\t\t\t RELEARN_FLAG\n");
	if ((BatteryMode & CHARGE_CONTROLLER_ENABLED) != 0)
		printf("\t\t\t CHARGE_CONTROLLER_ENABLED\n");
	if ((BatteryMode & PRIMARY_BATTERY) != 0)
		printf("\t\t\t PRIMARY_BATTERY\n");
	if ((BatteryMode & ALARM_MODE) != 0)
		printf("\t\t\t ALARM_MODE\n");
	if ((BatteryMode & CHARGER_MODE) != 0)
		printf("\t\t\t CHARGER_MODE\n");
	if ((BatteryMode & CAPACITY_MODE) != 0)
		printf("\t\t\t CAPACITY_MODE\n");	
	
	printf("AtRate \t\t\t\t = %d mA\n", AtRate);
	printf("AtRateTimeToFull \t\t = %d min\n", AtRateTimeToFull);
	printf("AtRateTimeToEmpty \t\t = %d min\n", AtRateTimeToEmpty);
	printf("AtRateOK \t\t\t = %d\n", AtRateOK);
	printf("Temperature \t\t\t = %d\n", Temperature);
	printf("Voltage \t\t\t = %d mV\n", Voltage);
	printf("Current \t\t\t = %d mA\n", Current);
	printf("AverageCurrent \t\t\t = %d mA\n", AverageCurrent);
	printf("MaxError \t\t\t = %d prc\n", MaxError);
	printf("RelativeStateOfCharge \t\t = %d prc\n", RelativeStateOfCharge);
	printf("AbsoluteStateOfCharge \t\t = %d prc\n", AbsoluteStateOfCharge);
	printf("RemainingCapacity \t\t = %d mAh\n", RemainingCapacity);	
	printf("FullChargeCapacity \t\t = %d mAh\n", FullChargeCapacity);	
	printf("RunTimeToEmpty \t\t\t = %d min\n", RunTimeToEmpty);
	
	printf("AverageTimeToEmpty \t\t = %d min\n", AverageTimeToEmpty);
	printf("AverageTimeToFull \t\t = %d\n", AverageTimeToFull);
	printf("ChargingCurrent \t\t = %d\n", ChargingCurrent);
	printf("ChargingVoltage \t\t = %d\n", ChargingVoltage);
	
	printf("BatteryStatus \t\t\t = %d\n", BatteryStatus);
	if ((BatteryStatus & OVER_CHARGED_ALARM) != 0)
		printf("\t\t\t OVER_CHARGED_ALARM\n");
	
	if ((BatteryStatus & TERMINATE_CHARGE_ALARM) != 0)
		printf("\t\t\t TERMINATE_CHARGE_ALARM\n");
	if ((BatteryStatus & TERMINATE_CHARGE_ALARM) != 0)
		printf("\t\t\t TERMINATE_CHARGE_ALARM\n");
	if ((BatteryStatus & OVER_TEMP_ALARM) != 0)
		printf("\t\t\t OVER_TEMP_ALARM\n");
	if ((BatteryStatus & TERMINATE_DISCHARGE_ALARM) != 0)
		printf("\t\t\t TERMINATE_DISCHARGE_ALARM\n");
	if ((BatteryStatus & REMAINING_CAPACITY_ALARM) != 0)
		printf("\t\t\t REMAINING_CAPACITY_ALARM\n");
	if ((BatteryStatus & REMAINING_TIME_ALARM) != 0)
		printf("\t\t\t REMAINING_TIME_ALARM\n");
	if ((BatteryStatus & INITIALIZED) != 0)
		printf("\t\t\t INITIALIZED\n");
	if ((BatteryStatus & DISCHARGING) != 0)
		printf("\t\t\t DISCHARGING\n");
	if ((BatteryStatus & FULLY_CHARGED) != 0)
		printf("\t\t\t FULLY_CHARGED\n");
	
	if ((BatteryStatus & FULLY_DISCHARGED) != 0)
		printf("\t\t\t FULLY_DISCHARGED\n");
	if ((BatteryStatus & UnknownError) != 0)
		printf("\t\t\t UnknownError\n");
	if ((BatteryStatus & BadSize) != 0)
		printf("\t\t\t BadSize\n");
	if ((BatteryStatus & Overflow_Underflow) != 0)
		printf("\t\t\t Overflow_Underflow\n");
	if ((BatteryStatus & AccessDenied) != 0)
		printf("\t\t\t AccessDenied\n");
	if ((BatteryStatus & UnsupportedCommand) != 0)
		printf("\t\t\t UnsupportedCommand\n");
	if ((BatteryStatus & ReservedCommand) != 0)
		printf("\t\t\t ReservedCommand\n");
	if ((BatteryStatus & Busy) != 0)
		printf("\t\t\t Busy\n");
		
	printf("CycleCount \t\t\t = %d\n", CycleCount);
	printf("DesignCapacity \t\t\t = %d\n", DesignCapacity);
	printf("DesignVoltage \t\t\t = %d\n", DesignVoltage);
	printf("SpecificationInfo \t\t = %d\n", SpecificationInfo);
	printf("ManufactureDate \t\t = %d\n", ManufactureDate);
	printf("SerialNumber \t\t\t = %d\n", SerialNumber);
	
	printf("ManufacturerName: %s\n", ManufacturerName);
	printf("DeviceName: %s\n", DeviceName);
	printf("DeviceChemistry: %s\n", DeviceChemistry);

	printf("ManufacturerData \t\t = %d\n", ManufacturerData);
	
	
	printf("PackStatus \t\t\t = %d\n", PackStatus);
	
	if ((PackStatus & (1 << 0)) != 0)
		printf("\t CVUV");
	if ((PackStatus & (1 << 1)) != 0)
		printf("\t CVOV");
	if ((PackStatus & (1 << 2)) != 0)
		printf("\t DOK");
	if ((PackStatus & (1 << 3)) != 0)
		printf("\t COK");
	if ((PackStatus & (1 << 4)) != 0)
		printf("\t VDQ");
	if ((PackStatus & (1 << 5)) != 0)
		printf("\t EINT");
	if ((PackStatus & (1 << 6)) != 0)
		printf("\t EDV2");
	if ((PackStatus & (1 << 7)) != 0)
		printf("\t OCE");
	printf("\n");
	
	printf("PackConfiguration \t\t = %d\n", PackConfiguration);
	printf("VCELL4 \t\t = %d\n", VCELL4);
	printf("VCELL3 \t\t = %d\n", VCELL3);
	printf("VCELL2 \t\t = %d\n", VCELL2);
	printf("VCELL1 \t\t = %d\n", VCELL1);	
}

void bq2060a::ResetControll()
{
	smbus_write_word(smbus_info, 0x4f, 0xff5a);
	smbus_write_word(smbus_info, 0x7d, 0x0000);
	smbus_write_word(smbus_info, 0x7d, 0x0080);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void bq2060a::ChargeSynchronization()
{
	smbus_write_word(smbus_info, 0x00, 0x064d);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void bq2060a::EnableVFCCalibration()
{
	smbus_write_word(smbus_info, 0x00, 0x067E);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void bq2060a::StopVFCCalibration()
{
	smbus_write_word(smbus_info, 0x00, 0x0660);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void bq2060a::SelectEEPROM()
{
	smbus_write_word(smbus_info, 0x00, 0x0606);
}

void bq2060a::ReadEEPROM()
{
	SelectEEPROM();
	printf("Read EEPROM\n");
	printf("--------------------------------------------------------------------------\n");
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	uint8_t val;
	//val = 0xa4;
	//i2c_write_eeprom(I2C_NUM_1, 0x0a, &val, 1);
	val = 0x38;
	//i2c_write_eeprom(I2C_NUM_1, 0x0b, &val, 1);
	/**/
	uint8_t data[128];
	data[0] = 0;
	data[1] = 2;
	
	i2c_read_eeprom(I2C_NUM_1, data, 128);
	for (int i = 0; i < 128; i++)
	{
		if ((i % 16) == 0)
			printf("\n");
		printf("0x%.2x ", data[i]);
	}
	printf("\n");
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	printf("--------------------------------------------------------------------------\n");
	printf("Read EEPROM\n");
	smbus_write_word(smbus_info, 0x00, 0x0000);
	//smbus_read_word(smbus_info, 0x1c, &SerialNumber);
	//smbus_read_word(smbus_info, 0x1c, &SerialNumber);
	//smbus_read_word(smbus_info, 0x1c, &SerialNumber);
}

esp_err_t bq2060a::i2c_read_eeprom(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
	if (size == 0) {
		return ESP_OK;
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x51 << 1) | READ_BIT, ACK_CHECK_EN);
	if (size > 1) {
		i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t)ACK_VAL);
	}
	i2c_master_read_byte(cmd, data_rd + size - 1, (i2c_ack_type_t)NACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}



esp_err_t bq2060a::i2c_write_eeprom(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x51 << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

esp_err_t bq2060a::i2c_write_eeprom(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data_wr, size_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x51 << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr , ACK_CHECK_EN);
	i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}