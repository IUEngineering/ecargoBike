#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "ina228.h"


/*
 * SHUNT_CAL is a conversion constant that represents the shunt resistance
 * used to calculate current value in Amps. This also sets the resolution
 * (CURRENT_LSB) for the current register.
 *
 * SHUNT_CAL is 15 bits wide (0 - 32768)
 *
 * SHUNT_CAL = 13107.2 x 10^6 x CURRENT_LSB x Rshunt
 *
 * CURRENT_LSB = Max Expected Current / 2^19
 */


#define CURRENT_LSB 	0.0000190735
#define SHUNT_CAL       2500

void ina228_init(ina228_config *config)
{   
    printf("1!\n");
    // SCCB I2C @ 100 kHz
	i2c_init(config->i2c, 100 * 1000);
	gpio_set_function(config->sda, GPIO_FUNC_I2C);
	gpio_set_function(config->scl, GPIO_FUNC_I2C);
    printf("2!\n");
    gpio_pull_up(config->sda);
    gpio_pull_up(config->scl);
    printf("3!\n");


    ina228_reset(config);
    printf("5!\n");

	printf("Manufacturer ID:    0x%04X\r\n",ina228_reg_read(config, INA228_MANUFACTURER_ID));
	printf("Device ID:          0x%04X\r\n",ina228_reg_read(config, INA228_DEVICE_ID));
    printf("6!\n");

    ina228_reg_write(config, INA228_SHUNT_CAL, SHUNT_CAL);
}

void ina228_reset(ina228_config *config){
    uint8_t buf[3];
    buf[0] = INA228_CONFIG;
    buf[1] = (INA228_CFG_RST >> 8) & 0xFF;
    buf[2] = INA228_CFG_RST & 0xFF;

    i2c_write_blocking(config->i2c, INA228_ADDRESS, buf, 3, false);
}

uint8_t ina228_reg_read(ina228_config *config, uint8_t reg){
	i2c_write_blocking(config->i2c, INA228_ADDRESS, &reg, 1, true);

	uint8_t buf;
	i2c_read_blocking(config->i2c, INA228_ADDRESS, &buf, 2, false);

	return buf;
}

void ina228_reg_write(ina228_config *config, uint8_t reg, uint16_t value){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF; // MSB
    buf[2] = value & 0xFF;        // LSB

    i2c_write_blocking(config->i2c, INA228_ADDRESS, buf, 3, false);
}



// float ina228_voltage(uint8_t i2c)
// {
// 	int32_t iBusVoltage;
// 	float fBusVoltage;
// 	bool sign;

// 	i2c_read_blocking(i2c, INA228_ADDRESS, INA228_VBUS, (uint8_t *)&iBusVoltage, 3);
// 	sign = iBusVoltage & 0x80;
// 	iBusVoltage = __bswap32(iBusVoltage & 0xFFFFFF) >> 12;
// 	if (sign) iBusVoltage += 0xFFF00000;
// 	fBusVoltage = (iBusVoltage) * 0.0001953125;

// 	return (fBusVoltage);
// }

// float ina228_dietemp(uint8_t i2c)
// {
// 	uint16_t iDieTemp;
// 	float fDieTemp;

// 	iDieTemp = i2c_read_short(i2c, INA228_ADDRESS, INA228_DIETEMP);
// 	fDieTemp = (iDieTemp) * 0.0078125;

// 	return (fDieTemp);
// }

// float ina228_shuntvoltage(uint8_t i2c)
// {
// 	int32_t iShuntVoltage;
// 	float fShuntVoltage;
// 	bool sign;

// 	i2c_read_blocking(i2c, INA228_ADDRESS, INA228_VSHUNT, (uint8_t *)&iShuntVoltage, 3);
// 	sign = iShuntVoltage & 0x80;
// 	iShuntVoltage = __bswap32(iShuntVoltage & 0xFFFFFF) >> 12;
// 	if (sign) iShuntVoltage += 0xFFF00000;

// 	fShuntVoltage = (iShuntVoltage) * 0.0003125;		// Output in mV when ADCRange = 0
// 	//fShuntVoltage = (iShuntVoltage) * 0.000078125;	// Output in mV when ADCRange = 1

// 	return (fShuntVoltage);
// }

// float ina228_current(uint8_t i2c)
// {
// 	int32_t iCurrent;
// 	float fCurrent;
// 	bool sign;

// 	i2c_read_blocking(i2c, INA228_ADDRESS, INA228_CURRENT, (uint8_t *)&iCurrent, 3);
// 	sign = iCurrent & 0x80;
// 	iCurrent = __bswap32(iCurrent & 0xFFFFFF) >> 12;
// 	if (sign) iCurrent += 0xFFF00000;
// 	fCurrent = (iCurrent) * CURRENT_LSB;

// 	return (fCurrent);
// }

// float ina228_power(uint8_t i2c)
// {
// 	uint32_t iPower;
// 	float fPower;

// 	i2c_read_blocking(i2c, INA228_ADDRESS, INA228_POWER, (uint8_t *)&iPower, 3);
// 	iPower = __bswap32(iPower & 0xFFFFFF) >> 8;
// 	fPower = 3.2 * CURRENT_LSB * iPower;

// 	return (fPower);
// }

// /*
//  * Returns energy in Joules.
//  * 1 Watt = 1 Joule per second
//  * 1 W/hr = Joules / 3600
//  */

// float ina228_energy(uint8_t i2c)
// {
// 	uint64_t iEnergy;
// 	float fEnergy;

// 	i2c_read_blocking(i2c, INA228_ADDRESS, INA228_ENERGY, (uint8_t *)&iEnergy, 5);
// 	iEnergy = __bswap64(iEnergy & 0xFFFFFFFFFF) >> 24;

// 	fEnergy = 16 * 3.2 * CURRENT_LSB * iEnergy;

// 	return (fEnergy);
// }

// /*
//  * Returns electric charge in Coulombs.
//  * 1 Coulomb = 1 Ampere per second.
//  * Hence Amp-Hours (Ah) = Coulombs / 3600
//  */

// float ina228_charge(uint8_t i2c)
// {
// 	int64_t iCharge;
// 	float fCharge;
// 	bool sign;

// 	i2c_read_blocking(i2c, INA228_ADDRESS, INA228_CHARGE, (uint8_t *)&iCharge, 5);
// 	sign = iCharge & 0x80;
// 	iCharge = __bswap64(iCharge & 0xFFFFFFFFFF) >> 24;
// 	if (sign) iCharge += 0xFFFFFF0000000000;

// 	fCharge = CURRENT_LSB * iCharge;

// 	return (fCharge);
// }


