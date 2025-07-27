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

#define BUS_VOLTAGE_LSB     0.0001953125f  // Bus voltage LSB (195.3125 µV/bit)
#define MAX_CURRENT         10             // Current in Amps
#define CURRENT_LSB         (MAX_CURRENT / (float)(1UL << 19))
#define SHUNT_VALUE         0.015
#define SHUNT_CAL_CONSTANT  13107200.0f



// Write 1 byte to the specified register
int reg_write(i2c_inst_t *i2c,  const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {
    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}


void ina228_init(ina228_config *config)
{   
    // SCCB I2C @ 100 kHz
	i2c_init(config->i2c, 100 * 1000);
	gpio_set_function(config->sda, GPIO_FUNC_I2C);
	gpio_set_function(config->scl, GPIO_FUNC_I2C);
    gpio_pull_up(config->sda);
    gpio_pull_up(config->scl);


    ina228_reset(config);
    sleep_ms(20);  // Wait 20 after reset
    uint8_t config_reg[2];
    reg_read(config->i2c, INA228_ADDRESS, INA228_CONFIG, config_reg, 2);
    printf("Config register after reset: 0x%02X%02X\n", config_reg[0], config_reg[1]);
    uint8_t id_buf[2];
    reg_read(config->i2c, INA228_ADDRESS, INA228_DEVICE_ID, id_buf, 2);
    uint16_t device_id = (id_buf[0] << 8) | id_buf[1];

    uint8_t man_buf[2];
    reg_read(config->i2c, INA228_ADDRESS, INA228_MANUFACTURER_ID, man_buf, 2);
    uint16_t manufacturer_id = (man_buf[0] << 8) | man_buf[1];

    printf("Manufacturer ID: 0x%04X\n", manufacturer_id);
    printf("Device ID:       0x%04X\n", device_id);


    ina228_updateShuntCalRegister(config);

    // ina228_reg_write(config, INA228_SHUNT_CAL, SHUNT_CAL);
}

void ina228_reset(ina228_config *config){
    uint8_t buf[3];
    buf[0] = INA228_CONFIG;
    buf[1] = (INA228_CFG_RST >> 8) & 0xFF;
    buf[2] = INA228_CFG_RST & 0xFF;

    i2c_write_blocking(config->i2c, INA228_ADDRESS, buf, 3, false);
}

void ina228_updateShuntCalRegister(ina228_config *config) {
    float scale = 1;
        if (ina228_getADCRange(config)) {
        scale = 4;
    }
    float shunt_cal = SHUNT_CAL_CONSTANT * CURRENT_LSB * SHUNT_VALUE * scale;
    uint16_t cal_value = (uint16_t)shunt_cal;
    uint8_t data[2] = {
        (cal_value >> 8) & 0xFF,  // MSB
        cal_value & 0xFF          // LSB
    };  

    reg_write(config->i2c, INA228_ADDRESS, INA228_SHUNT_CAL, data, 2);
    uint8_t cal_buf[2];
    reg_read(config->i2c, INA228_ADDRESS, INA228_SHUNT_CAL, cal_buf, 2);
    uint16_t readback = (cal_buf[0] << 8) | cal_buf[1];
    printf("SHUNT_CAL = %u (expected ~%u)\n", readback, cal_value);
// 
}

uint8_t ina228_getADCRange(ina228_config *config) {
  uint8_t ADCRange[2];
  reg_read(config->i2c, INA228_ADDRESS, INA228_CONFIG, ADCRange, 2);
  return (ADCRange[1] >> 4) & 1;
}

float ina228_voltage(ina228_config *config){ 
    uint8_t regReadbusVoltage[3];
    float busVoltage;
    int32_t rawVoltage24 = 0;

    reg_read(config->i2c, INA228_ADDRESS, INA228_VBUS, regReadbusVoltage, 3);
    
    int32_t signed1 = rawVoltage24;
    if (signed1 & 0x800000) {  // if sign bit set
        signed1 |= 0xFF000000; // sign extend to 32-bit
    }
    return (float)(signed1 >> 4) * 195.3125f / 1e6f;


    // Assemble 24-bit signed integer
    // rawVoltage24 = (regReadbusVoltage[0] << 16) |
    //                (regReadbusVoltage[1] << 8)  |
    //                (regReadbusVoltage[2]);

    // return (float)((uint32_t)rawVoltage24 >> 4) * 195.3125 / 1e6;
}


uint32_t ina228_voltage_raw(ina228_config *config){ 
    uint8_t regReadbusVoltage[3];
    int32_t rawVoltage24 = 0;

    reg_read(config->i2c, INA228_ADDRESS, INA228_VBUS, regReadbusVoltage, 3);
    
    // Assemble 24-bit signed integer
    rawVoltage24 = (regReadbusVoltage[0] << 16) |
                   (regReadbusVoltage[1] << 8)  |
                   (regReadbusVoltage[2]);

    return (uint32_t)rawVoltage24;
}


float ina228_current(ina228_config *config){ 
    uint8_t regReadCurrent[2];
    int16_t rawCurrent16 = 0;

    reg_read(config->i2c, INA228_ADDRESS, INA228_CURRENT, regReadCurrent, 2);

    rawCurrent16 = (regReadCurrent[0] << 8) | regReadCurrent[1];

    // Current LSB and scaling might differ, adjust as per datasheet
    return (float)rawCurrent16 * CURRENT_LSB * 1000.0;  // mA
}


uint16_t ina228_current_raw(ina228_config *config){ 
    uint8_t regReadCurrent[2];
    reg_read(config->i2c, INA228_ADDRESS, INA228_CURRENT, regReadCurrent, 2);
    return (regReadCurrent[0] << 8) | regReadCurrent[1];
}
