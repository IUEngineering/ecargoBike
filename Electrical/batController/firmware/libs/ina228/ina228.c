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

#define SHUNT_CAL           2500


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

    uint8_t id[1];
    uint8_t man[1];
    
    reg_read(config->i2c, INA228_ADDRESS, INA228_VBUS, id, 1);
    reg_read(config->i2c, INA228_ADDRESS, INA228_VBUS, man, 1);

    printf("Manufacturer ID:    0x%04X\r\n", man[0]);
	printf("Device ID:          0x%04X\r\n", id[0]);


    // ina228_reg_write(config, INA228_SHUNT_CAL, SHUNT_CAL);
}

void ina228_reset(ina228_config *config){
    uint8_t buf[3];
    buf[0] = INA228_CONFIG;
    buf[1] = (INA228_CFG_RST >> 8) & 0xFF;
    buf[2] = INA228_CFG_RST & 0xFF;

    i2c_write_blocking(config->i2c, INA228_ADDRESS, buf, 3, false);
}

float ina228_voltage(ina228_config *config){ 
    uint8_t regReadbusVoltage[3];
    float busVoltage;
    int32_t rawVoltage24 = 0;

    reg_read(config->i2c, INA228_ADDRESS, INA228_VBUS, regReadbusVoltage, 3);
    
    // Assemble 24-bit signed integer
    rawVoltage24 = (regReadbusVoltage[0] << 16) |
                   (regReadbusVoltage[1] << 8)  |
                   (regReadbusVoltage[2]);

    return (float)((uint32_t)rawVoltage24 >> 4) * 195.3125 / 1e6;
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

uint32_t ina228_current_raw(ina228_config *config){ 
    uint8_t regReadbusCurrent[3];
    int32_t rawCurrent24 = 0;

    reg_read(config->i2c, INA228_ADDRESS, INA228_CURRENT, regReadbusCurrent, 3);
    
    // Assemble 24-bit signed integer
    rawCurrent24 = (regReadbusCurrent[0] << 16) |
                   (regReadbusCurrent[1] << 8)  |
                   (regReadbusCurrent[2]);

    return (uint32_t)rawCurrent24;
}