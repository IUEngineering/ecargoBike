#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "ina228.h"


#define BUS_VOLTAGE_LSB         0.0001953125f  // Bus voltage LSB (195.3125 µV/bit)
#define CURRENT_LSB             (100e-6)
#define SHUNT_CAL_CONSTANT      13107200.0f
#define SHUNT_RESISTOR          0.015
#define SHUNT_CAL               (uint16_t)(SHUNT_CAL_CONSTANT * CURRENT_LSB * SHUNT_RESISTOR)


int ina228_init(ina228_config *config)
{
    // I2C setup
    i2c_init(config->i2c, 100 * 1000);
    gpio_set_function(config->sda, GPIO_FUNC_I2C);
    gpio_set_function(config->scl, GPIO_FUNC_I2C);
    gpio_pull_up(config->sda);
    gpio_pull_up(config->scl);

    if (ina228_reset(config) != 0) {
        printf("Failed to reset INA228.\n");
        return -1;
    }

    uint8_t id[2];
    uint8_t man[2];

    if (reg_read(config->i2c, INA228_ADDRESS, 0xFE, man, 2) < 0 ||
        reg_read(config->i2c, INA228_ADDRESS, 0xFF, id, 2) < 0) {
        printf("Failed to read Manufacturer or Device ID.\n");
        return -1;
    }

    printf("Manufacturer ID: 0x%04X\r\n", (man[0] << 8) | man[1]);
    printf("Device ID:       0x%04X\r\n", (id[0] << 8) | id[1]);

    ina228_updateShuntCalRegister(config);

    return 0;
}

void ina228_updateShuntCalRegister(ina228_config *config) {
    float scale = 1;
        if (ina228_getADCRange(config)) {
        scale = 4;
    }
    float shunt_cal = SHUNT_CAL;
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

float ina228_voltage(ina228_config *config) {
    uint8_t regReadbusVoltage[3];
    int result = reg_read(config->i2c, INA228_ADDRESS, INA228_VBUS, regReadbusVoltage, 3);
    if (result < 0) {
        printf("Failed to read bus voltage\n");
        return -1.0f;
    }

    int32_t rawVoltage24 = (regReadbusVoltage[0] << 16) |
                           (regReadbusVoltage[1] << 8)  |
                           (regReadbusVoltage[2]);

    // Right-shift 4 bits since VBUS is a 20-bit value in bits 23:4
    return (float)((uint32_t)rawVoltage24 >> 4) * 195.3125f / 1e6f;
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

float ina228_current(ina228_config *config) { 
    uint8_t regReadCurrent[3];
    int32_t rawCurrent = 0;

    if (reg_read(config->i2c, INA228_ADDRESS, INA228_CURRENT, regReadCurrent, 3) < 0) {
        printf("Failed to read current register\n");
        return -1.0f;
    }

    // Assemble 24-bit signed value
    rawCurrent = (regReadCurrent[0] << 16) |
                 (regReadCurrent[1] << 8)  |
                 (regReadCurrent[2]);

    // Sign-extend if negative
    if (rawCurrent & 0x800000) {
        rawCurrent |= 0xFF000000;
    }

    float current = -rawCurrent * CURRENT_LSB;  // in Amps
    return current * 1000.0f;  // convert to mA if needed
}

uint32_t ina228_current_raw(ina228_config *config) { 
    uint8_t regReadCurrent[3];
    if (reg_read(config->i2c, INA228_ADDRESS, INA228_CURRENT, regReadCurrent, 3) < 0)
        return 0;

    int32_t rawCurrent = (regReadCurrent[0] << 16) |
                         (regReadCurrent[1] << 8)  |
                         (regReadCurrent[2]);

    // Sign-extend
    if (rawCurrent & 0x800000) {
        rawCurrent |= 0xFF000000;
    }

    return (uint32_t)rawCurrent;
}



int reg_read(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {
    if (nbytes < 1) return -1;

    int write_result = i2c_write_blocking(i2c, addr, &reg, 1, true);
    if (write_result < 0) {
        printf("I2C reg address write failed for reg 0x%02X\n", reg);
        return write_result;
    }

    int read_result = i2c_read_blocking(i2c, addr, buf, nbytes, false);
    if (read_result < 0) {
        printf("I2C read failed from reg 0x%02X\n", reg);
    }

    return read_result;
}

int ina228_reset(ina228_config *config){
    uint8_t buf[2];
    buf[0] = (INA228_CFG_RST >> 8) & 0xFF;
    buf[1] = INA228_CFG_RST & 0xFF;

    int result = reg_write(config->i2c, INA228_ADDRESS, INA228_CONFIG, buf, 2);
    if (result < 0) {
        printf("INA228 reset failed\n");
        return -1;
    }

    sleep_ms(2); // short wait for reset to complete
    return 0;
}

int reg_write(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {
    if (nbytes < 1) return -1; // invalid input

    uint8_t msg[nbytes + 1];
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    int result = i2c_write_blocking(i2c, addr, msg, nbytes + 1, false);
    if (result < 0) {
        printf("I2C write failed to address 0x%02X, reg 0x%02X\n", addr, reg);
    }

    return result;
}