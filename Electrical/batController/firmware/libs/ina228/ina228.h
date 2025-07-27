#ifndef INA228_H
#define INA228_H

#include "hardware/i2c.h"


// Hardware registers - using #define
#define INA228_ADDRESS          0x40
#define INA228_CONFIG           0x00
#define INA228_ADC_CONFIG       0x01
#define INA228_SHUNT_CAL        0x02
#define INA228_SHUNT_TEMPCO     0x03
#define INA228_VSHUNT           0x04
#define INA228_VBUS             0x05
#define INA228_DIETEMP          0x06
#define INA228_CURRENT          0x07
#define INA228_POWER            0x08
#define INA228_ENERGY           0x09
#define INA228_CHARGE           0x0A
#define INA228_DIAG_ALRT        0x0B
#define INA228_SOVL             0x0C
#define INA228_SUVL             0x0D
#define INA228_BOVL             0x0E
#define INA228_BUVL             0x0F
#define INA228_TEMP_LIMIT       0x10
#define INA228_PWR_LIMIT        0x11
#define INA228_MANUFACTURER_ID  0x3E
#define INA228_DEVICE_ID        0x3F
#define INA228_CFG_RST          0x8000


typedef struct {
	i2c_inst_t *i2c;
	uint sda;
	uint scl;
	float voltage;
	float dietemp;
	float shuntvoltage;
} ina228_config;

int ina228_init(ina228_config *config);
int ina228_reset(ina228_config *config);
float ina228_voltage(ina228_config *config);
uint32_t ina228_voltage_raw(ina228_config *config);
float ina228_current(ina228_config *config);
uint16_t ina228_current_raw(ina228_config *config);
int reg_write(i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes);

int reg_read(   i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);
void ina228_updateShuntCalRegister(ina228_config *config);
uint8_t ina228_getADCRange(ina228_config *config);


#endif // INA228_H
