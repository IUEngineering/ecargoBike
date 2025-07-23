#ifndef INA228_H
#define INA228_H

#include "hardware/i2c.h"


// Hardware registers - using #define
#define INA228_ADDRESS          _u(0x40)
#define INA228_CONFIG           _u(0x00)
#define INA228_ADC_CONFIG       _u(0x01)
#define INA228_SHUNT_CAL        _u(0x02)
#define INA228_SHUNT_TEMPCO     _u(0x03)
#define INA228_VSHUNT           _u(0x04)
#define INA228_VBUS             _u(0x05)
#define INA228_DIETEMP          _u(0x06)
#define INA228_CURRENT          _u(0x07)
#define INA228_POWER            _u(0x08)
#define INA228_ENERGY           _u(0x09)
#define INA228_CHARGE           _u(0x0A)
#define INA228_DIAG_ALRT        _u(0x0B)
#define INA228_SOVL             _u(0x0C)
#define INA228_SUVL             _u(0x0D)
#define INA228_BOVL             _u(0x0E)
#define INA228_BUVL             _u(0x0F)
#define INA228_TEMP_LIMIT       _u(0x10)
#define INA228_PWR_LIMIT        _u(0x11)
#define INA228_MANUFACTURER_ID  _u(0x3E)
#define INA228_DEVICE_ID        _u(0x3F)
#define INA228_CFG_RST          _u(0x8000)


typedef struct {
	i2c_inst_t *i2c;
	uint sda;
	uint scl;
} ina228_config;

void ina228_init(ina228_config *config);
// uint8_t ina228_reg_read(ina228_config *config, uint8_t reg);
uint8_t ina228_reg_read(ina228_config *config, uint8_t reg);
void ina228_reg_write(ina228_config *config, uint8_t reg, uint16_t value);
void ina228_reset(ina228_config *config);

// void ina228_reg_write(ina228_config *config, uint8_t reg, uint16_t value);
// float ina228_voltage(struct ina228_config *config);
// float ina228_dietemp(struct ina228_config *config);
// float ina228_shuntvoltage(struct ina228_config *config);
// float ina228_current(struct ina228_config *config);
// float ina228_power(struct ina228_config *config);
// float ina228_energy(struct ina228_config *config);
// float ina228_charge(struct ina228_config *config);



#endif // INA228_H
