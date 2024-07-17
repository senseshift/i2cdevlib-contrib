#ifndef __I2CDEVLIB_INA219_H__
#define __I2CDEVLIB_INA219_H__

#include "i2cdevbus.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define INA219_I2CADDR_BASE (0x40) // A0 and A1 tied to GND
#define INA219_I2CADDR_GET(A0, A1) (0x40 | ((A0) != 0 ? 0x01 : 0x00) | ((A1) != 0 ? 0x04 : 0x00))

#define INA219_CREATE_MASK(shift, length) (((1 << (length)) - 1) << (shift))

typedef enum ina219_reg_t {
    /// Configuration Register
    INA219_REG_CONFIG = (0x00),
    /// Shunt Voltage Register
    INA219_REG_SHUNT_VOLTAGE = (0x01),
    /// Bus Voltage Register
    INA219_REG_BUS_VOLTAGE = (0x02),
    /// Power Register
    INA219_REG_POWER = (0x03),
    /// Current Register
    INA219_REG_CURRENT = (0x04),
    /// Calibration Register
    INA219_REG_CALIBRATION = (0x05)
} ina219_reg_t;

#define INA219_CONFIG_DEFAULT (0x399F) // Datasheet 8.6.1

#define INA219_CONFIG_RESET_SHIFT (15)
#define INA219_CONFIG_RESET_LENGTH (1)
#define INA219_CONFIG_RESET INA219_CREATE_MASK(INA219_CONFIG_RESET_SHIFT, INA219_CONFIG_RESET_LENGTH)

#define INA219_CONFIG_BUS_VOLTAGE_RANGE_SHIFT (13)
#define INA219_CONFIG_BUS_VOLTAGE_RANGE_LENGTH (1)
#define INA219_CONFIG_BUS_VOLTAGE_RANGE_MASK INA219_CREATE_MASK(INA219_CONFIG_BUS_VOLTAGE_RANGE_SHIFT, INA219_CONFIG_BUS_VOLTAGE_RANGE_LENGTH)
typedef enum ina219_bus_voltage_range_t {
    /// 0-16V Range
    INA219_CONFIG_BUS_VOLTAGE_RANGE_16V = (0x0000), // 0b0000000000000000
    /// 0-32V Range
    INA219_CONFIG_BUS_VOLTAGE_RANGE_32V = (0x2000) // 0b0010000000000000
} ina219_bus_voltage_range_t;

#define INA219_CONFIG_GAIN_SHIFT (11)
#define INA219_CONFIG_GAIN_LENGTH (2)
#define INA218_CONFIG_GAIN_MASK INA219_CREATE_MASK(INA219_CONFIG_GAIN_SHIFT, INA219_CONFIG_GAIN_LENGTH)
typedef enum ina219_gain_t {
    /// Gain 1, 40mV Range
    INA219_CONFIG_GAIN_1_40MV = (0x0000), // 0b0000000000000000
    /// Gain 2, 80mV Range
    INA219_CONFIG_GAIN_2_80MV = (0x0800), // 0b0000100000000000
    /// Gain 4, 160mV Range
    INA219_CONFIG_GAIN_4_160MV = (0x1000), // 0b0001000000000000
    /// Gain 8, 320mV Range
    INA219_CONFIG_GAIN_8_320MV = (0x1800) // 0b0001100000000000
} ina219_gain_t;

#define INA219_CONFIG_BADC_SHIFT (7)
#define INA219_CONFIG_BADC_LENGTH (4)
#define INA219_CONFIG_BUS_ADC_MASK INA219_CREATE_MASK(INA219_CONFIG_BADC_SHIFT, INA219_CONFIG_BADC_LENGTH)
typedef enum ina219_bus_adc_t {
    /// 9-bit bus resolution (0-511)
    INA219_CONFIG_BUS_ADC_9BIT = (0x0000),
    /// 10-bit bus resolution (0-1023)
    INA219_CONFIG_BUS_ADC_10BIT = (0x0080),
    /// 11-bit bus resolution (0-2047)
    INA219_CONFIG_BUS_ADC_11BIT = (0x0100),
    /// 12-bit bus resolution (0-4095)
    INA219_CONFIG_BUS_ADC_12BIT = (0x0180),
    /// 2 x 12-bit bus samples averaged together
    INA219_CONFIG_BUS_ADC_12BIT_2S = (0x0480),
    /// 4 x 12-bit bus samples averaged together
    INA219_CONFIG_BUS_ADC_12BIT_4S = (0x0500),
    /// 8 x 12-bit bus samples averaged together
    INA219_CONFIG_BUS_ADC_12BIT_8S = (0x0580),
    /// 16 x 12-bit bus samples averaged together
    INA219_CONFIG_BUS_ADC_12BIT_16S = (0x0600),
    /// 32 x 12-bit bus samples averaged together
    INA219_CONFIG_BUS_ADC_12BIT_32S = (0x0680),
    /// 64 x 12-bit bus samples averaged together
    INA219_CONFIG_BUS_ADC_12BIT_64S = (0x0700),
    /// 128 x 12-bit bus samples averaged together
    INA219_CONFIG_BUS_ADC_12BIT_128S = (0x0780)
} ina219_bus_adc_t;

#define INA219_CONFIG_SADC_SHIFT (3)
#define INA219_CONFIG_SADC_LENGTH (4)
#define INA219_CONFIG_SHUNT_ADC_MASK INA219_CREATE_MASK(INA219_CONFIG_SADC_SHIFT, INA219_CONFIG_SADC_LENGTH)
typedef enum ina219_shunt_adc_t {
    /// 9-bit shunt resolution (0-511)
    INA219_CONFIG_SHUNT_ADC_9BIT = (0x0000),
    /// 10-bit shunt resolution (0-1023)
    INA219_CONFIG_SHUNT_ADC_10BIT = (0x0008),
    /// 11-bit shunt resolution (0-2047)
    INA219_CONFIG_SHUNT_ADC_11BIT = (0x0010),
    /// 12-bit shunt resolution (0-4095)
    INA219_CONFIG_SHUNT_ADC_12BIT = (0x0018),
    /// 2 x 12-bit shunt samples averaged together
    INA219_CONFIG_SHUNT_ADC_12BIT_2S = (0x0048),
    /// 4 x 12-bit shunt samples averaged together
    INA219_CONFIG_SHUNT_ADC_12BIT_4S = (0x0050),
    /// 8 x 12-bit shunt samples averaged together
    INA219_CONFIG_SHUNT_ADC_12BIT_8S = (0x0058),
    /// 16 x 12-bit shunt samples averaged together
    INA219_CONFIG_SHUNT_ADC_12BIT_16S = (0x0060),
    /// 32 x 12-bit shunt samples averaged together
    INA219_CONFIG_SHUNT_ADC_12BIT_32S = (0x0068),
    /// 64 x 12-bit shunt samples averaged together
    INA219_CONFIG_SHUNT_ADC_12BIT_64S = (0x0070),
    /// 128 x 12-bit shunt samples averaged together
    INA219_CONFIG_SHUNT_ADC_12BIT_128S = (0x0078)
} ina219_shunt_adc_t;

#define INA219_CONFIG_MODE_SHIFT (0)
#define INA219_CONFIG_MODE_LENGTH (3)
#define INA219_CONFIG_MODE_MASK INA219_CREATE_MASK(INA219_CONFIG_MODE_SHIFT, INA219_CONFIG_MODE_LENGTH)
typedef enum ina219_mode_t {
    /// Power-Down
    INA219_CONFIG_MODE_POWERDOWN = (0x0000), // 0b0000000000000000
    /// Shunt Voltage, Triggered
    INA219_CONFIG_MODE_SHUNT_TRIGGERED = (0x0001), // 0b0000000000000001
    /// Bus Voltage, Triggered
    INA219_CONFIG_MODE_BUS_TRIGGERED = (0x0002), // 0b0000000000000010
    /// Shunt and Bus, Triggered
    INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED = (0x0003), // 0b0000000000000011
    /// ADC Off
    INA219_CONFIG_MODE_ADC_OFF = (0x0004), // 0b0000000000000100
    /// Shunt Voltage, Continuous
    INA219_CONFIG_MODE_SHUNT_CONTINUOUS = (0x0005), // 0b0000000000000101
    /// Bus Voltage, Continuous
    INA219_CONFIG_MODE_BUS_CONTINUOUS = (0x0006), // 0b0000000000000110
    /// Shunt and Bus, Continuous
    INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS = (0x0007) // 0b0000000000000111
} ina219_mode_t;

uint16_t ina219_config_set_bus_voltage_range(uint16_t config, ina219_bus_voltage_range_t range) {
    return (config & ~INA219_CONFIG_BUS_VOLTAGE_RANGE_MASK) | (static_cast<uint16_t>(range));
}

ina219_bus_voltage_range_t ina219_config_get_bus_voltage_range(uint16_t config) {
    return static_cast<ina219_bus_voltage_range_t>(config & INA219_CONFIG_BUS_VOLTAGE_RANGE_MASK);
}

uint16_t ina219_config_set_gain(uint16_t config, ina219_gain_t gain) {
    return (config & ~INA218_CONFIG_GAIN_MASK) | (static_cast<uint16_t>(gain));
}

ina219_gain_t ina219_config_get_gain(uint16_t config) {
    return static_cast<ina219_gain_t>(config & INA218_CONFIG_GAIN_MASK);
}

uint16_t ina219_config_set_bus_adc(uint16_t config, ina219_bus_adc_t adc) {
    return (config & ~INA219_CONFIG_BUS_ADC_MASK) | (static_cast<uint16_t>(adc));
}

ina219_bus_adc_t ina219_config_get_bus_adc(uint16_t config) {
    return static_cast<ina219_bus_adc_t>(config & INA219_CONFIG_BUS_ADC_MASK);
}

uint16_t ina219_config_set_shunt_adc(uint16_t config, ina219_shunt_adc_t adc) {
    return (config & ~INA219_CONFIG_SHUNT_ADC_MASK) | (static_cast<uint16_t>(adc));
}

ina219_shunt_adc_t ina219_config_get_shunt_adc(uint16_t config) {
    return static_cast<ina219_shunt_adc_t>(config & INA219_CONFIG_SHUNT_ADC_MASK);
}

uint16_t ina219_config_set_mode(uint16_t config, ina219_mode_t mode) {
    return (config & ~INA219_CONFIG_MODE_MASK) | (static_cast<uint16_t>(mode));
}

ina219_mode_t ina219_config_get_mode(uint16_t config) {
    return static_cast<ina219_mode_t>(config & INA219_CONFIG_MODE_MASK);
}

#ifdef __cplusplus
};
#endif // __cplusplus

#endif //__I2CDEVLIB_INA219_H__
