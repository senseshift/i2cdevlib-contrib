#ifndef __I2CDEVLIB_MAX170XX_H__
#define __I2CDEVLIB_MAX170XX_H__

#include "i2cdevbus.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define MAX1704X_I2CADDR_BASE (0x36)

typedef enum max170xx_reg {
    /**
     * Cell voltage
     */
    MAX170XX_REG_VCELL = 0x02,

    /// State of charge
    MAX170XX_REG_SOC = 0x04,
    MAX170XX_REG_MODE = 0x06,
    MAX170XX_REG_VERSION = 0x08,

    /// MAC17048/MAX17049 only
    MAX170XX_REG_HIBRT = 0x0A,
    MAX170XX_REG_CONFIG = 0x0C,

    /// MAC17048/MAX17049 only
    MAX170XX_REG_VALERT = 0x14,

    /// MAC17048/MAX17049 only
    MAX170XX_REG_CRATE = 0x16,

    /// MAC17048/MAX17049 only
    MAX170XX_REG_VRESET_ID = 0x18,
    MAX170XX_REG_CHIP_ID = 0x19,
    MAX170XX_REG_STATUS = 0x1A,

    MAX170XX_REG_COMMAND = 0xFE
} max170xx_reg_t;

#define MAX1704X_MODE_QUICKSTART (0x4000)

#define MAX1704X_COMMAND_POR_43_44 (0x0054)
#define MAX1704X_COMMAND_POR_X8_X9 (0x5400)

typedef struct max170xx_dev {
    i2cdev_bus_t* bus;
    uint8_t addr;
} max170xx_dev_t;

inline int max170xx_read_version(max170xx_dev_t* dev, uint16_t* version)
{
    return i2cdev_reg_read_u16_be(dev->bus, dev->addr, MAX170XX_REG_VERSION, version);
}

inline int max170xx_check(max170xx_dev_t* dev)
{
    uint16_t version;

    int ret = max170xx_read_version(dev, &version);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    // todo: actually check the version

    return I2CDEV_RESULT_OK;
}

inline int max170xx_quick_start(max170xx_dev_t* dev)
{
    return i2cdev_reg_write_u16_be(dev->bus, dev->addr, MAX170XX_REG_MODE, MAX1704X_MODE_QUICKSTART);
}

inline int max170xx_read_raw_voltage(max170xx_dev_t* dev, uint16_t* voltage)
{
    return i2cdev_reg_read_u16_be(dev->bus, dev->addr, MAX170XX_REG_VCELL, voltage);
}

inline int max170xx_read_raw_soc(max170xx_dev_t* dev, uint16_t* soc)
{
    return i2cdev_reg_read_u16_be(dev->bus, dev->addr, MAX170XX_REG_SOC, soc);
}

inline int max17043_read_voltage(max170xx_dev_t* dev, float* voltage)
{
    uint16_t raw_voltage;
    int ret = max170xx_read_raw_voltage(dev, &raw_voltage);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    *voltage = (raw_voltage >> 4) / 800.0f;

    return I2CDEV_RESULT_OK;
}

inline int max17043_read_soc(max170xx_dev_t* dev, float* soc)
{
    uint16_t raw_soc;
    int ret = max170xx_read_raw_soc(dev, &raw_soc);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    *soc = ((raw_soc & 0xFF00) >> 8) + ((raw_soc & 0x00FF) / 256.0f);

    return I2CDEV_RESULT_OK;
}

inline int max17043_reset(max170xx_dev_t* dev)
{
    return i2cdev_reg_write_u16_be(dev->bus, dev->addr, MAX170XX_REG_COMMAND, MAX1704X_COMMAND_POR_43_44);
}

inline int max17044_read_voltage(max170xx_dev_t* dev, float* voltage)
{
    uint16_t raw_voltage;
    int ret = max170xx_read_raw_voltage(dev, &raw_voltage);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    *voltage = (raw_voltage >> 4) / 400.0f;

    return I2CDEV_RESULT_OK;
}

inline int max17044_read_soc(max170xx_dev_t* dev, float* soc)
{
    uint16_t raw_soc;
    int ret = max170xx_read_raw_soc(dev, &raw_soc);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    *soc = ((raw_soc & 0xFF00) >> 8) + ((raw_soc & 0x00FF) / 256.0f);

    return I2CDEV_RESULT_OK;
}

inline int max17044_reset(max170xx_dev_t* dev)
{
    return i2cdev_reg_write_u16_be(dev->bus, dev->addr, MAX170XX_REG_COMMAND, MAX1704X_COMMAND_POR_43_44);
}

inline int max170x8_read_voltage(max170xx_dev_t* dev, float* voltage)
{
    uint16_t raw_voltage;
    int ret = max170xx_read_raw_voltage(dev, &raw_voltage);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    *voltage = raw_voltage * 5.0f / 64000.0f;

    return I2CDEV_RESULT_OK;
}

inline int max170x8_read_soc(max170xx_dev_t* dev, float* soc)
{
    uint16_t raw_soc;
    int ret = max170xx_read_raw_soc(dev, &raw_soc);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    *soc = raw_soc / 256.0f;

    return I2CDEV_RESULT_OK;
}

inline int max170x8_reset(max170xx_dev_t* dev)
{
    return i2cdev_reg_write_u16_be(dev->bus, dev->addr, MAX170XX_REG_COMMAND, MAX1704X_COMMAND_POR_X8_X9);
}

inline int max170x9_read_voltage(max170xx_dev_t* dev, float* voltage)
{
    uint16_t raw_voltage;
    int ret = max170xx_read_raw_voltage(dev, &raw_voltage);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    *voltage = raw_voltage * 5.0f / 64000.0f;

    return I2CDEV_RESULT_OK;
}

inline int max170x9_read_soc(max170xx_dev_t* dev, float* soc)
{
    uint16_t raw_soc;
    int ret = max170xx_read_raw_soc(dev, &raw_soc);
    if (ret != I2CDEV_RESULT_OK) {
        return ret;
    }

    *soc = raw_soc / 256.0f;

    return I2CDEV_RESULT_OK;
}

inline int max170x9_reset(max170xx_dev_t* dev)
{
    return i2cdev_reg_write_u16_be(dev->bus, dev->addr, MAX170XX_REG_COMMAND, MAX1704X_COMMAND_POR_X8_X9);
}

#ifdef __cplusplus
};
#endif // __cplusplus

#endif //__I2CDEVLIB_MAX170XX_H__
