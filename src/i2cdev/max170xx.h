#ifndef __I2CDEVLIB_MAX170XX_H__
#define __I2CDEVLIB_MAX170XX_H__

#include "i2cdevbus.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define MAX1704X_I2CADDR_BASE (0x36)

typedef enum max170xx_reg_t {
    /// Cell voltage
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

    MAX1704X_REG_COMMAND = 0xFE
} max1704x_reg_t;

#define MAX1704X_MODE_QUICKSTART (0x4000)

#define MAX1704_COMMAND_POR_43_44 (0x0054)
#define MAX1704_COMMAND_POR_X8_X9 (0x5400)

#ifdef __cplusplus
};
#endif // __cplusplus

#endif //__I2CDEVLIB_MAX170XX_H__
