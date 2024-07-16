#ifndef __I2CDEVBUS_H__
#define __I2CDEVBUS_H__

#include "stdint.h"

/// 1000ms default read timeout (modify with `I2CDevBus::DEFAULT_READ_TIMEOUT_MS = [ms];`)
#ifndef I2CDEV_DEFAULT_READ_TIMEOUT_MS
#define I2CDEV_DEFAULT_READ_TIMEOUT_MS 1000
#endif

/// Set true to always update the I2C device registers (even if the value hasn't changed)
#ifndef I2CDEV_ALWAYS_UPDATE
#define I2CDEV_ALWAYS_UPDATE false
#endif

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct i2cdev_bus_handle_t i2cdev_bus_handle_t;

typedef enum {
    I2CDEV_RESULT_OK = 0,
    I2CDEV_RESULT_ERROR = -1,
    I2CDEV_RESULT_WOULD_BLOCK = -2,
} i2cdev_result_t;

typedef uint8_t i2cdev_dev_addr_t;
typedef uint8_t i2cdev_reg_addr_t;

#ifdef __cplusplus
};
#endif // __cplusplus

#endif // __I2CDEVBUS_H__