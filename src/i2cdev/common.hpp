#ifndef I2CDEVLIB_COMMON_HPP
#define I2CDEVLIB_COMMON_HPP

#include "i2cdevbus.h"

namespace i2cdev
{
    template <typename T, typename E = i2cdev_result_t>
    struct Result
    {
        T value;
        E error;

        bool ok() const { return error == I2CDEV_RESULT_OK; }
    };
}

#endif // I2CDEVLIB_COMMON_HPP
