#ifndef __I2CDEVLIB_MAX170XX_HPP__
#define __I2CDEVLIB_MAX170XX_HPP__

#include "i2cdev/max170xx.h"
#include "i2cdev/common.hpp"

namespace i2cdev {
    class MAX170XX : public max170xx_dev_t {
      public:
    #ifdef I2CDEVLIB_DEFAULT_BUS
        MAX170XX(uint8_t addr = MAX1704X_I2CADDR_BASE, i2cdev_bus_t* bus = &I2CDEVLIB_DEFAULT_BUS)
#else
        MAX170XX(uint8_t addr, i2cdev_bus_t* bus)
#endif
        {
          this->addr = addr;
          this->bus = bus;
        }

        [[nodiscard]] auto check() -> i2cdev_result_t
        {
          return static_cast<i2cdev_result_t>(max170xx_check(this));
        }

        [[nodiscard]] inline auto quickStart() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170xx_quick_start(this));
        }

        [[nodiscard]] virtual auto readVoltage(float *voltage) -> i2cdev_result_t;

        auto getVoltage() -> Result<float>
        {
            float voltage;
            const auto ret = this->readVoltage(&voltage);
            return {voltage, ret};
        }

        [[nodiscard]] virtual auto readSoc(float *soc) -> i2cdev_result_t;

        auto getSoc() -> Result<float>
        {
            float soc;
            const auto ret = this->readSoc(&soc);
            return {soc, ret};
        }

        [[nodiscard]] inline auto readRawVoltage(uint16_t *voltage) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170xx_read_raw_voltage(this, voltage));
        }

        [[nodiscard]] inline auto readRawSoc(uint16_t *soc) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170xx_read_raw_soc(this, soc));
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t;
    };

    class MAX17043 : public MAX170XX {
      public:
        using MAX170XX::MAX170XX;

        [[nodiscard]] auto readVoltage(float *voltage) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max17043_read_voltage(this, voltage));
        }

        [[nodiscard]] auto readSoc(float *soc) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max17043_read_soc(this, soc));
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max17043_reset(this));
        }
    };

    class MAX17044 : public MAX170XX {
      public:
        using MAX170XX::MAX170XX;

        [[nodiscard]] inline auto readVoltage(float *voltage) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max17044_read_voltage(this, voltage));
        }

        [[nodiscard]] inline auto readSoc(float *soc) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max17044_read_soc(this, soc));
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max17044_reset(this));
        }
    };

    class MAX170x8 : public MAX170XX {
      public:
        using MAX170XX::MAX170XX;

        [[nodiscard]] inline auto readVoltage(float *voltage) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170x8_read_voltage(this, voltage));
        }

        [[nodiscard]] inline auto readSoc(float *soc) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170x8_read_soc(this, soc));
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170x8_reset(this));
        }
    };

    using MAX17048 = MAX170x8;
    using MAX17058 = MAX170x8;

    class MAX170x9 : public MAX170XX {
      public:
        using MAX170XX::MAX170XX;

        [[nodiscard]] inline auto readVoltage(float *voltage) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170x9_read_voltage(this, voltage));
        }

        [[nodiscard]] inline auto readSoc(float *soc) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170x9_read_soc(this, soc));
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(max170x9_reset(this));
        }
    };

    using MAX17049 = MAX170x9;
    using MAX17059 = MAX170x9;
}

#endif //__I2CDEVLIB_MAX170XX_HPP__
