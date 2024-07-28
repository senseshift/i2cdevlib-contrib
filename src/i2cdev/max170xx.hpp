#ifndef __I2CDEVLIB_MAX170XX_HPP__
#define __I2CDEVLIB_MAX170XX_HPP__

#include "i2cdev/max170xx.h"
#include "i2cdevbus.hpp"

namespace i2cdev {
    class MAX170XX {
      public:
#ifdef I2CDEV_DEFAULT_BUS
        MAX170XX(uint8_t addr = MAX1704X_I2CADDR_BASE, I2CDevBus& bus = I2CDEV_DEFAULT_BUS) : _addr(addr), _bus(bus) {}
#else
        MAX170XX(uint8_t addr, I2CDevBus& bus) : _addr(addr), _bus(bus) {}
#endif

        [[nodiscard]] auto check() -> i2cdev_result_t;

        [[nodiscard]] inline auto quickStart() -> i2cdev_result_t;

      protected:
        uint8_t _addr;
        I2CDevBus& _bus;
    };

    class MAX17043 : public MAX170XX {
      public:
        using MAX170XX::MAX170XX;

        inline auto readVoltage() -> float
        {
            uint16_t data;
            if (this->_bus.readReg16(this->_addr, MAX170XX_REG_VCELL, &data) != I2CDEV_RESULT_OK) {
                return 0.0f;
            }

            return (data >> 4) / 800.0f;
        }

        inline auto readSoc() -> float
        {
            uint16_t data;
            if (this->_bus.readReg16(this->_addr, MAX170XX_REG_SOC, &data) != I2CDEV_RESULT_OK) {
                return 0.0f;
            }

            return ((data & 0xFF00) >> 8) + ((data & 0x00FF) / 256.0f);
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return this->_bus.writeReg16(this->_addr, MAX1704X_REG_COMMAND, static_cast<uint16_t>(MAX1704_COMMAND_POR_43_44));
        }
    };

    class MAX17044 : public MAX170XX {
      public:
        using MAX170XX::MAX170XX;

        inline auto readVoltage() -> float
        {
            uint16_t data;
            if (this->_bus.readReg16(this->_addr, MAX170XX_REG_VCELL, &data) != I2CDEV_RESULT_OK) {
                return 0.0f;
            }

            return (data >> 4) / 400.0f;
        }

        inline auto readSoc() -> float
        {
            uint16_t data;
            if (this->_bus.readReg16(this->_addr, MAX170XX_REG_SOC, &data) != I2CDEV_RESULT_OK) {
                return 0.0f;
            }

            return ((data & 0xFF00) >> 8) + ((data & 0x00FF) / 256.0f);
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return this->_bus.writeReg16(this->_addr, MAX1704X_REG_COMMAND, static_cast<uint16_t>(MAX1704_COMMAND_POR_43_44));
        }
    };

    class MAX170x8 : public MAX170XX {
      public:
        using MAX170XX::MAX170XX;

        inline auto readVoltage() -> float
        {
            uint16_t data;
            if (this->_bus.readReg16(this->_addr, MAX170XX_REG_VCELL, &data) != I2CDEV_RESULT_OK) {
                return 0.0f;
            }

            return data * 5.0f / 64000.0f;
        }

        inline auto readSoc() -> float
        {
            uint16_t data;
            if (this->_bus.readReg16(this->_addr, MAX170XX_REG_SOC, &data) != I2CDEV_RESULT_OK) {
                return 0.0f;
            }

            return data / 256.0f;
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return this->_bus.writeReg16(this->_addr, MAX1704X_REG_COMMAND, static_cast<uint16_t>(MAX1704_COMMAND_POR_X8_X9));
        }
    };

    using MAX17048 = MAX170x8;
    using MAX17058 = MAX170x8;

    class MAX170x9 : public MAX170XX {
      public:
        using MAX170XX::MAX170XX;

        inline auto readVoltage() -> float
        {
            uint16_t data;
            if (this->_bus.readReg16(this->_addr, MAX170XX_REG_VCELL, &data) != I2CDEV_RESULT_OK) {
                return 0.0f;
            }

            return data * 5.0f / 32000.0f;
        }

        inline auto readSoc() -> float
        {
            uint16_t data;
            if (this->_bus.readReg16(this->_addr, MAX170XX_REG_SOC, &data) != I2CDEV_RESULT_OK) {
                return 0.0f;
            }

            return data / 256.0f;
        }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return this->_bus.writeReg16(this->_addr, MAX1704X_REG_COMMAND, static_cast<uint16_t>(MAX1704_COMMAND_POR_X8_X9));
        }
    };

    using MAX17049 = MAX170x9;
    using MAX17059 = MAX170x9;

    inline auto MAX170XX::check() -> i2cdev_result_t {
        uint16_t data;
        i2cdev_result_t result;

        // wait for the device to be ready
        for (int i = 0; i < 5; i++) {
            result = this->_bus.readReg16(this->_addr, MAX170XX_REG_VERSION, &data);
            if (result == I2CDEV_RESULT_OK) {
                break;
            }

            i2cdev_platform_sleep_us(10000);
        }

        return result;
    }

    inline auto MAX170XX::quickStart() -> i2cdev_result_t {
        return this->_bus.writeReg16(this->_addr, MAX170XX_REG_MODE, static_cast<uint16_t>(MAX1704X_MODE_QUICKSTART));
    }
}

#endif //__I2CDEVLIB_MAX170XX_HPP__
