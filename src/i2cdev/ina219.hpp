#ifndef __I2CDEVLIB_INA219_HPP__
#define __I2CDEVLIB_INA219_HPP__

#include "i2cdev/ina219.h"
#include "i2cdevbus.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace i2cdev {
    class INA219 {
      private:
        uint8_t _addr;
        I2CDevBus* _bus;

        i2cdev_result_t _status;

        uint16_t _config;

        uint32_t _calValue;
        uint32_t _currentDivider_mA;
        float _powerMultiplier_mW;

      public:
#ifdef I2CDEV_DEFAULT_BUS
        INA219(uint8_t addr = INA219_I2CADDR_BASE, I2CDevBus* bus = &I2CDEV_DEFAULT_BUS) : _addr(addr), _bus(bus) {}
#else
        INA219(uint8_t addr, I2CDevBus *bus) : _addr(addr), _bus(bus) {}
#endif

        inline auto readShuntVoltage() -> float
        {
            return this->readRawShuntVoltage() * 0.01f;
        }

        inline auto readBusVoltage() -> float
        {
            return this->readRawBusVoltage() * 0.001f;
        }

        inline auto readCurrent() -> float
        {
            return this->readRawCurrent() / static_cast<float>(this->_currentDivider_mA);
        }

        inline auto readPower() -> float
        {
            return this->readRawPower() * this->_powerMultiplier_mW;
        }

        inline auto sleep() -> i2cdev_result_t
        {
            // write to register without updating internal mode, so we can wake up later
            return this->_bus->writeReg16(
                this->_addr,
                INA219_REG_CONFIG,
                static_cast<uint16_t>(INA219_CONFIG_MODE_POWERDOWN)
            );
        }

        inline auto wakeup() -> i2cdev_result_t
        {
            return this->setConfig(this->_config);
        }

        auto setCalibration_32V_2A() -> i2cdev_result_t
        {
            uint16_t calValue = 0x1000;
            auto result = this->_bus->writeReg16(
                this->_addr,
                INA219_REG_CALIBRATION,
                static_cast<uint16_t>(calValue)
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_calValue = calValue;

            result = this->setConfig(
                (INA219_CONFIG_BUS_VOLTAGE_RANGE_32V | INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BUS_ADC_12BIT | INA219_CONFIG_SHUNT_ADC_12BIT | INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS)
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_currentDivider_mA = 10;
            this->_powerMultiplier_mW = 2.0f;

            return result;
        }

        auto setCalibration_32V_1A() -> i2cdev_result_t
        {
            uint16_t calValue = 0x2800;
            auto result = this->_bus->writeReg16(
                this->_addr,
                INA219_REG_CALIBRATION,
                static_cast<uint16_t>(calValue)
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_calValue = calValue;

            result = this->setConfig(
                (INA219_CONFIG_BUS_VOLTAGE_RANGE_32V | INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BUS_ADC_12BIT | INA219_CONFIG_SHUNT_ADC_12BIT | INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS)
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_currentDivider_mA = 25;
            this->_powerMultiplier_mW = 0.8f;

            return result;
        }

        auto setCalibration_16V_400mA() -> i2cdev_result_t
        {
            uint16_t calValue = 0x2000;
            auto result = this->_bus->writeReg16(
                this->_addr,
                INA219_REG_CALIBRATION,
                static_cast<uint16_t>(calValue)
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_calValue = calValue;

            result = this->setConfig(
                (INA219_CONFIG_BUS_VOLTAGE_RANGE_32V | INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BUS_ADC_12BIT | INA219_CONFIG_SHUNT_ADC_12BIT | INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS)
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_currentDivider_mA = 20;
            this->_powerMultiplier_mW = 1.0f;

            return result;
        }

      private:

        auto setConfig(uint16_t config) -> i2cdev_result_t
        {
            auto result = this->_bus->writeReg16(
                this->_addr,
                INA219_REG_CONFIG,
                config
            );

            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_config = config;

            return result;
        }

        auto readRawShuntVoltage() -> int16_t
        {
            uint16_t value;

            this->_status = this->_bus->readReg16(this->_addr, INA219_REG_SHUNT_VOLTAGE, 1, &value);
            if (this->_status != I2CDEV_RESULT_OK) {
                return 0;
            }

            return static_cast<int16_t>(value);
        }

        auto readRawBusVoltage() -> int16_t
        {
            uint16_t value;

            this->_status = this->_bus->readReg16(this->_addr, INA219_REG_BUS_VOLTAGE, 1, &value);
            if (this->_status != I2CDEV_RESULT_OK) {
                return 0;
            }

            // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
            return static_cast<int16_t>((value >> 3) * 4);
        }

        auto readRawPower() -> int16_t
        {
            uint16_t value;

            // Sometimes a sharp load will reset the INA219, which will reset the cal register,
            // meaning CURRENT and POWER will not be available...
            // Avoid this by always setting a INA219_REG_CALIBRATION even if it's an unfortunate extra step
            this->_status = this->_bus->writeReg16(this->_addr, INA219_REG_CALIBRATION, static_cast<uint16_t>(this->_calValue));
            if (this->_status != I2CDEV_RESULT_OK) {
                return 0;
            }

            this->_status = this->_bus->readReg16(this->_addr, INA219_REG_POWER, 1, &value);
            if (this->_status != I2CDEV_RESULT_OK) {
                return 0;
            }

            return static_cast<int16_t>(value);
        }

        auto readRawCurrent() -> int16_t
        {
            uint16_t value;

            // Sometimes a sharp load will reset the INA219, which will reset the cal register,
            // meaning CURRENT and POWER will not be available...
            // Avoid this by always setting a INA219_REG_CALIBRATION even if it's an unfortunate extra step
            this->_status = this->_bus->writeReg16(this->_addr, INA219_REG_CALIBRATION, static_cast<uint16_t>(this->_calValue));
            if (this->_status != I2CDEV_RESULT_OK) {
                return 0;
            }

            this->_status = this->_bus->readReg16(this->_addr, INA219_REG_CURRENT, 1, &value);
            if (this->_status != I2CDEV_RESULT_OK) {
                return 0;
            }

            return static_cast<int16_t>(value);
        }
    };
}

#endif //__I2CDEVLIB_INA219_HPP__
