#ifndef __I2CDEVLIB_PCA9685_HPP__
#define __I2CDEVLIB_PCA9685_HPP__

#include "i2cdev/pca9685.h"
#include "i2cdevbus.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace i2cdev {
    class PCA9685 {
      public:
#ifdef I2CDEV_DEFAULT_BUS
        PCA9685(uint8_t addr = PCA9685_I2CADDR_BASE, I2CDevBus* bus = &I2CDEV_DEFAULT_BUS) : _addr(addr), _bus(bus) {}
#else
        PCA9685(uint8_t addr, I2CDevBus* bus) : _addr(addr), _bus(bus) {}
#endif

        inline void setOscillatorFrequency(uint32_t freq) { this->_oscillator_freq = freq; }

        [[nodiscard]] inline auto reset() -> i2cdev_result_t
        {
            return this->_bus->writeReg8(
                this->_addr,
                PCA9685_REG_MODE1,
                static_cast<uint8_t>(PCA9685_MODE1_RESTART)
            );
        }

        [[nodiscard]] inline auto sleep() -> i2cdev_result_t
        {
            return this->_bus->updateReg16(
                this->_addr,
                PCA9685_REG_MODE1,
                PCA9685_MODE1_SLEEP,
                PCA9685_MODE1_SLEEP
            );
        }

        [[nodiscard]] inline auto wakeup() -> i2cdev_result_t
        {
            return this->_bus->updateReg16(
                this->_addr,
                PCA9685_REG_MODE1,
                PCA9685_MODE1_SLEEP,
                0
            );
        }

#ifdef ARDUINO
        [[nodiscard]] auto restart(void (*sleepUs)(uint32_t) = &delayMicroseconds) -> i2cdev_result_t
#else
        [[nodiscard]] auto restart(void (*sleepUs)(uint32_t)) -> i2cdev_result_t
#endif
        {
            uint8_t mode1;

            auto result = this->_bus->readReg8(this->_addr, PCA9685_REG_MODE1, &mode1);
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            // Check if the RESTART bit is set
            if ((mode1 & PCA9685_MODE1_RESTART) == 0) {
                return I2CDEV_RESULT_OK;
            }

            result = this->wakeup();
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            sleepUs(500);

            result = this->reset();

            return result;
        }

        [[nodiscard]] inline auto setFrequency(float freq) -> i2cdev_result_t
        {
            freq = (freq > PCA9685_FREQ_MAX) ? PCA9685_FREQ_MAX : (freq < PCA9685_FREQ_MIN) ? PCA9685_FREQ_MIN : freq;
            auto prescale = static_cast<uint8_t>(((this->_oscillator_freq / (4096.0 * freq)) + 0.5) - 1);

            return this->setPrescale(prescale);
        }

        [[nodiscard]] auto setPrescale(uint8_t prescale, bool extClk = false) -> i2cdev_result_t
        {
            if (prescale < PCA9685_PRESCALE_MIN) {
                return I2CDEV_RESULT_ERROR;
            }

            uint8_t mode1;
            auto result = this->_bus->readReg8(this->_addr, PCA9685_REG_MODE1, &mode1);
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            mode1 = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
            if (extClk) {
                mode1 |= PCA9685_MODE1_EXTCLK;
            }

            result = this->_bus->writeReg8(this->_addr, PCA9685_REG_MODE1, mode1);
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            result = this->_bus->writeReg8(this->_addr, PCA9685_REG_PRESCALE, prescale);
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_prescale = prescale;

            mode1 = (mode1 & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART | PCA9685_MODE1_AI;

            return this->_bus->writeReg8(this->_addr, PCA9685_REG_MODE1, mode1);
        }

        [[nodiscard]] auto readPrescale() -> i2cdev_result_t
        {
            uint8_t prescale;
            auto result = this->_bus->readReg8(this->_addr, PCA9685_REG_PRESCALE, &prescale);
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            this->_prescale = prescale;

            return result;
        }

        [[nodiscard]] auto setPwm(uint8_t led, int16_t on_value, int16_t off_value) -> i2cdev_result_t
        {
            if (led > 15) {
                return I2CDEV_RESULT_ERROR;
            }

            uint8_t data[4] = {
                static_cast<uint8_t>(on_value & 0xFF),
                static_cast<uint8_t>(on_value >> 8),
                static_cast<uint8_t>(off_value & 0xFF),
                static_cast<uint8_t>(off_value >> 8)
            };

            return this->_bus->writeReg8(
                this->_addr,
                PCA9685_REG_LED0_ON_L + (led * 4),
                4,
                data
            );
        }

        [[nodiscard]] auto setPin(uint8_t led, uint16_t value, bool invert = false) -> i2cdev_result_t
        {
            // Clamp value to 12 bits
            value = (value < 0) ? 0 : (value > 4095) ? 4095 : value;

            if (invert) {
                if (value == 0) {
                    return this->setPwm(led, 4096, 0);
                }
                if (value == 4095) {
                    return this->setPwm(led, 0, 4096);
                }
                return this->setPwm(led, 0, 4095 - value);
            } else {
                if (value == 4095) {
                    return this->setPwm(led, 4096, 0);
                }
                if (value == 0) {
                    return this->setPwm(led, 0, 4096);
                }
                return this->setPwm(led, 0, value);
            }
        }

        [[nodiscard]] inline auto writeMicroseconds(uint8_t led, uint16_t value) -> i2cdev_result_t
        {
            return this->setPin(
                led,
                static_cast<uint16_t>(
                    value / (1000000.0 / this->_oscillator_freq * (this->_prescale + 1))
                )
            );
        }

      private:
        uint8_t _addr;
        I2CDevBus* _bus;

        uint32_t _oscillator_freq = 25000000;
        uint8_t _prescale;
    };
}

#endif // __I2CDEVLIB_PCA9685_HPP__