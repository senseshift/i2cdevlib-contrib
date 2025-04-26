#ifndef I2CDEVLIB_PCA9685_HPP_
#define I2CDEVLIB_PCA9685_HPP_

#include "i2cdev/pca9685.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace i2cdev
{
    class PCA9685 : public pca9685_dev_t
    {
        uint32_t _oscillator_freq = PCA9685_OSCILLATOR_FREQ;
        uint8_t _prescale;

    public:
#ifdef I2CDEVLIB_DEFAULT_BUS
        PCA9685(uint8_t addr = PCA9685_I2CADDR_BASE, i2cdev_bus_t& bus = I2CDEVLIB_DEFAULT_BUS)
#else
        PCA9685(uint8_t addr, i2cdev_bus_t& bus)
#endif
        {
            this->addr = addr;
            this->i2cdev = &bus;
        }

        void setOscillatorFrequency(uint32_t freq) { this->_oscillator_freq = freq; }

        [[nodiscard]] auto reset() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(pca9685_reset(this));
        }

        [[nodiscard]] auto sleep() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(pca9685_sleep(this));
        }

        [[nodiscard]] auto wakeup() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(pca9685_wakeup(this));
        }

        [[nodiscard]] auto restart() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(pca9685_restart(this));
        }

        [[nodiscard]] auto setFrequency(float freq) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(pca9685_set_frequency(this, freq, this->_oscillator_freq));
        }

        [[nodiscard]] auto setPrescale(uint8_t prescale, bool extClk = false) -> i2cdev_result_t
        {
            const auto result = pca9685_set_prescale(this, prescale, extClk);
            if (result == I2CDEV_RESULT_OK)
            {
                this->_prescale = prescale;
            }

            return static_cast<i2cdev_result_t>(result);
        }

        [[nodiscard]] auto readPrescale() -> i2cdev_result_t
        {
            uint8_t prescale;
            const auto result = pca9685_read_prescale(this, &prescale);
            if (result == I2CDEV_RESULT_OK)
            {
                this->_prescale = prescale;
            }

            return static_cast<i2cdev_result_t>(result);
        }

        [[nodiscard]] auto setChannelOnOff(uint8_t led, int16_t on_value, int16_t off_value) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(pca9685_set_channel_on_off(this, led, on_value, off_value));
        }

        [[nodiscard]] auto setChannel(uint8_t led, uint16_t value, bool invert = false) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(pca9685_set_channel(this, led, value, invert));
        }

        [[nodiscard]] auto setChannel(uint8_t led, float value, bool invert = false) -> i2cdev_result_t
        {
            return this->setChannel(led, static_cast<uint16_t>(value * 4095.0f), invert);
        }

        /**
         * @brief Arduino-style API for Servos
         *
         * @param [in] channel The LED number (0-15)
         * @param [in] microseconds The pulse length in microseconds
         *
         * @retval I2CDEV_RESULT_OK if the operation was successful
         */
        [[nodiscard]] auto writeMicroseconds(uint8_t channel, uint16_t microseconds) -> i2cdev_result_t
        {
            auto ticks = static_cast<uint16_t>(
                microseconds / (1e6f / _oscillator_freq * (_prescale + 1))
            );

            return this->setChannel(channel, ticks);
        }
    };
}

#endif // I2CDEVLIB_PCA9685_HPP_
