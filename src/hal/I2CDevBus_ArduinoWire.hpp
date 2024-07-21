#ifndef __I2CDEVBUS_ARDUINOWIRE_HPP__
#define __I2CDEVBUS_ARDUINOWIRE_HPP__

#include "i2cdevbus.hpp"
#include <Wire.h>

inline __attribute__((always_inline)) void i2cdev_platform_sleep_us(uint32_t us) {
    if (us > 1000) {
        // delay in ms, then in us
        delay(us / 1000);
        delayMicroseconds(us % 1000);
    } else {
        delayMicroseconds(us);
    }
}

class ArduinoI2CDevBus : public I2CDevBus {
public:
    explicit ArduinoI2CDevBus(TwoWire* wire = &Wire) : wire_(wire) {}

private:
    TwoWire* wire_;

    auto readReg8(
            uint8_t devAddr,
            uint8_t regAddr,
            size_t length,
            uint8_t* data,
            uint16_t timeout = DEFAULT_READ_TIMEOUT_MS
    ) -> i2cdev_result_t override
    {
        I2CDEVLIB_LOG_D("readReg8: devAddr=0x%02X, regAddr=0x%02X, data=%s", devAddr, regAddr, i2cdevlib::hexdump(data, length));

        this->wire_->beginTransmission(devAddr);
        this->wire_->write(regAddr);
        this->wire_->endTransmission();

#if defined(ARDUINO_ARCH_AVR)
        this->wire_->requestFrom(devAddr, static_cast<uint8_t>(length), 1);
#else
        this->wire_->requestFrom(devAddr, length, true);
#endif

        uint8_t received = 0;
        const auto start = millis();

        while (this->wire_->available() && (timeout == 0 || millis() - start < timeout)) {
            data[received++] = this->wire_->read();

            // Return if we have received all the bytes we need
            if (received == length) {
                return I2CDEV_RESULT_OK;
            }
        }

        if (timeout > 0 && millis() - start >= timeout && received < length) {
            return I2CDEV_RESULT_ERROR;
        }

        return I2CDEV_RESULT_OK;
    }

    auto readReg16(
            uint8_t devAddr,
            uint8_t regAddr,
            size_t length,
            uint16_t* data,
            uint16_t timeout = DEFAULT_READ_TIMEOUT_MS
    ) -> i2cdev_result_t override
    {
        I2CDEVLIB_LOG_D("readReg16: devAddr=0x%02X, regAddr=0x%02X, data=%s", devAddr, regAddr, i2cdevlib::hexdump(data, length));

        this->wire_->beginTransmission(devAddr);
        this->wire_->write(regAddr);
        this->wire_->endTransmission();

#if defined(ARDUINO_ARCH_AVR)
        this->wire_->requestFrom(devAddr, static_cast<uint8_t>(length * 2), 1);
#else
        this->wire_->requestFrom(devAddr, length * 2, true);
#endif

        uint8_t received = 0;
        const auto start = millis();

        while (this->wire_->available() >= 2 && (timeout == 0 || millis() - start < timeout)) {
            data[received++] = this->wire_->read() << 8 | this->wire_->read();

            // Return if we have received all the bytes we need
            if (received == length * 2) {
                return I2CDEV_RESULT_OK;
            }
        }

        if (timeout > 0 && millis() - start >= timeout && received < length * 2) {
            return I2CDEV_RESULT_ERROR;
        }

        return I2CDEV_RESULT_OK;
    }

    auto writeReg8(uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t* data)
    -> i2cdev_result_t override
    {
        I2CDEVLIB_LOG_D("writeReg8: devAddr=0x%02X, regAddr=0x%02X, data=%s", devAddr, regAddr, i2cdevlib::hexdump(data, length));

        this->wire_->beginTransmission(devAddr);

        // Send address
        if (this->wire_->write(regAddr) != 1) {
            return I2CDEV_RESULT_ERROR;
        }

        // Send data
        if (this->wire_->write(data, length) != length) {
            return I2CDEV_RESULT_ERROR;
        }

        if (this->wire_->endTransmission() != 0) {
            return I2CDEV_RESULT_ERROR;
        }

        return I2CDEV_RESULT_OK;
    }

    auto writeReg16(uint8_t devAddr, uint8_t regAddr, size_t length, const uint16_t* data)
    -> i2cdev_result_t override
    {
        I2CDEVLIB_LOG_D("writeReg16: devAddr=0x%02X, regAddr=0x%02X, data=%s", devAddr, regAddr, i2cdevlib::hexdump(data, length));

        this->wire_->beginTransmission(devAddr);

        // Send address
        if (this->wire_->write(regAddr) != 1) {
            return I2CDEV_RESULT_ERROR;
        }

        // Send data
        for (size_t i = 0; i < length; i++) {
            // Send MSB
            if (this->wire_->write(static_cast<uint8_t>(data[i] >> 8)) != 1) {
                return I2CDEV_RESULT_ERROR;
            }

            // Send LSB
            if (this->wire_->write(static_cast<uint8_t>(data[i] & 0xFF)) != 1) {
                return I2CDEV_RESULT_ERROR;
            }
        }

        if (this->wire_->endTransmission() != 0) {
            return I2CDEV_RESULT_ERROR;
        }

        return I2CDEV_RESULT_OK;
    }
};

ArduinoI2CDevBus I2CDev = ArduinoI2CDevBus(&Wire);

#ifdef ARDUINO_ESP32_DEV
ArduinoI2CDevBus I2CDev1 = ArduinoI2CDevBus(&Wire1);
#endif // ARDUINO_ESP32_DEV

#ifndef I2CDEV_DEFAULT_BUS
#define I2CDEV_DEFAULT_BUS I2CDev
#endif // I2CDEV_DEFAULT_BUS

#endif // __I2CDEVBUS_ARDUINOWIRE_HPP__