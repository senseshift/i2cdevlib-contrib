#ifndef I2CDEVBUS_HAL_ARDUINO_HPP
#define I2CDEVBUS_HAL_ARDUINO_HPP

#if !defined(__cplusplus)
#error "This header requires a C++ compiler."
#endif // __cplusplus

#include "i2cdevbus.h"

#include <Arduino.h>
#include <Wire.h>

// #ifndef BUFFER_LENGTH
// #define BUFFER_LENGTH 32
// #endif // BUFFER_LENGTH
#ifndef I2CDEVLIB_BUFFER_LENGTH
# ifdef BUFFER_LENGTH
// As defined in Wire.h on Arduino AVR boards
#  define I2CDEVLIB_BUFFER_LENGTH BUFFER_LENGTH
# elif defined(I2C_BUFFER_LENGTH)
// As defined in Wire.h on ESP32 boards
#  define I2CDEVLIB_BUFFER_LENGTH I2C_BUFFER_LENGTH
# else
#  define I2CDEVLIB_BUFFER_LENGTH 32
# endif
#endif // I2CDEVLIB_BUFFER_LENGTH

#ifndef I2CDEVLIB_TRANSFER_TIMEOUT_MSEC
#define I2CDEVLIB_TRANSFER_TIMEOUT_MSEC 500
#endif // I2CDEVLIB_TRANSFER_TIMEOUT_MSEC

static inline int i2cdev_arduino_transfer(const i2cdev_bus_t *bus,
                                          const i2cdev_msg_t *msgs,
                                          size_t num_msgs,
                                          i2cdev_dev_addr_t addr) {
    if (!num_msgs) {
        return I2CDEV_RESULT_OK;
    }

    // The context should be a pointer to a TwoWire instance.
    TwoWire *wire = reinterpret_cast<TwoWire *>(bus->context);
    if (!wire) {
        return -I2CDEV_RESULT_ENODEV;
    }

    msgs[0].flags |= I2CDEV_MSG_RESTART;

    // Check for validity of all messages before transfer
    for (size_t i = 0; i < num_msgs; i++) {
        const i2cdev_msg *current = &msgs[i];
        if (current->len == 0) {
            return -I2CDEV_RESULT_EINVAL;
        }

        // 10-but address is not supported
        if (current->flags & I2CDEV_ADDR_10BIT) {
            return -I2CDEV_RESULT_ENOSYS;
        }

        if (i != num_msgs - 1) {
            const i2cdev_msg_t *next = &msgs[i + 1];

            // check if there is any stop event in the middle of the transaction
            if (current->flags & I2CDEV_MSG_STOP) {
                return -I2CDEV_RESULT_EINVAL;
            }

            // messages of different direction require restart event
            const bool currentIsRead = (current->flags & I2CDEV_MSG_READ) != 0;
            const bool nextIsRead = (next->flags & I2CDEV_MSG_READ) != 0;
            if (currentIsRead != nextIsRead && !(next->flags & I2CDEV_MSG_RESTART)) {
                return -I2CDEV_RESULT_EINVAL;
            }
        }
    }

    // Start the transfer
    for (size_t i = 0; i < num_msgs; i++) {
        const i2cdev_msg_t *current = &msgs[i];

        const bool isRead = (current->flags & I2CDEV_MSG_READ) != 0;
        const bool isWrite = (current->flags & I2CDEV_MSG_WRITE) != 0;

        const bool sendStop = (current->flags & I2CDEV_MSG_STOP) != 0;
        const bool restart = (current->flags & I2CDEV_MSG_RESTART) != 0;

        if (isWrite) {
            // WRITE in chunks
            for (size_t off = 0; off < current->len; off += I2CDEVLIB_BUFFER_LENGTH) {
                const size_t chunk = min((size_t) I2CDEVLIB_BUFFER_LENGTH, current->len - off);

                wire->beginTransmission((uint8_t) addr);

                const size_t written = wire->write(current->buf + off, chunk);
                if (written != chunk) {
                    return -I2CDEV_RESULT_EIO;
                }

                const int r = wire->endTransmission(restart ? false : sendStop);
                if (r != 0) {
                    // See https://reference.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
                    // for the meaning of the return values
                    if (r == 5) {
                        return -I2CDEV_RESULT_ETIMEDOUT;
                    }

                    return -I2CDEV_RESULT_EIO;
                }
            }
        } else if (isRead) {
            const unsigned long start = millis();

            for (size_t off = 0; off < current->len; off += I2CDEVLIB_BUFFER_LENGTH) {
                const size_t chunk = min((size_t) I2CDEVLIB_BUFFER_LENGTH, current->len - off);

                wire->requestFrom((uint8_t) addr, chunk);

                size_t got = wire->available();

                while (got < chunk) {
                    delay(1);

                    if (millis() - start > I2CDEVLIB_TRANSFER_TIMEOUT_MSEC) {
                        return -I2CDEV_RESULT_ETIMEDOUT;
                    }

                    got = wire->available();
                }

                for (size_t j = 0; j < chunk; j++) {
                    current->buf[off + j] = wire->read();
                }
            }
        }
    }

    return I2CDEV_RESULT_OK;
}

// Arduino driver API instance
static const i2cdev_driver_api_t i2cdev_arduino_driver_api = {
    .transfer = i2cdev_arduino_transfer,
};

static i2cdev_bus_t i2cdev_arduino_create_bus(TwoWire *wire) {
    const i2cdev_bus_t bus = {
        .api = &i2cdev_arduino_driver_api,
        .context = wire,
    };

    return bus;
}

i2cdev_bus_t i2cdev_arduino_bus = i2cdev_arduino_create_bus(&Wire);

#ifdef ARDUINO_ARCH_ESP32
i2cdev_bus_t i2cdev_arduino_bus1 = i2cdev_arduino_create_bus(&Wire1);
#endif // ARDUINO_ARCH_ESP32

#ifndef I2CDEVLIB_DEFAULT_BUS
#define I2CDEVLIB_DEFAULT_BUS i2cdev_arduino_bus
#endif // I2CDEVLIB_DEFAULT_BUS

#endif //I2CDEVBUS_HAL_ARDUINO_HPP
