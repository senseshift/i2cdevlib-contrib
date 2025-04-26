#ifndef I2CDEVBUS_HAL_ARDUINO_HPP
#define I2CDEVBUS_HAL_ARDUINO_HPP

#if !defined(__cplusplus)
#error "This header requires a C++ compiler."
#endif

#ifdef ARDUINO_ARCH_ESP32
#  define I2CDEVLIB_LOG_V(...) log_v(__VA_ARGS__)
#  define I2CDEVLIB_LOG_D(...) log_d(__VA_ARGS__)
#  define I2CDEVLIB_LOG_I(...) log_i(__VA_ARGS__)
#  define I2CDEVLIB_LOG_W(...) log_w(__VA_ARGS__)
#  define I2CDEVLIB_LOG_E(...) log_e(__VA_ARGS__)
#endif // ARDUINO_ARCH_ESP32

#include <Arduino.h>
#include <Wire.h>
#include "i2cdevbus.h"

#ifndef I2CDEVLIB_BUFFER_LENGTH
#ifdef BUFFER_LENGTH
#define I2CDEVLIB_BUFFER_LENGTH BUFFER_LENGTH
#elif defined(I2C_BUFFER_LENGTH)
#define I2CDEVLIB_BUFFER_LENGTH I2C_BUFFER_LENGTH
#else
#define I2CDEVLIB_BUFFER_LENGTH 32
#endif
#endif

#ifndef I2CDEVLIB_TRANSFER_TIMEOUT_MSEC
#define I2CDEVLIB_TRANSFER_TIMEOUT_MSEC 500
#endif

static inline int i2cdev_arduino_transfer(
    const i2cdev_bus_t *bus,
    i2cdev_msg_t *msgs,
    size_t num_msgs,
    i2cdev_dev_addr_t addr)
{
    I2CDEVLIB_LOG_I("[I2CDEV] Transfer start: addr=0x%02X, msgs=%u", addr, num_msgs);

    if (num_msgs == 0)
    {
        I2CDEVLIB_LOG_E("[I2CDEV] No messages to process");
        return I2CDEV_RESULT_OK;
    }

    TwoWire *wire = reinterpret_cast<TwoWire *>(bus->context);
    if (!wire)
    {
        I2CDEVLIB_LOG_E("[I2CDEV] Error: bus context null");
        return I2CDEV_RESULT_ENODEV;
    }

    // 1) Validate messages
    for (size_t i = 0; i < num_msgs; i++)
    {
        const auto *cur = &msgs[i];
        I2CDEVLIB_LOG_I("[I2CDEV] Validate msg %u: len=%u flags=0x%02X", i, cur->len, cur->flags);
        if (cur->len == 0)
        {
            I2CDEVLIB_LOG_E("[I2CDEV] Validation Error: zero-length message");
            return I2CDEV_RESULT_EINVAL;
        }
        if (cur->flags & I2CDEV_ADDR_10BIT)
        {
            I2CDEVLIB_LOG_E("[I2CDEV] Validation Error: 10-bit address not supported");
            return I2CDEV_RESULT_ENOSYS;
        }
        if (i + 1 < num_msgs && (cur->flags & I2CDEV_MSG_STOP))
        {
            I2CDEVLIB_LOG_E("[I2CDEV] Validation Error: STOP on intermediate msg %u", i);
            return I2CDEV_RESULT_EINVAL;
        }
    }

    // 2) Execute each message, chunked
    for (size_t i = 0; i < num_msgs; i++)
    {
        const auto *cur = &msgs[i];
        bool isRead = cur->flags & I2CDEV_MSG_READ;
        bool isWrite = !isRead;
        bool sendStop = cur->flags & I2CDEV_MSG_STOP;

        I2CDEVLIB_LOG_I("[I2CDEV] Processing msg %u: %s, %s STOP",
                      i,
                      isWrite ? "WRITE" : "READ",
                      sendStop ? "with" : "without");

        if (isWrite)
        {
            // Handle two-message register writes (reg address + data) in one transaction to avoid Arduino repeated-start errors
            if (!sendStop && (i + 1) < num_msgs &&
                !(msgs[i+1].flags & I2CDEV_MSG_READ) && (msgs[i+1].flags & I2CDEV_MSG_STOP))
            {
                const auto *next = &msgs[i + 1];
                I2CDEVLIB_LOG_I("[I2CDEV] Combined WRITE reg+data");
                wire->beginTransmission((uint8_t)addr);
                // write register address
                if (wire->write(msgs[i].buf, msgs[i].len) != msgs[i].len)
                {
                    I2CDEVLIB_LOG_E("[I2CDEV] ERROR: combined write() reg short");
                    return I2CDEV_RESULT_EIO;
                }
                // write register data
                if (wire->write(next->buf, next->len) != next->len)
                {
                    I2CDEVLIB_LOG_E("[I2CDEV] ERROR: combined write() data short");
                    return I2CDEV_RESULT_EIO;
                }
                int rc = wire->endTransmission(true);
                I2CDEVLIB_LOG_I("[I2CDEV] endTransmission(combined, doStop=1) => %d", rc);
                if (rc != 0)
                {
                    return (rc == 5) ? I2CDEV_RESULT_ETIMEDOUT : I2CDEV_RESULT_EIO;
                }
                i++; // skip the data message
                continue;
            }
            for (size_t off = 0; off < cur->len; off += I2CDEVLIB_BUFFER_LENGTH)
            {
                size_t chunk = min((size_t) I2CDEVLIB_BUFFER_LENGTH, cur->len - off);
                I2CDEVLIB_LOG_I("[I2CDEV] WRITE chunk: offset=%u len=%u", off, chunk);
                wire->beginTransmission((uint8_t)addr);
                if (wire->write(cur->buf + off, chunk) != chunk)
                {
                    I2CDEVLIB_LOG_E("[I2CDEV] ERROR: write() returned short");
                    return I2CDEV_RESULT_EIO;
                }
                bool isLast = (off + chunk >= cur->len);
                bool doStop = isLast && sendStop;
                int rc = wire->endTransmission(doStop);
                I2CDEVLIB_LOG_I("[I2CDEV] endTransmission(doStop=%u) => %d", doStop, rc);
                if (rc != 0)
                {
                    if (rc == 5)
                    {
                        I2CDEVLIB_LOG_E("[I2CDEV] ERROR: write timeout");
                        return I2CDEV_RESULT_ETIMEDOUT;
                    }
                    I2CDEVLIB_LOG_I("[I2CDEV] ERROR: write I/O");
                    return I2CDEV_RESULT_EIO;
                }
            }
        }
        else if (isRead)
        {
            unsigned long start = millis();
            for (size_t off = 0; off < cur->len; off += I2CDEVLIB_BUFFER_LENGTH)
            {
                size_t chunk = min((size_t) I2CDEVLIB_BUFFER_LENGTH, cur->len - off);
                bool doStop = (off + chunk >= cur->len) && sendStop;
                I2CDEVLIB_LOG_I("[I2CDEV] READ chunk: offset=%u len=%u doStop=%u", off, chunk, doStop);
#if defined(ARDUINO_ARCH_AVR)
                wire->requestFrom((uint8_t)addr,
                                  (uint8_t)chunk,
                                  (uint8_t)doStop);
#else
                wire->requestFrom((uint8_t)addr, chunk, doStop);
#endif
                size_t got = wire->available();
                while (got < chunk)
                {
                    delay(1);
                    if (millis() - start > I2CDEVLIB_TRANSFER_TIMEOUT_MSEC)
                    {
                        I2CDEVLIB_LOG_E("[I2CDEV] ERROR: read timeout");
                        return I2CDEV_RESULT_ETIMEDOUT;
                    }
                    got = wire->available();
                }
                I2CDEVLIB_LOG_I("[I2CDEV] got %u bytes", got);
                for (size_t j = 0; j < chunk; j++)
                {
                    cur->buf[off + j] = wire->read();
                }
            }
        }
    }

    I2CDEVLIB_LOG_I("[I2CDEV] Transfer complete OK");
    return I2CDEV_RESULT_OK;
}

// Driver API instance
static const i2cdev_driver_api_t i2cdev_arduino_driver_api = {
    .transfer = i2cdev_arduino_transfer,
};

static i2cdev_bus_t i2cdev_arduino_create_bus(TwoWire *w)
{
    return i2cdev_bus_t{.api = &i2cdev_arduino_driver_api,
                        .context = w};
}

i2cdev_bus_t i2cdev_arduino_bus = i2cdev_arduino_create_bus(&Wire);
#ifdef ARDUINO_ARCH_ESP32
i2cdev_bus_t i2cdev_arduino_bus1 = i2cdev_arduino_create_bus(&Wire1);
#endif

#ifndef I2CDEVLIB_DEFAULT_BUS
#define I2CDEVLIB_DEFAULT_BUS i2cdev_arduino_bus
#endif

#endif // I2CDEVBUS_HAL_ARDUINO_HPP
