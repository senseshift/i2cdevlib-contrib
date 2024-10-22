#ifndef __I2CDEVBUS_HPP__
#define __I2CDEVBUS_HPP__

/// Inspired by the `jrowberg/i2cdevlib` project, but refactored to be more abstract and modern C++.

#ifdef ARDUINO
#include <Arduino.h>
#else // !ARDUINO
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <string>
#endif // ARDUINO

#include "i2cdevbus.h"

#ifndef I2CDEVLIB_LOG_V
# define I2CDEVLIB_LOG_V(...)
#endif

#ifndef I2CDEVLIB_LOG_D
# define I2CDEVLIB_LOG_D(...)
#endif

#ifndef I2CDEVLIB_LOG_I
# define I2CDEVLIB_LOG_I(...)
#endif

#ifndef I2CDEVLIB_LOG_W
# define I2CDEVLIB_LOG_W(...)
#endif

#ifndef I2CDEVLIB_LOG_E
# define I2CDEVLIB_LOG_E(...)
#endif

/// An abstract class that provides a generic interface for I2C communication.
///
/// Provides useful functionality for reading and writing to I2C registers.
class I2CDevBus {
public:
    /// Default read timeout in milliseconds.
    static const uint16_t DEFAULT_READ_TIMEOUT_MS = I2CDEV_DEFAULT_READ_TIMEOUT_MS;

    // region Single byte operations

    /// Read multiple bytes from an 8-bit device register.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to start reading from
    /// \param [in] length Number of bytes to read
    /// \param [out] data Buffer to store read data in
    /// \param [in] timeout Read operation timeout (default DEFAULT_READ_TIMEOUT). Set to 0 to disable timeout.
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] virtual auto readReg8(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            const size_t length,
            uint8_t* const data,
            const uint16_t timeout = I2CDevBus::DEFAULT_READ_TIMEOUT_MS
    ) -> i2cdev_result_t = 0;

    /// Read multiple bytes from an 8-bit device register.
    ///
    /// \tparam U A container type that stores the read data.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to read from
    /// \param [out] data Buffer to store read data in
    /// \param [in] timeout Read operation timeout (default DEFAULT_READ_TIMEOUT). Set to 0 to disable timeout.
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    template<typename U>
    [[nodiscard]] auto readReg8(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            U& data,
            const uint16_t timeout = I2CDevBus::DEFAULT_READ_TIMEOUT_MS
    ) -> i2cdev_result_t;

    /// Read a single byte from an 8-bit device register.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to read from
    /// \param [out] data Buffer to store read data in
    /// \param [in] timeout Read operation timeout (default DEFAULT_READ_TIMEOUT). Set to 0 to disable timeout.
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] inline auto readReg8(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            uint8_t* const data,
            const uint16_t timeout = I2CDevBus::DEFAULT_READ_TIMEOUT_MS
    ) -> i2cdev_result_t;

    /// Write multiple bytes to an 8-bit device register.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to start writing to
    /// \param [in] length Number of bytes to write
    /// \param [in] data Buffer to write data from
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] virtual auto writeReg8(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            const size_t length,
            const uint8_t* const data
    ) -> i2cdev_result_t = 0;

    /// Write multiple bytes to an 8-bit device register.
    ///
    /// \tparam U A container type that stores the write data.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to start writing to
    /// \param [in] data Buffer to write data from
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    template<typename U>
    [[nodiscard]] auto
    writeReg8(const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const U& data) -> i2cdev_result_t;

    /// Write a single byte to an 8-bit device register.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to write to
    /// \param [in] data Data to write to the register
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] inline auto writeReg8(
            const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const uint8_t data
    ) -> i2cdev_result_t;

    /// Update a single byte on an 8-bit device register.
    ///
    /// \note If the calculated new register value matches the value that
    /// was read this function will not generate a write operation.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to update
    /// \param [in] value New value to write to the register
    /// \param [in] mask Bitmask for updating internal register.
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] auto updateReg8(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            const uint8_t mask,
            const uint8_t value
    ) -> i2cdev_result_t;

    [[nodiscard]] inline auto updateReg8Bits(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            const uint8_t startBit,
            const uint8_t length,
            const uint8_t value
    ) -> i2cdev_result_t;

    [[nodiscard]] inline auto updateReg8Bit(
            const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const uint8_t bit, const bool value
    ) -> i2cdev_result_t;

    // endregion

    // region Two byte operations

    /// Read multiple words from a 16-bit device register.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to start reading from
    /// \param [in] length Number of words to read
    /// \param [out] data Buffer to store read data in
    /// \param [in] timeout Read operation timeout (default DEFAULT_READ_TIMEOUT). Set to 0 to disable timeout.
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] virtual auto readReg16(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            const size_t length,
            uint16_t* const data,
            const uint16_t timeout = I2CDevBus::DEFAULT_READ_TIMEOUT_MS
    ) -> i2cdev_result_t = 0;

    /// Read multiple words from a 16-bit device register.
    ///
    /// \tparam U A container type that stores the read data.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to read from
    /// \param [out] data Buffer to store read data in
    /// \param [in] timeout Read operation timeout (default DEFAULT_READ_TIMEOUT). Set to 0 to disable timeout.
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    template<typename U>
    [[nodiscard]] auto readReg16(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            U& data,
            const uint16_t timeout = I2CDevBus::DEFAULT_READ_TIMEOUT_MS
    ) -> i2cdev_result_t;

    /// Read a single word from a 16-bit device register.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to read from
    /// \param [out] data Buffer to store read data in
    /// \param [in] timeout Read operation timeout (default DEFAULT_READ_TIMEOUT). Set to 0 to disable timeout.
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] inline auto readReg16(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            uint16_t* const data,
            const uint16_t timeout = I2CDevBus::DEFAULT_READ_TIMEOUT_MS
    ) -> i2cdev_result_t;

    /// Write multiple words to a 16-bit device register.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to start writing to
    /// \param [in] length Number of words to write
    /// \param [in] data Buffer to write data from
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] virtual auto writeReg16(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            const size_t length,
            const uint16_t* data
    ) -> i2cdev_result_t = 0;

    /// Write multiple words to a 16-bit device register.
    ///
    /// \tparam U A container type that stores the write data.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to start writing to
    /// \param [in] data Buffer to write data from
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    template<typename U>
    [[nodiscard]] auto
    writeReg16(const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const U& data) -> i2cdev_result_t;

    /// Write a single word to a 16-bit device register.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to write to
    /// \param [in] data Data to write to the register
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] inline auto writeReg16(
            const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const uint16_t data
    ) -> i2cdev_result_t;

    /// Update a single word on a 16-bit device register.
    ///
    /// \note If the calculated new register value matches the value that
    /// was read this function will not generate a write operation.
    ///
    /// \param [in] devAddr I2C slave device address
    /// \param [in] regAddr Register address to update
    /// \param [in] value New value to write to the register
    /// \param [in] mask Bitmask for updating internal register.
    ///
    /// \return I2CDEV_RESULT_OK on success, negative error code on failure
    /// \retval I2CDEV_RESULT_OK If the read operation was successful
    [[nodiscard]] auto updateReg16(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            const uint16_t mask,
            const uint16_t value
    ) -> i2cdev_result_t;

    [[nodiscard]] inline auto updateReg16Bits(
            const i2cdev_dev_addr_t devAddr,
            const i2cdev_reg_addr_t regAddr,
            const uint8_t startBit,
            const uint8_t length,
            const uint16_t value
    ) -> i2cdev_result_t;

    [[nodiscard]] inline auto updateReg16Bit(
            const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const uint8_t bit, const bool value
    ) -> i2cdev_result_t;

    // endregion
};

template<typename U>
[[nodiscard]] auto I2CDevBus::readReg8(
        const i2cdev_dev_addr_t devAddr,
        const i2cdev_reg_addr_t regAddr,
        U& data,
        const uint16_t timeout
) -> i2cdev_result_t
{
#if !defined(__AVR__) && !defined(ARDUINO_ARCH_AVR)
    static_assert(std::is_same<typename U::value_type, uint8_t>::value, "U must be a container of uint8_t");
#endif
    return this->readReg8(devAddr, regAddr, data.size(), data.data(), timeout);
}

[[nodiscard]] inline auto I2CDevBus::readReg8(
        const i2cdev_dev_addr_t devAddr,
        const i2cdev_reg_addr_t regAddr,
        uint8_t* const data,
        const uint16_t timeout
) -> i2cdev_result_t
{
    return this->readReg8(devAddr, regAddr, 1, data, timeout);
}

template<typename U>
[[nodiscard]] auto I2CDevBus::writeReg8(
        const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const U& data
) -> i2cdev_result_t
{
#if !defined(__AVR__) && !defined(ARDUINO_ARCH_AVR)
    static_assert(std::is_same<typename U::value_type, uint8_t>::value, "U must be a container of uint8_t");
#endif
    return this->writeReg8(devAddr, regAddr, data.size(), data.data());
}

[[nodiscard]] inline auto I2CDevBus::writeReg8(
        const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const uint8_t data
) -> i2cdev_result_t
{
    return this->writeReg8(devAddr, regAddr, 1, &data);
}

[[nodiscard]] auto I2CDevBus::updateReg8(
        const i2cdev_dev_addr_t devAddr,
        const i2cdev_reg_addr_t regAddr,
        const uint8_t mask,
        const uint8_t value
) -> i2cdev_result_t
{
    uint8_t old_data;

    const auto read_result = this->readReg8(devAddr, regAddr, &old_data);
    if (read_result != I2CDEV_RESULT_OK) {
        return read_result;
    }

    const uint8_t new_data = (old_data & ~mask) | (value & mask);

#if !defined(I2CDEV_ALWAYS_UPDATE) || !I2CDEV_ALWAYS_UPDATE
    if (old_data == new_data) {
        return I2CDEV_RESULT_OK;
    }
#endif

    return this->writeReg8(devAddr, regAddr, new_data);
}

[[nodiscard]] inline auto I2CDevBus::updateReg8Bits(
        const i2cdev_dev_addr_t devAddr,
        const i2cdev_reg_addr_t regAddr,
        const uint8_t startBit,
        const uint8_t length,
        const uint8_t value
) -> i2cdev_result_t
{
    const uint8_t mask = (1 << length) - 1;
    return this->updateReg8(devAddr, regAddr, mask << startBit, value << startBit);
}

[[nodiscard]] inline auto I2CDevBus::updateReg8Bit(
        const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const uint8_t bit, const bool value
) -> i2cdev_result_t
{
    return this->updateReg8(devAddr, regAddr, value ? (1 << bit) : 0, 1 << bit);
}

template<typename U>
[[nodiscard]] auto I2CDevBus::readReg16(
        const i2cdev_dev_addr_t devAddr,
        const i2cdev_reg_addr_t regAddr,
        U& data,
        const uint16_t timeout
) -> i2cdev_result_t
{
#if !defined(__AVR__) && !defined(ARDUINO_ARCH_AVR)
    static_assert(std::is_same<typename U::value_type, uint16_t>::value, "U must be a container of uint16_t");
#endif

    return this->readReg16(devAddr, regAddr, data.size(), data.data(), timeout);
}

[[nodiscard]] inline auto I2CDevBus::readReg16(
        const i2cdev_dev_addr_t devAddr,
        const i2cdev_reg_addr_t regAddr,
        uint16_t* const data,
        const uint16_t timeout
) -> i2cdev_result_t
{
    return this->readReg16(devAddr, regAddr, 1, data, timeout);
}

template<typename U>
[[nodiscard]] auto I2CDevBus::writeReg16(
        const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const U& data
) -> i2cdev_result_t
{
#if !defined(__AVR__) && !defined(ARDUINO_ARCH_AVR)
    static_assert(std::is_same<typename U::value_type, uint16_t>::value, "U must be a container of uint16_t");
#endif

    return this->writeReg16(devAddr, regAddr, data.size(), data.data());
}

[[nodiscard]] inline auto I2CDevBus::writeReg16(
        const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const uint16_t data
) -> i2cdev_result_t
{
    return this->writeReg16(devAddr, regAddr, 1, &data);
}

[[nodiscard]] auto I2CDevBus::updateReg16(
        const i2cdev_dev_addr_t devAddr,
        const i2cdev_reg_addr_t regAddr,
        const uint16_t mask,
        const uint16_t value
) -> i2cdev_result_t
{
    uint16_t old_data;

    const auto read_result = this->readReg16(devAddr, regAddr, &old_data);
    if (read_result != I2CDEV_RESULT_OK) {
        return read_result;
    }

    const uint16_t new_data = (old_data & ~mask) | (value & mask);

#if !defined(I2CDEV_ALWAYS_UPDATE) || !I2CDEV_ALWAYS_UPDATE
    if (old_data == new_data) {
        return I2CDEV_RESULT_OK;
    }
#endif

    return this->writeReg16(devAddr, regAddr, new_data);
}

[[nodiscard]] inline auto I2CDevBus::updateReg16Bits(
        const i2cdev_dev_addr_t devAddr,
        const i2cdev_reg_addr_t regAddr,
        const uint8_t startBit,
        const uint8_t length,
        const uint16_t value
) -> i2cdev_result_t
{
    const uint16_t mask = (1U << length) - 1;
    return this->updateReg16(devAddr, regAddr, mask << startBit, value << startBit);
}

[[nodiscard]] inline auto I2CDevBus::updateReg16Bit(
        const i2cdev_dev_addr_t devAddr, const i2cdev_reg_addr_t regAddr, const uint8_t bit, const bool value
) -> i2cdev_result_t
{
    return this->updateReg16(devAddr, regAddr, value ? (1 << bit) : 0, 1 << bit);
}

namespace i2cdevlib {
    using I2CDevBus = ::I2CDevBus;

    /// hexdump() - Dump a buffer in hexadecimal format.
    ///
    /// \param [in] data Pointer to the buffer to dump
    /// \param [in] length Number of bytes to dump
    ///
    /// \return A string containing the hexadecimal dump of the buffer
    [[nodiscard]] auto hexdump(const uint8_t* data, const size_t length) -> char* {
        char* result = new char[length * 2 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(result + i * 2, "%02X", data[i]);
        }

        result[length * 2] = '\0';

        return result;
    }

    [[nodiscard]] inline auto hexdump(const uint16_t* data, const size_t length) -> char* {
        return hexdump(reinterpret_cast<const uint8_t*>(data), length * 2);
    }
}

#endif // __I2CDEVBUS_HPP__