#ifndef I2CDEVBUS_H_
#define I2CDEVBUS_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef uint16_t i2cdev_dev_addr_t;
typedef uint8_t i2cdev_reg_addr_t;

/**
 * @note We are using the same values as the Linux kernel for error codes.
 * @todo Include and re-use definitions from errno.h.
 */
typedef enum i2cdev_result {
    /**
     * @brief No error.
     */
    I2CDEV_RESULT_OK = 0,

    /**
     * @brief I/O error.
     */
    I2CDEV_RESULT_EIO = 5,

    /**
     * @brief No more contexts.
     */
    I2CDEV_RESULT_EAGAIN = 11,

    /**
     * @brief No such device.
     */
    I2CDEV_RESULT_ENODEV = 19,

    /**
     * @brief Invalid argument.
     */
    I2CDEV_RESULT_EINVAL = 22,

    /**
     * @brief Function not implemented.
     */
    I2CDEV_RESULT_ENOSYS = 88,

    /**
     * @brief Operation would block.
     */
    I2CDEV_RESULT_EWOULDBBLOCK = I2CDEV_RESULT_EAGAIN,
} i2cdev_result_t;

/**
 * @brief I2C message flags.
 * @see i2cdev_msg
 */
typedef enum i2cdev_msg_flags {
    /**
     * Write message to I2C bus.
     */
    I2CDEV_MSG_WRITE = 0b00000000,

    /**
     * Read message from I2C bus.
     */
    I2CDEV_MSG_READ = 0b00000001,

    /**
     * Send STOP after this message.
     */
    I2CDEV_MSG_STOP = 0b00000010,

    I2CDEV_MSG_RESTART = 0b00000100,

    /**
     * Use 10-bit address for this message, instead of the default 7-bit.
     */
    I2CDEV_ADDR_10BIT = 0b00001000,
} i2cdev_msg_flags_t;

/**
 * @brief Single I2C message.
 */
typedef struct i2cdev_msg {
    /**
     * Data buffer in bytes
     */
    uint8_t *buf;

    /**
     * Length of buffer in bytes
     */
    const size_t len;

    /**
     * Flags for this message
     */
    const uint8_t flags;
} i2cdev_msg_t;

typedef struct i2cdev_driver_api i2cdev_driver_api_t;

typedef struct i2cdev_bus {
    const i2cdev_driver_api_t *api;
    void *context;
} i2cdev_bus_t;

typedef int (*i2cdev_api_transfer_t)(const i2cdev_bus_t *bus,
                                     const i2cdev_msg_t *msgs,
                                     size_t num_msgs,
                                     i2cdev_dev_addr_t addr);

typedef struct i2cdev_driver_api {
    i2cdev_api_transfer_t transfer;
} i2cdev_driver_api_t;

typedef struct i2cdev_delay_api {
    void (*delay_ns)(uint32_t ns);

    void (*delay_us)(uint32_t us);

    void (*delay_ms)(uint32_t ms);
} i2cdev_delay_api_t;

/**
 * @brief Transfer data to another I2C device as a controller.
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [in] msgs Array of messages to transfer.
 * @param [in] num_msgs Number of messages to transfer.
 * @param [in] addr Address of the I2C target device.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval -I2CDEV_RESULT_EIO If an I/O error occurred.
 * @retval -I2CDEV_RESULT_ENOSYS If the transfer function is not implemented.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_transfer(const i2cdev_bus_t *bus,
                           const i2cdev_msg_t *msgs,
                           const size_t num_msgs,
                           const i2cdev_dev_addr_t addr) {
    if (!num_msgs) {
        return I2CDEV_RESULT_OK;
    }

    if (!bus || !bus->api || !bus->api->transfer) {
        return -I2CDEV_RESULT_ENOSYS;
    }

    return bus->api->transfer(bus, msgs, num_msgs, addr);
}

/**
 * @brief Write data to another I2C device as a controller.
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [in] buf Pointer to the buffer to write.
 * @param [in] num_bytes Number of bytes to write.
 * @param [in] addr Address of the I2C target device.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_write(const i2cdev_bus_t *bus,
                        const uint8_t *buf,
                        size_t num_bytes,
                        i2cdev_dev_addr_t addr) {
    i2cdev_msg_t msg = {
        .buf = (uint8_t *) buf,
        .len = num_bytes,
        .flags = I2CDEV_MSG_WRITE | I2CDEV_MSG_STOP,
    };

    return i2cdev_transfer(bus, &msg, 1, addr);
}

/**
 * @brief Read data from another I2C device as a controller.
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [out] buf Pointer to the buffer to store the read data.
 * @param [in] num_bytes Number of bytes to read.
 * @param [in] addr Address of the I2C target device.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_read(const i2cdev_bus_t *bus,
                       uint8_t *buf,
                       size_t num_bytes,
                       i2cdev_dev_addr_t addr) {
    struct i2cdev_msg msg = {
        .buf = buf,
        .len = num_bytes,
        .flags = I2CDEV_MSG_READ | I2CDEV_MSG_STOP,
    };

    return i2cdev_transfer(bus, &msg, 1, addr);
}

/**
 * @brief Write then read data from another I2C device as a controller.
 *
 * This is commonly used for register reads, in the sense of transactions pair:
 * "here's what I want", "give it to me".
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [in] addr Address of the I2C target device.
 * @param [in] write_buf Pointer to the buffer to write.
 * @param [in] num_write Number of bytes to write.
 * @param [out] read_buf Pointer to the buffer to read.
 * @param [in] num_read Number of bytes to read.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_write_read(const i2cdev_bus_t *bus,
                             i2cdev_dev_addr_t addr,
                             const uint8_t *write_buf,
                             size_t num_write,
                             uint8_t *read_buf,
                             size_t num_read) {
    struct i2cdev_msg msgs[2] = {
        {
            .buf = (uint8_t *) write_buf,
            .len = num_write,
            .flags = I2CDEV_MSG_WRITE,
        },
        {
            .buf = (uint8_t *) read_buf,
            .len = num_read,
            .flags = I2CDEV_MSG_RESTART | I2CDEV_MSG_READ | I2CDEV_MSG_STOP,
        },
    };

    return i2cdev_transfer(bus, msgs, 2, addr);
}

/**
 * @brief Read a single register from an I2C device.
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [in] addr Address of the I2C target device.
 * @param [in] reg Register address to read.
 * @param [out] val Pointer to the buffer to store the read value.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_reg_read_u8(const i2cdev_bus_t *bus,
                              i2cdev_dev_addr_t addr,
                              i2cdev_reg_addr_t reg,
                              uint8_t *val) {
    return i2cdev_write_read(bus, addr, &reg, 1, val, 1);
}

/**
 * @brief Write a single register to an I2C device.
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [in] addr Address of the I2C target device.
 * @param [in] reg Register address to write.
 * @param [in] val Value to write to the register.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_reg_write_u8(const i2cdev_bus_t *bus,
                               i2cdev_dev_addr_t addr,
                               i2cdev_reg_addr_t reg,
                               uint8_t val) {
    uint8_t buf[2] = {reg, val};

    return i2cdev_write(bus, buf, 2, addr);
}

/**
 * @brief Update a single register in an I2C device.
 *
 * @note If the calculated new register value matches the value that
 * was read this function will not generate a write operation.
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [in] addr Address of the I2C target device.
 * @param [in] reg Register address to update.
 * @param [in] mask Mask to apply to the register value.
 * @param [in] val Value to write to the register.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_reg_update_u8(const i2cdev_bus_t *bus,
                                i2cdev_dev_addr_t addr,
                                i2cdev_reg_addr_t reg,
                                uint8_t mask,
                                uint8_t val) {
    uint8_t tmp;

    int ret = i2cdev_reg_read_u8(bus, addr, reg, &tmp);
    if (ret) {
        return ret;
    }

    tmp = (tmp & ~mask) | (val & mask);
    if ((tmp & mask) == (val & mask)) {
        return I2CDEV_RESULT_OK;
    }

    return i2cdev_reg_write_u8(bus, addr, reg, tmp);
}

/**
 * @brief Read a single bit from a register in an I2C device.
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [in] addr Address of the I2C target device.
 * @param [in] reg Register address to read.
 * @param [in] bit The bit offset to read.
 * @param [out] set Pointer to store the bit value.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_reg_read_bit(const i2cdev_bus_t *bus,
                               i2cdev_dev_addr_t addr,
                               i2cdev_reg_addr_t reg,
                               const uint8_t bit,
                               bool *set) {
    uint8_t tmp;

    int ret = i2cdev_reg_read_u8(bus, addr, reg, &tmp);
    if (ret) {
        return ret;
    }

    *set = (tmp & (1 << bit)) != 0;

    return I2CDEV_RESULT_OK;
}

/**
 * @brief Write a single bit to a register in an I2C device.
 *
 * @param [in] bus Pointer to the bus structure for an I2C controller.
 * @param [in] addr Address of the I2C target device.
 * @param [in] reg Register address to write.
 * @param [in] bit The bit offset to write.
 * @param [in] set The value to write to the bit.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
static int i2cdev_reg_write_bit(const struct i2cdev_bus *bus,
                                i2cdev_dev_addr_t addr,
                                i2cdev_reg_addr_t reg,
                                const uint8_t bit,
                                const bool set) {
    return i2cdev_reg_update_u8(bus, addr, reg,
                                (1 << bit),
                                set ? (1 << bit) : 0);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // I2CDEVBUS_H_
