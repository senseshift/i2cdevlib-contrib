#ifndef I2CDEVLIB_PCA9685_H_
#define I2CDEVLIB_PCA9685_H_

#include "i2cdevbus.h"

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define PCA9685_I2CADDR_BASE (0x40)

#define PCA9685_CREATE_MASK(shift, length) (((1U << (length)) - 1U) << (shift))

typedef enum pca9685_reg {
    PCA9685_REG_MODE1 = 0x00,
    PCA9685_REG_MODE2 = 0x01,
    PCA9685_REG_SUBADR1 = 0x02,
    PCA9685_REG_SUBADR2 = 0x03,
    PCA9685_REG_SUBADR3 = 0x04,
    PCA9685_REG_ALLCALLADR = 0x05,
    PCA9685_REG_LED0_ON_L = 0x06,
    PCA9685_REG_LED0_ON_H = 0x07,
    PCA9685_REG_LED0_OFF_L = 0x08,
    PCA9685_REG_LED0_OFF_H = 0x09,
    PCA9685_REG_ALL_LED_ON_L = 0xFA,
    PCA9685_REG_ALL_LED_ON_H = 0xFB,
    PCA9685_REG_ALL_LED_OFF_L = 0xFC,
    PCA9685_REG_ALL_LED_OFF_H = 0xFD,
    PCA9685_REG_PRESCALE = 0xFE,
    PCA9685_REG_TESTMODE = 0xFF
} pca9685_reg_t;

#define PCA9685_MODE1_ALLCALL_SHIFT (0)
#define PCA9685_MODE1_ALLCALL_LENGTH (1)
#define PCA9685_MODE1_ALLCALL PCA9685_CREATE_MASK(PCA9685_MODE1_ALLCALL_SHIFT, PCA9685_MODE1_ALLCALL_LENGTH)

#define PCA9685_MODE1_SUB3_SHIFT (1)
#define PCA9685_MODE1_SUB3_LENGTH (1)
#define PCA9685_MODE1_SUB3 PCA9685_CREATE_MASK(PCA9685_MODE1_SUB3_SHIFT, PCA9685_MODE1_SUB3_LENGTH)

#define PCA9685_MODE1_SUB2_SHIFT (2)
#define PCA9685_MODE1_SUB2_LENGTH (1)
#define PCA9685_MODE1_SUB2 PCA9685_CREATE_MASK(PCA9685_MODE1_SUB2_SHIFT, PCA9685_MODE1_SUB2_LENGTH)

#define PCA9685_MODE1_SUB1_SHIFT (3)
#define PCA9685_MODE1_SUB1_LENGTH (1)
#define PCA9685_MODE1_SUB1 PCA9685_CREATE_MASK(PCA9685_MODE1_SUB1_SHIFT, PCA9685_MODE1_SUB1_LENGTH)

#define PCA9685_MODE1_SLEEP_SHIFT (4)
#define PCA9685_MODE1_SLEEP_LENGTH (1)
#define PCA9685_MODE1_SLEEP PCA9685_CREATE_MASK(PCA9685_MODE1_SLEEP_SHIFT, PCA9685_MODE1_SLEEP_LENGTH)

#define PCA9685_MODE1_AI_SHIFT (5)
#define PCA9685_MODE1_AI_LENGTH (1)
#define PCA9685_MODE1_AI PCA9685_CREATE_MASK(PCA9685_MODE1_AI_SHIFT, PCA9685_MODE1_AI_LENGTH)

#define PCA9685_MODE1_EXTCLK_SHIFT (6)
#define PCA9685_MODE1_EXTCLK_LENGTH (1)
#define PCA9685_MODE1_EXTCLK PCA9685_CREATE_MASK(PCA9685_MODE1_EXTCLK_SHIFT, PCA9685_MODE1_EXTCLK_LENGTH)

#define PCA9685_MODE1_RESTART_SHIFT (7)
#define PCA9685_MODE1_RESTART_LENGTH (1)
#define PCA9685_MODE1_RESTART PCA9685_CREATE_MASK(PCA9685_MODE1_RESTART_SHIFT, PCA9685_MODE1_RESTART_LENGTH)

#define PCA9685_MODE2_OUTNE_0 (0x01)
#define PCA9685_MODE2_OUTNE_1 (0x02)
#define PCA9685_MODE2_OUTDRV (0x04)
#define PCA9685_MODE2_OCH (0x08)
#define PCA9685_MODE2_INVRT (0x10)

// Datasheet 7.3.5
#define PCA9685_PRESCALE_MIN (0x03)
#define PCA9685_PRESCALE_MAX (0xff)
#define PCA9685_FREQ_MIN (24)
#define PCA9685_FREQ_MAX (1526)

#define PCA9685_OSCILLATOR_FREQ (25000000)

#define PCA9685_VALUE_MAX (4095)
#define PCA9685_VALUE_FULL_ON (4096)

typedef struct pca9685_dev {
    i2cdev_bus_t *i2cdev;
    uint8_t addr;
    i2cdev_delay_api_t *delay;
} pca9685_dev_t;

inline int pca9685_set_sleep_enabled(pca9685_dev_t *dev, bool enabled) {
    return i2cdev_reg_write_bit(dev->i2cdev, dev->addr, PCA9685_REG_MODE1,
                                PCA9685_MODE1_SLEEP_SHIFT, enabled);
}

inline int pca9685_wakeup(pca9685_dev_t *dev) {
    return pca9685_set_sleep_enabled(dev, false);
}

inline int pca9685_sleep(pca9685_dev_t *dev) {
    return pca9685_set_sleep_enabled(dev, true);
}

inline int pca9685_restart(pca9685_dev_t *dev) {
    bool restart;

    int ret = i2cdev_reg_read_bit(dev->i2cdev, dev->addr, PCA9685_REG_MODE1, PCA9685_MODE1_RESTART_SHIFT, &restart);
    if (ret) {
        return ret;
    }

    if (restart) {
        return 0;
    }

    ret = pca9685_wakeup(dev);
    if (ret) {
        return ret;
    }

    dev->delay->delay_us(500);

    ret = i2cdev_reg_write_bit(dev->i2cdev, dev->addr, PCA9685_REG_MODE1, PCA9685_MODE1_RESTART_SHIFT, true);

    return ret;
}

inline int pca9685_set_prescale(pca9685_dev_t *dev, uint8_t prescale, bool extclk = false) {
    if (prescale < PCA9685_PRESCALE_MIN) {
        return -I2CDEV_RESULT_EINVAL;
    }

    uint8_t mode1;
    int result = i2cdev_reg_read_u8(dev->i2cdev, dev->addr, PCA9685_REG_MODE1, &mode1);
    if (result) {
        return result;
    }

    mode1 = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    if (extclk) {
        mode1 |= PCA9685_MODE1_EXTCLK;
    }

    result = i2cdev_reg_write_u8(dev->i2cdev, dev->addr, PCA9685_REG_MODE1, mode1);
    if (result) {
        return result;
    }

    result = i2cdev_reg_write_u8(dev->i2cdev, dev->addr, PCA9685_REG_PRESCALE, prescale);
    if (result) {
        return result;
    }

    mode1 = (mode1 & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART | PCA9685_MODE1_AI;

    return i2cdev_reg_write_u8(dev->i2cdev, dev->addr, PCA9685_REG_MODE1, mode1);
}

inline int pca9685_read_prescale(pca9685_dev_t *dev, uint8_t *prescale) {
    return i2cdev_reg_read_u8(dev->i2cdev, dev->addr, PCA9685_REG_PRESCALE, prescale);
}

inline int pca9685_set_frequency(pca9685_dev_t *dev, float freq, float oscillator_freq = PCA9685_OSCILLATOR_FREQ) {
    // Clamp frequency
    freq = (freq > PCA9685_FREQ_MAX) ? PCA9685_FREQ_MAX : (freq < PCA9685_FREQ_MIN) ? PCA9685_FREQ_MIN : freq;

    // Datasheet 7.3.5
    const auto prescale = static_cast<uint8_t>(roundf(oscillator_freq / (4096.0 * freq)) - 1);

    return pca9685_set_prescale(dev, prescale);
}

inline int pca9685_set_channel_on_off(pca9685_dev_t *dev, uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) {
        return -I2CDEV_RESULT_EINVAL;
    }

    const uint8_t data[5] = {
        static_cast<uint8_t>(PCA9685_REG_LED0_ON_L + (channel * 4)),
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };

    return i2cdev_write(dev->i2cdev, data, sizeof(data), dev->addr);
}

inline int pca9685_set_channel(pca9685_dev_t *dev, uint8_t channel, uint16_t value, bool invert = false)
{
    // Clamp value to 12 bits
    value = (value > PCA9685_VALUE_MAX) ? PCA9685_VALUE_MAX : value;

    if (invert) {
        if (value == 0) {
            return pca9685_set_channel_on_off(dev, channel, PCA9685_VALUE_FULL_ON, 0);
        }
        if (value == PCA9685_VALUE_MAX) {
            return pca9685_set_channel_on_off(dev, channel, 0, PCA9685_VALUE_FULL_ON);
        }
        return pca9685_set_channel_on_off(dev, channel, 0, PCA9685_VALUE_MAX - value);
    } else {
        if (value == PCA9685_VALUE_MAX) {
            return pca9685_set_channel_on_off(dev, channel, PCA9685_VALUE_FULL_ON, 0);
        }
        if (value == 0) {
            return pca9685_set_channel_on_off(dev, channel, 0, PCA9685_VALUE_FULL_ON);
        }
        return pca9685_set_channel_on_off(dev, channel, 0, value);
    }
}

#ifdef __cplusplus
};
#endif // __cplusplus

#endif // I2CDEVLIB_PCA9685_H_
