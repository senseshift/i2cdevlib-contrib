#ifndef __I2CDEVLIB_DRV2605_H__
#define __I2CDEVLIB_DRV2605_H__

#include "i2cdevbus.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define DRV2605_I2CADDR (0x5A)

typedef enum drv2605_reg_t {
    DRV2605_REG_MODE = 0x01,
    DRV2605_REG_RTPIN = 0x02,
    DRV2605_REG_LIBRARY = 0x03,
    DRV2605_REG_WAVESEQ1 = 0x04,
    DRV2605_REG_WAVESEQ2 = 0x05,
    DRV2605_REG_WAVESEQ3 = 0x06,
    DRV2605_REG_WAVESEQ4 = 0x07,
    DRV2605_REG_WAVESEQ5 = 0x08,
    DRV2605_REG_WAVESEQ6 = 0x09,
    DRV2605_REG_WAVESEQ7 = 0x0A,
    DRV2605_REG_WAVESEQ8 = 0x0B,
    DRV2605_REG_GO = 0x0C,
    DRV2605_REG_OVERDRIVE = 0x0D,
    DRV2605_REG_SUSTAIN_POS = 0x0E,
    DRV2605_REG_SUSTAIN_NEG = 0x0F,
    DRV2605_REG_BREAK = 0x10,
    DRV2605_REG_AUDIO_CTRL = 0x11,
    DRV2605_REG_AUDIO_LVL = 0x12,
    DRV2605_REG_AUDIO_MAX = 0x13,
    DRV2605_REG_AUDIO_OUT_MIN = 0x14,
    DRV2605_REG_AUDIO_OUT_MAX = 0x15,
    DRV2605_REG_RATED_VOLT = 0x16,
    DRV2605_REG_CLAMP_VOLT = 0x17,
    DRV2605_REG_AUTO_CAL_COMP = 0x18,
    DRV2605_REG_AUTO_CAL_BACK_EMF = 0x19,
    DRV2605_REG_FEEDBACK_CTRL = 0x1A,
    DRV2605_REG_CTRL1 = 0x1B,
    DRV2605_REG_CTRL2 = 0x1C,
    DRV2605_REG_CTRL3 = 0x1D,
    DRV2605_REG_CTRL4 = 0x1E,
    DRV2605_REG_VBAT = 0x21,
    DRV2605_REG_LRARESON = 0x22
} drv2605_reg_t;

#ifdef __cplusplus
};
#endif // __cplusplus

#endif //__I2CDEVLIB_DRV2605_H__