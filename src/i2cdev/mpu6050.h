#ifndef I2CDEVLIB_MPU6050_H_
#define I2CDEVLIB_MPU6050_H_

#include "i2cdevbus.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define MPU6050_I2CADDR_AD0_LOW (0x68)
#define MPU6050_I2CADDR_AD0_HIGH (0x69)
#define MPU6050_I2CADDR_BASE (MPU6050_I2CADDR_AD0_LOW)

#define MPU6050_DEVICE_ID (0x68) // Expected MPU6050_REG_WHO_AM_I value

#define MPU6050_CREATE_MASK(shift, length) (((1U << (length)) - 1U) << (shift))

typedef enum mpu6050_reg
{
    MPU6050_REG_XG_OFFS_TC = 0x00,
    MPU6050_REG_YG_OFFS_TC = 0x01,
    MPU6050_REG_ZG_OFFS_TC = 0x02,

    MPU6050_REG_X_FINE_GAIN = 0x03,
    MPU6050_REG_Y_FINE_GAIN = 0x04,
    MPU6050_REG_Z_FINE_GAIN = 0x05,

    MPU6050_REG_XA_OFFS_H = 0x06,
    MPU6050_REG_XA_OFFS_L = 0x07,

    MPU6050_REG_YA_OFFS_H = 0x08,
    MPU6050_REG_YA_OFFS_L = 0x09,

    MPU6050_REG_ZA_OFFS_H = 0x0A,
    MPU6050_REG_ZA_OFFS_L = 0x0B,

    MPU6050_REG_SELF_TEST_X = 0x0D,
    MPU6050_REG_SELF_TEST_Y = 0x0E,
    MPU6050_REG_SELF_TEST_Z = 0x0F,
    MPU6050_REG_SELF_TEST_A = 0x10,

    MPU6050_REG_XG_OFFS_USRH = 0x13,
    MPU6050_REG_XG_OFFS_USRL = 0x14,
    MPU6050_REG_YG_OFFS_USRH = 0x15,
    MPU6050_REG_YG_OFFS_USRL = 0x16,
    MPU6050_REG_ZG_OFFS_USRH = 0x17,
    MPU6050_REG_ZG_OFFS_USRL = 0x18,

    MPU6050_REG_SMPLRT_DIV = 0x19,

    MPU6050_REG_CONFIG = 0x1A,
    MPU6050_REG_GYRO_CONFIG = 0x1B,
    MPU6050_REG_ACCEL_CONFIG = 0x1C,

    MPU6050_REG_FF_THR = 0x1D,
    MPU6050_REG_FF_DUR = 0x1E,

    MPU6050_REG_MOT_THR = 0x1F,
    MPU6050_REG_MOT_DUR = 0x20,

    MPU6050_REG_ZRMOT_THR = 0x21,
    MPU6050_REG_ZRMOT_DUR = 0x22,

    MPU6050_REG_FIFO_EN = 0x23,

    MPU6050_REG_I2C_MST_CTRL = 0x24,
    MPU6050_REG_I2C_SLV0_ADDR = 0x25,
    MPU6050_REG_I2C_SLV0_REG = 0x26,
    MPU6050_REG_I2C_SLV0_CTRL = 0x27,
    MPU6050_REG_I2C_SLV1_ADDR = 0x28,
    MPU6050_REG_I2C_SLV1_REG = 0x29,
    MPU6050_REG_I2C_SLV1_CTRL = 0x2A,
    MPU6050_REG_I2C_SLV2_ADDR = 0x2B,
    MPU6050_REG_I2C_SLV2_REG = 0x2C,
    MPU6050_REG_I2C_SLV2_CTRL = 0x2D,
    MPU6050_REG_I2C_SLV3_ADDR = 0x2E,
    MPU6050_REG_I2C_SLV3_REG = 0x2F,
    MPU6050_REG_I2C_SLV3_CTRL = 0x30,
    MPU6050_REG_I2C_SLV4_ADDR = 0x31,
    MPU6050_REG_I2C_SLV4_REG = 0x32,
    MPU6050_REG_I2C_SLV4_DO = 0x33,
    MPU6050_REG_I2C_SLV4_CTRL = 0x34,
    MPU6050_REG_I2C_SLV4_DI = 0x35,
    MPU6050_REG_I2C_MST_STATUS = 0x36,

    MPU6050_REG_INT_PIN_CFG = 0x37,
    MPU6050_REG_INT_ENABLE = 0x38,
    MPU6050_REG_DMP_INT_STATUS = 0x39,
    MPU6050_REG_INT_STATUS = 0x3A,

    MPU6050_REG_ACCEL_XOUT_H = 0x3B,
    MPU6050_REG_ACCEL_XOUT_L = 0x3C,
    MPU6050_REG_ACCEL_YOUT_H = 0x3D,
    MPU6050_REG_ACCEL_YOUT_L = 0x3E,
    MPU6050_REG_ACCEL_ZOUT_H = 0x3F,
    MPU6050_REG_ACCEL_ZOUT_L = 0x40,

    MPU6050_REG_TEMP_OUT_H = 0x41,
    MPU6050_REG_TEMP_OUT_L = 0x42,

    MPU6050_REG_GYRO_XOUT_H = 0x43,
    MPU6050_REG_GYRO_XOUT_L = 0x44,
    MPU6050_REG_GYRO_YOUT_H = 0x45,
    MPU6050_REG_GYRO_YOUT_L = 0x46,
    MPU6050_REG_GYRO_ZOUT_H = 0x47,
    MPU6050_REG_GYRO_ZOUT_L = 0x48,

    MPU6050_REG_EXT_SENS_DATA_00 = 0x49,
    MPU6050_REG_EXT_SENS_DATA_01 = 0x4A,
    MPU6050_REG_EXT_SENS_DATA_02 = 0x4B,
    MPU6050_REG_EXT_SENS_DATA_03 = 0x4C,
    MPU6050_REG_EXT_SENS_DATA_04 = 0x4D,
    MPU6050_REG_EXT_SENS_DATA_05 = 0x4E,
    MPU6050_REG_EXT_SENS_DATA_06 = 0x4F,
    MPU6050_REG_EXT_SENS_DATA_07 = 0x50,
    MPU6050_REG_EXT_SENS_DATA_08 = 0x51,
    MPU6050_REG_EXT_SENS_DATA_09 = 0x52,
    MPU6050_REG_EXT_SENS_DATA_10 = 0x53,
    MPU6050_REG_EXT_SENS_DATA_11 = 0x54,
    MPU6050_REG_EXT_SENS_DATA_12 = 0x55,
    MPU6050_REG_EXT_SENS_DATA_13 = 0x56,
    MPU6050_REG_EXT_SENS_DATA_14 = 0x57,
    MPU6050_REG_EXT_SENS_DATA_15 = 0x58,
    MPU6050_REG_EXT_SENS_DATA_16 = 0x59,
    MPU6050_REG_EXT_SENS_DATA_17 = 0x5A,
    MPU6050_REG_EXT_SENS_DATA_18 = 0x5B,
    MPU6050_REG_EXT_SENS_DATA_19 = 0x5C,
    MPU6050_REG_EXT_SENS_DATA_20 = 0x5D,
    MPU6050_REG_EXT_SENS_DATA_21 = 0x5E,
    MPU6050_REG_EXT_SENS_DATA_22 = 0x5F,
    MPU6050_REG_EXT_SENS_DATA_23 = 0x60,

    MPU6050_REG_MOV_DETECT_STATUS = 0x61,

    MPU6050_REG_I2C_SLV0_DO = 0x63,
    MPU6050_REG_I2C_SLV1_DO = 0x64,
    MPU6050_REG_I2C_SLV2_DO = 0x65,
    MPU6050_REG_I2C_SLV3_DO = 0x66,

    MPU6050_REG_I2C_MST_DELAY_CTRL = 0x67,

    MPU6050_REG_SIGNAL_PATH_RESET = 0x68,

    MPU6050_REG_MOT_DETECT_CTRL = 0x69,

    MPU6050_REG_USER_CTRL = 0x6A,

    MPU6050_REG_PWR_MGMT_1 = 0x6B,
    MPU6050_REG_PWR_MGMT_2 = 0x6C,

    MPU6050_REG_BANK_SEL = 0x6D,

    MPU6050_REG_MEM_START_ADDR = 0x6E,
    MPU6050_REG_MEM_R_W = 0x6F,

    MPU6050_REG_DMP_CFG_1 = 0x70,
    MPU6050_REG_DMP_CFG_2 = 0x71,

    MPU6050_REG_FIFO_COUNT_H = 0x72,
    MPU6050_REG_FIFO_COUNT_L = 0x73,

    MPU6050_REG_FIFO_R_W = 0x74,

    /// Device ID
    /// The value of the WHO_AM_I register should be MPU6050_DEVICE_ID
    MPU6050_REG_WHO_AM_I = 0x75,
} mpu6050_reg_t;

#define MPU6050_PWR_MGMT_1_DEVICE_RESET_SHIFT (7)
#define MPU6050_PWR_MGMT_1_DEVICE_RESET_LENGTH (1)
#define MPU6050_PWR_MGMT_1_DEVICE_RESET MPU6050_CREATE_MASK(MPU6050_PWR_MGMT_1_DEVICE_RESET_SHIFT, MPU6050_PWR_MGMT_1_DEVICE_RESET_LENGTH)

#define MPU6050_PWR_MGMT_1_SLEEP_SHIFT (6)
#define MPU6050_PWR_MGMT_1_SLEEP_LENGTH (1)
#define MPU6050_PWR_MGMT_1_SLEEP MPU6050_CREATE_MASK(MPU6050_PWR_MGMT_1_SLEEP_SHIFT, MPU6050_PWR_MGMT_1_SLEEP_LENGTH)

#define MPU6050_PWR_MGMT_1_CLKSEL_SHIFT (2)
#define MPU6050_PWR_MGMT_1_CLKSEL_LENGTH (3)
#define MPU6050_PWR_MGMT_1_CLKSEL MPU6050_CREATE_MASK(MPU6050_PWR_MGMT_1_CLKSEL_SHIFT, MPU6050_PWR_MGMT_1_CLKSEL_LENGTH)

/**
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 */
typedef enum mpu6050_clock_source
{
    MPU6050_CLOCK_INTERNAL_8MHZ = 0b000, // Internal oscillator
    MPU6050_CLOCK_PLL_XGYRO = 0b001, // PLL with X Gyro reference
    MPU6050_CLOCK_PLL_YGYRO = 0b010, // PLL with Y Gyro reference
    MPU6050_CLOCK_PLL_ZGYRO = 0b011, // PLL with Z Gyro reference
    MPU6050_CLOCK_EXTERNAL_32KHZ = 0b100, // PLL with external 32.768kHz reference
    MPU6050_CLOCK_EXTERNAL_19MHZ = 0b101, // PLL with external 19.2MHz reference
    MPU6050_CLOCK_KEEP_RESET = 0b111, // Stops the clock and keeps the timing generator in reset
} mpu6050_clock_source_t;

#define MPU6050_SIGNAL_PATH_RESET_GYRO_RESET_SHIFT (2)
#define MPU6050_SIGNAL_PATH_RESET_GYRO_RESET_LENGTH (1)
#define MPU6050_SIGNAL_PATH_RESET_GYRO_RESET MPU6050_CREATE_MASK(MPU6050_SIGNAL_PATH_RESET_GYRO_RESET_SHIFT, MPU6050_SIGNAL_PATH_RESET_GYRO_RESET_LENGTH)

#define MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET_SHIFT (1)
#define MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET_LENGTH (1)
#define MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET MPU6050_CREATE_MASK(MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET_SHIFT, MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET_LENGTH)

#define MPU6050_SIGNAL_PATH_RESET_TEMP_RESET_SHIFT (0)
#define MPU6050_SIGNAL_PATH_RESET_TEMP_RESET_LENGTH (1)
#define MPU6050_SIGNAL_PATH_RESET_TEMP_RESET MPU6050_CREATE_MASK(MPU6050_SIGNAL_PATH_RESET_TEMP_RESET_SHIFT, MPU6050_SIGNAL_PATH_RESET_TEMP_RESET_LENGTH)

#define MPU6050_GYRO_CONFIG_FS_SEL_SHIFT (3)
#define MPU6050_GYRO_CONFIG_FS_SEL_LENGTH (2)
#define MPU6050_GYRO_CONFIG_FS_SEL_MASK MPU6050_CREATE_MASK(MPU6050_GYRO_CONFIG_FS_SEL_SHIFT, MPU6050_GYRO_CONFIG_FS_SEL_LENGTH)

typedef enum mpu6050_gyro_range
{
    MPU6050_GYRO_RANGE_250_DEG = 0b00,
    MPU6050_GYRO_RANGE_500_DEG = 0b01,
    MPU6050_GYRO_RANGE_1000_DEG = 0b10,
    MPU6050_GYRO_RANGE_2000_DEG = 0b11,
} mpu6050_gyro_range_t;

#define MPU6050_ACCEL_CONFIG_AFS_SEL_SHIFT (3)
#define MPU6050_ACCEL_CONFIG_AFS_SEL_LENGTH (2)
#define MPU6050_ACCEL_CONFIG_AFS_SEL_MASK MPU6050_CREATE_MASK(MPU6050_ACCEL_CONFIG_AFS_SEL_SHIFT, MPU6050_ACCEL_CONFIG_AFS_SEL_LENGTH)

typedef enum mpu6050_accel_range
{
    MPU6050_ACCEL_RANGE_2G = 0b00,
    MPU6050_ACCEL_RANGE_4G = 0b01,
    MPU6050_ACCEL_RANGE_8G = 0b10,
    MPU6050_ACCEL_RANGE_16G = 0b11,
} mpu6050_accel_range_t;

#define MPU6050_REG_CONFIG_EXT_SYNC_SHIFT (3)
#define MPU6050_REG_CONFIG_EXT_SYNC_LENGTH (3)
#define MPU6050_REG_CONFIG_EXT_SYNC MPU6050_CREATE_MASK(MPU6050_REG_CONFIG_EXT_SYNC_SHIFT, MPU6050_REG_CONFIG_EXT_SYNC_LENGTH)

/**
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 */
typedef enum mpu6050_ext_sync
{
    MPU6050_EXT_SYNC_DISABLED = 0b000,
    MPU6050_EXT_SYNC_TEMP_OUT_L = 0b001,
    MPU6050_EXT_SYNC_GYRO_XOUT_L = 0b010,
    MPU6050_EXT_SYNC_GYRO_YOUT_L = 0b011,
    MPU6050_EXT_SYNC_GYRO_ZOUT_L = 0b100,
    MPU6050_EXT_SYNC_ACCEL_XOUT_L = 0b101,
    MPU6050_EXT_SYNC_ACCEL_YOUT_L = 0b110,
    MPU6050_EXT_SYNC_ACCEL_ZOUT_L = 0b111,
} mpu6050_ext_sync_t;

#define MPU6050_REG_CONFIG_DLPF_CFG_SHIFT (0)
#define MPU6050_REG_CONFIG_DLPF_CFG_LENGTH (3)
#define MPU6050_REG_CONFIG_DLPF_CFG_MASK MPU6050_CREATE_MASK(MPU6050_REG_CONFIG_DLPF_CFG_SHIFT, MPU6050_REG_CONFIG_DLPF_CFG_LENGTH)

/**
 * @brief Digital Low Pass Filter (DLPF) settings.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @see mpu6050_set_filter_bandwidth()
 */
typedef enum mpu6050_filter_bandwidth
{
    MPU6050_BANDWIDTH_260_HZ = 0, // Accel: 260 Hz, Gyro: 256 Hz
    MPU6050_BANDWIDTH_184_HZ = 1, // Accel: 184 Hz, Gyro: 188 Hz
    MPU6050_BANDWIDTH_94_HZ = 2, // Accel: 94 Hz, Gyro: 98 Hz
    MPU6050_BANDWIDTH_44_HZ = 3, // Accel: 44 Hz, Gyro: 42 Hz
    MPU6050_BANDWIDTH_21_HZ = 4, // Accel: 21 Hz, Gyro: 20 Hz
    MPU6050_BANDWIDTH_10_HZ = 5, // Accel: 10 Hz, Gyro: 10 Hz
    MPU6050_BANDWIDTH_5_HZ = 6, // Accel: 5 Hz, Gyro: 5 Hz
} mpu6050_filter_bandwidth_t;

typedef enum mpu6050_highpass
{
    MPU6050_HIGHPASS_OFF = 0b0000,
    MPU6050_HIGHPASS_5_HZ = 0b0001,
    MPU6050_HIGHPASS_2_5_HZ = 0b0010,
    MPU6050_HIGHPASS_1_25_HZ = 0b0011,
    MPU6050_HIGHPASS_0_63_HZ = 0b0100,
    MPU6050_HIGHPASS_UNUSED = 0b0101,
    MPU6050_HIGHPASS_HOLD = 0b0110,
} mpu6050_highpass_t;

typedef enum mpu6050_cycle_rate
{
    MPU6050_CYCLE_RATE_1_25_HZ = 0b000,
    MPU6050_CYCLE_RATE_5_HZ = 0b001,
    MPU6050_CYCLE_RATE_20_HZ = 0b010,
    MPU6050_CYCLE_RATE_40_HZ = 0b011,
} mpu6050_cycle_rate_t;

typedef struct mpu6050_dev
{
    i2cdev_bus_t* bus;
    i2cdev_dev_addr_t addr;
    i2cdev_delay_api_t* delay;
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t gyro_range;
} mpu6050_dev_t;

typedef struct mpu6050_3axis_raw_data
{
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_3axis_raw_data_t;

typedef struct mpu6050_3axis_data
{
    float x;
    float y;
    float z;
} mpu6050_3axis_data_t;

typedef struct mpu6050_motion_data
{
    mpu6050_3axis_data_t accel;
    mpu6050_3axis_data_t gyro;
} mpu6050_motion_data_t;

typedef struct mpu6050_all_data
{
    mpu6050_3axis_data_t accel;
    mpu6050_3axis_data_t gyro;
    float temperature;
} mpu6050_all_data;

static mpu6050_3axis_data mpu6050_process_accel_data(mpu6050_3axis_raw_data_t* raw_data, mpu6050_accel_range_t range)
{
    mpu6050_3axis_data_t data;

    float scale = 1.0f;
    switch (range)
    {
    case MPU6050_ACCEL_RANGE_2G:
        scale = 16384.0f;
        break;
    case MPU6050_ACCEL_RANGE_4G:
        scale = 8192.0f;
        break;
    case MPU6050_ACCEL_RANGE_8G:
        scale = 4096.0f;
        break;
    case MPU6050_ACCEL_RANGE_16G:
        scale = 2048.0f;
        break;
    }

    data.x = (float)raw_data->x / scale;
    data.y = (float)raw_data->y / scale;
    data.z = (float)raw_data->z / scale;

    return data;
}

/**
 * @brief Convert temperature data to reasonable value.
 *
 * Converts temperature data from the MPU6050 to a float value in degrees Celsius.
 */
static float mpu6050_process_temperature(const int16_t temp)
{
    return (float)temp / 340.0f + 36.53f;
}

static mpu6050_3axis_data mpu6050_process_gyro_data(mpu6050_3axis_raw_data_t* raw_data, mpu6050_gyro_range_t range)
{
    mpu6050_3axis_data_t data;

    float scale = 1.0f;
    switch (range)
    {
    case MPU6050_GYRO_RANGE_250_DEG:
        scale = 131.0f;
        break;
    case MPU6050_GYRO_RANGE_500_DEG:
        scale = 65.5f;
        break;
    case MPU6050_GYRO_RANGE_1000_DEG:
        scale = 32.8f;
        break;
    case MPU6050_GYRO_RANGE_2000_DEG:
        scale = 16.4f;
        break;
    }

    data.x = (float)raw_data->x / scale;
    data.y = (float)raw_data->y / scale;
    data.z = (float)raw_data->z / scale;

    return data;
}

inline int mpu6050_check(mpu6050_dev_t* dev)
{
    uint8_t whoami;

    // read WHO_AM_I register
    const int ret = i2cdev_reg_read_u8(dev->bus, dev->addr, MPU6050_REG_WHO_AM_I, &whoami);
    if (ret < 0)
    {
        return ret;
    }

    if (whoami != MPU6050_DEVICE_ID)
    {
        return I2CDEV_RESULT_ENODEV;
    }

    return I2CDEV_RESULT_OK;
}

/**
 * @brief Put or wake up the MPU6050.
 */
inline int mpu6050_set_sleep_enabled(mpu6050_dev_t* dev, const bool enabled)
{
    return i2cdev_reg_write_bit(dev->bus, dev->addr, MPU6050_REG_PWR_MGMT_1,
                                MPU6050_PWR_MGMT_1_SLEEP_SHIFT, enabled);
}

/**
 * @brief Check if the MPU6050 is in sleep mode.
 *
 * @param [out] enabled Pointer to store the sleep status.
 */
inline int mpu6050_read_sleep_enabled(mpu6050_dev_t* dev, bool* enabled)
{
    return i2cdev_reg_read_bit(dev->bus, dev->addr, MPU6050_REG_PWR_MGMT_1,
                               MPU6050_PWR_MGMT_1_SLEEP_SHIFT, enabled);
}

/**
 * @brief Set the clock source for the MPU6050.
 *
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @see mpu6050_clock_source_t
 *
 * @param [in] dev Pointer to the device structure.
 * @param [in] source The clock source to set.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_set_clock_source(mpu6050_dev_t* dev, const mpu6050_clock_source_t source)
{
    return i2cdev_reg_update_u8(dev->bus, dev->addr, MPU6050_REG_PWR_MGMT_1,
                                MPU6050_PWR_MGMT_1_CLKSEL,
                                (uint8_t)source << MPU6050_PWR_MGMT_1_CLKSEL_SHIFT);
}

/**
 * @brief Read the clock source from the MPU6050.
 *
 * @see mpu6050_set_clock_source()
 *
 * @param [in] dev Pointer to the device structure.
 * @param [out] source Pointer to store the clock source.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_read_clock_source(mpu6050_dev_t* dev, mpu6050_clock_source_t* source)
{
    uint8_t tmp;
    int ret = i2cdev_reg_read_u8(dev->bus, dev->addr, MPU6050_REG_PWR_MGMT_1, &tmp);
    if (ret < 0)
    {
        return ret;
    }

    *source = (mpu6050_clock_source_t)((tmp & MPU6050_PWR_MGMT_1_CLKSEL) >> MPU6050_PWR_MGMT_1_CLKSEL_SHIFT);
    return I2CDEV_RESULT_OK;
}

/**
 * @brief Set the full-scale range for the gyroscope.
 *
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * FS_SEL | Full Scale Range
 * -------+----------------------
 * 0      | +/- 250 degrees/sec
 * 1      | +/- 500 degrees/sec
 * 2      | +/- 1000 degrees/sec
 * 3      | +/- 2000 degrees/sec
 * </pre>
 *
 * @see mpu6050_gyro_range_t
 *
 * @param [in] dev Pointer to the device structure.
 * @param [in] range The full-scale range to set.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_set_gyro_range(mpu6050_dev_t* dev, const mpu6050_gyro_range_t range)
{
    return i2cdev_reg_update_u8(dev->bus, dev->addr, MPU6050_REG_GYRO_CONFIG,
                                MPU6050_GYRO_CONFIG_FS_SEL_MASK,
                                (uint8_t)range << MPU6050_GYRO_CONFIG_FS_SEL_SHIFT);
}

inline int mpu6050_read_gyro_range(mpu6050_dev_t* dev, mpu6050_gyro_range_t* range)
{
    uint8_t tmp;
    int ret = i2cdev_reg_read_u8(dev->bus, dev->addr, MPU6050_REG_GYRO_CONFIG, &tmp);

    if (ret < 0)
    {
        return ret;
    }

    *range = (mpu6050_gyro_range_t)((tmp & MPU6050_GYRO_CONFIG_FS_SEL_MASK) >> MPU6050_GYRO_CONFIG_FS_SEL_SHIFT);

    return I2CDEV_RESULT_OK;
}

/**
 * @brief Set the full-scale range for the gyroscope.
 *
 * The AFS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * AFS_SEL | Full Scale Range
 * -------+----------------------
 * 0      | +/- 2g
 * 1      | +/- 4g
 * 2      | +/- 8g
 * 3      | +/- 16g
 * </pre>
 *
 * @see mpu6050_gyro_range_t
 *
 * @param [in] dev Pointer to the device structure.
 * @param [in] range The full-scale range to set.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_set_accelerometer_range(mpu6050_dev_t* dev, const mpu6050_accel_range_t range)
{
    return i2cdev_reg_update_u8(dev->bus, dev->addr, MPU6050_REG_ACCEL_CONFIG,
                                MPU6050_ACCEL_CONFIG_AFS_SEL_MASK,
                                (uint8_t)range << MPU6050_ACCEL_CONFIG_AFS_SEL_SHIFT);
}

/**
 * @brief Read the full-scale range for the gyroscope.
 *
 * @see mpu6050_set_accelerometer_range()
 *
 * @param [in] dev Pointer to the device structure.
 * @param [out] range Pointer to store the full-scale range.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_read_accelerometer_range(mpu6050_dev_t* dev, mpu6050_accel_range_t* range)
{
    uint8_t tmp;
    int ret = i2cdev_reg_read_u8(dev->bus, dev->addr, MPU6050_REG_ACCEL_CONFIG, &tmp);

    if (ret < 0)
    {
        return ret;
    }

    *range = (mpu6050_accel_range_t)((tmp & MPU6050_ACCEL_CONFIG_AFS_SEL_MASK) >> MPU6050_ACCEL_CONFIG_AFS_SEL_SHIFT);

    return I2CDEV_RESULT_OK;
}

/**
 * @brief Set external FSYNC configuration.
 *
 * Configures the external Frame Synchronization (FSYNC) pin sampling. An
 * external signal connected to the FSYNC pin can be sampled by configuring
 * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
 * strobes may be captured. The latched FSYNC signal will be sampled at the
 * Sampling Rate, as defined in register 25. After sampling, the latch will
 * reset to the current FSYNC signal state.
 *
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table.
 *
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 *
 * @see mpu6050_ext_sync_t
 *
 * @param [in] dev Pointer to the device structure.
 * @param [in] sync The external FSYNC configuration to set.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_set_ext_sync(mpu6050_dev_t* dev, const mpu6050_ext_sync_t sync)
{
    return i2cdev_reg_update_u8(dev->bus, dev->addr, MPU6050_REG_CONFIG,
                                MPU6050_REG_CONFIG_EXT_SYNC,
                                (uint8_t)sync << MPU6050_REG_CONFIG_EXT_SYNC_SHIFT);
}

/**
 * @brief Read the external FSYNC configuration.
 *
 * @see mpu6050_set_ext_sync()
 *
 * @param [in] dev Pointer to the device structure.
 * @param [out] sync Pointer to store the external FSYNC configuration.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_read_ext_sync(mpu6050_dev_t* dev, mpu6050_ext_sync_t* sync)
{
    uint8_t tmp;
    int ret = i2cdev_reg_read_u8(dev->bus, dev->addr, MPU6050_REG_CONFIG, &tmp);

    if (ret < 0)
    {
        return ret;
    }

    *sync = (mpu6050_ext_sync_t)((tmp & MPU6050_REG_CONFIG_EXT_SYNC) >> MPU6050_REG_CONFIG_EXT_SYNC_SHIFT);

    return I2CDEV_RESULT_OK;
}

/**
 * @brief Set the Digital Low Pass Filter (DLPF) bandwidth for the accelerometer and gyroscope.
 *
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @see mpu6050_filter_bandwidth_t
 *
 * @param [in] dev Pointer to the device structure.
 * @param [in] bandwidth The DLPF bandwidth to set.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_set_filter_bandwidth(mpu6050_dev_t* dev, const mpu6050_filter_bandwidth_t bandwidth)
{
    return i2cdev_reg_update_u8(dev->bus, dev->addr, MPU6050_REG_CONFIG,
                                MPU6050_REG_CONFIG_DLPF_CFG_MASK,
                                (uint8_t)bandwidth << MPU6050_REG_CONFIG_DLPF_CFG_SHIFT);
}

inline int mpu6050_read_filter_bandwidth(mpu6050_dev_t* dev, mpu6050_filter_bandwidth_t* bandwidth)
{
    uint8_t tmp;
    int ret = i2cdev_reg_read_u8(dev->bus, dev->addr, MPU6050_REG_CONFIG, &tmp);

    if (ret < 0)
    {
        return ret;
    }

    *bandwidth = (mpu6050_filter_bandwidth_t)((tmp & MPU6050_REG_CONFIG_DLPF_CFG_MASK) >>
        MPU6050_REG_CONFIG_DLPF_CFG_SHIFT);

    return I2CDEV_RESULT_OK;
}

inline int mpu6050_read_raw_accelerometer_measurements(mpu6050_dev_t* dev, mpu6050_3axis_raw_data* accel)
{
    uint8_t tmp[6];

    int ret = i2cdev_reg_burst_read_u8(dev->bus, dev->addr, MPU6050_REG_ACCEL_XOUT_H, tmp, sizeof(tmp));
    if (ret < 0)
    {
        return ret;
    }

    accel->x = (int16_t)((tmp[0] << 8) | tmp[1]);
    accel->y = (int16_t)((tmp[2] << 8) | tmp[3]);
    accel->z = (int16_t)((tmp[4] << 8) | tmp[5]);

    return I2CDEV_RESULT_OK;
}

inline int mpu6050_read_raw_temperature_measurements(mpu6050_dev_t* dev, int16_t* temp)
{
    uint8_t tmp[2];

    int ret = i2cdev_reg_burst_read_u8(dev->bus, dev->addr, MPU6050_REG_TEMP_OUT_H, tmp, sizeof(tmp));
    if (ret < 0)
    {
        return ret;
    }

    *temp = (int16_t)((tmp[0] << 8) | tmp[1]);

    return I2CDEV_RESULT_OK;
}

inline int mpu6050_read_raw_gyro_measurements(mpu6050_dev_t* dev, mpu6050_3axis_raw_data* gyro)
{
    uint8_t tmp[6];

    int ret = i2cdev_reg_burst_read_u8(dev->bus, dev->addr, MPU6050_REG_GYRO_XOUT_H, tmp, sizeof(tmp));
    if (ret < 0)
    {
        return ret;
    }

    gyro->x = (int16_t)((tmp[0] << 8) | tmp[1]);
    gyro->y = (int16_t)((tmp[2] << 8) | tmp[3]);
    gyro->z = (int16_t)((tmp[4] << 8) | tmp[5]);

    return I2CDEV_RESULT_OK;
}

inline int mpu6050_read_raw_all_measurements(mpu6050_dev* dev,
                                             mpu6050_3axis_raw_data* accel,
                                             int16_t* temp,
                                             mpu6050_3axis_raw_data* gyro)
{
    uint8_t tmp[14]; // 6 accel + 2 temp + 6 gyro

    int ret = i2cdev_reg_burst_read_u8(dev->bus, dev->addr, MPU6050_REG_ACCEL_XOUT_H, tmp, sizeof(tmp));
    if (ret < 0)
    {
        return ret;
    }

    accel->x = (int16_t)((tmp[0] << 8) | tmp[1]);
    accel->y = (int16_t)((tmp[2] << 8) | tmp[3]);
    accel->z = (int16_t)((tmp[4] << 8) | tmp[5]);

    *temp = (int16_t)((tmp[6] << 8) | tmp[7]);

    gyro->x = (int16_t)((tmp[8] << 8) | tmp[9]);
    gyro->y = (int16_t)((tmp[10] << 8) | tmp[11]);
    gyro->z = (int16_t)((tmp[12] << 8) | tmp[13]);

    return I2CDEV_RESULT_OK;
}

/**
 * @brief Read accelerometer and gyroscope measurements.
 *
 * @note This function still reads the temperature data, since it is faster to read all measurement registers,
 * but stores it in a throw-away variable.
 *
 * @see mpu6050_read_raw_all_measurements()
 *
 * @param [in] dev Pointer to the device structure.
 * @param [out] accel Pointer to store the accelerometer raw measurements.
 * @param [out] gyro Pointer to store the gyroscope raw measurements.
 *
 * @retval I2CDEV_RESULT_OK If successful.
 * @retval negative Error code if an error occurred.
 */
inline int mpu6050_read_motion_measurements(mpu6050_dev* dev,
                                            mpu6050_3axis_raw_data* accel,
                                            mpu6050_3axis_raw_data* gyro)
{
    int16_t temp;

    return mpu6050_read_raw_all_measurements(dev, accel, &temp, gyro);
}

inline int mpu6050_reset(mpu6050_dev_t* dev)
{
    return i2cdev_reg_update_u8(dev->bus, dev->addr, MPU6050_REG_PWR_MGMT_1,
                                MPU6050_PWR_MGMT_1_DEVICE_RESET,
                                MPU6050_PWR_MGMT_1_DEVICE_RESET);
}

#ifdef __cplusplus
};
#endif // __cplusplus

#endif //I2CDEVLIB_MPU6050_H_
