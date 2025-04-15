#ifndef __I2CDEVLIB_MPU6050_HPP__
#define __I2CDEVLIB_MPU6050_HPP__

#include "i2cdev/mpu6050.h"
#include "i2cdevbus.hpp"

namespace i2cdev {
    class MPU6050 {
      public:
#ifdef I2CDEV_DEFAULT_BUS
        MPU6050(uint8_t addr = MPU6050_I2CADDR_BASE, I2CDevBus& bus = I2CDEV_DEFAULT_BUS) : _addr(addr), _bus(bus) {}
#else
        MPU6050(uint8_t addr, I2CDevBus& bus) : _addr(addr), _bus(bus) {}
#endif

        [[nodiscard]] auto check() -> i2cdev_result_t;

        [[nodiscard]] auto reset() -> i2cdev_result_t;

        [[nodiscard]] auto setAccelerometerRange(mpu6050_accel_range_t range) -> i2cdev_result_t;

        auto getAccelerometerRange() -> mpu6050_accel_range_t;

        [[nodiscard]] auto setGyroscopeRange(mpu6050_gyro_range_t range) -> i2cdev_result_t;

        auto getGyroscopeRange() -> mpu6050_gyro_range_t;

        [[nodiscard]] auto setFilterBandwidth(mpu6050_bandwidth_t bandwidth) -> i2cdev_result_t;

        auto getFilterBandwidth() -> mpu6050_bandwidth_t;

        [[nodiscard]] auto setFsyncSampleOutput(mpu6050_fsync_t fsync) -> i2cdev_result_t;

        auto getFsyncSampleOutput() -> mpu6050_fsync_t;

        auto getAccelerometerMeasurements() -> mpu6050_3axis_data;

        auto getTemperatureMeasurements() -> float;

        auto getGyroscopeMeasurements() -> mpu6050_3axis_data;

        auto getAllMeasurements() -> mpu6050_all_data;

      protected:

        [[nodiscard]] auto readRawAccelerometerMeasurements(int16_t* accelX, int16_t* accelY, int16_t* accelZ) -> i2cdev_result_t;

        [[nodiscard]] auto readRawTemperatureMeasurements(int16_t* temp) -> i2cdev_result_t;

        [[nodiscard]] auto readRawGyroscopeMeasurements(int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ) -> i2cdev_result_t;

        [[nodiscard]] auto readRawMotionMeasurements(
            int16_t* accelX, int16_t* accelY, int16_t* accelZ,
            int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ
        ) -> i2cdev_result_t;

        [[nodiscard]] auto readRawAllMeasurements(
            int16_t* accelX, int16_t* accelY, int16_t* accelZ,
            int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ,
            int16_t* temp
        ) -> i2cdev_result_t;

      private:
        uint8_t _addr;
        I2CDevBus& _bus;

        mpu6050_accel_range_t _accel_range;
        mpu6050_gyro_range_t _gyro_range;
    };

    inline auto MPU6050::check() -> i2cdev_result_t {
        uint8_t data;
        i2cdev_result_t result;

        // wait for the device to be ready
        for (int i = 0; i < 5; i++) {
            result = this->_bus.readReg8(this->_addr, MPU6050_REG_WHO_AM_I, &data);
            if (result == I2CDEV_RESULT_OK) {
                break;
            }

            i2cdev_platform_sleep_us(10000);
        }

        if (result != I2CDEV_RESULT_OK || data != MPU6050_DEVICE_ID) {
            I2CDEVLIB_LOG_E("Device check failure. Result: %i, ID: %i", result, data);
            return I2CDEV_RESULT_EIO;
        }

        return result;
    }

    inline auto MPU6050::reset() -> i2cdev_result_t {
        i2cdev_result_t result;

        result = this->_bus.updateReg8(
            this->_addr,
            MPU6050_REG_PWR_MGMT_1,
            MPU6050_PWR_MGMT_1_DEVICE_RESET,
            MPU6050_PWR_MGMT_1_DEVICE_RESET
        );
        if (result != I2CDEV_RESULT_OK) {
            return result;
        }

        uint8_t data;
        do {
            result = this->_bus.readReg8(this->_addr, MPU6050_REG_PWR_MGMT_1, &data);
            if (result != I2CDEV_RESULT_OK) {
                I2CDEVLIB_LOG_E("Failed to read PWR_MGMT_1 after reset");
                return result;
            }

            i2cdev_platform_sleep_us(1000);
        } while (data & MPU6050_PWR_MGMT_1_DEVICE_RESET);

        i2cdev_platform_sleep_us(10000);

        // reset signal paths
        result = this->_bus.updateReg8(
            this->_addr,
            MPU6050_REG_SIGNAL_PATH_RESET,
            MPU6050_SIGNAL_PATH_RESET_GYRO_RESET | MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET | MPU6050_SIGNAL_PATH_RESET_TEMP_RESET,
            MPU6050_SIGNAL_PATH_RESET_GYRO_RESET | MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET | MPU6050_SIGNAL_PATH_RESET_TEMP_RESET
        );
        if (result != I2CDEV_RESULT_OK) {
            return result;
        }

        i2cdev_platform_sleep_us(1000);

        return result;
    }

    inline auto MPU6050::setAccelerometerRange(mpu6050_accel_range_t range) -> i2cdev_result_t {
        auto result = this->_bus.updateReg8Bits(
            this->_addr,
            MPU6050_REG_ACCEL_CONFIG,
            MPU6050_ACCEL_CONFIG_AFS_SEL_SHIFT,
            MPU6050_ACCEL_CONFIG_AFS_SEL_LENGTH,
            range
        );

        if (result != I2CDEV_RESULT_OK) {
            return result;
        }

        this->_accel_range = range;

        return result;
    }

    inline auto MPU6050::getAccelerometerRange() -> mpu6050_accel_range_t {
        uint8_t data;
        if (this->_bus.readReg8(this->_addr, MPU6050_REG_ACCEL_CONFIG, &data) != I2CDEV_RESULT_OK) {
            I2CDEVLIB_LOG_E("Failed to read ACCEL_CONFIG register");
            return this->_accel_range;
        }

        this->_accel_range = static_cast<mpu6050_accel_range_t>(
            (data >> MPU6050_ACCEL_CONFIG_AFS_SEL_SHIFT) & MPU6050_ACCEL_CONFIG_AFS_SEL_MASK
        );

        return this->_accel_range;
    }

    inline auto MPU6050::setGyroscopeRange(mpu6050_gyro_range_t range) -> i2cdev_result_t {
        auto result = this->_bus.updateReg8Bits(
            this->_addr,
            MPU6050_REG_GYRO_CONFIG,
            MPU6050_GYRO_CONFIG_FS_SEL_SHIFT,
            MPU6050_GYRO_CONFIG_FS_SEL_LENGTH,
            range
        );

        if (result != I2CDEV_RESULT_OK) {
            return result;
        }

        this->_gyro_range = range;

        return result;
    }

    inline auto MPU6050::getGyroscopeRange() -> mpu6050_gyro_range_t {
        uint8_t data;
        if (this->_bus.readReg8(this->_addr, MPU6050_REG_GYRO_CONFIG, &data) != I2CDEV_RESULT_OK) {
            I2CDEVLIB_LOG_E("Failed to read GYRO_CONFIG register");
            return this->_gyro_range;
        }

        this->_gyro_range = static_cast<mpu6050_gyro_range_t>(
            (data >> MPU6050_GYRO_CONFIG_FS_SEL_SHIFT) & MPU6050_GYRO_CONFIG_FS_SEL_MASK
        );

        return this->_gyro_range;
    }

    inline auto MPU6050::setFilterBandwidth(mpu6050_bandwidth_t bandwidth) -> i2cdev_result_t {
        return this->_bus.updateReg8Bits(
            this->_addr,
            MPU6050_REG_CONFIG,
            MPU6050_REG_CONFIG_DLPF_CFG_SHIFT,
            MPU6050_REG_CONFIG_DLPF_CFG_LENGTH,
            bandwidth
        );
    }

    inline auto MPU6050::getFilterBandwidth() -> mpu6050_bandwidth_t {
        uint8_t data;
        if (this->_bus.readReg8(this->_addr, MPU6050_REG_CONFIG, &data) != I2CDEV_RESULT_OK) {
            I2CDEVLIB_LOG_E("Failed to read CONFIG register");
            return static_cast<mpu6050_bandwidth_t>(0);
        }

        return static_cast<mpu6050_bandwidth_t>(
            (data >> MPU6050_REG_CONFIG_DLPF_CFG_SHIFT) & MPU6050_REG_CONFIG_DLPF_CFG_MASK
        );
    }

    inline auto MPU6050::getAccelerometerMeasurements() -> mpu6050_3axis_data {
        int16_t accelX, accelY, accelZ;
        if (this->readRawAccelerometerMeasurements(&accelX, &accelY, &accelZ) != I2CDEV_RESULT_OK) {
            return {NAN, NAN, NAN};
        }
        return mpu6050_process_accel_data(accelX, accelY, accelZ, this->_accel_range);
    }

    inline auto MPU6050::getTemperatureMeasurements() -> float {
        int16_t temp;
        if (this->readRawTemperatureMeasurements(&temp) != I2CDEV_RESULT_OK) {
            return NAN;
        }
        return mpu6050_process_temperature(temp);
    }

    inline auto MPU6050::getGyroscopeMeasurements() -> mpu6050_3axis_data {
        int16_t gyroX, gyroY, gyroZ;
        if (this->readRawGyroscopeMeasurements(&gyroX, &gyroY, &gyroZ) != I2CDEV_RESULT_OK) {
            return {NAN, NAN, NAN};
        }
        return mpu6050_process_gyro_data(gyroX, gyroY, gyroZ, this->_gyro_range);
    }

    inline auto MPU6050::getAllMeasurements() -> mpu6050_all_data {
        int16_t accelX, accelY, accelZ;
        int16_t gyroX, gyroY, gyroZ;
        int16_t temp;

        if (this->readRawAllMeasurements(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ, &temp) != I2CDEV_RESULT_OK) {
            return { NAN, NAN, NAN, NAN, NAN, NAN, NAN };
        }

        return {
            .accel = mpu6050_process_accel_data(accelX, accelY, accelZ, this->_accel_range),
            .gyro = mpu6050_process_gyro_data(gyroX, gyroY, gyroZ, this->_gyro_range),
            .temperature = mpu6050_process_temperature(temp),
        };
    }

    inline auto MPU6050::readRawAccelerometerMeasurements(int16_t *accelX, int16_t *accelY,
                                                          int16_t *accelZ) -> i2cdev_result_t {
        int16_t buffer[3];

        i2cdev_result_t result = this->_bus.readReg16(
            this->_addr,
            MPU6050_REG_ACCEL_XOUT_H,
            6,
            reinterpret_cast<uint16_t*>(buffer)
        );
        if (result != I2CDEV_RESULT_OK) {
            return result;
        }

        *accelX = buffer[0];
        *accelY = buffer[1];
        *accelZ = buffer[2];

        return result;
    }

    inline auto MPU6050::readRawTemperatureMeasurements(int16_t *temp) -> i2cdev_result_t {
        i2cdev_result_t result = this->_bus.readReg16(
            this->_addr,
            MPU6050_REG_TEMP_OUT_H,
            1,
            reinterpret_cast<uint16_t*>(temp)
        );
        if (result != I2CDEV_RESULT_OK) {
            return result;
        }

        return result;
    }

    inline auto MPU6050::readRawGyroscopeMeasurements(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) -> i2cdev_result_t {
        int16_t buffer[3];

        i2cdev_result_t result = this->_bus.readReg16(
            this->_addr,
            MPU6050_REG_GYRO_XOUT_H,
            3,
            reinterpret_cast<uint16_t*>(buffer)
        );
        if (result != I2CDEV_RESULT_OK) {
            return result;
        }

        *gyroX = buffer[0];
        *gyroY = buffer[1];
        *gyroZ = buffer[2];

        return result;
    }

    inline auto MPU6050::readRawMotionMeasurements(int16_t *accelX, int16_t *accelY, int16_t *accelZ,
                                                   int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) -> i2cdev_result_t {
        int16_t temp;

        return this->readRawAllMeasurements(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, &temp);
    }

    inline auto MPU6050::readRawAllMeasurements(int16_t *accelX, int16_t *accelY, int16_t *accelZ,
                                                int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ,
                                                int16_t *temp) -> i2cdev_result_t {
        int16_t buffer[7];

        i2cdev_result_t result = this->_bus.readReg16(
            this->_addr,
            MPU6050_REG_ACCEL_XOUT_H,
            7,
            reinterpret_cast<uint16_t*>(buffer)
        );
        if (result != I2CDEV_RESULT_OK) {
            return result;
        }

        *accelX = buffer[0];
        *accelY = buffer[1];
        *accelZ = buffer[2];

        *temp = buffer[3];

        *gyroX = buffer[4];
        *gyroY = buffer[5];
        *gyroZ = buffer[6];

        return result;
    }
}

#endif //__I2CDEVLIB_MPU6050_HPP__
