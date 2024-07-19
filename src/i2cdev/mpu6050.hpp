#ifndef __I2CDEVLIB_MPU6050_HPP__
#define __I2CDEVLIB_MPU6050_HPP__

#include "i2cdev/mpu6050.h"
#include "i2cdevbus.hpp"

namespace i2cdev {
    class MPU6050 {
      public:
#ifdef I2CDEV_DEFAULT_BUS
        MPU6050(uint8_t addr = MPU6050_I2CADDR_BASE, I2CDevBus* bus = &I2CDEV_DEFAULT_BUS) : _addr(addr), _bus(bus) {}
#else
        MPU6050(uint8_t addr, I2CDevBus *bus) : _addr(addr), _bus(bus) {}
#endif

        [[nodiscard]] auto check() -> i2cdev_result_t {
            uint8_t data;
            i2cdev_result_t result = this->_bus->readReg8(this->_addr, MPU6050_REG_WHO_AM_I, &data);

            return (result == I2CDEV_RESULT_OK && data == MPU6050_DEVICE_ID) ? result : I2CDEV_RESULT_ERROR;
        }

        [[nodiscard]] auto reset() -> i2cdev_result_t {
            i2cdev_result_t result;

            result = this->_bus->updateReg8(
                this->_addr,
                MPU6050_REG_PWR_MGMT_1,
                MPU6050_PWR_MGMT_1_DEVICE_RESET,
                MPU6050_PWR_MGMT_1_DEVICE_RESET
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            uint8_t data = MPU6050_PWR_MGMT_1_DEVICE_RESET;
            do {
                result = this->_bus->readReg8(this->_addr, MPU6050_REG_PWR_MGMT_1, &data);
                if (result != I2CDEV_RESULT_OK) {
                    return result;
                }
            } while (data & MPU6050_PWR_MGMT_1_DEVICE_RESET);

            // reset signal paths
            result = this->_bus->updateReg8(
                this->_addr,
                MPU6050_REG_SIGNAL_PATH_RESET,
                MPU6050_SIGNAL_PATH_RESET_GYRO_RESET | MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET | MPU6050_SIGNAL_PATH_RESET_TEMP_RESET,
                MPU6050_SIGNAL_PATH_RESET_GYRO_RESET | MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET | MPU6050_SIGNAL_PATH_RESET_TEMP_RESET
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            return result;
        }

        [[nodiscard]] auto setAccelerometerRange(mpu6050_accel_range_t range) -> i2cdev_result_t {
            auto result = this->_bus->updateReg8Bits(
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

        [[nodiscard]] auto setGyroscopeRange(mpu6050_gyro_range_t range) -> i2cdev_result_t {
            auto result = this->_bus->updateReg8Bits(
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

        [[nodiscard]] inline auto setFilterBandwidth(mpu6050_bandwidth_t bandwidth) -> i2cdev_result_t {
            return this->_bus->updateReg8Bits(
                this->_addr,
                MPU6050_REG_CONFIG,
                MPU6050_REG_CONFIG_DLPF_CFG_SHIFT,
                MPU6050_REG_CONFIG_DLPF_CFG_LENGTH,
                bandwidth
            );
        }

        mpu6050_3axis_data getAccelerometerMeasurements() {
            int16_t accelX, accelY, accelZ;
            if (this->readRawAccelerometerMeasurements(&accelX, &accelY, &accelZ) != I2CDEV_RESULT_OK) {
                return {NAN, NAN, NAN};
            }
            return mpu6050_process_accel_data(accelX, accelY, accelZ, this->_accel_range);
        }

        float getTemperatureMeasurements() {
            int16_t temp;
            if (this->readRawTemperatureMeasurements(&temp) != I2CDEV_RESULT_OK) {
                return NAN;
            }
            return mpu6050_process_temperature(temp);
        }

        mpu6050_3axis_data getGyroscopeMeasurements() {
            int16_t gyroX, gyroY, gyroZ;
            if (this->readRawGyroscopeMeasurements(&gyroX, &gyroY, &gyroZ) != I2CDEV_RESULT_OK) {
                return {NAN, NAN, NAN};
            }
            return mpu6050_process_gyro_data(gyroX, gyroY, gyroZ, this->_gyro_range);
        }

        mpu6050_all_data getAllMeasurements() {
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

      protected:

        [[nodiscard]] auto readRawAccelerometerMeasurements(int16_t* accelX, int16_t* accelY, int16_t* accelZ) -> i2cdev_result_t {
            i2cdev_result_t result = this->_bus->readReg8(
                this->_addr,
                MPU6050_REG_ACCEL_XOUT_H,
                6,
                _buffer
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            *accelX = (_buffer[0] << 8) | _buffer[1];
            *accelY = (_buffer[2] << 8) | _buffer[3];
            *accelZ = (_buffer[4] << 8) | _buffer[5];

            return result;
        }

        [[nodiscard]] auto readRawTemperatureMeasurements(int16_t* temp) -> i2cdev_result_t {
            i2cdev_result_t result = this->_bus->readReg8(
                this->_addr,
                MPU6050_REG_TEMP_OUT_H,
                2,
                _buffer
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            *temp = (_buffer[0] << 8) | _buffer[1];

            return result;
        }

        [[nodiscard]] auto readRawGyroscopeMeasurements(int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ) -> i2cdev_result_t {
            i2cdev_result_t result = this->_bus->readReg8(
                    this->_addr,
                    MPU6050_REG_GYRO_XOUT_H,
                    6,
                    _buffer
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            *gyroX = (_buffer[0] << 8) | _buffer[1];
            *gyroY = (_buffer[2] << 8) | _buffer[3];
            *gyroZ = (_buffer[4] << 8) | _buffer[5];

            return result;
        }

        [[nodiscard]] auto readRawMotionMeasurements(
            int16_t* accelX, int16_t* accelY, int16_t* accelZ,
            int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ
        ) -> i2cdev_result_t {
            i2cdev_result_t result = this->_bus->readReg8(
                this->_addr,
                MPU6050_REG_ACCEL_XOUT_H,
                14,
                _buffer
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            *accelX = (_buffer[0] << 8) | _buffer[1];
            *accelY = (_buffer[2] << 8) | _buffer[3];
            *accelZ = (_buffer[4] << 8) | _buffer[5];

            // bytes 6 and 7 are temperature

            *gyroX = (_buffer[8] << 8) | _buffer[9];
            *gyroY = (_buffer[10] << 8) | _buffer[11];
            *gyroZ = (_buffer[12] << 8) | _buffer[13];

            return result;
        }

        [[nodiscard]] auto readRawAllMeasurements(
            int16_t* accelX, int16_t* accelY, int16_t* accelZ,
            int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ,
            int16_t* temp
        ) -> i2cdev_result_t {
            i2cdev_result_t result = this->_bus->readReg8(
                this->_addr,
                MPU6050_REG_ACCEL_XOUT_H,
                14,
                _buffer
            );
            if (result != I2CDEV_RESULT_OK) {
                return result;
            }

            *accelX = (_buffer[0] << 8) | _buffer[1];
            *accelY = (_buffer[2] << 8) | _buffer[3];
            *accelZ = (_buffer[4] << 8) | _buffer[5];

            *temp = (_buffer[6] << 8) | _buffer[7];

            *gyroX = (_buffer[8] << 8) | _buffer[9];
            *gyroY = (_buffer[10] << 8) | _buffer[11];
            *gyroZ = (_buffer[12] << 8) | _buffer[13];

            return result;
        }

      private:
        uint8_t _addr;
        I2CDevBus* _bus;

        uint8_t _buffer[14];

        mpu6050_accel_range_t _accel_range;
        mpu6050_gyro_range_t _gyro_range;
    };
}

#endif //__I2CDEVLIB_MPU6050_HPP__
