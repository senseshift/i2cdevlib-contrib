#ifndef I2CDEVLIB_MPU6050_HPP_
#define I2CDEVLIB_MPU6050_HPP_

#include "i2cdev/mpu6050.h"
#include "i2cdev/common.hpp"

namespace i2cdev
{
    class MPU6050 : public mpu6050_dev_t
    {
        mpu6050_accel_range_t _accel_range;
        mpu6050_gyro_range_t _gyro_range;

    public:
#ifdef I2CDEVLIB_DEFAULT_BUS
        MPU6050(i2cdev_dev_addr_t addr = MPU6050_I2CADDR_BASE, i2cdev_bus_t* bus = &I2CDEVLIB_DEFAULT_BUS)
#else
        MPU6050(i2cdev_dev_addr_t addr, i2cdev_bus_t* bus)
#endif
        {
            this->addr = addr;
            this->bus = bus;
        }

        [[nodiscard]] auto check() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_check(this));
        }

        [[nodiscard]] auto available() -> bool
        {
            return this->check() == I2CDEV_RESULT_OK;
        }

        [[nodiscard]] auto setSleepEnabled(const bool enabled) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_set_sleep_enabled(this, enabled));
        }

        [[nodiscard]] auto sleep() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_sleep(this));
        }

        [[nodiscard]] auto wakeup() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_wakeup(this));
        }

        [[nodiscard]] auto reset() -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_reset(this));
        }

        [[nodiscard]] auto setAccelerometerRange(const mpu6050_accel_range_t range) -> i2cdev_result_t
        {
            const auto ret = static_cast<i2cdev_result_t>(mpu6050_set_accelerometer_range(this, range));

            if (ret == I2CDEV_RESULT_OK)
            {
                this->_accel_range = range;
            }

            return ret;
        }

        [[nodiscard]] auto readAccelerometerRange(mpu6050_accel_range_t* range) -> i2cdev_result_t
        {
            const auto ret = static_cast<i2cdev_result_t>(mpu6050_read_accelerometer_range(this, range));

            if (ret == I2CDEV_RESULT_OK)
            {
                this->_accel_range = *range;
            }

            return ret;
        }

        auto getAccelerometerRange() -> Result<mpu6050_accel_range_t>
        {
            mpu6050_accel_range_t range;
            const auto ret = this->readAccelerometerRange(&range);
            return {range, ret};
        }

        [[nodiscard]] auto setGyroscopeRange(const mpu6050_gyro_range_t range) -> i2cdev_result_t
        {
            const auto ret = static_cast<i2cdev_result_t>(mpu6050_set_gyro_range(this, range));

            if (ret == I2CDEV_RESULT_OK)
            {
                this->_gyro_range = range;
            }

            return ret;
        }

        [[nodiscard]] auto readGyroscopeRange(mpu6050_gyro_range_t* range) -> i2cdev_result_t
        {
            const auto ret = static_cast<i2cdev_result_t>(mpu6050_read_gyro_range(this, range));

            if (ret == I2CDEV_RESULT_OK)
            {
                this->_gyro_range = *range;
            }

            return ret;
        }

        auto getGyroscopeRange() -> Result<mpu6050_gyro_range_t>
        {
            mpu6050_gyro_range_t range;
            const auto ret = this->readGyroscopeRange(&range);
            return {range, ret};
        }

        [[nodiscard]] auto setFilterBandwidth(const mpu6050_filter_bandwidth_t bandwidth) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_set_filter_bandwidth(this, bandwidth));
        }

        [[nodiscard]] auto readFilterBandwidth(mpu6050_filter_bandwidth_t* bandwidth) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_read_filter_bandwidth(this, bandwidth));
        }

        auto getFilterBandwidth() -> Result<mpu6050_filter_bandwidth_t>
        {
            mpu6050_filter_bandwidth_t bandwidth;
            const auto ret = this->readFilterBandwidth(&bandwidth);
            return {bandwidth, ret};
        }

        [[nodiscard]] auto setExtSync(const mpu6050_ext_sync_t sync) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_set_ext_sync(this, sync));
        }

        [[nodiscard]] auto readExtSync(mpu6050_ext_sync_t* sync) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_read_ext_sync(this, sync));
        }

        auto getExtSync() -> Result<mpu6050_ext_sync_t>
        {
            mpu6050_ext_sync_t sync;
            const auto ret = this->readExtSync(&sync);
            return {sync, ret};
        }

        [[nodiscard]] auto readRawAccelerometerMeasurements(mpu6050_3axis_raw_data_t* data) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_read_raw_accelerometer_measurements(this, data));
        }

        auto getRawAccelerometerMeasurements() -> Result<mpu6050_3axis_raw_data_t>
        {
            mpu6050_3axis_raw_data_t data;
            const auto ret = this->readRawAccelerometerMeasurements(&data);
            return {data, ret};
        }

        [[nodiscard]] auto readAccelerometerMeasurements(mpu6050_3axis_data_t* data) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(
                mpu6050_read_accelerometer_measurements(this, data, this->_accel_range));
        }

        auto getAccelerometerMeasurements() -> Result<mpu6050_3axis_data_t>
        {
            mpu6050_3axis_data_t data;
            const auto ret = this->readAccelerometerMeasurements(&data);
            return {data, ret};
        }

        [[nodiscard]] auto readRawTemperatureMeasurements(int16_t* temp) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_read_raw_temperature_measurements(this, temp));
        }

        auto getRawTemperatureMeasurements() -> Result<int16_t>
        {
            int16_t temp;
            const auto ret = this->readRawTemperatureMeasurements(&temp);
            return {temp, ret};
        }

        [[nodiscard]] auto readTemperatureMeasurements(float* temp) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_read_temperature_measurements(this, temp));
        }

        auto getTemperatureMeasurements() -> Result<float>
        {
            float temp;
            const auto ret = this->readTemperatureMeasurements(&temp);
            return {temp, ret};
        }

        [[nodiscard]] auto readRawGyroscopeMeasurements(mpu6050_3axis_raw_data_t* data) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_read_raw_gyro_measurements(this, data));
        }

        auto getRawGyroscopeMeasurements() -> Result<mpu6050_3axis_raw_data_t>
        {
            mpu6050_3axis_raw_data_t data;
            const auto ret = this->readRawGyroscopeMeasurements(&data);
            return {data, ret};
        }

        [[nodiscard]] auto readGyroscopeMeasurements(mpu6050_3axis_data_t* data) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(
                mpu6050_read_gyro_measurements(this, data, this->_gyro_range));
        }

        auto getGyroscopeMeasurements() -> Result<mpu6050_3axis_data_t>
        {
            mpu6050_3axis_data_t data;
            const auto ret = this->readGyroscopeMeasurements(&data);
            return {data, ret};
        }

        [[nodiscard]] auto readRawMotionMeasurements(mpu6050_3axis_raw_data_t* accel,
                                                     mpu6050_3axis_raw_data_t* gyro) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_read_raw_motion_measurements(this, accel, gyro));
        }

        auto getRawMotionMeasurements() -> Result<mpu6050_motion_raw_data_t>
        {
            mpu6050_3axis_raw_data_t accel, gyro;
            const auto ret = this->readRawMotionMeasurements(&accel, &gyro);
            return {mpu6050_motion_raw_data_t{.accel = accel, .gyro = gyro}, ret};
        }

        [[nodiscard]] auto readMotionMeasurements(mpu6050_3axis_data_t* accel,
                                                  mpu6050_3axis_data_t* gyro) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(
                mpu6050_read_motion_measurements(this, accel, this->_accel_range, gyro, this->_gyro_range));
        }

        auto getMotionMeasurements() -> Result<mpu6050_motion_data_t>
        {
            mpu6050_3axis_data_t accel, gyro;
            const auto ret = this->readMotionMeasurements(&accel, &gyro);
            return {mpu6050_motion_data_t{.accel = accel, .gyro = gyro}, ret};
        }

        [[nodiscard]] auto readRawAllMeasurements(mpu6050_3axis_raw_data_t* accel,
                                                  int16_t* temp,
                                                  mpu6050_3axis_raw_data_t* gyro) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(mpu6050_read_raw_all_measurements(this, accel, temp, gyro));
        }

        auto getRawAllMeasurements() -> Result<mpu6050_all_raw_data>
        {
            mpu6050_3axis_raw_data_t accel, gyro;
            int16_t temp;
            const auto ret = this->readRawAllMeasurements(&accel, &temp, &gyro);
            return {mpu6050_all_raw_data{.accel = accel, .gyro = gyro, .temperature = temp}, ret};
        }

        [[nodiscard]] auto readAllMeasurements(mpu6050_3axis_data_t* accel,
                                               float* temp,
                                               mpu6050_3axis_data_t* gyro) -> i2cdev_result_t
        {
            return static_cast<i2cdev_result_t>(
                mpu6050_read_all_measurements(this, accel, this->_accel_range, temp, gyro, this->_gyro_range));
        }

        auto getAllMeasurements() -> Result<mpu6050_all_data>
        {
            mpu6050_3axis_data_t accel, gyro;
            float temp;
            const auto ret = this->readAllMeasurements(&accel, &temp, &gyro);
            return {mpu6050_all_data{.accel = accel, .gyro = gyro, .temperature = temp}, ret};
        }
    };
}

#endif // I2CDEVLIB_MPU6050_HPP_
