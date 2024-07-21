#include <I2CDevLib.h>
#include <i2cdev/mpu6050.hpp>

i2cdev::MPU6050 mpu6050;

void setup(void) {
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Wire.begin();

    Serial.println("I2CDevLibContrib MPU6050 test!");

    // Wait for the MPU6050 to be ready
    delay(1000);

    if (mpu6050.check() != I2CDEV_RESULT_OK) {
        Serial.println("Failed to find MPU6050 chip");
        while (true) {}
    }
    Serial.println("MPU6050 Found!");

    if (mpu6050.reset() != I2CDEV_RESULT_OK) {
        Serial.println("Failed to reset MPU6050 chip");
        while (true) {}
    }

    mpu6050.setAccelerometerRange(MPU6050_ACCEL_RANGE_8G);
    mpu6050.setGyroscopeRange(MPU6050_GYRO_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BANDWIDTH_21_HZ);

    Serial.println("");
    delay(100);
}

void loop() {
    auto measurements = mpu6050.getAllMeasurements();

    Serial.print("Accelerometer X: "); Serial.print(measurements.accel.x); Serial.print(" Y: "); Serial.print(measurements.accel.y); Serial.print(" Z: "); Serial.println(measurements.accel.z);
    Serial.print("Gyroscope     X: "); Serial.print(measurements.gyro.x); Serial.print(" Y: "); Serial.print(measurements.gyro.y); Serial.print(" Z: "); Serial.println(measurements.gyro.z);
    Serial.print("Temperature:     "); Serial.print(measurements.temperature); Serial.println(" C");
    Serial.println("");

    delay(500);
}