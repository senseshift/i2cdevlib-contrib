#include <I2CDevLib.h>
#include <i2cdev/max170xx.hpp>

i2cdev::MAX17048 max17048;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Wire.begin();

    Serial.println("I2CDevLibContrib MAX170XX test!");

    // Wait for the MAX170XX to be ready
    delay(1000);

    if (max17048.check() != I2CDEV_RESULT_OK) {
        Serial.println("Failed to find MAX170XX chip");
        while (true) {}
    }
    Serial.println("MAX170XX Found!");
    if (max17048.quickStart() != I2CDEV_RESULT_OK) {
        Serial.println("Failed to quick start MAX170XX chip");
        while (true) {}
    }
}

void loop() {
    auto voltage = max17048.readVoltage();
    auto soc = max17048.readSoc();

    Serial.print("Voltage: "); Serial.print(voltage); Serial.print(" V");
    Serial.print("    SoC: "); Serial.print(soc); Serial.println(" %");

    delay(500);
}