#include <i2cdevbus_hal_arduino.hpp>
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
    auto voltage_result = max17048.getVoltage();
    auto soc_result = max17048.getSoc();

    if (!voltage_result.ok() || !soc_result.ok()) {
        Serial.println("Error reading values from MAX170XX!");
        delay(1000);
    }

    auto voltage = voltage_result.value;
    auto soc = soc_result.value;

    Serial.print("Voltage: "); Serial.print(voltage); Serial.print(" V");
    Serial.print("    SoC: "); Serial.print(soc); Serial.println(" %");

    delay(500);
}