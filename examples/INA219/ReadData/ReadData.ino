#include <I2CDevLib.h>
#include <i2cdev/ina219.hpp>

using namespace i2cdev;

INA219 ina219 = INA219();
// INA219 ina219 = INA219(0x41);
// INA219 ina219 = INA219(0x40, new ArduinoI2CDevBus(&Wire));

void setup() {
    Serial.begin(115200);
    Serial.println("I2CDevLib Example - INA219 Test");

    if (!Wire.begin()) {
        Serial.println("[E] Failed to init Wire!");
        while(true) {};
    }

    auto result = ina219.setCalibration_32V_2A();
    if (result != I2CDEV_RESULT_OK) {
        Serial.printf("[E] Failed to setup INA219: %i\n", result);
        while(true) {};
    }

    delay(10);
}

float shuntVoltage, busVoltage, current, power, loadVoltage;

void loop() {
    shuntVoltage = ina219.readShuntVoltage();
    busVoltage = ina219.readBusVoltage();
    current = ina219.readCurrent();
    power = ina219.readPower();
    loadVoltage = busVoltage + (shuntVoltage / 1000);

    Serial.printf("Bus Voltage: %5.3f V\n", busVoltage);
    Serial.printf("Shunt Voltage: %5.3f mV\n", shuntVoltage);
    Serial.printf("Load Voltage: %5.3f V\n", loadVoltage);
    Serial.printf("Current: %5.3f A\n", current);
    Serial.printf("Power: %5.3f mW\n", power);
    Serial.print('\n');

    delay(2000);
}