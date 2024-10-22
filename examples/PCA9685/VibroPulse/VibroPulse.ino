#include <I2CDevLib.h>
#include <i2cdev/pca9685.hpp>

#define NUM_MOTORS (4)

#define PWM_FREQ (1600)

using namespace i2cdev;

PCA9685 pca9685 = PCA9685();
// PCA9685 pca9685 = PCA9685(0x41);
// PCA9685 pca9685 = PCA9685(0x40, ArduinoI2CDevBus(Wire));

i2cdev_result_t result;

void setup() {
    Serial.begin(115200);
    Serial.println("I2CDevLib Example - PCA9685 Test");

    Wire.begin();

    // auto result = pca9685.setPrescale(100);
    result = pca9685.setFrequency(PWM_FREQ);
    if (result != I2CDEV_RESULT_OK) {
        Serial.println("[E] Failed to set PCA9685 prescale");
        while(true) {};
    }
    result = pca9685.wakeup();
    if (result != I2CDEV_RESULT_OK) {
        Serial.println("[E] Failed wakeup PCA9685");
        while(true) {};
    }

    delay(10);
}

void pulse(uint8_t pin) {
    pca9685.setChannel(pin, 4095);
    delay(250);
    pca9685.setChannel(pin, 0);
}

uint8_t currentVibro = 0;

void loop() {
    Serial.print("Current vibro: "); Serial.println(currentVibro);

    pulse(currentVibro);
    delay(100);
    pulse(currentVibro);

    delay(500);

    currentVibro = (++currentVibro) % (NUM_MOTORS);
}
