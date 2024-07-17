#include <I2CDevLib.h>
#include <i2cdev/pca9685.hpp>

#define NUM_SERVOS (16)

#define SERVO_FREQ (50)

#define SERVO_PULSE_LEN_MIN (150)
#define SERVO_PULSE_LEN_MAX (600)

#define SERVO_US_MIN (600)
#define SERVO_US_MAX (2400)

using namespace i2cdev;

PCA9685 pca9685 = PCA9685();
// PCA9685 pca9685 = PCA9685(0x41);
// PCA9685 pca9685 = PCA9685(0x40, new ArduinoI2CDevBus(&Wire));

i2cdev_result_t result;

void setup() {
    Serial.begin(115200);
    Serial.println("I2CDevLib Example - PCA9685 Test");

    if (!Wire.begin()) {
        Serial.println("[E] Failed to init Wire!");
        while(true) {};
    }

    // auto result = pca9685.setPrescale(100);
    result = pca9685.setFrequency(SERVO_FREQ);
    if (result != I2CDEV_RESULT_OK) {
        Serial.printf("[E] Failed to set PCA9685 prescale: %i\n", result);
        while(true) {};
    }
    result = pca9685.wakeup();
    if (result != I2CDEV_RESULT_OK) {
        Serial.printf("[E] Failed wakeup PCA9685: %i\n", result);
        while(true) {};
    }

    delay(10);
}

uint8_t currentServo = 0;

void loop() {
    Serial.printf("Current servo: %i\n", currentServo);

    Serial.println("\t > SERVO_PULSE_LEN_MIN => SERVO_PULSE_LEN_MAX");
    for (uint16_t pulseLen = SERVO_PULSE_LEN_MIN; pulseLen < SERVO_PULSE_LEN_MAX; pulseLen++) {
        result = pca9685.setChannel(currentServo, pulseLen);
        if (result != I2CDEV_RESULT_OK) {
            Serial.printf("[E] Failed to set PCA9685 pin: %i\n", result);
        }
        // delay(5);
        // Serial.printf("\t > SERVO_PULSE_LEN_MIN => SERVO_PULSE_LEN_MAX: %i\n", pulseLen);
    }

    delay(500);

    Serial.println("\t > SERVO_PULSE_LEN_MAX => SERVO_PULSE_LEN_MIN");
    for (uint16_t pulseLen = SERVO_PULSE_LEN_MAX; pulseLen > SERVO_PULSE_LEN_MIN; pulseLen--) {
        result = pca9685.setChannel(currentServo, pulseLen);
        if (result != I2CDEV_RESULT_OK) {
            Serial.printf("[E] Failed to set PCA9685 pin: %i\n", result);
        }
        // delay(5);
        // Serial.printf("\t > SERVO_PULSE_LEN_MAX => SERVO_PULSE_LEN_MIN: %i\n", pulseLen);
    }

    delay(500);

    Serial.println("\t > SERVO_US_MIN => SERVO_US_MAX");
    for (uint16_t uSec = SERVO_US_MIN; uSec < SERVO_US_MAX; uSec++) {
        result = pca9685.writeMicroseconds(currentServo, uSec);
        if (result != I2CDEV_RESULT_OK) {
            Serial.printf("[E] Failed to set PCA9685 microseconds: %i\n", result);
        }
        // delay(5);
        // Serial.printf("\t > SERVO_US_MIN => SERVO_US_MAX: %i\n", uSec);
    }

    delay(500);

    Serial.println("\t > SERVO_US_MAX => SERVO_US_MIN");
    for (uint16_t uSec = SERVO_US_MAX; uSec > SERVO_US_MIN; uSec--) {
        result = pca9685.writeMicroseconds(currentServo, uSec);
        if (result != I2CDEV_RESULT_OK) {
            Serial.printf("[E] Failed to set PCA9685 microseconds: %i\n", result);
        }
        // delay(5);
        // Serial.printf("\t > SERVO_US_MAX => SERVO_US_MIN: %i\n", uSec);
    }

    delay(500);

    currentServo = (++currentServo) % (NUM_SERVOS);
}
