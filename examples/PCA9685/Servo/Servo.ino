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

    Wire.begin();

    // auto result = pca9685.setPrescale(100);
    result = pca9685.setFrequency(SERVO_FREQ);
    if (result != I2CDEV_RESULT_OK) {
        Serial.println("[E] Failed to set PCA9685 prescale");
        while(true) {};
    }
    result = pca9685.wakeup();
    if (result != I2CDEV_RESULT_OK) {
        Serial.println("[E] Failed to wake up PCA9685");
        while(true) {};
    }

    delay(10);
}

uint8_t currentServo = 0;

void loop() {
    Serial.print("Current servo: "); Serial.println(currentServo);

    Serial.println("\t > SERVO_PULSE_LEN_MIN => SERVO_PULSE_LEN_MAX");
    for (uint16_t pulseLen = SERVO_PULSE_LEN_MIN; pulseLen < SERVO_PULSE_LEN_MAX; pulseLen++) {
        result = pca9685.setChannel(currentServo, pulseLen);
        if (result != I2CDEV_RESULT_OK) {
            Serial.println("[E] Failed to set PCA9685 pin");
        }
    }

    delay(500);

    Serial.println("\t > SERVO_PULSE_LEN_MAX => SERVO_PULSE_LEN_MIN");
    for (uint16_t pulseLen = SERVO_PULSE_LEN_MAX; pulseLen > SERVO_PULSE_LEN_MIN; pulseLen--) {
        result = pca9685.setChannel(currentServo, pulseLen);
        if (result != I2CDEV_RESULT_OK) {
            Serial.println("[E] Failed to set PCA9685 pin");
        }
    }

    delay(500);

    Serial.println("\t > SERVO_US_MIN => SERVO_US_MAX");
    for (uint16_t uSec = SERVO_US_MIN; uSec < SERVO_US_MAX; uSec++) {
        result = pca9685.writeMicroseconds(currentServo, uSec);
        if (result != I2CDEV_RESULT_OK) {
            Serial.println("[E] Failed to set PCA9685 microseconds");
        }
    }

    delay(500);

    Serial.println("\t > SERVO_US_MAX => SERVO_US_MIN");
    for (uint16_t uSec = SERVO_US_MAX; uSec > SERVO_US_MIN; uSec--) {
        result = pca9685.writeMicroseconds(currentServo, uSec);
        if (result != I2CDEV_RESULT_OK) {
            Serial.println("[E] Failed to set PCA9685 microseconds");
        }
    }

    delay(500);

    currentServo = (++currentServo) % (NUM_SERVOS);
}
