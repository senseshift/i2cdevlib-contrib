#ifndef __I2CDEVLIB_H__
#define __I2CDEVLIB_H__

#ifdef ARDUINO_ESP32_DEV
#  define I2CDEVLIB_LOG_V(...) log_v(__VA_ARGS__)
#  define I2CDEVLIB_LOG_D(...) log_d(__VA_ARGS__)
#  define I2CDEVLIB_LOG_I(...) log_i(__VA_ARGS__)
#  define I2CDEVLIB_LOG_W(...) log_w(__VA_ARGS__)
#  define I2CDEVLIB_LOG_E(...) log_e(__VA_ARGS__)
#endif // ARDUINO_ESP32_DEV

#include "i2cdevbus.hpp"

#if defined(ARDUINO)
#  include <Arduino.h>
#  include <hal/I2CDevBus_ArduinoWire.hpp>
#endif // ARDUINO

#endif // __I2CDEVLIB_H__