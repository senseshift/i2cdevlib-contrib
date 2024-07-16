#ifndef __I2CDEVLIB_H__
#define __I2CDEVLIB_H__

#include "i2cdevbus.hpp"

#if defined(ARDUINO)
#  include <Arduino.h>
#  include <hal/I2CDevBus_ArduinoWire.hpp>
#endif // ARDUINO

#endif // __I2CDEVLIB_H__