// Host-test mock of the Arduino core, just enough for pca9685.cpp and
// servos.cpp. Lives on the include path BEFORE the real core headers.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

inline void delay(unsigned long) {}
