#ifndef ARDU_TO_PICO_H
#define ARDU_TO_PICO_H

#include "pico/stdlib.h"  // For gpio_* and sleep_* functions
#include <stdarg.h>
#include <string.h>
#include <cstring>  // for std::strcpy
#include <algorithm>  // for std::min, std::max


// Emulate Arduino digitalWrite and digitalRead using Pico SDK
#define digitalWrite(pin, value)    gpio_put((pin), (value))
#define digitalRead(pin)            gpio_get((pin))
#define pinMode(pin, mode)          gpio_init(pin); gpio_set_dir((pin), ((mode) == OUTPUT ? GPIO_OUT : GPIO_IN))
#define delay(ms)                   sleep_ms(ms)
#define delayMicroseconds(us)       sleep_us(us)

// Optional Arduino-like constants
#define OUTPUT GPIO_OUT
#define INPUT  GPIO_IN
#define HIGH   1
#define LOW    0

#endif  // ARDU_TO_PICO_H

