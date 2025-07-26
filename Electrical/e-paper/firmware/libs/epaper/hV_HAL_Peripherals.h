#ifndef hV_HAL_PERIPHERALS_RELEASE
///
/// @brief Release
///
#define hV_HAL_PERIPHERALS_RELEASE 812

///
/// @brief SDK library
/// @see References
/// * Arduino SDK https://www.arduino.cc/reference/en/
/// * Energia SDK https://energia.nu/reference/
///
// #include <Arduino.h>

// ///
// /// @brief SDK other libraries
// ///
// #include <SPI.h>
// #include <Wire.h>

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
///
/// @brief Other libraries
///
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

// #define mySerial Serial

#define HIGH    1
#define LOW     0
#define SCK     14
#define MOSI    13
#define MISO    12

///
/// @brief General initialisation
///
void hV_HAL_begin();

///
/// @brief Wait for
/// @param pin pin number
/// @param state state to reach, default = HIGH
///
void waitFor(uint8_t pin, uint8_t state = HIGH);

///
/// @brief Configure and start SPI
/// @param speed SPI speed in Hz, 8000000 = default
/// @note Other parameters are
/// * Bit order: MSBFIRST
/// * Data mode: SPI_MODE0
/// @note With check for unique initialisation
///
void hV_HAL_SPI_begin(uint32_t speed = 8000000);

///
/// @brief End SPI
/// @note With check for unique deinitialisation
///
void hV_HAL_SPI_end();

///
/// @brief Combined write and read of a single byte
/// @param data byte
/// @return read byte
/// @warning No check for previous initialisation
///
uint8_t hV_HAL_SPI_transfer(uint8_t data);







///
///
/// @brief Minimum of two numbers
///
/// @param a first number
/// @param b second number
/// @return minimum of a and b
///
/// @note Macro more robust than bugged implementation on some platforms
///
#define hV_HAL_min(a, b) ((a) < (b) ? (a) : (b))

///
/// @brief Maximum of two numbers
///
/// @param a first number
/// @param b second number
/// @return maximum of a and b
///
/// @note Macro more robust than bugged implementation on some platforms
///
#define hV_HAL_max(a, b) ((a) > (b) ? (a) : (b))

///
/// @brief Swap numbers
/// @param[out] x first number
/// @param[out] y second number
///
/// @note Macro more robust than template for some platforms
///
/// @code
/// template <typename T> T
/// hV_HAL_swap(T &x, T &y)
/// {
///     T w = x;
///     x = y;
///     y = w;
/// }
/// @endcode
///
/// @note `__typeof__` recommended over `typeof`
///
#define hV_HAL_swap(x, y) do { __typeof__(x) WORK = x; x = y; y = WORK; } while (0)

/// @}

#endif // hV_HAL_PERIPHERALS_RELEASE

