//
// hV_HAL_Peripherals.cpp
// C++ code
// ----------------------------------
//
// Project highView Library Suite
//
// Created by Rei Vilo, 08 Jan 2024
//
// Copyright (c) Rei Vilo, 2010-2025
// Licence All rights reserved
//
// * Basic edition: for hobbyists and for basic usage
// Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
//
// * Evaluation edition: for professionals or organisations, evaluation only, no commercial usage
// All rights reserved
//
// * Commercial edition: for professionals or organisations, commercial usage
// All rights reserved
//
// Release 800: Added 3-wire SPI
// Release 801: Added SPI configuration
// Release 803: Improved stability
// Release 804: Improved power management
// Release 805: Improved stability
// Release 810: Added patches for some platforms
//

// Library header
#include "hV_HAL_Peripherals.h"

//
// === General section
//
#ifndef SPI_CLOCK_MAX
#define SPI_CLOCK_MAX 16000000
#endif

struct h_pinSPI3_t
{
    uint8_t pinClock;
    uint8_t pinData;
};

h_pinSPI3_t h_pinSPI3;

void hV_HAL_begin()
{
    // Empty
}
//
// === End of General section
//

//
// === GPIO section
//
void waitFor(uint8_t pin, uint8_t state)
{
    while (gpio_get(pin) != state)
    {
        sleep_ms(32); // non-blocking
    }
}
//
// === End of GPIO section
//

//
// === Time section
//

//
// === End of Time section
//

//
// === SPI section
//

void hV_HAL_SPI_begin(uint32_t speed)
{
        spi_inst_t *spi = spi0;

        // _settingScreen = {speed, MSBFIRST, SPI_MODE0};
        // Initialize SPI port at 1 MHz
        spi_init(spi, 1000 * 1000);

        // Set SPI format
        spi_set_format( spi0,   // SPI instance
                        8,      // Number of bits per transfer
                        SPI_CPOL_0,      // Polarity (CPOL)
                        SPI_CPHA_0,      // Phase (CPHA)
                        SPI_MSB_FIRST);
        // Initialize SPI pins
        gpio_set_function(SCK,  GPIO_FUNC_SPI);
        gpio_set_function(MOSI, GPIO_FUNC_SPI);
        gpio_set_function(MISO, GPIO_FUNC_SPI);

        // Board ESP32-Pico-DevKitM-2 crashes if pins are not specified.
        // SPI.begin(14, 12, 13); // SCK MISO MOSI

        // SPI.beginTransaction(_settingScreen);
}

void hV_HAL_SPI_end()
{
    // Disable SPI0
        spi_deinit(spi0);

        // Return SPI pins to GPIO (optional, depending on your use case)
        gpio_init(SCK); // SCK
        gpio_init(MOSI); // MOSI
        gpio_init(MISO); // MISO

}

uint8_t hV_HAL_SPI_transfer(uint8_t data)
{
    uint8_t rx;
    spi_write_read_blocking(spi0, &data, &rx, 1);  // Send & receive 1 byte
    return rx;
}

//
// === End of SPI section
//
