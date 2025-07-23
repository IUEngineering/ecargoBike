/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "ina228.h"
#include "tusb.h"


ina228_config config;


void config_ina228(){
    config.i2c = i2c0;
    config.sda = 4;
    config.scl = 5;

    ina228_init(&config);
}

// Turn the LED on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#endif
}

int main() {
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    printf("waiting for usb host");
    while (!tud_cdc_connected()) {
      printf(".");
      sleep_ms(500);
    }

    sleep_ms(500);
    printf("\nusb host detected!\n");

    config_ina228();
    

    while (true) {
        // printf("Hello, world!\n");
        pico_set_led(true);
        printf("M: The voltage is: %f\n", ina228_voltage(&config));

        sleep_ms(250);
        pico_set_led(false);
        sleep_ms(250);

    }
}
