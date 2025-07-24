/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "ina228.h"
#include "tusb.h"
#include "can2040.h"
#include "hardware/clocks.h"

// Determine if the target is an rp2350
#ifdef PICO_RP2350
  #include "RP2350.h"
#else
  #include "RP2040.h"
#endif


ina228_config config;

void config_ina228(){
    config.i2c = i2c1;
    config.sda = 6;
    config.scl = 7;

    ina228_init(&config);
}

void pico_set_led(bool led_on) {
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

static uint32_t pio_num = 0;
static uint32_t irq_num = 1;
static uint32_t bitrate = 500000;
static uint32_t gpio_rx = 4;
static uint32_t gpio_tx = 5;

static struct can2040 cbus;

//Callback to handle messages
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    char buffer[128];
    if (notify == CAN2040_NOTIFY_RX){
      // Set up a test message to send
        struct can2040_msg tmsg;
        tmsg.id = 0x209;
        tmsg.dlc = 8;
        tmsg.data32[0] = ina228_voltage_raw(&config);
        tmsg.data32[1] = ina228_current_raw(&config);
        int sts = can2040_transmit(&cbus, &tmsg);
        printf("Sent message with ina voltage & current\n");
      // snprintf(buffer, sizeof(buffer), "CAN: rx msg: (id: %0x, size: %0x, data: %0x, %0x)\n", msg->id & 0x7ff, msg->dlc, msg->data32[0], msg->data32[1]);
      // printf("%s", buffer);
    } else if (notify == CAN2040_NOTIFY_TX) {
      snprintf(buffer, sizeof(buffer), "CAN: tx msg: (id: %0x, size: %0x, data: %0x, %0x)\n", msg->id & 0x7ff, msg->dlc, msg->data32[0], msg->data32[1]);
      printf("%s", buffer); 
    } else if (notify & CAN2040_NOTIFY_ERROR) {
      //For some reason printing a serial message here causes a crash
      return;
    }
}

static void PIOx_IRQHandler(void){
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup(void){
    // get the actual system clock frequency dynamically 
    //  to account for overclocking.
    // pico1 = 125000000
    // pico2 = 150000000
    uint32_t sys_clock = clock_get_hz(clk_sys);

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
    NVIC_SetPriority(PIO0_IRQ_0_IRQn, irq_num);
    NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}



int main() {
    stdio_init_all();

    
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // printf("waiting for usb host");
    // while (!tud_cdc_connected()) {
    //   printf(".");
    //   sleep_ms(500);
    // };

    // printf("\nusb host detected!\n");

    printf("Initializing CAN Bus...\n"); 
    canbus_setup();

    printf("Initializing INA228...\n"); 
    config_ina228();



    
    while(true){
      pico_set_led(1);
      sleep_ms(100);
     
    }
    can2040_stop(&cbus);
    pico_set_led(0);

    return 0;
}

//ORANGE IS L
//GREEN IS H