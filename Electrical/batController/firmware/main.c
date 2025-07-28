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
#include "ws2812.pio.h"
#include "pico/multicore.h"


// Determine if the target is an rp2350
#ifdef PICO_RP2350
  #include "RP2350.h"
#else
  #include "RP2040.h"
#endif

// Structure to hold measurement data
typedef struct {
    uint32_t voltage_raw;
    uint32_t current_raw;
    float voltage;
    float current;
} measurement_data_t;

// Shared data between cores
volatile measurement_data_t measurement_data;
volatile bool measurement_ready = false;
volatile bool measurement_requested = false;
static bool underVoltageEvent = false;

uint8_t timerCounter = 0;

ina228_config config;

void config_ina228(){
    config.i2c = i2c1;
    config.sda = 6;
    config.scl = 7;

    ina228_init(&config);
}


// CAN
static uint32_t pio_num = 0;
static uint32_t irq_num = 1;
static uint32_t bitrate = 500000;
static uint32_t gpio_rx = 4;
static uint32_t gpio_tx = 5;

static struct can2040 cbus;
static volatile bool can_needs_response = false;
volatile bool can_message_received = false;
volatile bool can_message_sent = false;


//Callback to handle messages
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    char buffer[128];

    if (notify == CAN2040_NOTIFY_RX) {
        can_message_received = true;
    } 
    else if (notify == CAN2040_NOTIFY_TX) {
        can_message_sent = true;
    }
    else if (notify & CAN2040_NOTIFY_ERROR) {
      //For some reason printing a serial message here causes a crash
      return;
    }
}

static void PIOx_IRQHandler(void){
  can2040_pio_irq_handler(&cbus);
  // printf("[IRQ] PIO CAN IRQ triggered\n");
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
void blink_on_input(bool tx, bool rx) {
    uint8_t r = tx ? 255 : 0;
    uint8_t g = rx ? 100 : 0;

    if (r || g) {
        put_rgb(r, g, 0);
        sleep_ms(10);
        put_rgb(0, 0, 0);
    }
}

void core1_entry() {
    // Initialize INA228 on core1
    printf("Initializing INA228 on Core1...\n"); 
    config_ina228();
    
    while (true) {
        // Check if measurement is requested from core0
        if (measurement_requested && !measurement_ready) {
            // Perform the measurements
            measurement_data.voltage_raw = ina228_voltage_raw(&config);
            measurement_data.voltage = ina228_voltage(&config);
            
            // Signal that measurement is ready
            measurement_ready = true;
        }
        
        // Handle LED blinking
        if (can_message_received || can_message_sent) {
            blink_on_input(can_message_sent, can_message_received);
            can_message_sent = false;
            can_message_received = false;
        }

        sleep_ms(1); // avoid busy-waiting
    }
}

// Timer callback for periodic CAN transmission (runs on core 0)
bool send_can_data_callback(repeating_timer_t *rt) {
    if (timerCounter <= 254) {
        timerCounter++;
    }
    
    measurement_requested = true;
    measurement_ready = false;
    can_needs_response = true;
    struct can2040_msg tmsg = {
        .id = 0x09a,
        .dlc = 4,
        .data32 = {
            0x04, 0x00
        }
    };

    if (measurement_data.voltage < 31.000f && timerCounter >= 10) {
        underVoltageEvent = true;
    }

    if (!underVoltageEvent) {
        can2040_transmit(&cbus, &tmsg);
    }

    printf("CAN TX [Timer]: %.3f V (0x%06X)\n",
           measurement_data.voltage,
           measurement_data.voltage_raw 
        );
    return true;  // Return true to keep repeating
}

int main() {
    stdio_init_all();
    setup_ws2812();

    while (!tud_cdc_connected()) {
        printf(".");
        sleep_ms(500);
    }
    put_rgb(255, 0, 0); // Red
    sleep_ms(500);
    put_rgb(0, 255, 0); // Green
    sleep_ms(500);
    put_rgb(0, 0, 255); // Blue
    sleep_ms(500);
    put_rgb(0, 0, 0);   // Off

    multicore_launch_core1(core1_entry);
    canbus_setup();

    printf("System ready.\n");
    static repeating_timer_t can_timer;
    add_repeating_timer_ms(200, send_can_data_callback, NULL, &can_timer);

    while (true) {
        sleep_ms(1000); // Main thread idle
    }
}
