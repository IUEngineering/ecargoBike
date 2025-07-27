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

volatile bool can_message_received = false;
volatile bool can_message_sent = false;


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


ina228_config config;

void config_ina228(){
    config.i2c = i2c1;
    config.sda = 6;
    config.scl = 7;

    ina228_init(&config);
}


static uint32_t pio_num = 0;
static uint32_t irq_num = 1;
static uint32_t bitrate = 500000;
static uint32_t gpio_rx = 4; // (BLUE)
static uint32_t gpio_tx = 5; // (ORANGE)

static struct can2040 cbus;

static volatile bool can_needs_response = false;


//Callback to handle messages
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    char buffer[128];
    // if (notify == CAN2040_NOTIFY_RX){
    //   // can_message_received = true;
    //   // // Set up a test message to send
    //   //   struct can2040_msg tmsg;
    //   //   tmsg.id = 0x209;
    //   //   tmsg.dlc = 8;
    //   //   // tmsg.data32[0] = ina228_voltage_raw(&config);
    //   //   // tmsg.data32[1] = ina228_current_raw(&config);
    //   //   tmsg.data32[0] = 0x0000;
    //   //   tmsg.data32[1] = 0x0000;
    //   //   int sts = can2040_transmit(&cbus, &tmsg);
    //     // printf("Sent message with INA %f [A] & %f [V] RAW \n", ina228_current(&config), ina228_voltage(&config));
    //     can_message_received = true;
    //     printf("Can recieved\n");
    //     // Request measurement from core1
    //     measurement_requested = true;
    //     measurement_ready = false;
        
    //     // Set up a test message to send with the measured data
    //     struct can2040_msg tmsg;
    //     tmsg.id = 0x209;
    //     tmsg.dlc = 8;
    //     tmsg.data32[0] = measurement_data.voltage_raw;
    //     tmsg.data32[1] = measurement_data.current_raw;
        
    //     int sts = can2040_transmit(&cbus, &tmsg);
    //     printf("Sent message with INA %f [A] & %f [V] RAW \n", 
    //            measurement_data.current, measurement_data.voltage);
        
    //     // Reset flags
    //     measurement_requested = false;
    //     measurement_ready = false;
      
    // } else if (notify == CAN2040_NOTIFY_TX) {
    //   can_message_sent = true;
    //   snprintf(buffer, sizeof(buffer), "CAN: tx msg: (id: %0x, size: %0x, data: %0x, %0x)\n", msg->id & 0x7ff, msg->dlc, msg->data32[0], msg->data32[1]);
    //   printf("%s", buffer); 

    if (notify == CAN2040_NOTIFY_RX) {
        can_message_received = true;
        measurement_requested = true;
        measurement_ready = false;
        can_needs_response = true;  // Let main loop handle response
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
    uint8_t g = rx ? 200 : 0;

    if (r || g) {
        put_rgb(r, g, 0);
        sleep_ms(10);
        put_rgb(0, 0, 0);
    }
}

// void core1_entry() {
    
//     while (true) {
        
//         if (can_message_received || can_message_sent) {
//             blink_on_input(can_message_sent, can_message_received);
//             can_message_sent = false;
//             can_message_received = false;
//         }

//         sleep_ms(1); // avoid busy-waiting
//     }
// }

void core1_entry() {
    // Initialize INA228 on core1
    printf("Initializing INA228 on Core1...\n"); 
    config_ina228();
    
    while (true) {
        // Check if measurement is requested from core0
        if (measurement_requested && !measurement_ready) {
            // Perform the measurements
            measurement_data.voltage_raw = ina228_voltage_raw(&config);
            measurement_data.current_raw = ina228_current_raw(&config);
            measurement_data.voltage = ina228_voltage(&config);
            measurement_data.current = ina228_current(&config);
            
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

// // int main() {
//     stdio_init_all();

//     setup_ws2812();

    
//     printf("waiting for usb host");
//     while (!tud_cdc_connected()) {
//       printf(".");
//       sleep_ms(500);
//     };
    
//     put_rgb(255, 0, 0); // Red
//     sleep_ms(500);
//     put_rgb(0, 255, 0); // Green
//     sleep_ms(500);
//     put_rgb(0, 0, 255); // Blue
//     sleep_ms(500);
//     put_rgb(0, 0, 0);   // Off
    
//     multicore_launch_core1(core1_entry);  // << Start Core 1
//     printf("\nusb host detected!\n");

//     printf("Initializing CAN Bus...\n"); 
//     canbus_setup();

//     // printf("Initializing INA228...\n"); 
//     // config_ina228();
    
//     while(true){
//       sleep_ms(10);
//     }
//     can2040_stop(&cbus);

//     return 0;
// }

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

    while (true) {
        // Handle CAN response after INA228 read
        if (can_needs_response && measurement_ready) {
            struct can2040_msg tmsg = {
                .id = 0x209,
                .dlc = 8,
                .data32 = {
                    measurement_data.voltage_raw,
                    measurement_data.current_raw
                }
            };
            can2040_transmit(&cbus, &tmsg);
            printf("Sent CAN msg: %.3f V, %.3f mA\n and RAW voltage reg: 0x%06X, current reg: 0x%06X\n" ,
                   measurement_data.voltage,
                   measurement_data.current, 
                  measurement_data.voltage_raw, 
                measurement_data.current_raw);


            // Reset flags
            can_needs_response = false;
            measurement_requested = false;
            measurement_ready = false;
        }

        sleep_ms(1);
    }
}
