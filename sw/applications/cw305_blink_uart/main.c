/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
 * Description: This program can be used to test the LED3 (blue led) on board, connected to X-HEEP GPIO 2,
   and the UART interface.
 */

#include <stdio.h>
#include <stdlib.h>
#include "core_v_mini_mcu.h"
#include "gpio.h"
#include "x-heep.h"

#define GPIO_TOGGLE 2

/* By default, printfs are activated for FPGA and disabled for simulation. */
#define PRINTF_IN_FPGA  1
#define PRINTF_IN_SIM   1

#if TARGET_SIM && PRINTF_IN_SIM
        #define PRINTF(fmt, ...)    printf(fmt, ## __VA_ARGS__)
#elif PRINTF_IN_FPGA && !TARGET_SIM
    #define PRINTF(fmt, ...)    printf(fmt, ## __VA_ARGS__)
#else
    #define PRINTF(...)
#endif


int main(int argc, char *argv[])
{
    gpio_result_t gpio_res;
    gpio_cfg_t pin_cfg = {
        .pin = GPIO_TOGGLE,
        .mode = GpioModeOutPushPull
    };
    gpio_res = gpio_config (pin_cfg);
    if (gpio_res != GpioOk)
        PRINTF("Gpio initialization failed!\n");

    while (1) {
        gpio_write(GPIO_TOGGLE, true);
        PRINTF("GPIO %d set to HIGH\r\n", GPIO_TOGGLE);
        for(int i=0;i<1000000;i++) asm volatile("nop");
        gpio_write(GPIO_TOGGLE, false);
        PRINTF("GPIO %d set to LOW\r\n", GPIO_TOGGLE);
        for(int i=0;i<1000000;i++) asm volatile("nop");
    }

    return EXIT_SUCCESS;
}
