/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 11/04/2025
 * Description: This program reads the value of GPIO 3 and writes it to GPIO 4.
 */

#include <stdio.h>
#include <stdlib.h>
#include "core_v_mini_mcu.h"
#include "gpio.h"
#include "x-heep.h"

#define GPIO_INPUT_TRIGGER 3
#define GPIO_SCOPE_TRIGGER 4

/* By default, printfs are activated for FPGA and disabled for simulation. */
#define PRINTF_IN_FPGA  1
#define PRINTF_IN_SIM   0

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

    gpio_cfg_t pin_cfg3 = {
        .pin = GPIO_INPUT_TRIGGER,
        .mode = GpioModeIn,
        .en_input_sampling = true,
    };
    gpio_res = gpio_config (pin_cfg3);
    if (gpio_res != GpioOk){
        PRINTF("Gpio %d initialization failed!\r\n", GPIO_INPUT_TRIGGER);
        return EXIT_FAILURE;
    }

    gpio_cfg_t pin_cfg4 = {
        .pin = GPIO_SCOPE_TRIGGER,
        .mode = GpioModeOutPushPull
    };
    gpio_res = gpio_config (pin_cfg4);
    if (gpio_res != GpioOk){
        PRINTF("Gpio %d initialization failed!\r\n", GPIO_SCOPE_TRIGGER);
        return EXIT_FAILURE;
    }

    bool pin_value = 0;

    gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
    PRINTF("GPIO %d is %d.\r\n", GPIO_INPUT_TRIGGER, pin_value);
    gpio_write(GPIO_SCOPE_TRIGGER, pin_value);
    PRINTF("GPIO %d is driven %d.\r\n", GPIO_SCOPE_TRIGGER, pin_value);

    return EXIT_SUCCESS;
}
