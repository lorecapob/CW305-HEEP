// Copyright EPFL contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <stdlib.h>
#include "core_v_mini_mcu.h"
#include "gpio.h"
#include "x-heep.h"

#define GPIO_INPUT_TRIGGER 3
#define GPIO_SCOPE_TRIGGER 4

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

    gpio_cfg_t pin_cfg3 = {
        .pin = GPIO_INPUT_TRIGGER,
        .mode = GpioModeIn
    };
    gpio_res = gpio_config (pin_cfg3);
    if (gpio_res != GpioOk){
        PRINTF("Gpio %d initialization failed!\n", GPIO_INPUT_TRIGGER);
        return EXIT_FAILURE;
    }

    gpio_cfg_t pin_cfg4 = {
        .pin = GPIO_SCOPE_TRIGGER,
        .mode = GpioModeOutPushPull
    };
    gpio_res = gpio_config (pin_cfg4);
    if (gpio_res != GpioOk){
        PRINTF("Gpio %d initialization failed!\n", GPIO_SCOPE_TRIGGER);
        return EXIT_FAILURE;
    }

    if (gpio_read(GPIO_INPUT_TRIGGER, true)) {
        PRINTF("GPIO %d is low.\n", GPIO_INPUT_TRIGGER);
        gpio_write(GPIO_SCOPE_TRIGGER, false);
        PRINTF("Driven GPIO %d is low.\n", GPIO_SCOPE_TRIGGER);
    } else {
        PRINTF("GPIO %d is high.\n", GPIO_INPUT_TRIGGER);
        gpio_write(GPIO_SCOPE_TRIGGER, true);
        PRINTF("Driven GPIO %d is high.\n", GPIO_SCOPE_TRIGGER);
    }

    return EXIT_SUCCESS;
}
