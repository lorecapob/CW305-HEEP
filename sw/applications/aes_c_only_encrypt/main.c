#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Enable ECB, CTR and CBC mode. Note this can be done before including aes.h or at compile-time.
// E.g. with GCC by using the -D flag: gcc -c aes.c -DCBC=0 -DCTR=1 -DECB=1
#define CBC 0
#define CTR 0
#define ECB 1

#include "aes.h"
#include "sbox.h"

#define SBOX_VERSION 0

#define ITERATIONS 1 // Number of iterations for the Success Rate Test
#define POWER_TRACES 5000 // Number of power traces collected for each iteration
#define FIXED_PLAINTEXT 0 // 1 = use fixed plaintext, 0 = random plaintext. Needed for the TVLA test.

// --------- X-HEEP includes and defines ---------
#define XHEEP_PRINT 0

#include "core_v_mini_mcu.h"
#include "x-heep.h"
#include "gpio.h"

#define GPIO_INPUT_TRIGGER 3
#define GPIO_SCOPE_TRIGGER 4
// ----------------------------------------------

static void phex(uint8_t* str);


int main(void)
{
    // GPIO initialization
    gpio_result_t gpio_res;

    gpio_cfg_t pin_cfg3 = {
        .pin = GPIO_INPUT_TRIGGER,
        .mode = GpioModeIn,
        .en_input_sampling = true,
    };
    gpio_res = gpio_config (pin_cfg3);
    if (gpio_res != GpioOk){
        printf("Gpio %d initialization failed!\r\n", GPIO_INPUT_TRIGGER);
        return EXIT_FAILURE;
    }

    gpio_cfg_t pin_cfg4 = {
        .pin = GPIO_SCOPE_TRIGGER,
        .mode = GpioModeOutPushPull
    };
    gpio_res = gpio_config (pin_cfg4);
    if (gpio_res != GpioOk){
        printf("Gpio %d initialization failed!\r\n", GPIO_SCOPE_TRIGGER);
        return EXIT_FAILURE;
    }
    bool pin_value = 0;

    // Choose the S-box to use
    AES_init_sbox(SBOX_VERSION);
    AES_init_inv_sbox(SBOX_VERSION);

    // Secret key
    uint8_t key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
    // uint8_t key[] = { 0x3c, 0x8f, 0x26, 0x27, 0x39, 0xbf, 0xe3, 0xb1, 0xbc, 0x08, 0x26, 0x99, 0x1a, 0xd0, 0x52, 0x4d };
#if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
    printf("Key: ");
    phex(key);
    printf("\n");
#endif

    // Initial Plaintext
    uint8_t plain_text[16] = { 0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a };

    // Plaintext - Fixed value, equal to the first random plaintext generated above
    uint8_t fixed_plain_text[16];
    memcpy(fixed_plain_text, plain_text, sizeof(plain_text));
#if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
    printf("Fixed Plaintext: ");
    phex(fixed_plain_text);
    printf("\n");
#endif

    // AES initialization
    struct AES_ctx ctx;
    AES_init_ctx(&ctx, key);

    for (int k = 0; k < ITERATIONS; k++) {
    #if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
        printf("Iteration %d\n\n", k);
    #endif

        // Wait for the trigger signal
        while (!pin_value) {
            gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
        }
        // Wait for the trigger signal to be low
        while (pin_value) {
            gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
        }

        for (int i = 0; i < POWER_TRACES; i++) {

        #if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
            printf("Trace %d\n", i);
            printf("Plaintext: ");
            phex(plain_text);
        #endif

            // Wait for the trigger signal
            while (!pin_value) {
                gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
            }

            // When this function is called, the plaintext is encrypted in place and the ciphertext 
            // is stored in the same plain_text variable.
            AES_ECB_encrypt(&ctx, plain_text);

            // Wait for the trigger signal to go low again
            while (pin_value) {
                gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
            }

        #if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
            printf("Ciphertext: ");
            phex(plain_text);
            printf("\n");
        #endif

        #if defined(FIXED_PLAINTEXT) && (FIXED_PLAINTEXT == 1)
            // Set the plaintext back to the fixed value
            memcpy(plain_text, fixed_plain_text, sizeof(plain_text));
        #endif
        }
    }
#if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
    printf("Finished!\n");
#endif

    return 0;
}

// prints string as hex
static void phex(uint8_t* str)
{
    uint8_t len = 16;

    unsigned char i;
    for (i = 0; i < len; ++i)
        printf("%.2x", str[i]);
    printf("\n");
}
