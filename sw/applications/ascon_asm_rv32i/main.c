/*
    Author: Lorenzo Capobianco
    Date: 11/07/2025
    Description:
    This program performs the first round of the ASCON authenticated encryption algorithm for RV32I CPUs.
    In particular, the state registers are initialized with the key, nonce and the initialization vector.
    Then the first round of the ASCON algorithm is executed, which consists of an addition of a round constant,
    a substitution layer and a linear diffusion layer.
    At the end of the first round, the state registers 3 and 4 are used as new nonce value for the next iteration.
    There is no need to perform the full ASCON algorithm, as the goal is to collect power traces
    for the first round only, which is were the secret key is used, so that the
    power traces can be used for side-channel analysis.
    The program uses GPIO pins to trigger the scope and to read the trigger signal.
    The GPIO pin 3 is used to read the trigger signal, while GPIO pin 4
    is used to trigger the scope.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "api.h"
#include "crypto_aead.h"

// --------- X-HEEP includes and defines ---------
#define XHEEP_PRINT 1

#include "core_v_mini_mcu.h"
#include "x-heep.h"
#include "gpio.h"

#define GPIO_INPUT_TRIGGER 3
#define GPIO_SCOPE_TRIGGER 4
// ----------------------------------------------

// Number of power traces collected for each iteration
#define POWER_TRACES 2


void print(unsigned char c, unsigned char* x, unsigned long long xlen) {
  unsigned long long i;
  printf("%c[%d]=", c, (int)xlen);
  for (i = 0; i < xlen; ++i) printf("%02x", x[i]);
  printf("\n");
}

int main() {
  // GPIO initialization. The program polls GPIO 3 for the trigger signal
  // and uses GPIO 4 to trigger the scope.
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

  unsigned char n[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                         11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                         22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
  unsigned char k[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                         11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                         22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
  unsigned char a[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                         11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                         22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
  unsigned char m[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                         11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                         22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
  unsigned char c[32], h[32], t[32];
  unsigned long long alen = 16;
  unsigned long long mlen = 16;
  unsigned long long clen;
  int result = 0;

  // Encryption loop
  for (int i = 0; i < POWER_TRACES; i++) {
  #if (defined XHEEP_PRINT) && (XHEEP_PRINT == 1)
    printf("input:\n");
    print('k', k, CRYPTO_KEYBYTES);
    print('n', n, CRYPTO_NPUBBYTES);
  #endif
    // Wait for the trigger signal
    while (!pin_value) {
      gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
    }
    
    result |= crypto_aead_encrypt(c, &clen, m, mlen, a, alen, (void*)0, n, k);

    // Wait for the trigger signal to go low again
    while (pin_value) {
      gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
    }
  #if (defined XHEEP_PRINT) && (XHEEP_PRINT == 1)
    printf("output:\n");
    print('k', k, CRYPTO_KEYBYTES);
    print('n', n, CRYPTO_NPUBBYTES);
  #endif
  }

  exit(result);
}
