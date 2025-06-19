#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "api.h"
#include "crypto_aead.h"

// --------- X-HEEP includes and defines ---------
#define XHEEP_PRINT 0

#include "core_v_mini_mcu.h"
#include "x-heep.h"
#include "gpio.h"

#define GPIO_INPUT_TRIGGER 3
#define GPIO_SCOPE_TRIGGER 4
// ----------------------------------------------

#define POWER_TRACES 10000 // Number of power traces collected for each iteration


void print(unsigned char c, unsigned char* x, unsigned long long xlen) {
  unsigned long long i;
  printf("%c[%d]=", c, (int)xlen);
  for (i = 0; i < xlen; ++i) printf("%02x", x[i]);
  printf("\n");
}

int main() {

  // GPIO initialization. THe program polls GPIO 3 for the trigger signal
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

  // Initialize nonce, key, associated data, and message (in this order)
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
  
  // Output ciphertext, hash and tag
  unsigned char c[32], h[32], t[32];

  // Lengths of the associated data, message and ciphertext
  unsigned long long alen = 16;
  unsigned long long mlen = 16;
  unsigned long long clen;
  int result = 0;
  
// Encryption and decryption loop
for (int i = 0; i < POWER_TRACES; i++) {

  // Wait for the trigger signal
  while (!pin_value) {
    gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
  }
  // // Set the trigger signal for the scope
  // gpio_write(GPIO_SCOPE_TRIGGER, 1);

#if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
  printf("input:\n");
  print('k', k, CRYPTO_KEYBYTES);
  print('n', n, CRYPTO_NPUBBYTES);
  print('a', a, alen);
  print('m', m, mlen);
  printf("\n");
#endif

  result |= crypto_aead_encrypt(c, &clen, m, mlen, a, alen, (void*)0, n, k);

  // // Reset the trigger signal for the scope
  // gpio_write(GPIO_SCOPE_TRIGGER, 0);
  // Wait for the trigger signal to go low again
  while (pin_value) {
    gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
  }

#if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
  printf("encrypt:\n");
  print('c', c, clen - CRYPTO_ABYTES);
  print('t', c + clen - CRYPTO_ABYTES, CRYPTO_ABYTES);
  printf("\n");
#endif

  // Update the new message with the previous ciphertext
  memcpy(m, c, mlen);

  //   result |= crypto_aead_decrypt(m, &mlen, (void*)0, c, clen, a, alen, n, k);

  // #if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
  //   printf("decrypt:\n");
  //   print('m', m, mlen);
  //   printf("\n");
  // #endif
}

  exit(result);
}
