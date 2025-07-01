/*
    Author: Lorenzo Capobianco
    Date: 30/06/2025
    Description:
    This program performs the first round of the ASCON authenticated encryption algorithm.
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
#include "ascon.h"
#include "word.h"
#include "constants.h"
#include "round.h"
#include "printstate.h"

// --------- X-HEEP includes and defines ---------
#define XHEEP_PRINT 0

#include "core_v_mini_mcu.h"
#include "x-heep.h"
#include "gpio.h"

#define GPIO_INPUT_TRIGGER 3
#define GPIO_SCOPE_TRIGGER 4
// ----------------------------------------------

// Number of power traces collected for each iteration
#define POWER_TRACES 50000


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

  // Initialize nonce, key
  unsigned char nonce[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                         11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                         22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
  unsigned char key[32] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                         11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                         22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
  
// Encryption loop
for (int i = 0; i < POWER_TRACES; i++) {
  // Wait for the trigger signal
  while (!pin_value) {
    gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
  }

  // Initial part of the ASCON algorithm
#if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
  /* print input bytes */
  printf("encrypt\n");
  printbytes("k", key, CRYPTO_KEYBYTES);
  printbytes("n", nonce, CRYPTO_NPUBBYTES);
#endif

  /* load key and nonce */
  const uint64_t K0 = LOADBYTES(key, 8);
  const uint64_t K1 = LOADBYTES(key + 8, 8);
  const uint64_t N0 = LOADBYTES(nonce, 8);
  const uint64_t N1 = LOADBYTES(nonce + 8, 8);

  /* initialize */
  ascon_state_t s;
  s.x[0] = ASCON_128A_IV;
  s.x[1] = K0;
  s.x[2] = K1;
  s.x[3] = N0;
  s.x[4] = N1;

#if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
  printstate("Initial state", &s);
#endif

  ROUND(&s, 0xf0);

  // Wait for the trigger signal to go low again
  while (pin_value) {
    gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
  }

#if defined(XHEEP_PRINT) && (XHEEP_PRINT == 1)
  printstate("state after first round", &s);
#endif

  // Update nonce for the next iteration
  memcpy(nonce, &s.x[4], 8);
  memcpy(nonce + 8, &s.x[3], 8);

}

  exit(EXIT_SUCCESS);
}
