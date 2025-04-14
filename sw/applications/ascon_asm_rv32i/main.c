//
// NIST-developed software is provided by NIST as a public service.
// You may use, copy and distribute copies of the software in any medium,
// provided that you keep intact this entire notice. You may improve,
// modify and create derivative works of the software or any portion of
// the software, and you may copy and distribute such modifications or
// works. Modified works should carry a notice stating that you changed
// the software and should note the date and nature of any such change.
// Please explicitly acknowledge the National Institute of Standards and
// Technology as the source of the software.
//
// NIST-developed software is expressly provided "AS IS." NIST MAKES NO
// WARRANTY OF ANY KIND, EXPRESS, IMPLIED, IN FACT OR ARISING BY OPERATION
// OF LAW, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT AND DATA
// ACCURACY. NIST NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE
// SOFTWARE WILL BE UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE
// CORRECTED. NIST DOES NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE
// USE OF THE SOFTWARE OR THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE
// CORRECTNESS, ACCURACY, RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
//
// You are solely responsible for determining the appropriateness of using and
// distributing the software and you assume all risks associated with its use,
// including but not limited to the risks and costs of program errors,
// compliance with applicable laws, damage to or loss of data, programs or
// equipment, and the unavailability or interruption of operation. This software
// is not intended to be used in any situation where a failure could cause risk
// of injury or damage to property. The software developed by NIST employees is
// not subject to copyright protection within the United States.
//

// This file has been modified. The history of changes can be found at:
// https://github.com/ascon/ascon-c/commits/main/tests/genkat_aead.c

// Modified by Lorenzo Capobianco.
// Note: all the printf were once fprintf to a file pointer.

// disable deprecation for sprintf and fopen
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "csr.h"
#include "core_v_mini_mcu.h"
#include "gpio.h"
#include "x-heep.h"

#include "api.h"
#include "crypto_aead.h"

#define GPIO_INPUT_TRIGGER 3
#define GPIO_SCOPE_TRIGGER 4

#define KAT_SUCCESS 0
#define KAT_CRYPTO_FAILURE -4

#ifndef MAX_MESSAGE_LENGTH
#define MAX_MESSAGE_LENGTH 32
#endif
#define MAX_ASSOCIATED_DATA_LENGTH 32

// Set to 1 to enable printf, or 0 to disable (faster simulation)
#define ENABLE_PRINTF 1

#if ENABLE_PRINTF
    #define PRINTF(fmt, ...)    printf(fmt, ## __VA_ARGS__)
#else
    #define PRINTF(...)
#endif

void init_buffer(unsigned char offset, unsigned char* buffer,
                 unsigned long long numbytes);

void fprint_bstr(const char* label, const unsigned char* data,
                 unsigned long long length);

int generate_test_vectors();

int main() {
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
  unsigned int cycles;

  // Enable mcycle csr (NumCycles Performance counter)
  CSR_CLEAR_BITS(CSR_REG_MCOUNTINHIBIT, 0x1);

  // Wait for the trigger signal
  while (!pin_value) {
    gpio_read(GPIO_INPUT_TRIGGER, &pin_value);
  }

  // Reset mcycle csr
  CSR_WRITE(CSR_REG_MCYCLE, 0);

  // Execute the algorithm
  int ret = generate_test_vectors();
  if (ret != KAT_SUCCESS) {
    PRINTF("test vector generation failed with code %d\r\n", ret);
  }

  // Read the number of cycles
  CSR_READ(CSR_REG_MCYCLE, &cycles);
  
  // Always on printf to signal the end of the program and the number of cycles required
  printf("Program finished in %d cycles\r\n", cycles);

  return ret;
}

int generate_test_vectors() {
  unsigned char key[CRYPTO_KEYBYTES];
  unsigned char nonce[CRYPTO_NPUBBYTES];
  unsigned char* msg;
  unsigned char* msg2;
  unsigned char* ad;
  unsigned char* ct;
  unsigned long long mlen, adlen;
  unsigned long long clen, mlen2;
  int count = 1;
  int func_ret, ret_val = KAT_SUCCESS;

  init_buffer(0x00, key, sizeof(key));
  init_buffer(0x10, nonce, sizeof(nonce));

  for (mlen = 0; mlen <= MAX_MESSAGE_LENGTH; mlen++) {
    msg = malloc(mlen);
    msg2 = malloc(mlen);
    ct = malloc(mlen + CRYPTO_ABYTES);
    init_buffer(0x20, msg, mlen);

    for (adlen = 0; adlen <= MAX_ASSOCIATED_DATA_LENGTH; adlen++) {
      ad = malloc(adlen);
      init_buffer(0x30, ad, adlen);

      PRINTF("Count = %d\r\n", count++);
      fprint_bstr("Key = ", key, CRYPTO_KEYBYTES);
      fprint_bstr("Nonce = ", nonce, CRYPTO_NPUBBYTES);
      fprint_bstr("PT = ", msg, mlen);
      fprint_bstr("AD = ", ad, adlen);

      if ((func_ret = crypto_aead_encrypt(ct, &clen, msg, mlen, ad, adlen, NULL,
                                          nonce, key)) != 0) {
        PRINTF("crypto_aead_encrypt returned <%d>\r\n", func_ret);
        ret_val = KAT_CRYPTO_FAILURE;
        free(ad);
        break;
      }

      fprint_bstr("CT = ", ct, clen);
      PRINTF("\r\n");

      if ((func_ret = crypto_aead_decrypt(msg2, &mlen2, NULL, ct, clen, ad,
                                          adlen, nonce, key)) != 0) {
        PRINTF("crypto_aead_decrypt returned <%d>\r\n", func_ret);
        ret_val = KAT_CRYPTO_FAILURE;
        free(ad);
        break;
      }

      if (mlen != mlen2) {
        PRINTF( "crypto_aead_decrypt returned bad 'mlen': Got <%" PRIu32
                ">, expected <%" PRIu32 ">\r\n",
                (uint32_t)mlen2, (uint32_t)mlen);
        ret_val = KAT_CRYPTO_FAILURE;
        free(ad);
        break;
      }

      if (memcmp(msg, msg2, mlen)) {
        PRINTF("crypto_aead_decrypt did not recover the plaintext\r\n");
        ret_val = KAT_CRYPTO_FAILURE;
        free(ad);
        break;
      }

      // test failing verification
      ct[0] ^= 1;
      if ((func_ret = crypto_aead_decrypt(msg2, &mlen2, NULL, ct, clen, ad,
                                          adlen, nonce, key)) == 0) {
        PRINTF("crypto_aead_decrypt should have failed\r\n");
        ret_val = KAT_CRYPTO_FAILURE;
        free(ad);
        break;
      }
      free(ad);
    }
    free(msg);
    free(msg2);
    free(ct);
    if (ret_val != KAT_SUCCESS) break;
  }

  return ret_val;
}

void fprint_bstr(const char* label, const unsigned char* data,
                 unsigned long long length) {
  unsigned long long i;
  PRINTF("%s", label);
  for (i = 0; i < length; i++) PRINTF("%02X", data[i]);
  PRINTF("\r\n");
}

void init_buffer(unsigned char offset, unsigned char* buffer,
                 unsigned long long numbytes) {
  unsigned long long i;
  for (i = 0; i < numbytes; i++) buffer[i] = (unsigned char)(offset + i);
}
