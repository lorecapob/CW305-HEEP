#ifndef ROUND_H_
#define ROUND_H_

#include "ascon.h"
#include "constants.h"
#include "forceinline.h"
#include "printstate.h"
#include "word.h"

#include "sbox.h"

forceinline void ROUND(ascon_state_t* s, uint8_t C) {
  uint64_t xtemp;
  /* round constant */
  s->x[2] ^= C;

//   /* s-box layer */
//   s->x[0] ^= s->x[4];
//   s->x[4] ^= s->x[3];
//   s->x[2] ^= s->x[1];
//   xtemp = s->x[0] & ~s->x[4];
//   s->x[0] ^= s->x[2] & ~s->x[1];
//   s->x[2] ^= s->x[4] & ~s->x[3];
//   s->x[4] ^= s->x[1] & ~s->x[0];
//   s->x[1] ^= s->x[3] & ~s->x[2];
//   s->x[3] ^= xtemp;
//   s->x[1] ^= s->x[0];
//   s->x[3] ^= s->x[2];
//   s->x[0] ^= s->x[4];
//   s->x[2] = ~s->x[2];

  /* LUT S-Box layer */
  for (uint8_t i = 0; i < 64; i++) {
      // Extract column 0 (LSB of each word) and form the S-box input
      // as (x[0]_j, x[1]_j, x[2]_j, x[3]_j, x[4]_j)
      uint8_t sbox_input = ((s->x[0] & 1) << 4) |
                          ((s->x[1] & 1) << 3) |
                          ((s->x[2] & 1) << 2) |
                          ((s->x[3] & 1) << 1) |
                          ((s->x[4] & 1) << 0);

      // Pass through S-box
      uint8_t sbox_output = sbox(SBOX_ASCON, sbox_input);

      // Shift right and insert S-box output at MSB position
      for (uint8_t k = 0; k < 5; k++) {
          s->x[k] = (s->x[k] >> 1) | (((uint64_t)((sbox_output >> (4 - k)) & 1)) << 63);
      }
  }

  /* linear layer */
  // Set the trigger signal for the scope just for the first round
  gpio_write(4, 1);
  s->x[0] ^=
      (s->x[0] >> 19) ^ (s->x[0] << 45) ^ (s->x[0] >> 28) ^ (s->x[0] << 36);
  s->x[1] ^=
      (s->x[1] >> 61) ^ (s->x[1] << 3) ^ (s->x[1] >> 39) ^ (s->x[1] << 25);
  s->x[2] ^=
      (s->x[2] >> 1) ^ (s->x[2] << 63) ^ (s->x[2] >> 6) ^ (s->x[2] << 58);
  s->x[3] ^=
      (s->x[3] >> 10) ^ (s->x[3] << 54) ^ (s->x[3] >> 17) ^ (s->x[3] << 47);
  s->x[4] ^=
      (s->x[4] >> 7) ^ (s->x[4] << 57) ^ (s->x[4] >> 41) ^ (s->x[4] << 23);
  printstate(" round output", s);
}

forceinline void PROUNDS(ascon_state_t* s, int nr) {
  int i = START(nr);
  do {
    ROUND(s, RC(i));
    i += INC;
  } while (i != END);
}

#endif /* ROUND_H_ */
