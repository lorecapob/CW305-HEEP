#include "printstate.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

void print(const char* text) { printf("%s", text); }

void printbytes(const char* text, const uint8_t* b, uint64_t len) {
  uint64_t i;
  printf(" %s[%d]\t= {", text, len);
  for (i = 0; i < len; ++i) printf("0x%02x%s", b[i], i < len - 1 ? ", " : "");
  printf("}\n");
}

void printword(const char* text, const uint64_t x) {
  // The x variable is a 64-bit integer, split into two 32-bit parts for printing.
  uint32_t hi = (uint32_t)(x >> 32);
  uint32_t lo = (uint32_t)(x & 0xFFFFFFFF);
  printf("%s=0x%08x%08x", text, hi, lo);
}

void printstate(const char* text, const ascon_state_t* s) {
  int i;
  printf("%s:", text);
  for (i = strlen(text); i < 17; ++i) printf(" ");
  printword(" x0", s->x[0]);
  printword(" x1", s->x[1]);
  printword(" x2", s->x[2]);
  printword(" x3", s->x[3]);
  printword(" x4", s->x[4]);
#ifdef ASCON_PRINT_BI
  printf(" ");
  printf(" x0=%08x_%08x", s->w[0][1], s->w[0][0]);
  printf(" x1=%08x_%08x", s->w[1][1], s->w[1][0]);
  printf(" x2=%08x_%08x", s->w[2][1], s->w[2][0]);
  printf(" x3=%08x_%08x", s->w[3][1], s->w[3][0]);
  printf(" x4=%08x_%08x", s->w[4][1], s->w[4][0]);
#endif
  printf("\n");
}
