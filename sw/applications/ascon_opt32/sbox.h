/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 04/09/2025
 * Description: ASCON S-Box header file.
 */

#ifndef SBOX_H
#define SBOX_H

#include <stdint.h>

#define SBOX_ASCON 0
#define SBOX_BILGIN 1
#define SBOX_ALLOUZI 2
#define SBOX_LU_4 3
#define SBOX_LU_5 4
#define SBOX_LU_6 5
#define SBOX_LU_7 6

// Function to get the S-box
const uint8_t sbox(uint8_t version, uint8_t sbox_input);

#endif // SBOX_H
