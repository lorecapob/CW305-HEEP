/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 06/05/2025
 * Description: AES S-Box header file.
 */

#ifndef SBOX_H
#define SBOX_H

#include <stdint.h>

// Function to get the S-box
const uint8_t* get_sbox(int version);

// Function to get the inverse S-box
const uint8_t* get_inv_sbox(int version);

#endif // SBOX_H
