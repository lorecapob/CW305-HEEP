// Generated register defines for pdm2pcm

// Copyright information found in source file:
// Copyright 2022 EPFL

// Licensing information found in source file:
// 
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

#ifndef _PDM2PCM_REG_DEFS_
#define _PDM2PCM_REG_DEFS_

#ifdef __cplusplus
extern "C" {
#endif
// Register width
#define PDM2PCM_PARAM_REG_WIDTH 32

// Control register
#define PDM2PCM_CLKDIVIDX_REG_OFFSET 0x0
#define PDM2PCM_CLKDIVIDX_COUNT_MASK 0xffff
#define PDM2PCM_CLKDIVIDX_COUNT_OFFSET 0
#define PDM2PCM_CLKDIVIDX_COUNT_FIELD \
  ((bitfield_field32_t) { .mask = PDM2PCM_CLKDIVIDX_COUNT_MASK, .index = PDM2PCM_CLKDIVIDX_COUNT_OFFSET })

// Control register
#define PDM2PCM_CONTROL_REG_OFFSET 0x4
#define PDM2PCM_CONTROL_ENABL_BIT 0
#define PDM2PCM_CONTROL_CLEAR_BIT 1

// Status register
#define PDM2PCM_STATUS_REG_OFFSET 0x8
#define PDM2PCM_STATUS_EMPTY_BIT 0
#define PDM2PCM_STATUS_FULLL_BIT 1

// Thermometric value of the activated stages (The 1s should be contiguous
// and right-aligned)
#define PDM2PCM_CIC_ACTIVATED_STAGES_REG_OFFSET 0xc
#define PDM2PCM_CIC_ACTIVATED_STAGES_CIC_ACTIVATED_STAGES_MASK 0x3f
#define PDM2PCM_CIC_ACTIVATED_STAGES_CIC_ACTIVATED_STAGES_OFFSET 0
#define PDM2PCM_CIC_ACTIVATED_STAGES_CIC_ACTIVATED_STAGES_FIELD \
  ((bitfield_field32_t) { .mask = PDM2PCM_CIC_ACTIVATED_STAGES_CIC_ACTIVATED_STAGES_MASK, .index = PDM2PCM_CIC_ACTIVATED_STAGES_CIC_ACTIVATED_STAGES_OFFSET })

// delay in each comb block (D)
#define PDM2PCM_CIC_DELAY_COMB_REG_OFFSET 0x10
#define PDM2PCM_CIC_DELAY_COMB_CIC_DELAY_COMB_MASK 0xf
#define PDM2PCM_CIC_DELAY_COMB_CIC_DELAY_COMB_OFFSET 0
#define PDM2PCM_CIC_DELAY_COMB_CIC_DELAY_COMB_FIELD \
  ((bitfield_field32_t) { .mask = PDM2PCM_CIC_DELAY_COMB_CIC_DELAY_COMB_MASK, .index = PDM2PCM_CIC_DELAY_COMB_CIC_DELAY_COMB_OFFSET })

// Samples count after which to decimate in the CIC filter.
#define PDM2PCM_DECIMCIC_REG_OFFSET 0x14
#define PDM2PCM_DECIMCIC_COUNT_MASK 0xf
#define PDM2PCM_DECIMCIC_COUNT_OFFSET 0
#define PDM2PCM_DECIMCIC_COUNT_FIELD \
  ((bitfield_field32_t) { .mask = PDM2PCM_DECIMCIC_COUNT_MASK, .index = PDM2PCM_DECIMCIC_COUNT_OFFSET })

// Memory area: PCM Receive data
#define PDM2PCM_RXDATA_REG_OFFSET 0x18
#define PDM2PCM_RXDATA_SIZE_WORDS 1
#define PDM2PCM_RXDATA_SIZE_BYTES 4
#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // _PDM2PCM_REG_DEFS_
// End generated register defines for pdm2pcm