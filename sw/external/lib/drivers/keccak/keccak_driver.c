#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>

#include "x-heep.h"

#include "keccak_x_heep.h"
#include "core_v_mini_mcu.h"
#include "keccak_driver.h"
#include "keccak_ctrl_auto.h"
#include "keccak_data_auto.h"
#include "x-heep.h"

#include "stats.h"

// To manage interrupt
#include "handler.h"
#include "csr.h"
#include "rv_plic.h"
#include "rv_plic_regs.h"
#include "rv_plic_structs.h"
#include "hart.h"

// To manage DMA
#include "dma.h"

#define KECCAK_BUSY 0
#define DATA_SIZE 50

/* By default, printfs are activated for FPGA and disabled for simulation. */
#define PRINTF_IN_FPGA  1
#define PRINTF_IN_SIM   0

#if TARGET_SIM && PRINTF_IN_SIM
        #define PRINTF(fmt, ...)    printf(fmt, ## __VA_ARGS__)
#elif PRINTF_IN_FPGA && !TARGET_SIM
    #define PRINTF(fmt, ...)    printf(fmt, ## __VA_ARGS__)
#else
    #define PRINTF(...)
#endif

int8_t cycles = 0;

// INTERRUPT HANDLERS
void dma_intr_handler_trans_done(uint8_t channel)
{
    cycles++;
}

#ifndef USE_DMA
#define USE_DMA 1
#endif

// Interrupt controller variables
plic_result_t plic_res;

/* ISR that just sets a flag */
volatile int keccak_done = 0;
static void isr_keccak_done(uint32_t id)
{
    /* 1.  Clear the pending bit *inside* the accelerator           */
    //((volatile uint32_t *)KECCAK_STATUS_START_ADDR)[0] =
    //    1 << KECCAK_STATUS_DONE_BIT;

    /* 2.  Tell the main thread we finished                          */
    keccak_done = 1;
}

// UTILITIES

#define WAIT_DMA                              \
    while (!dma_is_ready(0))                   \
    {                                         \
        CSR_CLEAR_BITS(CSR_REG_MSTATUS, 0x8); \
        if (dma_is_ready(0) == 0)              \
        {                                     \
            wait_for_interrupt();             \
        }                                     \
        CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);   \
    }


#define RUN_DMA                                                                               \
    trans.flags = 0x0;                                                                        \
    res = dma_validate_transaction(&trans, DMA_ENABLE_REALIGN, DMA_PERFORM_CHECKS_INTEGRITY); \
    PRINTF("tran: %u \t%s\n\r", res, res == DMA_CONFIG_OK ? "Ok!" : "Error!");                \
    res = dma_load_transaction(&trans);                                                       \
    PRINTF("load: %u \t%s\n\r", res, res == DMA_CONFIG_OK ? "Ok!" : "Error!");                \
    res = dma_launch(&trans);                                                                 \
    PRINTF("laun: %u \t%s\n\r", res, res == DMA_CONFIG_OK ? "Ok!" : "Error!");


  
void KeccakF1600_StatePermute(uint32_t* Din, uint32_t* Dout)
{
#pragma message ("USE_DMA set to: " USE_DMA);
  uint32_t volatile *Din_reg_start = (uint32_t*)KECCAK_DIN_START_ADDR;
  uint32_t volatile *ctrl_reg = (uint32_t*)KECCAK_CTRL_START_ADDR;
  uint32_t volatile *status_reg = (uint32_t*)KECCAK_STATUS_START_ADDR;
  uint32_t current_status;
  uint32_t volatile *Dout_reg_start = (uint32_t*)KECCAK_DOUT_START_ADDR;
  
  // Performance regs variables
  unsigned int instr, cycles, ldstall, jrstall, imstall;
  
  uint32_t* ext_addr_4B_PTR = (uint32_t*)KECCAK_DIN_START_ADDR;
 
  // Keccak accelerator send interrupt on ext_intr line 0
  PRINTF("Interrupt id : %d\n", EXT_INTR_0);
  PRINTF("Init the PLIC...");
  plic_res = plic_Init();                                 // initialise PLIC fabric
  if (plic_res != kPlicOk) {
      PRINTF("Init PLIC failed\n\r;");
      return -1;
  }
  PRINTF("Number of clock cycles : %d\n", cycles);
  // Set Keccak priority to 1 (target threshold is by default 0) to trigger an interrupt to the target (the processor)
    plic_res = plic_irq_set_priority(EXT_INTR_0, 1); // Keccak done IRQ
    if (plic_res != kPlicOk) {
      PRINTF("Failed\n\r;");
      return -1;
  }
  // Enable the interrupt in reg 0 
  PRINTF("Enable Keccak interrupt...");
  plic_res = plic_irq_set_enabled(EXT_INTR_0, kPlicToggleEnabled);
  if (plic_res != kPlicOk) {
    PRINTF("Failed\n\r;");
    return -1;
  }
PRINTF("!");
  plic_assign_external_irq_handler(EXT_INTR_0, isr_keccak_done); 
  // Enable interrupt on processor side
  // Enable global interrupt for machine-level interrupts
  CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);
  // Set mie.MEIE bit to one to enable machine-level external interrupts
  const uint32_t mask = 1 << 11;//IRQ_EXT_ENABLE_OFFSET;
  CSR_SET_BITS(CSR_REG_MIE, mask);

  // Starting the performance counter
  CSR_WRITE(CSR_REG_MCYCLE, 0);

  #if USE_DMA == 1
  PRINTF("Keccak : using DMA\n");
  // The DMA is initialized (i.e. Any current transaction is cleaned.)
  dma_init(NULL);
    
  dma_config_flags_t res;

  PRINTF("din_src_ptr: %04x, keccak_din_ptr : %04x\n", Din_4B, ext_addr_4B_PTR);

  // First DMA transaction consist on loading Din in Keccak register file
   
  dma_target_t tgt_src = {
                              .ptr        = Din,
                              .inc_d1_du     = 1,
                              .trig       = DMA_TRIG_MEMORY,
                              .type       = DMA_DATA_TYPE_WORD,
                              };
  dma_target_t tgt_dst = {
                              .ptr        = ext_addr_4B_PTR,
                              .inc_d1_du     = 1,
                              .trig       = DMA_TRIG_MEMORY,
                              .type       = DMA_DATA_TYPE_WORD
                              };

  dma_target_t tgt_addr = {
                              .ptr        = ext_addr_4B_PTR,
                              .inc_d1_du     = 1,
                              .trig       = DMA_TRIG_MEMORY,
                              };

  dma_trans_t trans = {
                              .src        = &tgt_src,
                              .dst        = &tgt_dst,
                              .src_addr   = &tgt_addr,
                              .size_d1_du = DATA_SIZE,          /* <-- here */
                              .src_type   = DMA_DATA_TYPE_WORD, /* keeps HAL checks happy */
                              .dst_type   = DMA_DATA_TYPE_WORD,
                              .mode       = DMA_TRANS_MODE_SINGLE,
                              .win_du     = 0,
                              .sign_ext  = 0,
                              .end        = DMA_TRANS_END_INTR,
                              .dim        = DMA_DIM_CONF_1D,
                              };
  // Create a target pointing at the buffer to be copied. Whole WORDs, no skippings, in memory, no environment.  

PRINTF("\n\n=====================================\n\n");
PRINTF("    TESTING SINGLE MODE WITH KECCAK  ");
PRINTF("\n\n=====================================\n\n");

  RUN_DMA                                                                                                          \
  WAIT_DMA    

  PRINTF(">> Finished transaction Din. \n");

  #else
 
  //PRINTF("Keccak : not using DMA\n");
  for (int i = 0; i<50; i++)
  {
     Din_reg_start[i] = Din[i];
  }

  #endif
 
  keccak_done = 0; // Reset the done flag
  asm volatile ("": : : "memory");
  *ctrl_reg = 1 << KECCAK_CTRL_CTRL_START_BIT; // start core
  asm volatile ("": : : "memory");
  *ctrl_reg = 0 << KECCAK_CTRL_CTRL_START_BIT;
 
  PRINTF("Keccak started...\n");
  // Wait till keccak is done
  while(keccak_done==0) {
      wait_for_interrupt();
  }
  PRINTF("Keccak finished...\n");
	 
  #if USE_DMA == 1

  ext_addr_4B_PTR = (uint32_t*)KECCAK_DOUT_START_ADDR;
  tgt_src.ptr = ext_addr_4B_PTR;
  tgt_dst.ptr = Dout;

  PRINTF("dout_dst_ptr: %04x, keccak_dout_ptr : %04x\n", Dout_4B, ext_addr_4B_PTR);

  // Second DMA transaction consist on reading Dout from Keccak register file

  RUN_DMA
  WAIT_DMA    
  PRINTF(">> Finished transaction Dout. \n");
     
  #else
  for (volatile int i = 0; i<DATA_SIZE; i++){
     Dout[i] = Dout_reg_start[i];
     PRINTF("Dout[%d]=%04X\n", i, Dout[i]);
  }

  #endif

  // stop the HW counter used for monitoring
  CSR_READ(CSR_REG_MCYCLE, &cycles);
  PRINTF("Number of clock cycles : %d\n", cycles);
  PRINTF("Number of instructions : %d\nNumber of clock cycles: %d\nCPI: %f%f\n",instr_cnt, cycles_cnt, (float) instr_cnt/cycles_cnt);
  
}

