// Copyright 2024 Politecnico di Torino.
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// File: gr_heep_top.sv
// Author: Luigi Giuffrida
// Date: 16/10/2024
// Description: GR-heep top-level module

module gr_heep_top (
  // X-HEEP interface
  inout wire rst_ni,
  inout wire boot_select_i,
  inout wire execute_from_flash_i,
  inout wire jtag_tck_i,
  inout wire jtag_tms_i,
  inout wire jtag_trst_ni,
  inout wire jtag_tdi_i,
  inout wire jtag_tdo_o,
  inout wire uart_rx_i,
  inout wire uart_tx_o,
  inout wire exit_valid_o,
  inout wire gpio_0_io,
  inout wire gpio_1_io,
  inout wire gpio_2_o,

























  inout wire clk_i,
  inout wire exit_value_o,

  // Added to connect the bridge
  // Request section for the OBI interface
  input logic        req_i,
  input logic        we_i,
  input logic [ 3:0] be_i,
  input logic [31:0] addr_i,
  input logic [31:0] wdata_i,

  // Response section for the OBI interface
  output logic        gnt_o,
  output logic        rvalid_o,
  output logic [31:0] rdata_o
);
  import obi_pkg::*;
  import reg_pkg::*;
  import gr_heep_pkg::*;
  import core_v_mini_mcu_pkg::*;

  // PARAMETERS
  localparam int unsigned ExtXbarNmasterRnd = (gr_heep_pkg::ExtXbarNMaster > 0) ?
    gr_heep_pkg::ExtXbarNMaster : 32'd1;
  localparam int unsigned ExtDomainsRnd = core_v_mini_mcu_pkg::EXTERNAL_DOMAINS == 0 ?
    32'd1 : core_v_mini_mcu_pkg::EXTERNAL_DOMAINS;

  // INTERNAL SIGNALS
  // ----------------
  // Synchronized reset
  logic rst_nin_sync;

  // Exit value
  logic [31:0] exit_value;

  // X-HEEP external master ports
  obi_req_t heep_core_instr_req;
  obi_resp_t heep_core_instr_rsp;
  obi_req_t heep_core_data_req;
  obi_resp_t heep_core_data_rsp;
  obi_req_t heep_debug_master_req;
  obi_resp_t heep_debug_master_rsp;
  obi_req_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_read_req;
  obi_resp_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_read_rsp;
  obi_req_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_write_req;
  obi_resp_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_write_rsp;
  obi_req_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_addr_req;
  obi_resp_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_addr_rsp;

  // X-HEEP slave ports
  obi_req_t [ExtXbarNmasterRnd-1:0] heep_slave_req;
  obi_resp_t [ExtXbarNmasterRnd-1:0] heep_slave_rsp;

  // External master ports
  obi_req_t [ExtXbarNmasterRnd-1:0] gr_heep_master_req;
  obi_resp_t [ExtXbarNmasterRnd-1:0] gr_heep_master_resp;

  // X-HEEP external peripheral master ports
  reg_req_t heep_peripheral_req;
  reg_rsp_t heep_peripheral_rsp;

  // Interrupt vector
  logic [core_v_mini_mcu_pkg::NEXT_INT-1:0] ext_int_vector;

  // Power Manager signals
  logic cpu_subsystem_powergate_switch_n;
  logic cpu_subsystem_powergate_switch_ack_n;
  logic peripheral_subsystem_powergate_switch_n;
  logic peripheral_subsystem_powergate_switch_ack_n;

  // External SPC interface signals
  reg_req_t [AoSPCNum-1:0] ext_ao_peripheral_req;
  reg_rsp_t [AoSPCNum-1:0] ext_ao_peripheral_resp;


  // Pad controller
  reg_req_t pad_req;
  reg_rsp_t pad_rsp;
  logic [core_v_mini_mcu_pkg::NUM_PAD-1:0][0:0] pad_muxes;

  // External power domains
  logic [ExtDomainsRnd-1:0] external_subsystem_powergate_switch_n;
  logic [ExtDomainsRnd-1:0] external_subsystem_powergate_switch_ack_n;
  logic [ExtDomainsRnd-1:0] external_subsystem_powergate_iso_n;

  // External RAM banks retentive mode control
  logic [ExtDomainsRnd-1:0] external_ram_banks_set_retentive_n;

  // External domains reset
  logic [ExtDomainsRnd-1:0] external_subsystem_rst_n;

  // External domains clock-gating
  logic [ExtDomainsRnd-1:0] external_subsystem_clkgate_en_n;

  logic ext_debug_req;
  logic ext_debug_reset_n;

  // Tie the CV-X-IF coprocessor signals to a default value that will
  // receive petitions but reject all offloaded instructions
  // CV-X-IF is unused in core-v-mini-mcu as it has the cv32e40p CPU
  if_xif #() ext_xif ();

  initial begin
    ext_xif.compressed_ready = 1'b1;
    ext_xif.compressed_resp  = '0;

    ext_xif.issue_ready      = 1'b1;
    ext_xif.issue_resp       = '0;

    ext_xif.mem_valid        = 1'b0;
    ext_xif.mem_req          = '0;

    ext_xif.result_valid     = 1'b0;
    ext_xif.result           = '0;
  end

  // CORE-V-MINI-MCU input/output pins
  logic rst_nin_x, rst_nout_x, rst_noe_x;

  logic boot_select_in_x, boot_select_out_x, boot_select_oe_x;

  logic execute_from_flash_in_x, execute_from_flash_out_x, execute_from_flash_oe_x;

  logic jtag_tck_in_x, jtag_tck_out_x, jtag_tck_oe_x;

  logic jtag_tms_in_x, jtag_tms_out_x, jtag_tms_oe_x;

  logic jtag_trst_nin_x, jtag_trst_nout_x, jtag_trst_noe_x;

  logic jtag_tdi_in_x, jtag_tdi_out_x, jtag_tdi_oe_x;

  logic jtag_tdo_in_x, jtag_tdo_out_x, jtag_tdo_oe_x;

  logic uart_rx_in_x, uart_rx_out_x, uart_rx_oe_x;

  logic uart_tx_in_x, uart_tx_out_x, uart_tx_oe_x;

  logic exit_valid_in_x, exit_valid_out_x, exit_valid_oe_x;

  logic gpio_0_in_x, gpio_0_out_x, gpio_0_oe_x;

  logic gpio_1_in_x, gpio_1_out_x, gpio_1_oe_x;

  logic gpio_2_in_x, gpio_2_out_x, gpio_2_oe_x;

  logic spi_flash_sck_in_x, spi_flash_sck_out_x, spi_flash_sck_oe_x;

  logic spi_flash_cs_0_in_x, spi_flash_cs_0_out_x, spi_flash_cs_0_oe_x;

  logic spi_flash_cs_1_in_x, spi_flash_cs_1_out_x, spi_flash_cs_1_oe_x;
  logic gpio_31_in_x, gpio_31_out_x, gpio_31_oe_x;
  logic spi_flash_cs_1_in_x_muxed, spi_flash_cs_1_out_x_muxed, spi_flash_cs_1_oe_x_muxed;

  logic spi_flash_sd_0_in_x, spi_flash_sd_0_out_x, spi_flash_sd_0_oe_x;

  logic spi_flash_sd_1_in_x, spi_flash_sd_1_out_x, spi_flash_sd_1_oe_x;

  logic spi_flash_sd_2_in_x, spi_flash_sd_2_out_x, spi_flash_sd_2_oe_x;

  logic spi_flash_sd_3_in_x, spi_flash_sd_3_out_x, spi_flash_sd_3_oe_x;

  logic spi_sck_in_x, spi_sck_out_x, spi_sck_oe_x;

  logic spi_cs_0_in_x, spi_cs_0_out_x, spi_cs_0_oe_x;

  logic spi_cs_1_in_x, spi_cs_1_out_x, spi_cs_1_oe_x;

  logic spi_sd_0_in_x, spi_sd_0_out_x, spi_sd_0_oe_x;

  logic spi_sd_1_in_x, spi_sd_1_out_x, spi_sd_1_oe_x;

  logic spi_sd_2_in_x, spi_sd_2_out_x, spi_sd_2_oe_x;

  logic spi_sd_3_in_x, spi_sd_3_out_x, spi_sd_3_oe_x;

  logic pdm2pcm_pdm_in_x, pdm2pcm_pdm_out_x, pdm2pcm_pdm_oe_x;

  logic pdm2pcm_clk_in_x, pdm2pcm_clk_out_x, pdm2pcm_clk_oe_x;

  logic spi2_cs_0_in_x, spi2_cs_0_out_x, spi2_cs_0_oe_x;

  logic spi2_cs_1_in_x, spi2_cs_1_out_x, spi2_cs_1_oe_x;

  logic spi2_sck_in_x, spi2_sck_out_x, spi2_sck_oe_x;

  logic spi2_sd_0_in_x, spi2_sd_0_out_x, spi2_sd_0_oe_x;

  logic spi2_sd_1_in_x, spi2_sd_1_out_x, spi2_sd_1_oe_x;

  logic spi2_sd_2_in_x, spi2_sd_2_out_x, spi2_sd_2_oe_x;

  logic spi2_sd_3_in_x, spi2_sd_3_out_x, spi2_sd_3_oe_x;

  logic i2c_scl_in_x, i2c_scl_out_x, i2c_scl_oe_x;

  logic i2c_sda_in_x, i2c_sda_out_x, i2c_sda_oe_x;

  logic clk_in_x, clk_out_x, clk_oe_x;

  logic exit_value_in_x, exit_value_out_x, exit_value_oe_x;


  // Drive to zero bypassed pins
  assign spi_flash_sck_in_x  = 1'b0;
  assign spi_flash_cs_0_in_x = 1'b0;
  assign spi_flash_cs_1_in_x = 1'b0;
  assign gpio_31_in_x        = 1'b0;
  assign spi_flash_sd_0_in_x = 1'b0;
  assign spi_flash_sd_1_in_x = 1'b0;
  assign spi_flash_sd_2_in_x = 1'b0;
  assign spi_flash_sd_3_in_x = 1'b0;
  assign spi_sck_in_x        = 1'b0;
  assign spi_cs_0_in_x       = 1'b0;
  assign spi_cs_1_in_x       = 1'b0;
  assign spi_sd_0_in_x       = 1'b0;
  assign spi_sd_1_in_x       = 1'b0;
  assign spi_sd_2_in_x       = 1'b0;
  assign spi_sd_3_in_x       = 1'b0;
  assign pdm2pcm_pdm_in_x    = 1'b0;
  assign pdm2pcm_clk_in_x    = 1'b0;
  assign spi2_cs_0_in_x      = 1'b0;
  assign spi2_cs_1_in_x      = 1'b0;
  assign spi2_sck_in_x       = 1'b0;
  assign spi2_sd_0_in_x      = 1'b0;
  assign spi2_sd_1_in_x      = 1'b0;
  assign spi2_sd_2_in_x      = 1'b0;
  assign spi2_sd_3_in_x      = 1'b0;
  assign i2c_scl_in_x        = 1'b0;
  assign i2c_sda_in_x        = 1'b0;

  // --------------
  // SYSTEM MODULES
  // --------------

  // Reset generator
  // ---------------
  rstgen u_rstgen (
    .clk_i      (clk_in_x),
    .rst_ni     (rst_nin_x),
    .test_mode_i(1'b0),          // not implemented
    .rst_no     (rst_nin_sync),
    .init_no    ()               // unused
  );

  // CORE-V-MINI-MCU (microcontroller)
  // ---------------------------------
  core_v_mini_mcu #(
    .COREV_PULP      (CpuCorevPulp),
    .FPU             (CpuFpu),
    .ZFINX           (CpuRiscvZfinx),
    .EXT_XBAR_NMASTER(ExtXbarNMaster),
    .X_EXT           (CpuCorevXif),
    .AO_SPC_NUM      (AoSPCNum),
    .EXT_HARTS       (1)
  ) u_core_v_mini_mcu (
    .rst_ni(rst_nin_sync),
    .clk_i (clk_i),

    // MCU pads

    .boot_select_i(boot_select_in_x),

    .execute_from_flash_i(execute_from_flash_in_x),

    .jtag_tck_i(jtag_tck_in_x),

    .jtag_tms_i(jtag_tms_in_x),

    .jtag_trst_ni(jtag_trst_nin_x),

    .jtag_tdi_i(jtag_tdi_in_x),

    .jtag_tdo_o(jtag_tdo_out_x),

    .uart_rx_i(uart_rx_in_x),

    .uart_tx_o(uart_tx_out_x),

    .exit_valid_o(exit_valid_out_x),

    .gpio_0_i   (gpio_0_in_x),
    .gpio_0_o   (gpio_0_out_x),
    .gpio_0_oe_o(gpio_0_oe_x),

    .gpio_1_i   (gpio_1_in_x),
    .gpio_1_o   (gpio_1_out_x),
    .gpio_1_oe_o(gpio_1_oe_x),

    .gpio_2_o(gpio_2_out_x),

    .spi_flash_sck_i   (spi_flash_sck_in_x),
    .spi_flash_sck_o   (spi_flash_sck_out_x),
    .spi_flash_sck_oe_o(spi_flash_sck_oe_x),

    .spi_flash_cs_0_i   (spi_flash_cs_0_in_x),
    .spi_flash_cs_0_o   (spi_flash_cs_0_out_x),
    .spi_flash_cs_0_oe_o(spi_flash_cs_0_oe_x),

    .spi_flash_cs_1_i   (spi_flash_cs_1_in_x),
    .spi_flash_cs_1_o   (spi_flash_cs_1_out_x),
    .spi_flash_cs_1_oe_o(spi_flash_cs_1_oe_x),
    .gpio_31_i          (gpio_31_in_x),
    .gpio_31_o          (gpio_31_out_x),
    .gpio_31_oe_o       (gpio_31_oe_x),

    .spi_flash_sd_0_i   (spi_flash_sd_0_in_x),
    .spi_flash_sd_0_o   (spi_flash_sd_0_out_x),
    .spi_flash_sd_0_oe_o(spi_flash_sd_0_oe_x),

    .spi_flash_sd_1_i   (spi_flash_sd_1_in_x),
    .spi_flash_sd_1_o   (spi_flash_sd_1_out_x),
    .spi_flash_sd_1_oe_o(spi_flash_sd_1_oe_x),

    .spi_flash_sd_2_i   (spi_flash_sd_2_in_x),
    .spi_flash_sd_2_o   (spi_flash_sd_2_out_x),
    .spi_flash_sd_2_oe_o(spi_flash_sd_2_oe_x),

    .spi_flash_sd_3_i   (spi_flash_sd_3_in_x),
    .spi_flash_sd_3_o   (spi_flash_sd_3_out_x),
    .spi_flash_sd_3_oe_o(spi_flash_sd_3_oe_x),

    .spi_sck_i   (spi_sck_in_x),
    .spi_sck_o   (spi_sck_out_x),
    .spi_sck_oe_o(spi_sck_oe_x),

    .spi_cs_0_i   (spi_cs_0_in_x),
    .spi_cs_0_o   (spi_cs_0_out_x),
    .spi_cs_0_oe_o(spi_cs_0_oe_x),

    .spi_cs_1_i   (spi_cs_1_in_x),
    .spi_cs_1_o   (spi_cs_1_out_x),
    .spi_cs_1_oe_o(spi_cs_1_oe_x),

    .spi_sd_0_i   (spi_sd_0_in_x),
    .spi_sd_0_o   (spi_sd_0_out_x),
    .spi_sd_0_oe_o(spi_sd_0_oe_x),

    .spi_sd_1_i   (spi_sd_1_in_x),
    .spi_sd_1_o   (spi_sd_1_out_x),
    .spi_sd_1_oe_o(spi_sd_1_oe_x),

    .spi_sd_2_i   (spi_sd_2_in_x),
    .spi_sd_2_o   (spi_sd_2_out_x),
    .spi_sd_2_oe_o(spi_sd_2_oe_x),

    .spi_sd_3_i   (spi_sd_3_in_x),
    .spi_sd_3_o   (spi_sd_3_out_x),
    .spi_sd_3_oe_o(spi_sd_3_oe_x),

    .pdm2pcm_pdm_i   (pdm2pcm_pdm_in_x),
    .pdm2pcm_pdm_o   (pdm2pcm_pdm_out_x),
    .pdm2pcm_pdm_oe_o(pdm2pcm_pdm_oe_x),

    .pdm2pcm_clk_i   (pdm2pcm_clk_in_x),
    .pdm2pcm_clk_o   (pdm2pcm_clk_out_x),
    .pdm2pcm_clk_oe_o(pdm2pcm_clk_oe_x),

    .spi2_cs_0_i   (spi2_cs_0_in_x),
    .spi2_cs_0_o   (spi2_cs_0_out_x),
    .spi2_cs_0_oe_o(spi2_cs_0_oe_x),

    .spi2_cs_1_i   (spi2_cs_1_in_x),
    .spi2_cs_1_o   (spi2_cs_1_out_x),
    .spi2_cs_1_oe_o(spi2_cs_1_oe_x),

    .spi2_sck_i   (spi2_sck_in_x),
    .spi2_sck_o   (spi2_sck_out_x),
    .spi2_sck_oe_o(spi2_sck_oe_x),

    .spi2_sd_0_i   (spi2_sd_0_in_x),
    .spi2_sd_0_o   (spi2_sd_0_out_x),
    .spi2_sd_0_oe_o(spi2_sd_0_oe_x),

    .spi2_sd_1_i   (spi2_sd_1_in_x),
    .spi2_sd_1_o   (spi2_sd_1_out_x),
    .spi2_sd_1_oe_o(spi2_sd_1_oe_x),

    .spi2_sd_2_i   (spi2_sd_2_in_x),
    .spi2_sd_2_o   (spi2_sd_2_out_x),
    .spi2_sd_2_oe_o(spi2_sd_2_oe_x),

    .spi2_sd_3_i   (spi2_sd_3_in_x),
    .spi2_sd_3_o   (spi2_sd_3_out_x),
    .spi2_sd_3_oe_o(spi2_sd_3_oe_x),

    .i2c_scl_i   (i2c_scl_in_x),
    .i2c_scl_o   (i2c_scl_out_x),
    .i2c_scl_oe_o(i2c_scl_oe_x),

    .i2c_sda_i   (i2c_sda_in_x),
    .i2c_sda_o   (i2c_sda_out_x),
    .i2c_sda_oe_o(i2c_sda_oe_x),


    // `ifdef FPGA
    //     .spi_flash_cs_1_o    (),
    //     .spi_flash_cs_1_i    ('0),
    //     .spi_flash_cs_1_oe_o (),
    // `endif

    // CORE-V eXtension Interface
    .xif_compressed_if(ext_xif.cpu_compressed),
    .xif_issue_if     (ext_xif.cpu_issue),
    .xif_commit_if    (ext_xif.cpu_commit),
    .xif_mem_if       (ext_xif.cpu_mem),
    .xif_mem_result_if(ext_xif.cpu_mem_result),
    .xif_result_if    (ext_xif.cpu_result),

    // Pad controller interface
    .pad_req_o (pad_req),
    .pad_resp_i(pad_rsp),

    // External slave ports
    .ext_xbar_master_req_i (heep_slave_req),
    .ext_xbar_master_resp_o(heep_slave_rsp),

    // External master ports
    .ext_core_instr_req_o   (heep_core_instr_req),
    .ext_core_instr_resp_i  (heep_core_instr_rsp),
    .ext_core_data_req_o    (heep_core_data_req),
    .ext_core_data_resp_i   (heep_core_data_rsp),
    .ext_debug_master_req_o (heep_debug_master_req),
    .ext_debug_master_resp_i(heep_debug_master_rsp),
    .ext_dma_read_req_o     (heep_dma_read_req),
    .ext_dma_read_resp_i    (heep_dma_read_rsp),
    .ext_dma_write_req_o    (heep_dma_write_req),
    .ext_dma_write_resp_i   (heep_dma_write_rsp),
    .ext_dma_addr_req_o     (heep_dma_addr_req),
    .ext_dma_addr_resp_i    (heep_dma_addr_rsp),

    // External peripherals slave ports
    .ext_peripheral_slave_req_o (heep_peripheral_req),
    .ext_peripheral_slave_resp_i(heep_peripheral_rsp),

    // SPC signals
    .ext_ao_peripheral_slave_req_i (ext_ao_peripheral_req),
    .ext_ao_peripheral_slave_resp_o(ext_ao_peripheral_resp),

    // Power switches connected by the backend
    .cpu_subsystem_powergate_switch_no           (cpu_subsystem_powergate_switch_n),
    .cpu_subsystem_powergate_switch_ack_ni       (cpu_subsystem_powergate_switch_ack_n),
    .peripheral_subsystem_powergate_switch_no    (peripheral_subsystem_powergate_switch_n),
    .peripheral_subsystem_powergate_switch_ack_ni(peripheral_subsystem_powergate_switch_ack_n),

    .external_subsystem_powergate_switch_no    (external_subsystem_powergate_switch_n),
    .external_subsystem_powergate_switch_ack_ni(external_subsystem_powergate_switch_ack_n),
    .external_subsystem_powergate_iso_no       (external_subsystem_powergate_iso_n),

    // Control signals for external peripherals
    .external_subsystem_rst_no          (external_subsystem_rst_n),
    .external_ram_banks_set_retentive_no(external_ram_banks_set_retentive_n),
    .external_subsystem_clkgate_en_no   (external_subsystem_clkgate_en_n),

    // External interrupts
    .intr_vector_ext_i(ext_int_vector),

    .ext_dma_slot_tx_i('0),
    .ext_dma_slot_rx_i('0),

    .ext_debug_req_o         (ext_debug_req),
    .ext_debug_reset_no      (ext_debug_reset_n),
    .ext_cpu_subsystem_rst_no(),

    .ext_dma_stop_i('0),
    .dma_done_o    (),

    .exit_value_o(exit_value)
  );

  assign cpu_subsystem_powergate_switch_ack_n        = cpu_subsystem_powergate_switch_n;
  assign peripheral_subsystem_powergate_switch_ack_n = peripheral_subsystem_powergate_switch_n;


  // External peripherals
  // --------------------


  // External peripherals bus
  // ------------------------

  // Added for the bridge
  assign heep_slave_req[0].req                       = req_i;
  assign heep_slave_req[0].we                        = we_i;
  assign heep_slave_req[0].be                        = be_i;
  assign heep_slave_req[0].addr                      = addr_i;
  assign heep_slave_req[0].wdata                     = wdata_i;

  assign gnt_o                                       = heep_slave_rsp[0].gnt;
  assign rvalid_o                                    = heep_slave_rsp[0].rvalid;
  assign rdata_o                                     = heep_slave_rsp[0].rdata;


  // External interrupts
  // -------------------

  assign ext_int_vector                              = '0;

  // Pad ring
  // --------
  assign exit_value_out_x                            = exit_value[0];
  pad_ring u_pad_ring (
    .rst_nio              (rst_ni),
    .rst_no               (rst_nin_x),
    .boot_select_io       (boot_select_i),
    .boot_select_o        (boot_select_in_x),
    .execute_from_flash_io(execute_from_flash_i),
    .execute_from_flash_o (execute_from_flash_in_x),
    .jtag_tck_io          (jtag_tck_i),
    .jtag_tck_o           (jtag_tck_in_x),
    .jtag_tms_io          (jtag_tms_i),
    .jtag_tms_o           (jtag_tms_in_x),
    .jtag_trst_nio        (jtag_trst_ni),
    .jtag_trst_no         (jtag_trst_nin_x),
    .jtag_tdi_io          (jtag_tdi_i),
    .jtag_tdi_o           (jtag_tdi_in_x),
    .jtag_tdo_io          (jtag_tdo_o),
    .jtag_tdo_i           (jtag_tdo_out_x),
    .uart_rx_io           (uart_rx_i),
    .uart_rx_o            (uart_rx_in_x),
    .uart_tx_io           (uart_tx_o),
    .uart_tx_i            (uart_tx_out_x),
    .exit_valid_io        (exit_valid_o),
    .exit_valid_i         (exit_valid_out_x),
    .gpio_0_io            (gpio_0_io),
    .gpio_0_o             (gpio_0_in_x),
    .gpio_0_i             (gpio_0_out_x),
    .gpio_0_oe_i          (gpio_0_oe_x),
    .gpio_1_io            (gpio_1_io),
    .gpio_1_o             (gpio_1_in_x),
    .gpio_1_i             (gpio_1_out_x),
    .gpio_1_oe_i          (gpio_1_oe_x),
    .gpio_2_io            (gpio_2_o),
    .gpio_2_i             (gpio_2_out_x),

























    .clk_io       (clk_i),
    .clk_o        (clk_in_x),
    .exit_value_io(exit_value_o),
    .exit_value_i (exit_value_out_x),

    // Pad attributes
    .pad_attributes_i('0)
  );

  // Constant pad signals
  assign rst_nout_x               = 1'b0;
  assign rst_noe_x                = 1'b0;
  assign boot_select_out_x        = 1'b0;
  assign boot_select_oe_x         = 1'b0;
  assign execute_from_flash_out_x = 1'b0;
  assign execute_from_flash_oe_x  = 1'b0;
  assign jtag_tck_out_x           = 1'b0;
  assign jtag_tck_oe_x            = 1'b0;
  assign jtag_tms_out_x           = 1'b0;
  assign jtag_tms_oe_x            = 1'b0;
  assign jtag_trst_nout_x         = 1'b0;
  assign jtag_trst_noe_x          = 1'b0;
  assign jtag_tdi_out_x           = 1'b0;
  assign jtag_tdi_oe_x            = 1'b0;
  assign jtag_tdo_oe_x            = 1'b1;
  assign uart_rx_out_x            = 1'b0;
  assign uart_rx_oe_x             = 1'b0;
  assign uart_tx_oe_x             = 1'b1;
  assign exit_valid_oe_x          = 1'b1;
  assign gpio_2_oe_x              = 1'b1;
  assign clk_out_x                = 1'b0;
  assign clk_oe_x                 = 1'b0;
  assign exit_value_oe_x          = 1'b1;


  // Shared pads multiplexing
  always_comb begin
    spi_flash_cs_1_in_x = 1'b0;
    gpio_31_in_x        = 1'b0;
    unique case (pad_muxes[core_v_mini_mcu_pkg::PAD_SPI_FLASH_CS_1])
      0: begin
        spi_flash_cs_1_out_x_muxed = spi_flash_cs_1_out_x;
        spi_flash_cs_1_oe_x_muxed  = spi_flash_cs_1_oe_x;
        spi_flash_cs_1_in_x        = spi_flash_cs_1_in_x_muxed;
      end
      1: begin
        spi_flash_cs_1_out_x_muxed = gpio_31_out_x;
        spi_flash_cs_1_oe_x_muxed  = gpio_31_oe_x;
        gpio_31_in_x               = spi_flash_cs_1_in_x_muxed;
      end
      default: begin
        spi_flash_cs_1_out_x_muxed = spi_flash_cs_1_out_x;
        spi_flash_cs_1_oe_x_muxed  = spi_flash_cs_1_oe_x;
        spi_flash_cs_1_in_x        = spi_flash_cs_1_in_x_muxed;
      end
    endcase
  end


  // Pad control
  // -----------
  pad_control #(
    .reg_req_t(reg_req_t),
    .reg_rsp_t(reg_rsp_t),
    .NUM_PAD  (NUM_PAD)
  ) u_pad_control (
    .clk_i      (clk_i),
    .rst_ni     (rst_nin_sync),
    .reg_req_i  (pad_req),
    .reg_rsp_o  (pad_rsp),
    .pad_muxes_o(pad_muxes)
  );

endmodule  // gr_heep_top
