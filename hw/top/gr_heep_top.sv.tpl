// Copyright 2024 Politecnico di Torino.
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// File: gr_heep_top.sv
// Author: Luigi Giuffrida
// Date: 16/10/2024
// Description: GR-heep top-level module

module gr_heep_top (
  // Added to connect the bridge2Xheep

  // Request section for the OBI interface
  input logic        req_i,
  input logic        we_i,
  input logic [ 3:0] be_i,
  input logic [31:0] addr_i,
  input logic [31:0] wdata_i,

  // Response section for the OBI interface
  output logic        gnt_o,
  output logic        rvalid_o,
  output logic [31:0] rdata_o,

    // X-HEEP interface
% for pad in total_pad_list:
${pad.x_heep_system_interface}
% endfor
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
  obi_req_t  heep_core_instr_req;
  obi_resp_t heep_core_instr_rsp;
  obi_req_t  heep_core_data_req;
  obi_resp_t heep_core_data_rsp;
  obi_req_t  heep_debug_master_req;
  obi_resp_t heep_debug_master_rsp;
  obi_req_t  [DMA_NUM_MASTER_PORTS-1:0] heep_dma_read_req;
  obi_resp_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_read_rsp;
  obi_req_t  [DMA_NUM_MASTER_PORTS-1:0] heep_dma_write_req;
  obi_resp_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_write_rsp;
  obi_req_t  [DMA_NUM_MASTER_PORTS-1:0] heep_dma_addr_req;
  obi_resp_t [DMA_NUM_MASTER_PORTS-1:0] heep_dma_addr_rsp;

  /* verilator lint_off UNUSED */
  /* verilator lint_off UNDRIVEN */
  // TODO: Remove the verilator lint_off pragmas when the signals are used 

  // X-HEEP slave ports
  obi_req_t  [ExtXbarNmasterRnd-1:0] heep_slave_req;
  obi_resp_t [ExtXbarNmasterRnd-1:0] heep_slave_rsp;

  // External master ports
  obi_req_t  [ExtXbarNmasterRnd-1:0] gr_heep_master_req;
  obi_resp_t [ExtXbarNmasterRnd-1:0] gr_heep_master_resp;

  // External slave ports
  obi_req_t  [ExtXbarNSlaveRnd-1:0] ext_slave_req;
  obi_resp_t [ExtXbarNSlaveRnd-1:0] ext_slave_resp; 

  // X-HEEP external peripheral master ports
  reg_req_t heep_peripheral_req;
  reg_rsp_t heep_peripheral_rsp;

  // Interrupt vector
  logic [core_v_mini_mcu_pkg::NEXT_INT-1:0] ext_int_vector;
  logic ext_intr_peripheral;

  // Power Manager signals
  logic cpu_subsystem_powergate_switch_n;
  logic cpu_subsystem_powergate_switch_ack_n;
  logic peripheral_subsystem_powergate_switch_n;
  logic peripheral_subsystem_powergate_switch_ack_n;

  // External SPC interface signals
  reg_req_t [AoSPCNum-1:0] ext_ao_peripheral_req;
  reg_rsp_t  [AoSPCNum-1:0] ext_ao_peripheral_resp;
  
  // External peripherals select
  logic     [LogExtPeriphNSlave-1:0] gr_heep_periph_select;
  reg_req_t [ExtPeriphNSlaveRnd-1:0] gr_heep_peripheral_req;
  reg_rsp_t [ExtPeriphNSlaveRnd-1:0] gr_heep_peripheral_rsp;

  // Pad controller
  reg_req_t pad_req;
  reg_rsp_t pad_rsp;
% if pads_attributes != None:
  logic [core_v_mini_mcu_pkg::NUM_PAD-1:0][${pads_attributes['bits']}] pad_attributes;
% endif
% if total_pad_muxed > 0:
  logic [core_v_mini_mcu_pkg::NUM_PAD-1:0][${max_total_pad_mux_bitlengh-1}:0] pad_muxes;
% endif

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
    ext_xif.compressed_ready   = 1'b1;
    ext_xif.compressed_resp    = '0;

    ext_xif.issue_ready        = 1'b1;
    ext_xif.issue_resp         = '0;

    ext_xif.mem_valid          = 1'b0;
    ext_xif.mem_req            = '0;

    ext_xif.result_valid       = 1'b0;
    ext_xif.result             = '0;
  end

  // CORE-V-MINI-MCU input/output pins
% for pad in total_pad_list:
${pad.internal_signals}
% endfor

  // Drive to zero bypassed pins
% for pad in total_pad_list:
% if pad.pad_type == 'bypass_inout' or pad.pad_type == 'bypass_input':
% for i in range(len(pad.pad_type_drive)):
% if pad.driven_manually[i] == False:
  assign ${pad.in_internal_signals[i]} = 1'b0;
% endif
% endfor
% endif
% endfor

  // --------------
  // SYSTEM MODULES
  // --------------

  // Reset generator
  // ---------------
  rstgen u_rstgen (
    .clk_i      (clk_in_x),
    .rst_ni     (rst_nin_x),
    .test_mode_i(1'b0 ), // not implemented
    .rst_no     (rst_nin_sync),
    .init_no    () // unused
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
    .rst_ni (rst_nin_sync),
    .clk_i  (clk_i),

    // MCU pads
% for pad in pad_list:
${pad.core_v_mini_mcu_bonding}
% endfor

// `ifdef FPGA
//     .spi_flash_cs_1_o    (),
//     .spi_flash_cs_1_i    ('0),
//     .spi_flash_cs_1_oe_o (),
// `endif

    // CORE-V eXtension Interface
    .xif_compressed_if (ext_xif.cpu_compressed),
    .xif_issue_if      (ext_xif.cpu_issue),
    .xif_commit_if     (ext_xif.cpu_commit),
    .xif_mem_if        (ext_xif.cpu_mem),
    .xif_mem_result_if (ext_xif.cpu_mem_result),
    .xif_result_if     (ext_xif.cpu_result),

    // Pad controller interface
    .pad_req_o  (pad_req),
    .pad_resp_i (pad_rsp),

    // External slave ports
    .ext_xbar_master_req_i (heep_slave_req),
    .ext_xbar_master_resp_o (heep_slave_rsp),

    // External master ports
    .ext_core_instr_req_o (heep_core_instr_req),
    .ext_core_instr_resp_i (heep_core_instr_rsp),
    .ext_core_data_req_o (heep_core_data_req),
    .ext_core_data_resp_i (heep_core_data_rsp),
    .ext_debug_master_req_o (heep_debug_master_req),
    .ext_debug_master_resp_i (heep_debug_master_rsp),
    .ext_dma_read_req_o (heep_dma_read_req),
    .ext_dma_read_resp_i (heep_dma_read_rsp),
    .ext_dma_write_req_o (heep_dma_write_req),
    .ext_dma_write_resp_i (heep_dma_write_rsp),
    .ext_dma_addr_req_o (heep_dma_addr_req),
    .ext_dma_addr_resp_i (heep_dma_addr_rsp),

    // External peripherals slave ports
    .ext_peripheral_slave_req_o  (heep_peripheral_req),
    .ext_peripheral_slave_resp_i (heep_peripheral_rsp),

    // SPC signals
    .ext_ao_peripheral_slave_req_i(ext_ao_peripheral_req),
    .ext_ao_peripheral_slave_resp_o(ext_ao_peripheral_resp),

    // Power switches connected by the backend
    .cpu_subsystem_powergate_switch_no            (cpu_subsystem_powergate_switch_n),
    .cpu_subsystem_powergate_switch_ack_ni        (cpu_subsystem_powergate_switch_ack_n),
    .peripheral_subsystem_powergate_switch_no     (peripheral_subsystem_powergate_switch_n),
    .peripheral_subsystem_powergate_switch_ack_ni (peripheral_subsystem_powergate_switch_ack_n),

    .external_subsystem_powergate_switch_no(external_subsystem_powergate_switch_n),
    .external_subsystem_powergate_switch_ack_ni(external_subsystem_powergate_switch_ack_n),
    .external_subsystem_powergate_iso_no(external_subsystem_powergate_iso_n),

    // Control signals for external peripherals
    .external_subsystem_rst_no (external_subsystem_rst_n),
    .external_ram_banks_set_retentive_no (external_ram_banks_set_retentive_n),
    .external_subsystem_clkgate_en_no (external_subsystem_clkgate_en_n),

    // External interrupts
    .intr_vector_ext_i (ext_int_vector),
    .intr_ext_peripheral_i(ext_intr_peripheral),

    .hw_fifo_done_i('0),
    .hw_fifo_req_o(),
    .hw_fifo_resp_i('0),

    .ext_dma_slot_tx_i('0),
    .ext_dma_slot_rx_i('0),

    .ext_debug_req_o(ext_debug_req),
    .ext_debug_reset_no(ext_debug_reset_n),
    .ext_cpu_subsystem_rst_no(),

    .ext_dma_stop_i('0),
    .dma_done_o(),
    
    .exit_value_o (exit_value)
  );

  assign cpu_subsystem_powergate_switch_ack_n = cpu_subsystem_powergate_switch_n;
  assign peripheral_subsystem_powergate_switch_ack_n = peripheral_subsystem_powergate_switch_n;

  // External peripherals
  // --------------------
  gr_heep_peripherals u_gr_heep_peripherals (
    .clk_i (clk_i),
    .rst_ni (rst_nin_sync),
    .gr_heep_master_req_o(),
    .gr_heep_master_resp_i(),
    .gr_heep_slave_req_i(ext_slave_req),
    .gr_heep_slave_resp_o(ext_slave_resp),
    .gr_heep_peripheral_req_i(gr_heep_peripheral_req),
    .gr_heep_peripheral_rsp_o(gr_heep_peripheral_rsp),
    .gr_heep_peripheral_int_o(ext_int_vector[0])
  );

  // Unused signals
  // --------------
  assign gr_heep_master_req = '0;
  assign ext_int_vector[11:1] = '0;

  localparam int unsigned IdxWidth = cf_math_pkg::idx_width(ExtXbarNSlave);

  // External BUS
  // ------------
  ext_bus #(
    .EXT_XBAR_NMASTER (ExtXbarNMasterRnd),
    .EXT_XBAR_NSLAVE  (ExtXbarNSlaveRnd)
  ) u_ext_bus (
    .clk_i (clk_i),
    .rst_ni (rst_nin_sync),

    // Address map
    .addr_map_i(ExtSlaveAddrRules),
    .default_idx_i(ExtSlaveDefaultIdx[IdxWidth-1:0]),

    // X-HEEP master ports
    .heep_core_instr_req_i(heep_core_instr_req),
    .heep_core_instr_resp_o(heep_core_instr_rsp),

    .heep_core_data_req_i(heep_core_data_req),
    .heep_core_data_resp_o(heep_core_data_rsp),

    .heep_debug_master_req_i(heep_debug_master_req),
    .heep_debug_master_resp_o(heep_debug_master_rsp),

    .heep_dma_read_req_i(heep_dma_read_req),
    .heep_dma_read_resp_o(heep_dma_read_rsp),

    .heep_dma_write_req_i(heep_dma_write_req),
    .heep_dma_write_resp_o(heep_dma_write_rsp),

    .heep_dma_addr_req_i(heep_dma_addr_req),
    .heep_dma_addr_resp_o(heep_dma_addr_rsp),

    // X-HEEP master ports
    .ext_master_req_i(gr_heep_master_req),
    .ext_master_resp_o(),

    // X-HEEP slave ports
    .heep_slave_req_o(heep_slave_req),
    .heep_slave_resp_i(heep_slave_rsp),

    // External slave ports
    .ext_slave_req_o(ext_slave_req),
    .ext_slave_resp_i(ext_slave_resp)

  );

  // External peripherals bus
  // ------------------------
  addr_decode #(
      .NoIndices(gr_heep_pkg::ExtPeriphNSlaveRnd),
      .NoRules(gr_heep_pkg::ExtPeriphNSlaveRnd),
      .addr_t(logic [31:0]),
      .rule_t(addr_map_rule_pkg::addr_map_rule_t)
  ) u_gr_heep_perpheral_decoder (
      .addr_i(heep_peripheral_req.addr),
      .addr_map_i(gr_heep_pkg::ExtPeriphAddrRules),
      .idx_o(gr_heep_periph_select),
      .dec_valid_o(),
      .dec_error_o(),
      .en_default_idx_i(1'b0),
      .default_idx_i('0)
  );

  reg_demux #(
      .NoPorts(gr_heep_pkg::ExtPeriphNSlaveRnd),
      .req_t  (reg_pkg::reg_req_t),
      .rsp_t  (reg_pkg::reg_rsp_t)
  ) u_gr_heep_reg_demux (
      .clk_i,
      .rst_ni,
      .in_select_i(gr_heep_periph_select),
      .in_req_i(heep_peripheral_req),
      .in_rsp_o(heep_peripheral_rsp),
      .out_req_o(gr_heep_peripheral_req),
      .out_rsp_i(gr_heep_peripheral_rsp)
  );
  
  // Added for the bridge
  assign heep_slave_req[0].req    = req_i;
  assign heep_slave_req[0].we     = we_i;
  assign heep_slave_req[0].be     = be_i;
  assign heep_slave_req[0].addr   = addr_i;
  assign heep_slave_req[0].wdata  = wdata_i;

  assign gnt_o    = heep_slave_rsp[0].gnt;
  assign rvalid_o = heep_slave_rsp[0].rvalid;
  assign rdata_o  = heep_slave_rsp[0].rdata;


  // External interrupts
  // -------------------
  assign ext_intr_peripheral = '0;
  // assign ext_int_vector = '0;
   
  // Pad ring
  // --------
  assign exit_value_out_x = exit_value[0];
  pad_ring u_pad_ring (
% for pad in total_pad_list:
${pad.pad_ring_bonding_bonding}
% endfor

    // Pad attributes
% if pads_attributes != None:
    .pad_attributes_i(pad_attributes)
% else:
    .pad_attributes_i('0)
% endif
  );

  // Constant pad signals
${pad_constant_driver_assign}

  // Shared pads multiplexing
${pad_mux_process}

  // Pad control
  // -----------
  pad_control #(
    .reg_req_t (reg_req_t),
    .reg_rsp_t (reg_rsp_t),
    .NUM_PAD   (NUM_PAD)
  ) u_pad_control (
    .clk_i            (clk_i),
    .rst_ni           (rst_nin_sync),
    .reg_req_i        (pad_req),
    .reg_rsp_o        (pad_rsp)
% if total_pad_muxed > 0 or pads_attributes != None:
      ,
% endif
% if pads_attributes != None:
      .pad_attributes_o(pad_attributes)
% if total_pad_muxed > 0:
      ,
% endif
% endif
% if total_pad_muxed > 0:
      .pad_muxes_o(pad_muxes)
% endif
  );

endmodule // gr_heep_top
