// Copyright 2024 Politecnico di Torino.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 2.0 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-2.0. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// File: gr_heep_peripherals.sv
// Author(s):
//   Luigi Giuffrida
// Date: 08/11/2024
// Description: Template for the GR-heep peripherals module

module gr_heep_peripherals (
    input logic clk_i,
% if (xbar_nmasters == 0 and xbar_nslaves == 0 and periph_nslaves == 0 and ext_interrupts == 0):
    input logic rst_ni
% else:
    input logic rst_ni,
% endif

    // Bridge interface
    input  logic        bridge_instr_valid_i,
    input  logic        bridge_addr_valid_i,

    output logic        bridge_rst_new_address_valid_o,
    output logic        bridge_rst_instr_valid_o,

    output logic        bridge_busy_o,

    input  logic [31:0] bridge_instruction_i,
    input  logic [31:0] bridge_new_section_address_i,

    output logic        bridge_OBI_rvalid_o,
    output logic [31:0] bridge_OBI_rdata_o,

% if (xbar_nmasters > 0):
    // External peripherals master ports
    output obi_pkg::obi_req_t  [gr_heep_pkg::ExtXbarNMasterRnd-1:0] gr_heep_master_req_o,
    input obi_pkg::obi_resp_t [gr_heep_pkg::ExtXbarNMasterRnd-1:0] gr_heep_master_resp_i${'' if (xbar_nslaves + periph_nslaves + ext_interrupts == 0) else ','}
% endif

% if (xbar_nslaves > 0):
    // External peripherals slave ports
    input obi_pkg::obi_req_t  [gr_heep_pkg::ExtXbarNSlaveRnd-1:0] gr_heep_slave_req_i,
    output obi_pkg::obi_resp_t [gr_heep_pkg::ExtXbarNSlaveRnd-1:0] gr_heep_slave_resp_o${'' if (periph_nslaves + ext_interrupts == 0) else ','}
% endif

% if (periph_nslaves > 0):
    // External peripherals configuration ports
    input reg_pkg::reg_req_t [gr_heep_pkg::ExtPeriphNSlaveRnd-1:0] gr_heep_peripheral_req_i,
    output reg_pkg::reg_rsp_t [gr_heep_pkg::ExtPeriphNSlaveRnd-1:0] gr_heep_peripheral_rsp_o${'' if (ext_interrupts == 0) else ','}
% endif

% if (ext_interrupts > 0):
    // External peripherals interrupt ports
    output logic [gr_heep_pkg::ExtInterrupts-1:0] gr_heep_peripheral_int_o
%endif
);

  // Instantiate here the external peripherals

  bridge2xheep u_bridge2xheep (
    .clk(clk_i),
    .rst_n(rst_ni),

    // HEEP Side - The bridge is connected to the X-HEEP master port 0
    .req(gr_heep_master_req_o[0].req),
    .we(gr_heep_master_req_o[0].we),
    .be(gr_heep_master_req_o[0].be),
    .addr(gr_heep_master_req_o[0].addr),
    .wdata(gr_heep_master_req_o[0].wdata),
    .gnt(gr_heep_master_resp_i[0].gnt),
    .rvalid(gr_heep_master_resp_i[0].rvalid),
    .rdata(gr_heep_master_resp_i[0].rdata),

    // External world side (FPGA side)
    .instr_valid(bridge_instr_valid_i),
    .addr_valid(bridge_addr_valid_i),
    .rst_new_address_valid(bridge_rst_new_address_valid_o),
    .rst_instr_valid(bridge_rst_instr_valid_o),
    .busy(bridge_busy_o),
    .instruction(bridge_instruction_i),
    .new_section_address(bridge_new_section_address_i),
    .OBI_rvalid(bridge_OBI_rvalid_o),
    .OBI_rdata(bridge_OBI_rdata_o)
  );

endmodule
