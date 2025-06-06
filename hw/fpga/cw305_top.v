/* 
ChipWhisperer Artix Target - Example of connections between example registers
and rest of system.

Copyright (c) 2016-2020, NewAE Technology Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted without restriction. Note that modules within
the project may have additional restrictions, please carefully inspect
additional licenses.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of NewAE Technology Inc.
*/

/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
  * Description: Top level module for the CW305 board.
 */

//`timescale 1ns / 1ps
`default_nettype none 

module cw305_top #(
    parameter pBYTECNT_SIZE = 2, // this parameter defines the number of bits used to address the bytes in the register. 2 bits => 2^2 => 4 bytes, so each register can store 4 bytes
    parameter pADDR_WIDTH = 21,  // the first (pADDR_WIDTH - pBYTECNT_SIZE) MSB bits are used to address the register, the remaining LSB bits are used to address the bytes in the register

   // Added for the bridge
   parameter pINSTR_WIDTH = 32
)(
    // USB Interface
    input wire                          usb_clk,        // Clock
    inout wire [7:0]                    usb_data,       // Data for write/read

    input wire [pADDR_WIDTH-1:0]        usb_addr,       // Address
    input wire                          usb_rdn,        // !RD, low when addr valid for read
    input wire                          usb_wrn,        // !WR, low when data+addr valid for write
    input wire                          usb_cen,        // !CE, active low chip enable
    input wire                          usb_trigger,    // High when trigger requested

    // Buttons/LEDs on Board
    input wire                          j16_sel,        // DIP switch J16
    input wire                          k16_sel,        // DIP switch K16
    input wire                          k15_sel,        // DIP switch K15
    input wire                          l14_sel,        // DIP Switch L14
    input wire                          pushbutton,     // Pushbutton SW4, connected to R1, used here as reset
    output wire                         led1,           // red LED
    output wire                         led2,           // green LED
    output wire                         led3,           // blue LED

    // PLL
    input wire                          pll_clk1,       //PLL Clock Channel #1
    //input wire                        pll_clk2,       //PLL Clock Channel #2 (unused in this example)

    // 20-Pin Connector Stuff
    output wire                         tio_trigger,
    // output wire                         tio_clkout,
    // input  wire                         tio_clkin,
    inout  wire                         IO_0,
    inout  wire                         IO_1,

    // DEBUG SIGNALS
   `ifdef VERILATOR

      // Exit signals. Needed for the testbench as output port
      output wire        exit_valid_o,
      output wire [31:0] exit_value_o,

      // UART
      inout wire         gr_heep_uart_rx,
      output wire        gr_heep_uart_tx,

      // SPI Flash
      output wire        spi_flash_sck,
      output wire [1:0]  spi_flash_csb,
      inout  wire [3:0]  spi_flash_sd_io,

      // SPI
      output wire        spi_sck,
      output wire [1:0]  spi_csb,
      inout  wire [3:0]  spi_sd_io,

      // GPIO
      inout  wire [31:0] gpio,
   `endif

    // Debug UART connected to the 20-PIN connector
    inout wire         debug_heep_uart_rx,
    output wire        debug_heep_uart_tx
  );


   wire        internal_exit_valid_o;
   wire [31:0] internal_exit_value_o;

   // UART
   wire        internal_gr_heep_uart_rx;
   wire        internal_gr_heep_uart_tx;

   // SPI Flash
   wire        internal_spi_flash_sck;
   wire [1:0]  internal_spi_flash_csb;
   wire [3:0]  internal_spi_flash_sd_io;

   // SPI
   wire        internal_spi_sck;
   wire [1:0]  internal_spi_csb;
   wire [3:0]  internal_spi_sd_io;

   // GPIO
   wire [31:0] internal_gpio;

   `ifdef VERILATOR
      assign exit_valid_o = internal_exit_valid_o;
      assign exit_value_o = internal_exit_value_o;
      assign gr_heep_uart_rx = internal_gr_heep_uart_rx;
      assign gr_heep_uart_tx = internal_gr_heep_uart_tx;
      assign spi_flash_sck = internal_spi_flash_sck;
      assign spi_flash_csb = internal_spi_flash_csb;
      assign spi_flash_sd_io = internal_spi_flash_sd_io;
      assign spi_sck = internal_spi_sck;
      assign spi_csb = internal_spi_csb;
      assign spi_sd_io = internal_spi_sd_io;
      assign gpio = internal_gpio;
   `endif

    // Added for the bridge
    wire [pINSTR_WIDTH-1:0] bridge_instruction;
    wire                    bridge_rst_instr_valid;
    wire                    bridge_rst_instr_valid_usb;
    wire [pINSTR_WIDTH-1:0] bridge_new_address;
    wire                    bridge_rst_new_address_valid;
    wire                    bridge_rst_new_address_valid_usb;

    wire [pINSTR_WIDTH-1:0] bridge_data;
    wire                    bridge_data_valid;
    wire [7:0] bridge_status_usb;
    wire [7:0] bridge_status_heep;
    // --------------------------------

    wire isout;
    reg [pADDR_WIDTH-pBYTECNT_SIZE-1:0] reg_address;
    wire [pADDR_WIDTH-pBYTECNT_SIZE-1:0] reg_address_fe;
    wire [pBYTECNT_SIZE-1:0] reg_bytecnt;
    wire reg_addrvalid;
    reg [7:0] write_data;
    wire [7:0] write_data_fe;
    wire [7:0] read_data;
    wire reg_read;
    reg reg_write;
    wire reg_write_fe;
    wire [4:0] clk_settings;
    wire heep_clk;
    wire usb_clk_buf;
    wire [7:0] usb_dout;

    wire resetn = pushbutton;
    wire reset = !resetn;


    // USB CLK Heartbeat
    reg [24:0] usb_timer_heartbeat;
    always @(posedge usb_clk_buf) usb_timer_heartbeat <= usb_timer_heartbeat +  25'd1;
    assign led1 = usb_timer_heartbeat[24];

    // HEEP CLK Heartbeat
    reg [22:0] heep_clk_heartbeat;
    always @(posedge heep_clk) heep_clk_heartbeat <= heep_clk_heartbeat +  23'd1;
    assign led2 = heep_clk_heartbeat[22];

    // Tri-state buffer for USB data
    assign usb_data = isout? usb_dout : 8'bZ;

    cw305_usb_reg_fe #(
       .pBYTECNT_SIZE           (pBYTECNT_SIZE),
       .pADDR_WIDTH             (pADDR_WIDTH)
    ) U_usb_reg_fe (
       .rst                     (reset),
       .usb_clk                 (usb_clk_buf), 
       .usb_din                 (usb_data), 
       .usb_dout                (usb_dout), 
       .usb_rdn                 (usb_rdn), 
       .usb_wrn                 (usb_wrn),
       .usb_cen                 (usb_cen),
       .usb_alen                (1'b0),                 // unused
       .usb_addr                (usb_addr),
       .usb_isout               (isout), 
       .reg_address             (reg_address_fe), 
       .reg_bytecnt             (reg_bytecnt), 
       .reg_datao               (write_data_fe), 
       .reg_datai               (read_data),
       .reg_read                (reg_read), 
       .reg_write               (reg_write_fe), 
       .reg_addrvalid           (reg_addrvalid)
    );


    cw305_reg_aes #(
       .pBYTECNT_SIZE           (pBYTECNT_SIZE),
       .pADDR_WIDTH             (pADDR_WIDTH)
    ) U_reg_aes (
       .reset_i                 (reset),
       .crypto_clk              (heep_clk),
       .usb_clk                 (usb_clk_buf), 
       .reg_address             (reg_address[pADDR_WIDTH-pBYTECNT_SIZE-1:0]), 
       .reg_bytecnt             (reg_bytecnt), 
       .read_data               (read_data), 
       .write_data              (write_data),
       .reg_read                (reg_read), 
       .reg_write               (reg_write), 
       .reg_addrvalid           (reg_addrvalid),

       .exttrigger_in           (usb_trigger),

      //.I_heep_data              (bridge_data),
      .I_reset_new_addr_valid   (bridge_rst_new_address_valid),
      .I_reset_instr_valid      (bridge_rst_instr_valid),

       .O_clksettings           (clk_settings),
       .O_user_led              (/*led3*/), // Unconnected since the LED is driven by the HEEP core

       // Added for the bridge
       .O_instruction           (bridge_instruction),
       .O_address               (bridge_new_address),
       .O_status                (bridge_status_usb)
       
    );


    clocks U_clocks (
       .usb_clk                 (usb_clk),
       .usb_clk_buf             (usb_clk_buf),
       .I_j16_sel               (j16_sel),
       .I_k16_sel               (k16_sel),
       .I_clock_reg             (clk_settings),
       // .I_cw_clkin              (tio_clkin),
       .I_cw_clkin              (1'b0), // No external clock connected to the 20-pin connector
       .I_pll_clk1              (pll_clk1),
       .O_cw_clkout             (),
       .O_cryptoclk             (heep_clk)
    );

  // Static configuration. The board has a 4-item DIP switch, but the values j16_sel and k16_sel
  // are used to select the clock sources, so only the signals k15_sel and l14_sel are available
  // for the user.
  // wire boot_select_i = k15_sel;
  // wire execute_from_flash_i = l14_sel;

  // Alternative configuration. The boot mode is fixed at the synthesis time.
  // 
  wire boot_select_i = 1'b0;
  wire execute_from_flash_i = 1'b0;


  // INTERNAL SIGNALS
  // ----------------
  // JTAG
  wire jtag_tck     = 1'b0;
  wire jtag_tms     = 1'b0;
  wire jtag_trst_n  = 1'b0;
  wire jtag_tdi     = 1'b0;
  wire jtag_tdo     = 1'b0;

  // Bridge signals
  wire        req_i;
  wire        we_i;
  wire [ 3:0] be_i;
  wire [31:0] addr_i;
  wire [31:0] wdata_i;

  wire        gnt_o;
  wire        rvalid_o;
  wire [31:0] rdata_o;
  wire        OBI_rvalid_o;
  wire [31:0] OBI_rdata_o;


  // DUT
  // ---
  gr_heep_top u_gr_heep_top (
    // Bridge interface
    .bridge_instr_valid_i           (bridge_status_heep[1]),
    .bridge_addr_valid_i            (bridge_status_heep[2]),
    .bridge_rst_new_address_valid_o (bridge_rst_new_address_valid),
    .bridge_rst_instr_valid_o       (bridge_rst_instr_valid),
    .bridge_busy_o                  (/*bridge_status_heep[0]*/),
    .bridge_instruction_i           (bridge_instruction),
    .bridge_new_section_address_i   (bridge_new_address),
    .bridge_OBI_rvalid_o            (bridge_data_valid),
    .bridge_OBI_rdata_o             (bridge_data),

    // X-HEEP interface
    .rst_ni               (resetn),
    .boot_select_i        (boot_select_i),
    .execute_from_flash_i (execute_from_flash_i),
    .jtag_tck_i           (jtag_tck),
    .jtag_tms_i           (jtag_tms),
    .jtag_trst_ni         (jtag_trst_n),
    .jtag_tdi_i           (jtag_tdi),
    .jtag_tdo_o           (jtag_tdo),
    .uart_rx_i            (internal_gr_heep_uart_rx),
    .uart_tx_o            (internal_gr_heep_uart_tx),
    .exit_valid_o         (internal_exit_valid_o),

    //`ifdef VERILATOR
    .gpio_0_io            (internal_gpio[0]),
    .gpio_1_io            (internal_gpio[1]),
    //`endif

    .gpio_2_o             (internal_gpio[2]),
    .gpio_3_i             (internal_gpio[3]),
    .gpio_4_o             (internal_gpio[4]),

    //`ifdef VERILATOR
    // .gpio_5_io           (internal_gpio[5]),
    // .gpio_6_io           (internal_gpio[6]),
    // .gpio_7_io           (internal_gpio[7]),
    // .gpio_8_io           (internal_gpio[8]),
    // .gpio_9_io           (internal_gpio[9]),
    // .gpio_10_io          (internal_gpio[10]),
    // .gpio_11_io          (internal_gpio[11]),
    // .gpio_12_io          (internal_gpio[12]),
    // .gpio_13_io          (internal_gpio[13]),
    // .gpio_14_io          (internal_gpio[14]),
    // .gpio_15_io          (internal_gpio[15]),
    // .gpio_16_io          (internal_gpio[16]),
    // .gpio_17_io          (internal_gpio[17]),
    // .gpio_18_io          (internal_gpio[18]),
    // .gpio_19_io          (internal_gpio[19]),
    // .gpio_20_io          (internal_gpio[20]),
    // .gpio_21_io          (internal_gpio[21]),
    // .gpio_22_io          (internal_gpio[22]),
    // .gpio_23_io          (internal_gpio[23]),
    // .gpio_24_io          (internal_gpio[24]),
    // .gpio_25_io          (internal_gpio[25]),
    // .gpio_26_io          (internal_gpio[26]),
    // .gpio_27_io          (internal_gpio[27]),
    // .gpio_28_io          (internal_gpio[28]),
    // .gpio_29_io          (internal_gpio[29]),
    // .gpio_30_io          (internal_gpio[30]),
    // .spi_flash_sck_io    (internal_spi_flash_sck),
    // .spi_flash_cs_0_io   (internal_spi_flash_csb[0]),
    // .spi_flash_cs_1_io   (internal_spi_flash_csb[1]),
    // .spi_flash_sd_0_io   (internal_spi_flash_sd_io[0]),
    // .spi_flash_sd_1_io   (internal_spi_flash_sd_io[1]),
    // .spi_flash_sd_2_io   (internal_spi_flash_sd_io[2]),
    // .spi_flash_sd_3_io   (internal_spi_flash_sd_io[3]),
    // .spi_sck_io          (internal_spi_sck),
    // .spi_cs_0_io         (internal_spi_csb[0]),
    // .spi_cs_1_io         (internal_spi_csb[1]),
    // .spi_sd_0_io         (internal_spi_sd_io[0]),
    // .spi_sd_1_io         (internal_spi_sd_io[1]),
    // .spi_sd_2_io         (internal_spi_sd_io[2]),
    // .spi_sd_3_io         (internal_spi_sd_io[3]),
    // .i2s_sck_io          (),
    // .i2s_ws_io           (),
    // .i2s_sd_io           (),
   //`endif

    .clk_i                (heep_clk),
    .exit_value_o         (internal_exit_value_o[0])
  );

  // Scope trigger
  assign tio_trigger = internal_gpio[4];

  // Software trigger for X-HEEP. It is used to trigger the program execution
  assign internal_gpio[3] = bridge_status_heep[3];

  // Debug LED
  assign led3 = internal_gpio[2];

  assign IO_1 = internal_gpio[1];
  assign IO_0 = internal_gpio[0];

  // Debug UART
  assign debug_heep_uart_rx = internal_gr_heep_uart_rx;
  assign debug_heep_uart_tx = internal_gr_heep_uart_tx;

  wire rst_new_addr_valid_to_regs;
  wire rst_instr_valid_to_regs;

  // Synchronizers for the handshake protocol between the reg_aes module (usb_clk) and the bridge (heep_clk)
  dfs u_dfs_1_instr_valid (
    .clk_src(usb_clk_buf),
    .clk_dst(heep_clk),
    .rst_src(resetn),
    .rst_dst(resetn),
    .din_src(bridge_status_usb[1]),
    .dout_dst(bridge_status_heep[1])
  );

  dfs u_dfs_1_new_addr_valid (
    .clk_src(usb_clk_buf),
    .clk_dst(heep_clk),
    .rst_src(resetn),
    .rst_dst(resetn),
    .din_src(bridge_status_usb[2]),
    .dout_dst(bridge_status_heep[2])
  );

  // Synchonizer for the software trigger signal
  dfs u_dfs_1_xheep_trigger (
    .clk_src(usb_clk_buf),
    .clk_dst(heep_clk),
    .rst_src(resetn),
    .rst_dst(resetn),
    .din_src(bridge_status_usb[3]),
    .dout_dst(bridge_status_heep[3])
  );

  // Synchonizers for the handshake protocol between the bridge (heep_clk) and the reg_aes module (usb_clk)
  dfs u_dfs_2_rst_instr_valid (
    .clk_src(heep_clk),
    .clk_dst(usb_clk_buf),
    .rst_src(resetn),
    .rst_dst(resetn),
    .din_src(bridge_rst_instr_valid),
    .dout_dst(bridge_rst_instr_valid_usb)
  );

  dfs u_dfs_rst_new_addr_valid (
    .clk_src(heep_clk),
    .clk_dst(usb_clk_buf),
    .rst_src(resetn),
    .rst_dst(resetn),
    .din_src(bridge_rst_new_address_valid),
    .dout_dst(bridge_rst_new_address_valid_usb)
  );

  // CU for the handshake protocol between the bridge (heep_clock) and the reg_aes module (usb_clk)
  handshake_CU u_handshake_CU (
    .usb_clk(usb_clk_buf),
    .rst_n(resetn),
    .rst_new_addr_valid_from_bridge_i(bridge_rst_new_address_valid_usb),
    .rst_instr_valid_from_bridge_i(bridge_rst_instr_valid_usb),
    .rst_new_addr_valid_to_regs_o(rst_new_addr_valid_to_regs),
    .rst_instr_valid_to_regs_o(rst_instr_valid_to_regs)
  );

  // The bridge drives a MUX that selects which data has to be written to the reg_aes module.
  // This is necessary to reset the status flags using the same interface used also by the 
  // usb_reg_fe module.
  always @(*) begin
    if (~rst_new_addr_valid_to_regs) begin
      reg_address = `REG_BRIDGE_STATUS;
      write_data  = bridge_status_usb & ~(8'b00000100);
      reg_write   = 1'b1;
    end

    else if (~rst_instr_valid_to_regs) begin
      reg_address = `REG_BRIDGE_STATUS;
      write_data  = bridge_status_usb & ~(8'b00000010);
      reg_write   = 1'b1;
    end

    else begin
      reg_address = reg_address_fe;
      write_data  = write_data_fe;
      reg_write   = reg_write_fe;
    end
  end

  // Exit value
  assign internal_exit_value_o[31:1] = u_gr_heep_top.u_core_v_mini_mcu.exit_value_o[31:1];

endmodule

`default_nettype wire

