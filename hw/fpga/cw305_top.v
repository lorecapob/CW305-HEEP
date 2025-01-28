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

//`timescale 1ns / 1ps
`default_nettype none 

module cw305_top #(
    parameter pBYTECNT_SIZE = 2, // this parameter defines the number of bits used to address the bytes in the register. 2 bits => 2^2 => 4 bytes, so each register can store 4 bytes
    parameter pADDR_WIDTH = 21,  // the first (pADDR_WIDTH - pBYTECNT_SIZE) MSB bits are used to address the register, the remaining LSB bits are used to address the bytes in the register
   //  parameter pPT_WIDTH = 128,
   //  parameter pCT_WIDTH = 128,
   //  parameter pKEY_WIDTH = 128

   // Added for the bridge
   parameter pINSTR_WIDTH = 32,
   parameter int unsigned CLK_FREQ = 32'd100_000  // kHz
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
    // output wire                         tio_trigger,
    // output wire                         tio_clkout,
    input  wire                         tio_clkin,

    // Block Interface to Crypto Core
// `ifdef USE_BLOCK_INTERFACE
//    ,output wire                         crypto_clk,
//     output wire                         crypto_rst,
//     output wire [pPT_WIDTH-1:0]         crypto_textout,
//     output wire [pKEY_WIDTH-1:0]        crypto_keyout,
//     input  wire [pCT_WIDTH-1:0]         crypto_cipherin,
//     output wire                         crypto_start,
//     input wire                          crypto_ready,
//     input wire                          crypto_done,
//     input wire                          crypto_busy,
//     input wire                          crypto_idle
// `endif

    // Exit signals. Needed for the testbench as output port
    output wire        exit_valid_o,
    output wire [31:0] exit_value_o
  );


   //  wire [pKEY_WIDTH-1:0] crypt_key;
   //  wire [pPT_WIDTH-1:0] crypt_textout;
   //  wire [pCT_WIDTH-1:0] crypt_cipherin;
   //  wire crypt_init;
   //  wire crypt_ready;
   //  wire crypt_start;
   //  wire crypt_done;
   //  wire crypt_busy;

    // Added for the bridge
    wire [pINSTR_WIDTH-1:0] bridge_instruction;
    wire                    bridge_rst_instr_valid;
    wire [pINSTR_WIDTH-1:0] bridge_new_address;
    wire                    bridge_rst_new_address_valid;

    wire [pINSTR_WIDTH-1:0] bridge_data;
    wire                    bridge_data_valid;
    wire [7:0] bridge_status;
    // --------------------------------

    wire isout;
    wire [pADDR_WIDTH-pBYTECNT_SIZE-1:0] reg_address;
    wire [pBYTECNT_SIZE-1:0] reg_bytecnt;
    wire reg_addrvalid;
    wire [7:0] write_data;
    wire [7:0] read_data;
    wire reg_read;
    wire reg_write;
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
       .reg_address             (reg_address), 
       .reg_bytecnt             (reg_bytecnt), 
       .reg_datao               (write_data), 
       .reg_datai               (read_data),
       .reg_read                (reg_read), 
       .reg_write               (reg_write), 
       .reg_addrvalid           (reg_addrvalid)
    );


    cw305_reg_aes #(
       .pBYTECNT_SIZE           (pBYTECNT_SIZE),
       .pADDR_WIDTH             (pADDR_WIDTH)
      //  .pPT_WIDTH               (pPT_WIDTH),
      //  .pCT_WIDTH               (pCT_WIDTH),
      //  .pKEY_WIDTH              (pKEY_WIDTH)
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

      //  .I_textout               (128'b0),               // unused
      //  .I_cipherout             (crypt_cipherin),
      //  .I_ready                 (crypt_ready),
      //  .I_done                  (crypt_done),
      //  .I_busy                  (crypt_busy),
      .I_heep_data              (bridge_data),
      .I_reset_new_addr_valid   (bridge_rst_new_address_valid),
      .I_reset_instr_valid      (bridge_rst_instr_valid),

       .O_clksettings           (clk_settings),
       .O_user_led              (led3),
      //  .O_key                   (crypt_key),
      //  .O_textin                (crypt_textout),
      //  .O_cipherin              (),                     // unused
      //  .O_start                 (crypt_start),
       // Added for the bridge
       .O_instruction           (bridge_instruction),
       .O_status                (bridge_status)
       
    );


    clocks U_clocks (
       .usb_clk                 (usb_clk),
       .usb_clk_buf             (usb_clk_buf),
       .I_j16_sel               (j16_sel),
       .I_k16_sel               (k16_sel),
       .I_clock_reg             (clk_settings),
       .I_cw_clkin              (tio_clkin),
       .I_pll_clk1              (pll_clk1),
       .O_cw_clkout             (),
       .O_cryptoclk             (heep_clk)
    );



  // Block interface is used by the IP Catalog. If you are using block-based
  // design define USE_BLOCK_INTERFACE.
// `ifdef USE_BLOCK_INTERFACE
//     assign crypto_clk = heep_clk;
//     assign crypto_rst = crypt_init;
//     assign crypto_keyout = crypt_key;
//     assign crypto_textout = crypt_textout;
//     assign crypt_cipherin = crypto_cipherin;
//     assign crypto_start = crypt_start;
//     assign crypt_ready = crypto_ready;
//     assign crypt_done = crypto_done;
//     assign crypt_busy = crypto_busy;
//     assign tio_trigger = ~crypto_idle;
// `endif

  // START CRYPTO MODULE CONNECTIONS
  // The following can have your crypto module inserted.
  // This is an example of the Google Vault AES module.
  // You can use the ILA to view waveforms if needed, which
  // requires an external USB-JTAG adapter (such as Xilinx Platform
  // Cable USB).


// #TODO: change this with grheep+bridge modules
// `ifdef GOOGLE_VAULT_AES
//    wire aes_clk;
//    wire [127:0] aes_key;
//    wire [127:0] aes_pt;
//    wire [127:0] aes_ct;
//    wire aes_load;
//    wire aes_busy;

//    assign aes_clk = heep_clk;
//    assign aes_key = crypt_key;
//    assign aes_pt = crypt_textout;
//    assign crypt_cipherin = aes_ct;
//    assign aes_load = crypt_start;
//    assign crypt_ready = 1'b1;
//    assign crypt_done = ~aes_busy;
//    assign crypt_busy = aes_busy;

//    // Example AES Core -- #TODO : 
//    aes_core aes_core (
//        .clk             (aes_clk),
//        .load_i          (aes_load),
//        .key_i           ({aes_key, 128'h0}),
//        .data_i          (aes_pt),
//        .size_i          (2'd0), //AES128
//        .dec_i           (1'b0),//enc mode
//        .data_o          (aes_ct),
//        .busy_o          (aes_busy)
//    );
//    assign tio_trigger = aes_busy;
// `endif

  // Static configuration. The board has a 4-item DIP switch, but the values j16_sel and k16_sel
  // are used to select the clock sources, so only the signals k15_sel and l14_sel are available
  // for the user.
  wire boot_select_i = k15_sel;
  wire execute_from_flash_i = l14_sel;

  

  // INTERNAL SIGNALS
  // ----------------
  // JTAG
  wire jtag_tck     = '0;
  wire jtag_tms     = '0;
  wire jtag_trst_n  = '0;
  wire jtag_tdi     = '0;
  wire jtag_tdo     = '0;

  // UART
  wire gr_heep_uart_tx;
  wire gr_heep_uart_rx;

  // GPIO
  wire [31:0] gpio;

  // SPI flash
  wire        spi_flash_sck;
  wire [ 1:0] spi_flash_csb;
  wire [ 3:0] spi_flash_sd_io;

  // SPI
  wire        spi_sck;
  wire [ 1:0] spi_csb;
  wire [ 3:0] spi_sd_io;

  // GPIO
  wire clk_div;

  // Bridge signals
  wire        req_i;
  wire        we_i;
  wire [ 3:0] be_i;
  wire [31:0] addr_i;
  wire [31:0] wdata_i;

  wire        gnt_o;
  wire        rvalid_o;
  wire [31:0] rdata_o;

  // wire        inst_valid_i;
  // wire [31:0] instruction_i;
  // wire        new_addr_valid_i;
  // wire [31:0] new_section_address_i;
  // wire        busy_o;
  wire        OBI_rvalid_o;
  wire [31:0] OBI_rdata_o;

  // UART DPI emulator
  uartdpi #(
    .BAUD('d256000),
    .FREQ(CLK_FREQ * 1000),  // Hz
    .NAME("uart")
  ) u_uartdpi (
    .clk_i (heep_clk),
    .rst_ni(resetn),
    .tx_o  (gr_heep_uart_rx),
    .rx_i  (gr_heep_uart_tx)
  );

  // SPI flash emulator
`ifndef VERILATOR
  spiflash u_flash_boot (
    .csb(spi_flash_csb[0]),
    .clk(spi_flash_sck),
    .io0(spi_flash_sd_io[0]),
    .io1(spi_flash_sd_io[1]),
    .io2(spi_flash_sd_io[2]),
    .io3(spi_flash_sd_io[3])
  );

  spiflash u_flash_device (
    .csb(spi_csb[0]),
    .clk(spi_sck),
    .io0(spi_sd_io[0]),
    .io1(spi_sd_io[1]),
    .io2(spi_sd_io[2]),
    .io3(spi_sd_io[3])
  );
`endif  /* VERILATOR */

  gpio_cnt #(
    .CntMax(32'd16)
  ) u_test_gpio (
    .clk_i (heep_clk),
    .rst_ni(resetn),
    .gpio_i(gpio[30]),
    .gpio_o(gpio[31])
  );

  // DUT
  // ---
  gr_heep_top u_gr_heep_top (
    .rst_ni              (resetn),
    .boot_select_i       (boot_select_i),
    .execute_from_flash_i(execute_from_flash_i),
    .jtag_tck_i          (jtag_tck),
    .jtag_tms_i          (jtag_tms),
    .jtag_trst_ni        (jtag_trst_n),
    .jtag_tdi_i          (jtag_tdi),
    .jtag_tdo_o          (jtag_tdo),
    .uart_rx_i           (gr_heep_uart_rx),
    .uart_tx_o           (gr_heep_uart_tx),
    .exit_valid_o        (exit_valid_o),
    .gpio_0_io           (gpio[0]),
    .gpio_1_io           (gpio[1]),
    .gpio_2_io           (gpio[2]),
    .gpio_3_io           (gpio[3]),
    .gpio_4_io           (gpio[4]),
    .gpio_5_io           (gpio[5]),
    .gpio_6_io           (gpio[6]),
    .gpio_7_io           (gpio[7]),
    .gpio_8_io           (gpio[8]),
    .gpio_9_io           (gpio[9]),
    .gpio_10_io          (gpio[10]),
    .gpio_11_io          (gpio[11]),
    .gpio_12_io          (gpio[12]),
    .gpio_13_io          (gpio[13]),
    .gpio_14_io          (gpio[14]),
    .gpio_15_io          (gpio[15]),
    .gpio_16_io          (gpio[16]),
    .gpio_17_io          (gpio[17]),
    .gpio_18_io          (gpio[18]),
    .gpio_19_io          (gpio[19]),
    .gpio_20_io          (gpio[20]),
    .gpio_21_io          (gpio[21]),
    .gpio_22_io          (gpio[22]),
    .gpio_23_io          (gpio[23]),
    .gpio_24_io          (gpio[24]),
    .gpio_25_io          (gpio[25]),
    .gpio_26_io          (gpio[26]),
    .gpio_27_io          (gpio[27]),
    .gpio_28_io          (gpio[28]),
    .gpio_29_io          (gpio[29]),
    .gpio_30_io          (gpio[30]),
    .spi_flash_sck_io    (spi_flash_sck),
    .spi_flash_cs_0_io   (spi_flash_csb[0]),
    .spi_flash_cs_1_io   (spi_flash_csb[1]),
    .spi_flash_sd_0_io   (spi_flash_sd_io[0]),
    .spi_flash_sd_1_io   (spi_flash_sd_io[1]),
    .spi_flash_sd_2_io   (spi_flash_sd_io[2]),
    .spi_flash_sd_3_io   (spi_flash_sd_io[3]),
    .spi_sck_io          (spi_sck),
    .spi_cs_0_io         (spi_csb[0]),
    .spi_cs_1_io         (spi_csb[1]),
    .spi_sd_0_io         (spi_sd_io[0]),
    .spi_sd_1_io         (spi_sd_io[1]),
    .spi_sd_2_io         (spi_sd_io[2]),
    .spi_sd_3_io         (spi_sd_io[3]),
    .i2s_sck_io          (),
    .i2s_ws_io           (),
    .i2s_sd_io           (),
    .clk_i           (heep_clk),
    .exit_value_o        (exit_value_o[0]),

    // Bridge signals
    .req_i                (req_i),
    .we_i                 (we_i),
    .be_i                 (be_i),
    .addr_i               (addr_i),
    .wdata_i              (wdata_i),

    .gnt_o                (gnt_o),
    .rvalid_o             (rvalid_o),
    .rdata_o              (rdata_o)

  );

  // Some modifications are needed. All the OBI signals have to move from the PORT section to internal signals
  // and the bridge MCU side signals have to be added

  assign bridge_new_address = bridge_instruction;

  // Bridge instantiation
  bridge2xheep u_bridge2xheep (
    .clk(heep_clk),
    .rst_n(resetn),

    // HEEP Side
    .req(req_i),
    .we(we_i),
    .be(be_i),
    .addr(addr_i),
    .wdata(wdata_i),
    .gnt(gnt_o),
    .rvalid(rvalid_o),
    .rdata(rdata_o),

    // Bridge Side
    .instr_valid(bridge_status[1]),
    .addr_valid(bridge_status[2]),
    .rst_new_address_valid(bridge_rst_new_address_valid),
    .rst_instr_valid(bridge_rst_instr_valid),
    .busy(/*bridge_status[0]*/),
    .instruction(bridge_instruction),
    .new_section_address(bridge_new_address),
    .OBI_rvalid(bridge_data_valid),
    .OBI_rdata(bridge_data)
  );

  // Exit value
  assign exit_value_o[31:1] = u_gr_heep_top.u_core_v_mini_mcu.exit_value_o[31:1];

endmodule

`default_nettype wire

