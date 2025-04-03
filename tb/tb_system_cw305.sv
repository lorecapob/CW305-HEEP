/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
 * Description: Testbench wrapper for the CW305 system. It includes the CW305 top module and the UART DPI emulator.
                It also includes the SPI flash emulator and a GPIO counter for testing purposes.
 */

module tb_system_cw305 #(
    parameter pBYTECNT_SIZE = 2,
    parameter pADDR_WIDTH = 21,
    parameter pINSTR_WIDTH = 32,
    parameter int unsigned CLK_FREQ = 32'd100_000  // kHz
)(
    // USB Interface
    input wire                          clk_i,          // USB Clock
    inout wire [7:0]                    usb_data,       // Data for write/read

    input wire [pADDR_WIDTH-1:0]        usb_addr,       // Address
    input wire                          usb_rdn,        // !RD, low when addr valid for read
    input wire                          usb_wrn,        // !WR, low when data+addr valid for write
    input wire                          usb_cen,        // !CE, active low chip enable
    input wire                          usb_trigger,    // High when trigger requested

    // Buttons/LEDs on Board
    // input wire                          j16_sel,        // DIP switch J16
    // input wire                          k16_sel,        // DIP switch K16
    input wire                          boot_select_i,          // DIP switch K15
    input wire                          execute_from_flash_i,   // DIP Switch L14
    input wire                          rst_ni,         // Pushbutton SW4, connected to R1, used here as reset
    output wire                         led1,           // red LED
    output wire                         led2,           // green LED
    output wire                         led3,           // blue LED

    // PLL
    input wire                          pll_clk1,       //PLL Clock Channel #1
    // input wire                          pll_clk2,       //PLL Clock Channel #2 (unused in this example)

    // 20-Pin Connector Stuff
    // output wire                         tio_trigger,
    // output wire                         tio_clkout,
    // input  wire                         tio_clkin

    // Exit signals. Needed for the testbench as output port
    output wire        exit_valid_o,
    output wire [31:0] exit_value_o
);
    logic usb_clk = clk_i;
    logic pushbutton = rst_ni;

    // UART
    wire cw305_heep_uart_tx;
    wire cw305_heep_uart_rx;

    wire debug_heep_uart_rx;
    wire debug_heep_uart_tx;

    // SPI flash
    wire        spi_flash_sck;
    wire [ 1:0] spi_flash_csb;
    wire [ 3:0] spi_flash_sd_io;

    // SPI
    wire        spi_sck;
    wire [ 1:0] spi_csb;
    wire [ 3:0] spi_sd_io;

    // GPIO
    wire [31:0] gpio;

    // UART DPI emulator
    uartdpi #(
        .BAUD('d256000),
        .FREQ(CLK_FREQ * 1000),  // Hz
        .NAME("uart")
    ) u_uartdpi (
        .clk_i (pll_clk1),
        .rst_ni(rst_ni),
        .tx_o  (cw305_heep_uart_rx),
        .rx_i  (cw305_heep_uart_tx)
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
        .clk_i (pll_clk1),
        .rst_ni(rst_ni),
        .gpio_i(gpio[30]),
        .gpio_o(gpio[31])
    );

    cw305_top #(
        .pBYTECNT_SIZE(pBYTECNT_SIZE),
        .pADDR_WIDTH(pADDR_WIDTH),
        .pINSTR_WIDTH(pINSTR_WIDTH)
    ) U_cw305_top (
        // USB Interface
        .usb_clk(usb_clk),
        .usb_data(usb_data),
        .usb_addr(usb_addr),
        .usb_rdn(usb_rdn),
        .usb_wrn(usb_wrn),
        .usb_cen(usb_cen),
        .usb_trigger(usb_trigger),

        // Buttons/LEDs on Board
        .j16_sel(1'b0),
        .k16_sel(1'b0),
        .k15_sel(boot_select_i),
        .l14_sel(execute_from_flash_i),
        .pushbutton(pushbutton),
        .led1(led1),
        .led2(led2),
        .led3(led3),

        // PLL
        .pll_clk1(pll_clk1),
        //.pll_clk2(pll_clk2),

        // 20-Pin Connector Stuff
        // .tio_trigger(), // Unused
        // .tio_clkout(),  // Unused
        .tio_clkin(usb_clk), // Connected to USB clock just for simulation purpose
        .IO_0(),
        .IO_1(),
        .debug_heep_uart_rx(debug_heep_uart_rx),
        .debug_heep_uart_tx(debug_heep_uart_tx),

        // Exit signals
        .exit_valid_o(exit_valid_o),
        .exit_value_o(exit_value_o),

        // UART DPI
        .gr_heep_uart_tx(cw305_heep_uart_tx),
        .gr_heep_uart_rx(cw305_heep_uart_rx),

        // SPI flash emulator
        .spi_flash_sck(spi_flash_sck),
        .spi_flash_csb(spi_flash_csb),
        .spi_flash_sd_io(spi_flash_sd_io),

        .spi_sck(spi_sck),
        .spi_csb(spi_csb),
        .spi_sd_io(spi_sd_io),

        // GPIO counter
        .gpio(gpio)
    );

endmodule
