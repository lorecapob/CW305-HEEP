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
    // input wire                          pll_clk1,       //PLL Clock Channel #1
    // input wire                          pll_clk2,       //PLL Clock Channel #2 (unused in this example)

    // 20-Pin Connector Stuff
    // output wire                         tio_trigger,
    // output wire                         tio_clkout,
    // input  wire                         tio_clkin

    // Exit signals. Needed for the testbench as output port
    output wire        exit_valid_o,
    output wire [31:0] exit_value_o,

    // Bridge status register. Needed for the testbench
    output wire        bridge_instr_valid_status
);
    logic usb_clk = clk_i;
    logic pushbutton = rst_ni;

    cw305_top #(
        .pBYTECNT_SIZE(pBYTECNT_SIZE),
        .pADDR_WIDTH(pADDR_WIDTH),
        .pINSTR_WIDTH(pINSTR_WIDTH),
        .CLK_FREQ(CLK_FREQ)
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
        .pll_clk1(usb_clk), // Connected to USB clock just for simulation purpose
        //.pll_clk2(pll_clk2),

        // 20-Pin Connector Stuff
        // .tio_trigger(), // Unused
        // .tio_clkout(),  // Unused
        .tio_clkin(usb_clk), // Connected to USB clock just for simulation purpose

        // Exit signals
        .exit_valid_o(exit_valid_o),
        .exit_value_o(exit_value_o)
    );

    assign bridge_instr_valid_status = U_cw305_top.bridge_status[1];

endmodule
