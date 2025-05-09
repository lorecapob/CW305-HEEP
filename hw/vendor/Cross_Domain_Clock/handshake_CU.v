/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
 * Description: This Control Unit manages the handshake protocol between the source and destination clock domains 
                for the CW305 XHEEP project. The board has two clock domains: the USB clock domain, fixed at 96 MHz,
                and the FPGA clock domain, which can be configured to run at a frequency in the range from roughly 
                5MHz to 160MHz. The handshake protocol is used to ensure that the USB clock domain and the FPGA
                clock domain are in sync. For the XHEEP case, the USB can send new data only if XHEEP is ready to
                receive it. This is done by using a status regiter in the FPGA that is set by the USB and cleared 
                by the FPGA. This CU works with the USB clock domain and recieves the reset signal from the FPGA 
                clock domain.
*/

module handshake_CU (
    input   wire usb_clk,
    input   wire rst_n,
    input   wire rst_new_addr_valid_from_bridge_i,
    input   wire rst_instr_valid_from_bridge_i,
    output  reg  rst_new_addr_valid_to_regs_o,
    output  reg  rst_instr_valid_to_regs_o
);

// States definition
parameter RESET = 6'b000001;
parameter IDLE = 6'b000010;
parameter RST_ADDR = 6'b000100;
parameter WAIT_1 = 6'b001000;
parameter RST_INSTR = 6'b010000;
parameter WAIT_2 = 6'b100000;

// State register
reg [5:0] present_state, next_state;

always @(*) begin: next_state_logic
    case(present_state)
        RESET: begin
            next_state = IDLE;
        end

        IDLE: begin
            if (~rst_new_addr_valid_from_bridge_i) begin
                next_state = RST_ADDR;
            end
            else if (~rst_instr_valid_from_bridge_i) begin
                next_state = RST_INSTR;
            end
            else begin
                next_state = IDLE;
            end
        end

        RST_ADDR: begin
            next_state = WAIT_1;
        end

        WAIT_1: begin
            if (~rst_new_addr_valid_from_bridge_i) begin
                next_state = WAIT_1;
            end
            else begin
                next_state = IDLE;
            end
        end

        RST_INSTR: begin
            next_state = WAIT_2;
        end

        WAIT_2: begin
            if (~rst_instr_valid_from_bridge_i) begin
                next_state = WAIT_2;
            end
            else begin
                next_state = IDLE;
            end
        end

        default: begin
            next_state = RESET;
        end
    endcase

end

always @(posedge usb_clk or negedge rst_n) begin: present_state_update
    if (~rst_n) begin
        present_state <= RESET;
    end
    else begin
        present_state <= next_state;
    end
end

always @(*) begin: output_logic
    rst_new_addr_valid_to_regs_o = 1'b1;
    rst_instr_valid_to_regs_o = 1'b1;

    case(present_state)
        RST_ADDR: begin
            rst_new_addr_valid_to_regs_o = 1'b0;
        end

        RST_INSTR: begin
            rst_instr_valid_to_regs_o = 1'b0;
        end

        default: begin
            rst_new_addr_valid_to_regs_o = 1'b1;
            rst_instr_valid_to_regs_o = 1'b1;
        end

    endcase
end

endmodule
