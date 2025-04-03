/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
 * Description: Instruction register module. It stores the instruction to be sent to X-HEEP.
 */

module instr_reg #()
(
    input logic clk,
    input logic rst_n,
    input logic LOAD,
    //input logic ADDRESS
    input logic [31:0] instr_in,

    output logic [31:0] instr_out
);

always_ff @(posedge clk) begin
    if(~rst_n)
        instr_out <= 32'h00000000;
    else 
        if(LOAD)
            instr_out <= instr_in;
end

endmodule
