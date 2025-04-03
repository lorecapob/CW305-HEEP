/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
 * Description: Dual Flip-Flop Synchronizer. This module is used to synchronize signals 
                between different clock domains. It consist of an output flip flop from 
                the signal arriving from the source clock domain and two input flip flops 
                for the destination clock domain.
*/

module dfs (
    input wire  clk_src, //source clk
    input wire  clk_dst, //destination clk
    input wire  rst_src,
    input wire  rst_dst,
    input wire  din_src,  //input
    output reg dout_dst  //synchronized output
);

reg din_src_flop;
reg dmeta;

always@(posedge clk_src or negedge rst_src) begin
    if(!rst_src) begin
        din_src_flop <= 1'b0;
    end
    else begin
        din_src_flop <= din_src;
    end
end

always@(posedge clk_dst or negedge rst_dst) begin
    if(!rst_dst) begin
        dmeta <= 1'b0;
        dout_dst  <= 1'b0;
    end
    else begin 
        dmeta <= din_src_flop;
        dout_dst  <= dmeta;
    end
end

endmodule
