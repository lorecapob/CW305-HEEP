/*
 * Copyright 2025 Politecnico di Torino.
 * Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
 * SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
 *
 * Author: Lorenzo Capobianco
 * Date: 03/04/2025
 * Description: Bridge CU for the Bridge to XHEEP module
 */

module bridge2xheep_cu #()(
    input   logic clk,
    input   logic rst_n,

    // Status register flags
    input   logic instr_valid,
    input   logic addr_valid,
    output  logic rst_new_addr_valid,
    output  logic rst_instr_valid,
    output  logic busy,

    // ##### OBI protocol signals #####
    // OBI request
    output  logic req,
    output  logic we,
    output  logic [3:0] be,

    // OBI response
    input   logic gnt,

    // ##### Datapath signals #####
    output  logic CNT_RSTN,
    output  logic CNT_LD,
    output  logic CNT_EN,
    output  logic INST_REG_RSTN,
    output  logic INST_REG_LD

);

enum logic [7:0] {RESET, IDLE, SET_COUNTER, WAIT_1, LD_INSTR, REQ_SENT, GNT_RECEIVED, WAIT_2} currentState, nextState;

always_comb begin: next_State_logic
    
    case(currentState)
        RESET: begin
            nextState = IDLE;
        end

        IDLE: begin
            if(addr_valid)
                nextState = SET_COUNTER;
            else if(instr_valid)
                nextState = LD_INSTR;
            else
                nextState = IDLE;
        end

        SET_COUNTER: begin
            nextState = WAIT_1;
        end

        WAIT_1: begin
            if(~addr_valid)
                nextState = IDLE;
            else
                nextState = WAIT_1;
        end

        LD_INSTR: begin
            nextState = REQ_SENT;
        end

        REQ_SENT: begin
            if(gnt)
                nextState = GNT_RECEIVED;
            else
                nextState = REQ_SENT;
        end

        GNT_RECEIVED: begin
            nextState = WAIT_2;
        end

        WAIT_2: begin
            if(~instr_valid)
                nextState = IDLE;
            else
                nextState = WAIT_2;
        end

        default: begin
            nextState = RESET;
        end

    endcase
end

always_ff @(posedge clk or negedge rst_n) begin : state_transition_logic
    if(~rst_n)
        currentState <= RESET;
    else
        currentState <= nextState; 
end

always_comb begin: output_generation_logic

    req = 1'b0;
    we = 1'b0;
    be = 4'b0;
    CNT_RSTN = 1'b1;
    CNT_LD = 1'b0;
    CNT_EN = 1'b0;
    INST_REG_RSTN = 1'b1;
    INST_REG_LD = 1'b0;
    busy = 1'b0;
    rst_new_addr_valid = 1'b1;
    rst_instr_valid = 1'b1;

    case(currentState)
        RESET: begin
            CNT_RSTN = 1'b0;
            INST_REG_RSTN = 1'b0;
        end

        IDLE: begin

        end

        SET_COUNTER: begin
            CNT_LD = 1'b1;
            busy = 1'b1;
            rst_new_addr_valid = 1'b0;
        end

        WAIT_1: begin
            
        end

        LD_INSTR: begin
            INST_REG_LD = 1'b1;
            busy = 1'b1;
        end

        REQ_SENT: begin
            req = 1'b1;
            we = 1'b1;
            be = 4'b1111;
            busy = 1'b1;
        end

        GNT_RECEIVED: begin
            CNT_EN = 1'b1;
            busy = 1'b1;
            rst_instr_valid = 1'b0;
        end

        WAIT_2: begin

        end

        default: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            CNT_RSTN = 1'b0;
            CNT_LD = 1'b0;
            CNT_EN = 1'b0;
            INST_REG_RSTN = 1'b0;
            INST_REG_LD = 1'b0;
            busy = 1'b0;
        end
    endcase

end

endmodule
