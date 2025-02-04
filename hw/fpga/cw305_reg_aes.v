/* 
ChipWhisperer Artix Target - Example of connections between example registers
and rest of system.

Copyright (c) 2020, NewAE Technology Inc.
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

`default_nettype none
//`timescale 1ns / 1ps
`include "cw305_aes_defines.v"

module cw305_reg_aes #(
   parameter pADDR_WIDTH = 21,
   parameter pBYTECNT_SIZE = 7,

   // Added for the bridge
   parameter pINSTR_WIDTH = 32
)(

// Interface to cw305_usb_reg_fe:
   input  wire                                  usb_clk,
   input  wire                                  crypto_clk,
   input  wire                                  reset_i,
   input  wire [pADDR_WIDTH-pBYTECNT_SIZE-1:0]  reg_address,     // Address of register
   input  wire [pBYTECNT_SIZE-1:0]              reg_bytecnt,  // Current byte count
   output reg  [7:0]                            read_data,       //
   input  wire [7:0]                            write_data,      //
   input  wire                                  reg_read,        // Read flag. One clock cycle AFTER this flag is high
                                                                 // valid data must be present on the read_data bus
   input  wire                                  reg_write,       // Write flag. When high on rising edge valid data is
                                                                 // present on write_data
   input  wire                                  reg_addrvalid,   // Address valid flag

// from top:
   input  wire                                  exttrigger_in,

// register inputs:
   input reg [pINSTR_WIDTH-1:0]                 I_heep_data,
   
   // Added for the bridge
   input wire                                   I_reset_new_addr_valid,
   input wire                                   I_reset_instr_valid,


// register outputs:
   output reg  [4:0]                            O_clksettings,
   output reg                                   O_user_led,

   // Added for the bridge
   output reg [pINSTR_WIDTH-1:0]                O_instruction,
   output reg [pINSTR_WIDTH-1:0]                O_address,
   output reg [7:0]                             O_status


);

   reg  [7:0]                   reg_read_data;
   wire [31:0]                  buildtime = 0;

   
   //////////////////////////////////
   // read logic:
   //////////////////////////////////

   always @(*) begin
      if (reg_addrvalid && reg_read) begin
         case (reg_address)
            `REG_CLKSETTINGS:           reg_read_data = {{3{1'b0}}, O_clksettings}; // Zero-extend to 8 bits
            `REG_USER_LED:              reg_read_data = {{7{1'b0}}, O_user_led}; // Zero-extend to 8 bits
            `REG_BRIDGE_STATUS:         reg_read_data = O_status;
            `REG_HEEP_DATA:             reg_read_data = I_heep_data[reg_bytecnt*8 +: 8];
            `REG_BUILDTIME:             reg_read_data = buildtime[reg_bytecnt*8 +: 8];
            default:                    reg_read_data = 0;
         endcase
      end
      else
         reg_read_data = 0;
   end

   // Register output read data to ease timing. If you need read data one clock
   // cycle earlier, simply remove this stage:
   always @(posedge usb_clk)
      read_data <= reg_read_data;
   //assign read_data = reg_read_data;

   //////////////////////////////////
   // write logic (USB clock domain):
   //////////////////////////////////
   always @(posedge usb_clk) begin
      if (reset_i) begin
         O_clksettings <= 0;
         O_user_led <= 0;

         O_status <= 0;
         O_instruction <= 0;
      end

      else begin
         if (reg_addrvalid && reg_write) begin
            case (reg_address)
               `REG_CLKSETTINGS:        O_clksettings <= write_data[4:0];
               `REG_USER_LED:           O_user_led <= write_data[0];
               `REG_BRIDGE_STATUS:      O_status <= write_data;
               `REG_PROG_INSTR:         O_instruction[reg_bytecnt*8 +: 8] <= write_data;
               `REG_PROG_ADDRESS:       O_address[reg_bytecnt*8 +: 8] <= write_data;
               
            endcase
         end
      end
   end

   // Since the USB communications relies on the value of the instruction valid flag (status_reg[1])
   // and the address valid flag (status_reg[2]), we need to reset these flags after they have been read
   // by the bridge. A signal from the bridge can be used to do this.
   // There is no risk of race condition, since, as soon the instruction valid flag is set to 1, the USB
   // communication is blocked until that flag is set to 0.
   // During this interval, the bridge can read the instruction and reset the flag.
   always @(posedge usb_clk) begin
      if (~I_reset_new_addr_valid)
         O_status[2] <= 0;
   end
   
   always @(posedge usb_clk) begin
      if (~I_reset_instr_valid)
         O_status[1] <= 0;
   end


endmodule

`default_nettype wire
