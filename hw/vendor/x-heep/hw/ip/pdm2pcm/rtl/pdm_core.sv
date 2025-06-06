// Copyright 2022 EPFL
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

// Authors: Pierre Guillod <pierre.guillod@epfl.ch> ,EPFL, STI-SEL
//          Jérémie Moullet<jeremie.moullet@epfl.ch>,EPFL, STI-SEL
// Date: 05.2025
// Description: PDM to PCM converter core



module pdm_core #(
    // Number of stages of the CIC filter
    parameter integer STAGES_CIC = 4,
    // Width of the datapath
    parameter integer WIDTH = 18,
    // First decimator internal counter width
    parameter integer DECIM_COMBS_CNT_W = 4,
    // Widht of the comb delay parameter
    parameter integer DELAYCOMBWIDTH = 5

) (
    // Clock input
    input logic div_clk_i,
    // Reset input
    input logic rstn_i,
    // Enable input
    input logic en_i,
    // Clock output to the microphone
    output logic pdm_clk_o,
    // Which/How many CIC stage are activated (Thermometric, right-aligned)
    input logic [STAGES_CIC-1:0] par_cic_activated_stages,
    // First decimator decimation index
    input logic [DECIM_COMBS_CNT_W-1:0] par_decim_idx_combs,
    //Delay D in the combs stage
    input logic [DELAYCOMBWIDTH-1:0] par_delay_combs,

    // Input signal (PDM)
    input logic pdm_i,
    // Output signal (PCM)
    output logic [WIDTH-1:0] pcm_o,
    // Valid output data flag
    output logic pcm_data_valid_o
);

  logic             r_store;
  logic             r_send;
  logic             r_data;

  // Auxiliary signals to link the filter blocks
  logic [WIDTH-1:0] data;
  logic [WIDTH-1:0] integr_to_comb;
  logic [WIDTH-1:0] combs_to_hb1;

  logic             r_en;
  logic             s_clr;

  // Decimators output signals
  logic             combs_en;

  // Output synchronized with the last decimator
  assign pcm_data_valid_o = combs_en;

  ///////////////////////////////////////////////////////////////////////////
  //////////// PIECE OF CODE I NEED TO MAKE EASIER TO UNDERSTAND ////////////
  ///////////////////////////////////////////////////////////////////////////

  always_ff @(posedge div_clk_i or negedge rstn_i) begin : proc_r_store
    if (~rstn_i) begin
      r_store   <= 1;
      r_send    <= 0;
      r_data    <= 0;
      pdm_clk_o <= 0;
    end else begin
      if (en_i) begin
        r_store <= ~r_store;
        r_send  <= ~r_send;
        if (r_store) begin
          r_data    <= pdm_i;
          pdm_clk_o <= ~pdm_clk_o;
        end else begin
          r_store   <= 1'b1;
          r_send    <= 0;
          pdm_clk_o <= 0;
        end
      end
    end
  end

  assign s_clr = en_i & !r_en;

  always_ff @(posedge div_clk_i or negedge rstn_i) begin
    if (~rstn_i) r_en <= 'h0;
    else r_en <= en_i;
  end

  ///////////////////////////////////////////////////////////////////////////
  ////// END OF THE PIECE OF CODE I NEED TO MAKE EASIER TO UNDERSTAND ///////
  ///////////////////////////////////////////////////////////////////////////

  // Converts binary PDM {0,1} to bipolar PDM {-1,1}
  assign data = r_data ? 'h1 : {WIDTH{1'b1}};

  // Instantiation sequence
  //┌───────────────────────┐
  //│       CIC Filter      │
  //│┌─────┐ ┌─────┐ ┌─────┐│┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐
  //││Intgs├─►Decim├─►Combs├│►Hlfbd├─►Decim├─►Hlfbd├─►Decim├─► FIR │
  //│└─────┘ └─────┘ └─────┘│└─────┘ └─────┘ └─────┘ └─────┘ └─────┘
  //└───────────────────────┘
  // (made with asciiflow.com)

  cic_integrators #(STAGES_CIC, WIDTH) cic_integrators_inst (
      .clk_i (div_clk_i),
      .rstn_i(rstn_i),
      .clr_i (s_clr),
      .en_i  (r_send),
      .par_cic_activated_stages,
      .data_i(data),
      .data_o(integr_to_comb)
  );

  decimator #(DECIM_COMBS_CNT_W) decimator_before_hb1 (
      .clk_i(div_clk_i),
      .rst_i(rstn_i),
      .clr_i(s_clr),
      .par_decimation_index(par_decim_idx_combs),
      .en_i(r_send),
      .en_o(combs_en)
  );

  cic_combs #(STAGES_CIC, WIDTH, DELAYCOMBWIDTH) cic_combs_inst (
      .clk_i (div_clk_i),
      .rstn_i(rstn_i),
      .clr_i (s_clr),
      .en_i  (combs_en),
      .par_cic_activated_stages,
      .par_delay_combs,
      .data_i(integr_to_comb),
      .data_o(combs_to_hb1)
  );

  assign pcm_o = combs_to_hb1;

endmodule
