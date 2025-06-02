// keccak: control unit and datapath are instantiated. 
// Designed by Mattia Mirigaldi
// mattia.mirigaldi@polito.com


module keccak_f (
    input clk,
    input rst_n,  // asynchronous, active-low
    input start_i,
    input [1599:0] Din, // rate input (XORed before round 0)
    output [1599:0] Dout,  // full state after 24 rounds
    output status_d,
    output status_de,
    output keccak_intr);


logic permutation_finish;
logic start_dp;

// --------------------------------------------------------------------
    //  Datapath – Keccak-f[1600] permutation
    // --------------------------------------------------------------------
    keccak_dp u_dp (
        .clk      (clk),
        .rst_n    (rst_n),
        .start_i  (start_dp),          // driven by control unit
        .Din      (Din),
        .ready_o  (permutation_finish),
        .Dout     (Dout)
    );

    // --------------------------------------------------------------------
    //  Control unit – starts the permutation and raises IRQ / status
    // --------------------------------------------------------------------
    keccak_cu u_cu (
        .clk_i           (clk),
        .rst_ni         (rst_n),
        .start_i       (start_i),          // external command
        .ready_dp_i    (permutation_finish),
        .start_dp_o    (start_dp),
        .status_d      (status_d),
        .status_de     (status_de),
        .keccak_intr   (keccak_intr)
    );

endmodule



