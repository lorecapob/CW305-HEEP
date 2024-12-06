module instr_reg #()
(
    input logic clk,
    input logic rst_n,
    input logic LD,
    input logic [31:0] instr_in,

    output logic [31:0] instr_out
);

always_ff @(posedge clk) begin
    if(~rst_n)
        instr_out <= 32'h00000000;
    else 
        if(LD)
            instr_out <= instr_in;
end

endmodule
