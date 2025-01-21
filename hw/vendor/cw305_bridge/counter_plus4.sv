module counter_plus4
#(
    // parameter section
)
(
    input   logic clk,
    input   logic rst_n,

    input   logic LOAD,
    input   logic ENABLE,
    input   logic [31:0] cnt_in,
    output  logic [31:0] cnt_out
);

    logic [31:0] cnt;

    always_ff @(posedge clk)
    begin
        if(~rst_n)
            cnt <= 32'h00000000;
        else 
            if(LOAD)
                cnt <= cnt_in;
            else if(ENABLE)
                cnt <= cnt + 32'h00000004;
    end

    assign cnt_out = cnt;

endmodule
