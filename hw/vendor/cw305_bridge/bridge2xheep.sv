module bridge2xheep 
    import obi_pkg::*;
#(
    // parameter section
)
(
    input logic clk,
    input logic rst_n,

    // ####### HEEP side #######
    // OBI request
    output logic req,
    output logic we,
    output logic [3:0] be,
    output logic [31:0] addr,
    output logic [31:0] wdata,

    // OBI response
    input logic gnt,
    input logic rvalid,
    input logic [31:0] rdata,

    // ####### MCU side #######
    // Status register flags
    input logic inst_valid,
    //input logic data_read,

    output logic busy,
    //output logic data_valid,
    output logic OBI_rvalid,
    output logic [31:0] OBI_rdata,

    // Instruction register values
    input logic [31:0] instruction
);

enum logic [3:0] {RESET, IDLE, REQ_SENT, GNT_RECEIVED} currentState, nextState;
logic [31:0] internal_addr = 32'h00000180;

assign addr = internal_addr;
assign wdata = instruction;

assign OBI_rdata = rdata;
assign OBI_rvalid = rvalid;

always_comb begin: next_State_logic
    nextState = RESET;
    
    case(currentState)
        RESET: begin
            nextState = IDLE;
        end

        IDLE: begin
            if(inst_valid)
                nextState = REQ_SENT;
            else
                nextState = IDLE;
        end

        REQ_SENT: begin
            if(gnt)
                nextState = GNT_RECEIVED;
            else
                nextState = REQ_SENT;
        end

        GNT_RECEIVED: begin
            nextState = IDLE;
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
    case(currentState)
        RESET: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            //addr = 32'b0;
            //wdata = 32'b0;
            busy = 1'b0;
        end

        IDLE: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            //addr = 32'b0;
            //wdata = 32'b0;
            busy = 1'b0;
        end

        REQ_SENT: begin
            req = 1'b1;
            we = 1'b1;
            be = 4'b1111;
            //addr = internal_addr;
            //wdata = instruction;
            busy = 1'b1;
        end

        GNT_RECEIVED: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            //addr = 32'b0;
            //wdata = 32'b0;
            busy = 1'b0;
        end

        default: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            //addr = 32'b0;
            //wdata = 32'b0;
            busy = 1'b0;
        end
    endcase

end

// Update the address for the next instruction
always_ff @(posedge clk) begin
    if(currentState == GNT_RECEIVED) begin
        internal_addr <= internal_addr + 4;
    end

end


endmodule
