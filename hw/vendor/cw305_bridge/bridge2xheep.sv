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
    input logic instr_valid,
    input logic addr_valid,
    //input logic data_read,

    output logic busy,
    //output logic data_valid,
    output logic OBI_rvalid,
    output logic [31:0] OBI_rdata,

    // Instruction register values
    input logic [31:0] instruction,
    input logic [31:0] new_section_address
);

enum logic [5:0] {RESET, IDLE, SET_COUNTER, LD_INSTR, REQ_SENT, GNT_RECEIVED} currentState, nextState;
logic [31:0] internal_addr;

logic CNT_RSTN;
logic CNT_LD;
logic CNT_EN;

logic INST_REG_RSTN;
logic INST_REG_LD;

assign OBI_rdata = rdata;
assign OBI_rvalid = rvalid;

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
            nextState = IDLE;
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
            CNT_RSTN = 1'b0;
            CNT_LD = 1'b0;
            CNT_EN = 1'b0;
            INST_REG_RSTN = 1'b0;
            INST_REG_LD = 1'b0;
            busy = 1'b0;
        end

        IDLE: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            CNT_RSTN = 1'b1;
            CNT_LD = 1'b0;
            CNT_EN = 1'b0;
            INST_REG_RSTN = 1'b1;
            INST_REG_LD = 1'b0;
            busy = 1'b0;
        end

        SET_COUNTER: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            CNT_RSTN = 1'b1;
            CNT_LD = 1'b1;
            CNT_EN = 1'b0;
            INST_REG_RSTN = 1'b1;
            INST_REG_LD = 1'b0;
            busy = 1'b0;
        end

        LD_INSTR: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            CNT_RSTN = 1'b1;
            CNT_LD = 1'b0;
            CNT_EN = 1'b0;
            INST_REG_RSTN = 1'b1;
            INST_REG_LD = 1'b1;
            busy = 1'b1;
        end

        REQ_SENT: begin
            req = 1'b1;
            we = 1'b1;
            be = 4'b1111;
            CNT_RSTN = 1'b1;
            CNT_LD = 1'b0;
            CNT_EN = 1'b0;
            INST_REG_RSTN = 1'b1;
            INST_REG_LD = 1'b0;
            busy = 1'b1;
        end

        GNT_RECEIVED: begin
            req = 1'b0;
            we = 1'b0;
            be = 4'b0;
            CNT_RSTN = 1'b1;
            CNT_LD = 1'b0;
            CNT_EN = 1'b1;
            INST_REG_RSTN = 1'b1;
            INST_REG_LD = 1'b0;
            busy = 1'b1;
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

instr_reg #() internal_reg (
    .clk(clk),
    .rst_n(INST_REG_RSTN),
    .LD(INST_REG_LD),
    .instr_in(instruction),
    .instr_out(wdata)
);

counter_plus4 #() addr_counter (
    .clk(clk),
    .rst_n(CNT_RSTN),
    .LD_cnt(CNT_LD),
    .EN_cnt(CNT_EN),
    .cnt_in(new_section_address),
    .cnt_out(internal_addr)
);

assign addr = internal_addr;


endmodule
