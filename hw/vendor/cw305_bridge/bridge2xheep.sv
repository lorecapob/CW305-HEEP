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
    // output logic req,
    // output logic we,
    // output logic [3:0] be,
    // output logic [31:0] addr,
    // output logic [31:0] wdata,
    output obi_req_t req_from_bridge,

    // OBI response
    // input logic gnt,
    // input logic rvalid,
    // input logic [31:0] rdata,
    input obi_resp_t resp_from_xheep,

    // ####### MCU side #######
    // Status register flags
    input logic instr_valid,
    input logic addr_valid,
    //input logic data_read,
    output logic rst_new_address_valid,
    output logic rst_instr_valid,

    output logic busy,
    //output logic data_valid,
    output logic OBI_rvalid,
    output logic [31:0] OBI_rdata,

    // Instruction register values
    input logic [31:0] instruction,
    input logic [31:0] new_section_address
);



logic CNT_RSTN;
logic CNT_LD;
logic CNT_EN;

logic INST_REG_RSTN;
logic INST_REG_LD;

logic req;
logic we;
logic [3:0] be;
// logic [31:0] addr;
logic [31:0] internal_addr;
logic [31:0] wdata;

logic gnt;
logic rvalid;
logic [31:0] rdata;

assign req_from_bridge.req      = req;
assign req_from_bridge.we       = we;
assign req_from_bridge.be       = be;
assign req_from_bridge.addr     = internal_addr;
assign req_from_bridge.wdata    = wdata;

assign gnt      = resp_from_xheep.gnt;
assign rdata    = resp_from_xheep.rdata;
assign rvalid   = resp_from_xheep.rvalid;

assign OBI_rdata = rdata;
assign OBI_rvalid = rvalid;

bridge2xheep_cu #() bridge_cu (
    .clk(clk),
    .rst_n(rst_n),
    .instr_valid(instr_valid),
    .addr_valid(addr_valid),
    .rst_new_addr_valid(rst_new_address_valid),
    .rst_instr_valid(rst_instr_valid),
    .busy(busy),
    .req(req),
    .we(we),
    .be(be),
    .gnt(gnt),
    .CNT_RSTN(CNT_RSTN),
    .CNT_LD(CNT_LD),
    .CNT_EN(CNT_EN),
    .INST_REG_RSTN(INST_REG_RSTN),
    .INST_REG_LD(INST_REG_LD)
);

instr_reg #() internal_reg (
    .clk(clk),
    .rst_n(INST_REG_RSTN),
    .LOAD(INST_REG_LD),
    //.ADDRESS(),
    .instr_in(instruction),
    .instr_out(wdata)
);

counter_plus4 #() addr_counter (
    .clk(clk),
    .rst_n(CNT_RSTN),
    .LOAD(CNT_LD),
    .ENABLE(CNT_EN),
    .cnt_in(new_section_address),
    .cnt_out(internal_addr)
);

// assign addr = internal_addr;


endmodule
