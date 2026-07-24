`include "tl_sram_ctrl_pkg.vh"
`include "./tl_ad_mem_if.v"
`include "./raw_sram_ctrl_top.v"

module tlc_mem_wrapper_top #(
    parameter AW         = 64,
    parameter DW         = 64,
    parameter SOURCE_W   = 4,
    parameter SINK_W     = 1,
    parameter SZW        = 8,
    parameter MAX_BURST  = 16,
    parameter AIW        = SOURCE_W,
    parameter NUM_BANKS  = 4,
    parameter BANK_DEPTH = 262144,
    parameter RD_LATENCY = 1,
    parameter WR_LATENCY = 1,
    parameter INIT_FILE  = ""
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Memory-side TileLink request/response interface.
    // These names intentionally match the slave-manager-facing A/D signals.
    input  wire                  a_valid,
    output wire                  a_ready,
    input  wire [2:0]            a_opcode,
    input  wire [2:0]            a_param,
    input  wire [SZW-1:0]        a_size,
    input  wire [SOURCE_W-1:0]   a_source,
    input  wire [AW-1:0]         a_address,
    input  wire [DW/8-1:0]       a_mask,
    input  wire [DW-1:0]         a_data,
    input  wire                  a_corrupt,

    output wire                  d_valid,
    input  wire                  d_ready,
    output wire [2:0]            d_opcode,
    output wire [1:0]            d_param,
    output wire [SZW-1:0]        d_size,
    output wire [SOURCE_W-1:0]   d_source,
    output wire [SINK_W-1:0]     d_sink,
    output wire                  d_denied,
    output wire [DW-1:0]         d_data,
    output wire                  d_corrupt,

    // Optional visibility into the backend memory traffic.
    output wire                  mem_req_dbg,
    output wire                  mem_we_dbg,
    output wire [AW-1:0]         mem_addr_dbg,
    output wire [DW-1:0]         mem_wdata_dbg,
    output wire [DW/8-1:0]       mem_wmask_dbg,
    output wire                  mem_rvalid_dbg,
    output wire [DW-1:0]         mem_rdata_dbg,

    output wire [31:0]           total_rd_count,
    output wire [31:0]           total_wr_count,
    output wire [63:0]           total_rd_latency_sum
);

    wire                 mem_req;
    wire                 mem_ready;
    wire                 mem_we;
    wire [AW-1:0]        mem_addr;
    wire [DW-1:0]        mem_wdata;
    wire [DW/8-1:0]      mem_wmask;
    wire [AIW-1:0]       mem_source;
    wire                 mem_rvalid;
    wire [DW-1:0]        mem_rdata;
    wire [AIW-1:0]       mem_rsource;

    assign mem_req_dbg   = mem_req;
    assign mem_we_dbg    = mem_we;
    assign mem_addr_dbg  = mem_addr;
    assign mem_wdata_dbg = mem_wdata;
    assign mem_wmask_dbg = mem_wmask;
    assign mem_rvalid_dbg= mem_rvalid;
    assign mem_rdata_dbg = mem_rdata;

    // The memory-side A/D engine handles burst sequencing, atomics, and
    // converts TileLink-style requests into simple backend memory operations.
    tl_ad_mem_if #(
        .AW(AW),
        .DW(DW),
        .SOURCE_W(SOURCE_W),
        .SINK_W(SINK_W),
        .SZW(SZW),
        .MAX_BURST(MAX_BURST)
    ) u_mem_if (
        .clk(clk),
        .rst_n(rst_n),
        .a_valid(a_valid),
        .a_ready(a_ready),
        .a_opcode(a_opcode),
        .a_param(a_param),
        .a_size(a_size),
        .a_source(a_source),
        .a_address(a_address),
        .a_mask(a_mask),
        .a_data(a_data),
        .a_corrupt(a_corrupt),
        .d_valid(d_valid),
        .d_ready(d_ready),
        .d_opcode(d_opcode),
        .d_param(d_param),
        .d_size(d_size),
        .d_source(d_source),
        .d_sink(d_sink),
        .d_denied(d_denied),
        .d_data(d_data),
        .d_corrupt(d_corrupt),
        .mem_req(mem_req),
        .mem_we(mem_we),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wmask(mem_wmask),
        .mem_rdata(mem_rdata),
        .mem_rvalid(mem_rvalid),
        .mem_ready(mem_ready)
    );

    assign mem_source = {{(AIW-SOURCE_W){1'b0}}, a_source};

    // The raw SRAM backend preserves the existing banked memory hierarchy
    // without any TL-UH protocol logic at this boundary.
    raw_sram_ctrl_top #(
        .AW(AW),
        .DW(DW),
        .AIW(AIW),
        .NUM_BANKS(NUM_BANKS),
        .BANK_DEPTH(BANK_DEPTH),
        .RD_LATENCY(RD_LATENCY),
        .WR_LATENCY(WR_LATENCY),
        .INIT_FILE(INIT_FILE)
    ) u_raw_mem (
        .clk(clk),
        .rst_n(rst_n),
        .mem_req(mem_req),
        .mem_ready(mem_ready),
        .mem_we(mem_we),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wmask(mem_wmask),
        .mem_source(mem_source),
        .mem_rvalid(mem_rvalid),
        .mem_rdata(mem_rdata),
        .mem_rsource(mem_rsource),
        .total_rd_count(total_rd_count),
        .total_wr_count(total_wr_count),
        .total_rd_latency_sum(total_rd_latency_sum)
    );

endmodule
