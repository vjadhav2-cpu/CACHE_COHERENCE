`include "tl_sram_ctrl_pkg.vh"
`include "./bank_arbiter.v"
`include "./sram_bank.v"

module raw_sram_ctrl_top #(
    parameter AW         = 64,
    parameter DW         = 64,
    parameter AIW        = 3,
    parameter NUM_BANKS  = 4,
    parameter BANK_DEPTH = 262144,
    parameter RD_LATENCY = 1,
    parameter WR_LATENCY = 1,
    parameter INIT_FILE  = ""
) (
    input  wire              clk,
    input  wire              rst_n,

    // Raw memory request interface
    input  wire              mem_req,
    output wire              mem_ready,
    input  wire              mem_we,
    input  wire [AW-1:0]     mem_addr,
    input  wire [DW-1:0]     mem_wdata,
    input  wire [DW/8-1:0]   mem_wmask,
    input  wire [AIW-1:0]    mem_source,
    output wire              mem_rvalid,
    output wire [DW-1:0]     mem_rdata,
    output wire [AIW-1:0]    mem_rsource,

    output wire [31:0]       total_rd_count,
    output wire [31:0]       total_wr_count,
    output wire [63:0]       total_rd_latency_sum
);

    localparam BANK_AW = $clog2(BANK_DEPTH);

    wire [NUM_BANKS-1:0]      bank_req;
    wire [NUM_BANKS-1:0]      bank_we;
    wire [NUM_BANKS*BANK_AW-1:0] bank_addr_flat;
    wire [NUM_BANKS*DW-1:0]   bank_wdata_flat;
    wire [NUM_BANKS*(DW/8)-1:0] bank_wmask_flat;
    wire [NUM_BANKS*DW-1:0]   bank_rdata_flat;
    wire [NUM_BANKS-1:0]      bank_rvalid;
    wire [NUM_BANKS-1:0]      bank_ready;

    wire [NUM_BANKS*32-1:0]   bank_rd_count;
    wire [NUM_BANKS*32-1:0]   bank_wr_count;
    wire [NUM_BANKS*64-1:0]   bank_rd_latency;

    reg  [31:0]               rd_count_sum;
    reg  [31:0]               wr_count_sum;
    reg  [63:0]               latency_sum;
    integer                   j;

    always @(*) begin
        rd_count_sum = 32'd0;
        wr_count_sum = 32'd0;
        latency_sum  = 64'd0;
        for (j = 0; j < NUM_BANKS; j = j + 1) begin
            rd_count_sum = rd_count_sum + bank_rd_count[j*32 +: 32];
            wr_count_sum = wr_count_sum + bank_wr_count[j*32 +: 32];
            latency_sum  = latency_sum + bank_rd_latency[j*64 +: 64];
        end
    end

    assign total_rd_count       = rd_count_sum;
    assign total_wr_count       = wr_count_sum;
    assign total_rd_latency_sum = latency_sum;

    bank_arbiter #(
        .NUM_BANKS(NUM_BANKS),
        .AW(AW),
        .DW(DW),
        .BANK_AW(BANK_AW)
    ) u_arbiter (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(mem_req),
        .req_ready(mem_ready),
        .req_we(mem_we),
        .req_addr(mem_addr),
        .req_wdata(mem_wdata),
        .req_wmask(mem_wmask),
        .req_source(mem_source),
        .resp_valid(mem_rvalid),
        .resp_rdata(mem_rdata),
        .resp_source(mem_rsource),
        .bank_req(bank_req),
        .bank_we(bank_we),
        .bank_addr(bank_addr_flat),
        .bank_wdata(bank_wdata_flat),
        .bank_wmask(bank_wmask_flat),
        .bank_rdata(bank_rdata_flat),
        .bank_rvalid(bank_rvalid),
        .bank_ready(bank_ready)
    );

    genvar i;
    generate
        for (i = 0; i < NUM_BANKS; i = i + 1) begin : gen_banks
            sram_bank #(
                .DEPTH(BANK_DEPTH),
                .DW(DW),
                .AW(BANK_AW),
                .RD_LATENCY(RD_LATENCY),
                .WR_LATENCY(WR_LATENCY),
                .INIT_FILE(INIT_FILE)
            ) u_bank (
                .clk(clk),
                .rst_n(rst_n),
                .req(bank_req[i]),
                .we(bank_we[i]),
                .addr(bank_addr_flat[i*BANK_AW +: BANK_AW]),
                .wdata(bank_wdata_flat[i*DW +: DW]),
                .wmask(bank_wmask_flat[i*(DW/8) +: (DW/8)]),
                .rdata(bank_rdata_flat[i*DW +: DW]),
                .rvalid(bank_rvalid[i]),
                .ready(bank_ready[i]),
                .rd_count(bank_rd_count[i*32 +: 32]),
                .wr_count(bank_wr_count[i*32 +: 32]),
                .total_rd_latency(bank_rd_latency[i*64 +: 64])
            );
        end
    endgenerate

endmodule
