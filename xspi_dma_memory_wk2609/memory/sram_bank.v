module sram_bank #(
    parameter DEPTH      = 65536,
    parameter DW         = 64,
    parameter AW         = 16,
    parameter RD_LATENCY = 2,
    parameter WR_LATENCY = 1,
    parameter INIT_FILE  = ""
) (
    input  wire            clk,
    input  wire            rst_n,
    input  wire            req,
    input  wire            we,
    input  wire [  AW-1:0] addr,
    input  wire [  DW-1:0] wdata,
    input  wire [DW/8-1:0] wmask,
    output wire [  DW-1:0] rdata,
    output wire            rvalid,
    output wire            ready,
    output wire [    31:0] rd_count,
    output wire [    31:0] wr_count,
    output wire [    63:0] total_rd_latency
);

  // Extended statistics (optional, lft unconnected at top)
  wire [31:0] min_rd_latency;
  wire [31:0] max_rd_latency;
  wire [31:0] rd_bandwidth_bytes;
  wire [31:0] wr_bandwidth_bytes;

  sram_model #(
      .DEPTH(DEPTH),
      .DW(DW),
      .AW(AW),
      .RD_LATENCY(RD_LATENCY),
      .WR_LATENCY(WR_LATENCY),
      .INIT_FILE(INIT_FILE)
  ) u_mem (
      .clk(clk),
      .rst_n(rst_n),
      .req(req),
      .we(we),
      .addr(addr),
      .wdata(wdata),
      .wmask(wmask),
      .rdata(rdata),
      .rvalid(rvalid),
      .ready(ready),
      .rd_count(rd_count),
      .wr_count(wr_count),
      .total_rd_latency(total_rd_latency),
      .min_rd_latency(min_rd_latency),
      .max_rd_latency(max_rd_latency),
      .rd_bandwidth_bytes(rd_bandwidth_bytes),
      .wr_bandwidth_bytes(wr_bandwidth_bytes)
  );

endmodule
