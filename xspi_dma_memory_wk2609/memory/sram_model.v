// ============================================================================
//
// Timing parameters: for just simulating real time stuff.
//   - tAA (Address Access Time): Configurable, default 4ns
//   - tCO (Clock to Output): 2ns typical
//   - Setup/Hold: Modeled via latency cycles
//
// At 500MHz (2ns period): 2-cycle read latency = tAA=4ns
// At 250MHz (4ns period): 1-cycle read latency = tAA=4ns
// ============================================================================

module sram_model #(
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
    output reg  [  DW-1:0] rdata,
    output reg             rvalid,
    output wire            ready,

    // Performance monitoring
    output reg [31:0] rd_count,
    output reg [31:0] wr_count,
    output reg [63:0] total_rd_latency,

    // Extended statistics
    output reg [31:0] min_rd_latency,
    output reg [31:0] max_rd_latency,
    output reg [31:0] rd_bandwidth_bytes,
    output reg [31:0] wr_bandwidth_bytes
);

  // ========================================================================
  // Memory Array
  // ========================================================================
  reg [DW-1:0] mem[0:DEPTH-1];
  integer i;

  // ========================================================================
  // Latency Pipeline
  // ========================================================================
  localparam PIPE_DEPTH = (RD_LATENCY > 4) ? RD_LATENCY : 4;

  reg [PIPE_DEPTH-1:0] rd_pipe_valid;
  reg [DW-1:0] rd_pipe_data[0:PIPE_DEPTH-1];
  reg [AW-1:0] rd_pipe_addr[0:PIPE_DEPTH-1];

  reg [31:0] rd_start_cycle[0:PIPE_DEPTH-1];
  reg [31:0] cycle_count;

  // Backpressure (for future)
  reg busy;
  assign ready = !busy;

  // ========================================================================
  // Memory Initialization
  // ========================================================================
  initial begin
    if (INIT_FILE != "") begin
      $readmemh(INIT_FILE, mem);
      $display("SRAM: Loaded %s into memory", INIT_FILE);
    end
  end

  // ========================================================================
  // Main Memory Access Logic
  // ========================================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rvalid <= 1'b0;
      rdata <= {DW{1'b0}};
      rd_pipe_valid <= {PIPE_DEPTH{1'b0}};
      rd_count <= 32'd0;
      wr_count <= 32'd0;
      total_rd_latency <= 64'd0;
      min_rd_latency <= 32'hFFFF_FFFF;
      max_rd_latency <= 32'd0;
      rd_bandwidth_bytes <= 32'd0;
      wr_bandwidth_bytes <= 32'd0;
      cycle_count <= 32'd0;
      busy <= 1'b0;
    end else begin
      cycle_count <= cycle_count + 1;
      rvalid <= 1'b0;

      // Shift pipeline
      rd_pipe_valid <= {1'b0, rd_pipe_valid[PIPE_DEPTH-1:1]};
      for (i = 0; i < PIPE_DEPTH - 1; i = i + 1) begin
        rd_pipe_data[i]   <= rd_pipe_data[i+1];
        rd_start_cycle[i] <= rd_start_cycle[i+1];
      end

      // Process new request
      if (req && ready) begin
        if (we) begin
          // Write operation
          for (i = 0; i < DW / 8; i = i + 1) begin
            if (wmask[i]) begin
              mem[addr][i*8+:8] <= wdata[i*8+:8];
            end
          end
          wr_count <= wr_count + 1;
          $display("[SRAM_WRITE][%0t] addr=0x%0h wdata=0x%016h wmask=0x%0h", $time, addr, wdata, wmask);

          // Count written bytes
          for (i = 0; i < DW / 8; i = i + 1) begin
            if (wmask[i]) begin
              wr_bandwidth_bytes <= wr_bandwidth_bytes + 1;
            end
          end
        end else begin
          // Read operation - enter pipeline
          rd_pipe_data[RD_LATENCY-1] <= mem[addr];
          rd_pipe_valid[RD_LATENCY-1] <= 1'b1;
          rd_start_cycle[RD_LATENCY-1] <= cycle_count;
          rd_count <= rd_count + 1;
          rd_bandwidth_bytes <= rd_bandwidth_bytes + (DW / 8);
          $display("[SRAM_READ_REQ][%0t] addr=0x%0h rdata_now=0x%016h", $time, addr, mem[addr]);
        end
      end

      // Pipeline output stage
      if (rd_pipe_valid[0]) begin
        rdata  <= rd_pipe_data[0];
        rvalid <= 1'b1;

        // Update latency statistics
        begin : latency_stats
          reg [31:0] this_latency;
          this_latency = cycle_count - rd_start_cycle[0];
          total_rd_latency <= total_rd_latency + {{32{1'b0}}, this_latency};
          if (this_latency < min_rd_latency) min_rd_latency <= this_latency;
          if (this_latency > max_rd_latency) max_rd_latency <= this_latency;
        end
      end
    end
  end

  // ========================================================================
  // Timing Parameter Display
  // ========================================================================
`ifndef SYNTHESIS
  initial begin
    $display("========================================");
    $display("SRAM Timing Model Configuration:");
    $display("  Depth:       %0d entries", DEPTH);
    $display("  Width:       %0d bits", DW);
    $display("  RD_LATENCY:  %0d cycles", RD_LATENCY);
    $display("  WR_LATENCY:  %0d cycles", WR_LATENCY);
    $display("  At 500MHz:   tAA = %0dns", RD_LATENCY * 2);
    $display("  At 250MHz:   tAA = %0dns", RD_LATENCY * 4);
    $display("========================================");
  end
`endif

endmodule
