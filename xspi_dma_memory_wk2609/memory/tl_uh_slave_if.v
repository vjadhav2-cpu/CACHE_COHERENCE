`include "tl_sram_ctrl_pkg.vh"

module tl_uh_slave_if #(
    parameter AW = 64,
    parameter DW = 64,
    parameter AIW = 4,
    parameter SZW = 8,
    parameter MAX_BURST = 16  // Support up to 16 beats (128 bytes)
) (
    input wire clk,
    input wire rst_n,

    input  wire            tl_a_valid,
    output wire            tl_a_ready,
    input  wire [     2:0] tl_a_opcode,
    input  wire [     2:0] tl_a_param,
    input  wire [ SZW-1:0] tl_a_size,
    input  wire [ AIW-1:0] tl_a_source,
    input  wire [  AW-1:0] tl_a_address,
    input  wire [DW/8-1:0] tl_a_mask,
    input  wire [  DW-1:0] tl_a_data,
    input  wire            tl_a_corrupt,

    output reg            tl_d_valid,
    input  wire           tl_d_ready,
    output reg  [    2:0] tl_d_opcode,
    output reg  [    1:0] tl_d_param,
    output reg  [SZW-1:0] tl_d_size,
    output reg  [AIW-1:0] tl_d_source,
    output wire           tl_d_sink,
    output reg  [ DW-1:0] tl_d_data,
    output reg            tl_d_denied,
    output reg            tl_d_corrupt,

    output reg             mem_req,
    output reg             mem_we,
    output reg  [  AW-1:0] mem_addr,
    output reg  [  DW-1:0] mem_wdata,
    output reg  [DW/8-1:0] mem_wmask,
    input  wire [  DW-1:0] mem_rdata,
    input  wire            mem_rvalid,
    input  wire            mem_ready
);

  assign tl_d_sink = 1'b0;

  localparam [3:0] ST_IDLE = 4'd0;
  localparam [3:0] ST_READ = 4'd1;
  localparam [3:0] ST_WRITE = 4'd2;
  localparam [3:0] ST_ATOMIC_RD = 4'd3;
  localparam [3:0] ST_ATOMIC_WR = 4'd4;
  localparam [3:0] ST_RESPOND = 4'd5;
  localparam [3:0] ST_BURST_RD = 4'd6;
  localparam [3:0] ST_BURST_RD_RSP = 4'd7;
  localparam [3:0] ST_BURST_WR = 4'd8;
  localparam [3:0] ST_BURST_WR_RSP = 4'd9;

  reg  [     3:0] state;
  reg  [     2:0] saved_opcode;
  reg  [     2:0] saved_param;
  reg  [ SZW-1:0] saved_size;
  reg  [ AIW-1:0] saved_source;
  reg  [  AW-1:0] saved_address;
  reg  [  DW-1:0] saved_wdata;
  reg  [DW/8-1:0] saved_mask;
  reg  [  DW-1:0] atomic_orig;
  reg  [     4:0] burst_count;  // 5-bit for internal counting
  reg  [     4:0] burst_remaining;  // 5-bit for internal counting
  reg  [     4:0] burst_total;  // 5-bit for internal counting

  reg  [  DW-1:0] burst_buffer                                    [0:MAX_BURST-1];
  reg  [     3:0] burst_wr_idx;  // 4-bit for array index [0:15]
  reg  [     3:0] burst_rd_idx;  // 4-bit for array index [0:15]

  wire [  DW-1:0] atomic_result;
  wire [  DW-1:0] atomic_original;

  atomic_unit #(
      .DW(DW)
  ) u_atomic (
      .opcode(saved_opcode),
      .param(saved_param),
      .operand_a(mem_rdata),
      .operand_b(saved_wdata),
      .result(atomic_result),
      .original(atomic_original)
  );

  wire is_read = (tl_a_opcode == TL_A_GET);
  wire is_write = (tl_a_opcode == TL_A_PUT_FULL) || (tl_a_opcode == TL_A_PUT_PARTIAL);
  wire is_atomic = (tl_a_opcode == TL_A_ARITHMETIC) || (tl_a_opcode == TL_A_LOGICAL);
  wire is_hint = (tl_a_opcode == TL_A_INTENT);

  localparam [SZW-1:0] DATA_SIZE_LOG = 3;
  wire needs_burst = (tl_a_size > DATA_SIZE_LOG);
  wire [4:0] num_beats = needs_burst ? (5'd1 << (tl_a_size - DATA_SIZE_LOG)) : 5'd1;

  wire can_accept = (state == ST_IDLE) && mem_ready;
  wire can_accept_burst = (state == ST_BURST_WR) && ({1'b0, burst_wr_idx} < burst_total);
  assign tl_a_ready = can_accept || can_accept_burst;

  integer i;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state <= ST_IDLE;
      tl_d_valid <= 1'b0;
      tl_d_opcode <= 3'd0;
      tl_d_param <= 2'd0;
      tl_d_size <= {SZW{1'b0}};
      tl_d_source <= {AIW{1'b0}};
      tl_d_data <= {DW{1'b0}};
      tl_d_denied <= 1'b0;
      tl_d_corrupt <= 1'b0;
      mem_req <= 1'b0;
      mem_we <= 1'b0;
      mem_addr <= {AW{1'b0}};
      mem_wdata <= {DW{1'b0}};
      mem_wmask <= {(DW / 8) {1'b0}};
      saved_opcode <= 3'd0;
      saved_param <= 3'd0;
      saved_size <= {SZW{1'b0}};
      saved_source <= {AIW{1'b0}};
      saved_address <= {AW{1'b0}};
      saved_wdata <= {DW{1'b0}};
      saved_mask <= {(DW / 8) {1'b0}};
      atomic_orig <= {DW{1'b0}};
      burst_count <= 5'd0;
      burst_remaining <= 5'd0;
      burst_total <= 5'd0;
      burst_wr_idx <= 4'd0;
      burst_rd_idx <= 4'd0;
      for (i = 0; i < MAX_BURST; i = i + 1) begin
        burst_buffer[i] <= {DW{1'b0}};
      end
    end else begin
      mem_req <= 1'b0;

      if (tl_d_valid && tl_d_ready) begin
        tl_d_valid <= 1'b0;
      end

      case (state)
        ST_IDLE: begin
          if (tl_a_valid && can_accept) begin
            saved_opcode <= tl_a_opcode;
            saved_param <= tl_a_param;
            saved_size <= tl_a_size;
            saved_source <= tl_a_source;
            saved_address <= tl_a_address;
            saved_wdata <= tl_a_data;
            saved_mask <= tl_a_mask;
            burst_total <= num_beats;
            burst_count <= 5'd0;
            burst_wr_idx <= 4'd0;
            burst_rd_idx <= 4'd0;

            if (is_hint) begin
              tl_d_valid <= 1'b1;
              tl_d_opcode <= TL_D_HINT_ACK;
              tl_d_param <= 2'd0;
              tl_d_size <= tl_a_size;
              tl_d_source <= tl_a_source;
              tl_d_data <= {DW{1'b0}};
              tl_d_denied <= 1'b0;
              tl_d_corrupt <= 1'b0;
            end else if (is_read) begin
              mem_req  <= 1'b1;
              mem_we   <= 1'b0;
              mem_addr <= tl_a_address;
              if (needs_burst) begin
                burst_remaining <= num_beats;
                state <= ST_BURST_RD;
              end else begin
                state <= ST_READ;
              end
            end else if (is_write) begin
              if (needs_burst) begin
                burst_buffer[0] <= tl_a_data;
                burst_wr_idx <= 4'd1;
                burst_remaining <= num_beats - 5'd1;
                state <= ST_BURST_WR;
              end else begin
                mem_req <= 1'b1;
                mem_we <= 1'b1;
                mem_addr <= tl_a_address;
                mem_wdata <= tl_a_data;
                mem_wmask <= tl_a_mask;
                state <= ST_WRITE;
              end
            end else if (is_atomic) begin
              mem_req <= 1'b1;
              mem_we <= 1'b0;
              mem_addr <= tl_a_address;
              state <= ST_ATOMIC_RD;
            end
          end
        end

        ST_BURST_RD: begin
          if (mem_rvalid) begin
            burst_buffer[burst_count[3:0]] <= mem_rdata;
            burst_count <= burst_count + 5'd1;
            burst_remaining <= burst_remaining - 5'd1;
            if (burst_remaining == 5'd1) begin
              burst_rd_idx <= 4'd0;
              state <= ST_BURST_RD_RSP;
            end else begin
              mem_req  <= 1'b1;
              mem_we   <= 1'b0;
              mem_addr <= saved_address + ({{(AW - 8) {1'b0}}, burst_count + 5'd1, 3'b0});
            end
          end
        end

        ST_BURST_RD_RSP: begin
          if (!tl_d_valid || tl_d_ready) begin
            tl_d_valid <= 1'b1;
            tl_d_opcode <= TL_D_ACCESS_ACK_DATA;
            tl_d_param <= 2'd0;
            tl_d_size <= saved_size;
            tl_d_source <= saved_source;
            tl_d_data <= burst_buffer[burst_rd_idx];
            tl_d_denied <= 1'b0;
            tl_d_corrupt <= 1'b0;
            burst_rd_idx <= burst_rd_idx + 4'd1;
            if (burst_rd_idx == burst_total[3:0] - 4'd1) begin
              state <= ST_RESPOND;
            end
          end
        end

        ST_BURST_WR: begin
          if (tl_a_valid && can_accept_burst) begin
            burst_buffer[burst_wr_idx] <= tl_a_data;
            burst_wr_idx <= burst_wr_idx + 4'd1;
            burst_remaining <= burst_remaining - 5'd1;
            if (burst_remaining == 5'd1) begin
              burst_count <= 5'd0;
              state <= ST_BURST_WR_RSP;
            end
          end
        end

        ST_BURST_WR_RSP: begin
          if (mem_ready) begin
            mem_req <= 1'b1;
            mem_we <= 1'b1;
            mem_addr <= saved_address + ({{(AW - 8) {1'b0}}, burst_count, 3'b0});
            mem_wdata <= burst_buffer[burst_count[3:0]];
            mem_wmask <= saved_mask;
            burst_count <= burst_count + 5'd1;
            if (burst_count == burst_total - 5'd1) begin
              tl_d_valid <= 1'b1;
              tl_d_opcode <= TL_D_ACCESS_ACK;
              tl_d_param <= 2'd0;
              tl_d_size <= saved_size;
              tl_d_source <= saved_source;
              tl_d_data <= {DW{1'b0}};
              tl_d_denied <= 1'b0;
              tl_d_corrupt <= 1'b0;
              state <= ST_RESPOND;
            end
          end
        end

        ST_READ: begin
          if (mem_rvalid) begin
            tl_d_valid <= 1'b1;
            tl_d_opcode <= TL_D_ACCESS_ACK_DATA;
            tl_d_param <= 2'd0;
            tl_d_size <= saved_size;
            tl_d_source <= saved_source;
            tl_d_data <= mem_rdata;
            tl_d_denied <= 1'b0;
            tl_d_corrupt <= 1'b0;
            state <= ST_RESPOND;
          end
        end

        ST_WRITE: begin
          tl_d_valid <= 1'b1;
          tl_d_opcode <= TL_D_ACCESS_ACK;
          tl_d_param <= 2'd0;
          tl_d_size <= saved_size;
          tl_d_source <= saved_source;
          tl_d_data <= {DW{1'b0}};
          tl_d_denied <= 1'b0;
          tl_d_corrupt <= 1'b0;
          state <= ST_RESPOND;
        end

        ST_ATOMIC_RD: begin
          if (mem_rvalid) begin
            atomic_orig <= mem_rdata;
            mem_req <= 1'b1;
            mem_we <= 1'b1;
            mem_addr <= saved_address;
            mem_wdata <= atomic_result;
            mem_wmask <= saved_mask;
            state <= ST_ATOMIC_WR;
          end
        end

        ST_ATOMIC_WR: begin
          tl_d_valid <= 1'b1;
          tl_d_opcode <= TL_D_ACCESS_ACK_DATA;
          tl_d_param <= 2'd0;
          tl_d_size <= saved_size;
          tl_d_source <= saved_source;
          tl_d_data <= atomic_orig;
          tl_d_denied <= 1'b0;
          tl_d_corrupt <= 1'b0;
          state <= ST_RESPOND;
        end

        ST_RESPOND: begin
          if (tl_d_ready) begin
            tl_d_valid <= 1'b0;
            state <= ST_IDLE;
          end
        end

        default: begin
          state <= ST_IDLE;
        end
      endcase
    end
  end

endmodule
