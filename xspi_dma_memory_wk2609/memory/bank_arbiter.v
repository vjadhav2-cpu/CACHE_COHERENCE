`include "tl_sram_ctrl_pkg.vh"

module bank_arbiter #(
    parameter NUM_BANKS = 4,
    parameter AW = 64,
    parameter DW = 64,
    parameter BANK_AW = 16
) (
    input wire clk,
    input wire rst_n,

    input  wire            req_valid,
    output wire            req_ready,
    input  wire            req_we,
    input  wire [  AW-1:0] req_addr,
    input  wire [  DW-1:0] req_wdata,
    input  wire [DW/8-1:0] req_wmask,
    input  wire [     3:0] req_source,

    output reg          resp_valid,
    output reg [DW-1:0] resp_rdata,
    output reg [   3:0] resp_source,

    output wire [        NUM_BANKS-1:0] bank_req,
    output wire [        NUM_BANKS-1:0] bank_we,
    output wire [NUM_BANKS*BANK_AW-1:0] bank_addr,
    output wire [     NUM_BANKS*DW-1:0] bank_wdata,
    output wire [ NUM_BANKS*(DW/8)-1:0] bank_wmask,
    input  wire [     NUM_BANKS*DW-1:0] bank_rdata,
    input  wire [        NUM_BANKS-1:0] bank_rvalid,
    input  wire [        NUM_BANKS-1:0] bank_ready
);

  // Calculateing bank selection bits dynamically
  // Address format: [...] | [bank_sel] | [bank_offset] | [byte_offset]
  // byte_offset: bits [2:0] for 8-byte alignment (log2(DW/8) = log2(8) = 3)
  // bank_offset: BANK_AW bits for addressing within bank
  // bank_sel: log2(NUM_BANKS) bits for bank selection
  localparam BYTE_OFFSET_BITS = $clog2(DW / 8);  // 3 for 64-bit data
  localparam BANK_SEL_WIDTH = $clog2(NUM_BANKS);  // 2 for 4 banks
  localparam BANK_SEL_LO_CALC = BYTE_OFFSET_BITS + BANK_AW;  // 3 + BANK_AW
  localparam BANK_SEL_HI_CALC = BANK_SEL_LO_CALC + BANK_SEL_WIDTH - 1;

  wire [BANK_SEL_WIDTH-1:0] bank_sel = req_addr[BANK_SEL_HI_CALC:BANK_SEL_LO_CALC];
  wire [BANK_AW-1:0] bank_offset = req_addr[BANK_AW-1+BYTE_OFFSET_BITS:BYTE_OFFSET_BITS];

  reg req_pending;
  reg [1:0] pending_bank;
  reg [3:0] pending_source;

  wire selected_bank_ready = bank_ready[bank_sel];
  assign req_ready = !req_pending && selected_bank_ready;

  genvar b;
  generate
    for (b = 0; b < NUM_BANKS; b = b + 1) begin : gen_bank_signals
      assign bank_req[b] = req_valid && req_ready && (bank_sel == b[1:0]);
      assign bank_we[b] = req_we;
      assign bank_addr[b*BANK_AW+:BANK_AW] = bank_offset;
      assign bank_wdata[b*DW+:DW] = req_wdata;
      assign bank_wmask[b*(DW/8)+:(DW/8)] = req_wmask;
    end
  endgenerate

  wire [DW-1:0] pending_bank_rdata = bank_rdata[pending_bank*DW+:DW];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      req_pending <= 1'b0;
      pending_bank <= 2'd0;
      pending_source <= 4'd0;
      resp_valid <= 1'b0;
      resp_rdata <= {DW{1'b0}};
      resp_source <= 4'd0;
    end else begin
      resp_valid <= 1'b0;

      if (req_valid && req_ready && !req_we) begin
        req_pending <= 1'b1;
        pending_bank <= bank_sel;
        pending_source <= req_source;
      end

      if (req_pending && bank_rvalid[pending_bank]) begin
        resp_valid  <= 1'b1;
        resp_rdata  <= pending_bank_rdata;
        resp_source <= pending_source;
        req_pending <= 1'b0;
      end

      if (req_valid && req_ready && req_we) begin
        resp_valid  <= 1'b1;
        resp_rdata  <= {DW{1'b0}};
        resp_source <= req_source;
      end
    end
  end

endmodule
