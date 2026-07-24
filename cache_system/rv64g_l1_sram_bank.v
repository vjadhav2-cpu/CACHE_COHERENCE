// rv64g_l1_sram_bank.v - Single SRAM bank for banked L1 cache
// Part of Phase 1: Modularization for RVA23 Vector support
//
// Each bank holds 1/8 of the cache data:
//   - 2KB per bank (16KB total / 8 banks)
//   - 64-bit width (one element)
//   - 256 rows = 32 sets × 8 words/line
//
// Element-interleaved: bank selected by vector element index, not address bits
// Single R/W port with byte enables

`timescale 1ns/1ps
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off BLKSEQ */

module rv64g_l1_sram_bank #(
    parameter integer SETS          = 32,
    parameter integer WAYS          = 8,
    parameter integer WORDS_PER_LINE = 8,    // 64 bytes / 8 bytes
    parameter integer TAG_W         = 53,
    parameter integer INDEX_W       = 5
) (
    input               clk_i,
    input               rst_ni,

    // Invalidate all entries (flush)
    input               invalidate_all_i,

    // Access request
    input               req_i,          // Request valid
    input               we_i,           // Data write enable (0=read, 1=write)
    input               tag_we_i,       // Tag/state write enable (can be separate from data)
    input  [INDEX_W-1:0] index_i,       // Set index (5 bits -> 32 sets)
    input  [2:0]        word_i,         // Word within line (3 bits -> 8 words)
    input  [2:0]        way_i,          // Way select for write (3 bits -> 8 ways)
    input  [7:0]        be_i,           // Byte enables for 64-bit word
    input  [63:0]       wdata_i,        // Write data
    input  [TAG_W-1:0]  tag_i,          // Tag to write
    input  [1:0]        state_i,        // MESI state to write

    // Per-way parallel outputs (for tag comparison)
    output [WAYS*64-1:0]    rdata_way_o,    // All ways data for this index/word
    output [WAYS*TAG_W-1:0] tag_way_o,      // All ways tags for this index
    output [WAYS*2-1:0]     state_way_o,    // All ways state for this index

    // Selected way output (post-hit determination)
    output [63:0]       rdata_sel_o,    // Data from selected way
    output [TAG_W-1:0]  tag_sel_o,      // Tag from selected way
    output [1:0]        state_sel_o     // State from selected way
);

    // =========================================================================
    // Storage Arrays
    // =========================================================================
    // Data: [way][{index,word}] - flattened for Verilog-2001
    // Each way has SETS*WORDS_PER_LINE = 32*8 = 256 entries of 64 bits
    localparam integer DATA_DEPTH = SETS * WORDS_PER_LINE;
    localparam integer LINE_ADDR_W = INDEX_W + 3;  // index + word offset

    reg [63:0]       data_q  [0:WAYS-1][0:DATA_DEPTH-1];
    reg [TAG_W-1:0]  tag_q   [0:WAYS-1][0:SETS-1];
    reg [1:0]        state_q [0:WAYS-1][0:SETS-1];

    // =========================================================================
    // Address Computation
    // =========================================================================
    wire [LINE_ADDR_W-1:0] line_addr;
    assign line_addr = {index_i, word_i};

    // =========================================================================
    // Read Path - Combinational parallel per-way output
    // =========================================================================
    genvar w;
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_way_out
            assign rdata_way_o[(w+1)*64-1    : w*64]    = data_q[w][line_addr];
            assign tag_way_o[(w+1)*TAG_W-1   : w*TAG_W] = tag_q[w][index_i];
            assign state_way_o[(w+1)*2-1     : w*2]     = state_q[w][index_i];
        end
    endgenerate

    // Selected way outputs
    assign rdata_sel_o = data_q[way_i][line_addr];
    assign tag_sel_o   = tag_q[way_i][index_i];
    assign state_sel_o = state_q[way_i][index_i];

    // =========================================================================
    // Write Path - Synchronous with byte enables
    // =========================================================================
    reg [63:0] be_mask;
    integer b, i, j;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            // Reset state to Invalid; data/tag undefined per spec
            for (i = 0; i < WAYS; i = i + 1) begin
                for (j = 0; j < SETS; j = j + 1) begin
                    state_q[i][j] <= MESI_N;
                end
            end
        end else if (invalidate_all_i) begin
            // Flush: invalidate all lines
            for (i = 0; i < WAYS; i = i + 1) begin
                for (j = 0; j < SETS; j = j + 1) begin
                    state_q[i][j] <= MESI_N;
                end
            end
        end else if (req_i && (we_i || tag_we_i)) begin
            if (we_i) begin
                // Compute byte-enable mask
                be_mask = 64'b0;
                for (b = 0; b < 8; b = b + 1) begin
                    if (be_i[b]) be_mask[(b*8) +: 8] = 8'hFF;
                end

                // Byte-masked write to data array
                data_q[way_i][line_addr] <= (wdata_i & be_mask) |
                                            (data_q[way_i][line_addr] & ~be_mask);
            end
            if (tag_we_i) begin
                // Tag and state update
                tag_q[way_i][index_i]   <= tag_i;
                state_q[way_i][index_i] <= state_i;
            end
        end
    end

endmodule
