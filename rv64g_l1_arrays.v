// rv64g_l1_arrays.v - Data/tag/state arrays for L1 cache
`timescale 1ns/1ps
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */

module rv64g_l1_arrays #(
    parameter integer SETS = 32,
    parameter integer WAYS = 8,
    parameter integer TAG_W = 53, // 64 - 5(index) - 6(offset)
    parameter integer INDEX_W = 5
) (
	input               clk_i,
	input               rst_ni,
	input               invalidate_all_i,

	// Access controls
	input       [INDEX_W-1:0] index_i,       // set index
	input       [2:0]   word_sel_i,    // word within line (8 words)
	input       [2:0]   way_sel_i,     // selected way for write
	input               write_en_i,    // write enable for selected way/word
	input       [1:0]   state_i,       // state to write (N, B, T, TT)
	input       [7:0]   be_i,          // byte enables for 64-bit word
	input      [TAG_W-1:0]   tag_in_i,
	input      [63:0]   wdata_i,

	// Parallel per-way outputs for current index/word
	output     [63:0]   rdata_selected_o,
	output     [TAG_W-1:0]   tag_selected_o,
	output     [1:0]    state_selected_o,

	output [WAYS*64-1:0]   rdata_way_flat_o,
	output [WAYS*TAG_W-1:0]   tag_way_flat_o,
	output [WAYS*2-1:0]    state_way_flat_o
);

	localparam integer DATA_W          = 64;
	localparam integer WORDS_PER_LINE  = LINE_BYTES / 8;
	localparam integer LINE_ADDR_W     = INDEX_W + 3;  // index + 3 (word)

	// Storage arrays (flattened to 2D for Verilog-2001)
	// data_q[way][{index,word}]
	reg [DATA_W-1:0] data_q [0:WAYS-1][0:SETS*WORDS_PER_LINE-1];
	reg [TAG_W-1:0]  tag_q  [0:WAYS-1][0:SETS-1];
	reg [1:0]        state_q[0:WAYS-1][0:SETS-1];

	// Computed linear word index within a set group
	wire [LINE_ADDR_W-1:0] line_idx;
	assign line_idx = {index_i, word_sel_i};

	// Selected outputs
	assign rdata_selected_o = data_q[way_sel_i][line_idx];
	assign tag_selected_o   = tag_q[way_sel_i][index_i];
	assign state_selected_o = state_q[way_sel_i][index_i];

	// Flattened per-way outputs for current index/word
	genvar w;
	generate
		for (w = 0; w < WAYS; w = w + 1) begin : g_flat
			assign rdata_way_flat_o[(w+1)*DATA_W-1 : w*DATA_W] = data_q[w][line_idx];
			assign tag_way_flat_o[(w+1)*TAG_W-1    : w*TAG_W]  = tag_q[w][index_i];
			assign state_way_flat_o[(w+1)*2-1      : w*2]      = state_q[w][index_i];
		end
	endgenerate
    reg [63:0] be_mask;
    integer b;
	integer i, j;
	always @(posedge clk_i or negedge rst_ni) begin
		if (!rst_ni) begin
			// Reset only state; do not reset data/tag per spec
			for (i = 0; i < WAYS; i = i + 1) begin
				for (j = 0; j < SETS; j = j + 1) begin
					state_q[i][j] <= MESI_N;
				end
			end
		end else if (invalidate_all_i) begin
			for (i = 0; i < WAYS; i = i + 1) begin
				for (j = 0; j < SETS; j = j + 1) begin
					state_q[i][j] <= MESI_N;
				end
			end
		end else if (write_en_i) begin
			// Byte-enable aware RMW for 64b word (Verilog-2001 style part-select)
			be_mask = 64'b0;
			for (b = 0; b < 8; b = b + 1) begin
				if (be_i[b]) be_mask[(b*8) +: 8] = 8'hFF;
			end
			data_q[way_sel_i][line_idx] <= (wdata_i & be_mask) |
				(data_q[way_sel_i][line_idx] & ~be_mask);
			tag_q[way_sel_i][index_i]   <= tag_in_i;
			state_q[way_sel_i][index_i] <= state_i;
		end
	end

endmodule


