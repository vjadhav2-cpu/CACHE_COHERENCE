
`timescale 1ns/1ps
// rv64g_l2_arrays.v - Data/tag arrays for 256KiB, 16-way, 64B lines
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */

module rv64g_l2_arrays (
	input               clk_i,
	input               rst_ni,

	// Access controls
	input       [7:0]   index_i,       // set index (256 sets)
	input       [2:0]   word_sel_i,    // word within line (8 words)
	input       [3:0]   way_sel_i,     // selected way for write (0..15)
	input               data_we_i,     // write enable for data
	input               tag_we_i,      // write enable for tag
	input       [7:0]   be_i,          // byte enables for 64-bit word
	input      [49:0]   tag_in_i,
	input      [63:0]   wdata_i,

	// Parallel per-way outputs for current index/word
	output     [63:0]   rdata_selected_o,
	output     [49:0]   tag_selected_o,

	output [16*64-1:0]  rdata_way_flat_o,
	output [16*50-1:0]  tag_way_flat_o
);

	localparam integer DATA_W          = 64;
	localparam integer TAG_W           = 50;
	localparam integer WORDS_PER_LINE  = 8;   // 64B / 8B
	localparam integer WAYS            = 16;
	localparam integer SETS            = 256; // 256KiB / (64B * 16 ways) = 256 sets
	localparam integer LINE_ADDR_W     = 11;  // 8 (index) + 3 (word)

	// Storage arrays (flattened to 2D for Verilog-2001)
	// data_q[way][{index,word}] where {index,word} is 11-bit (0..2047)
	reg [DATA_W-1:0] data_q [0:WAYS-1][0:SETS*WORDS_PER_LINE-1];
	reg [TAG_W-1:0]  tag_q  [0:WAYS-1][0:SETS-1];

	// Computed linear word index within a set group
	wire [LINE_ADDR_W-1:0] line_idx;
	assign line_idx = {index_i, word_sel_i};

	// Selected outputs
	assign rdata_selected_o = data_q[way_sel_i][line_idx];
	assign tag_selected_o   = tag_q[way_sel_i][index_i];

	// Flattened per-way outputs for current index/word
	genvar w;
	generate
		for (w = 0; w < WAYS; w = w + 1) begin : g_flat
			assign rdata_way_flat_o[(w+1)*DATA_W-1 : w*DATA_W] = data_q[w][line_idx];
			assign tag_way_flat_o[(w+1)*TAG_W-1    : w*TAG_W]  = tag_q[w][index_i];
		end
	endgenerate
    reg [63:0] be_mask;
	integer b;
	always @(posedge clk_i) begin
		if (data_we_i) begin
			// Byte-enable aware RMW for 64b word
			
			be_mask = 64'b0;
			for (b = 0; b < 8; b = b + 1) begin
				if (be_i[b]) be_mask[(b*8) +: 8] = 8'hFF;
			end
			data_q[way_sel_i][line_idx] <= (wdata_i & be_mask) |
				(data_q[way_sel_i][line_idx] & ~be_mask);
		end
		if (tag_we_i) begin
			tag_q[way_sel_i][index_i]   <= tag_in_i;
		end
	end

endmodule



