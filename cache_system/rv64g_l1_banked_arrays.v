// rv64g_l1_banked_arrays.v - Banked L1 cache arrays with scalar/vector ports
// Combines Phase 1-3: 8 SRAM banks + arbiters + crossbar
//
// Provides:
//   - Port A: Scalar LSU (1 request/cycle, always wins on conflict)
//   - Port B: Vector LSU (up to 8 requests/cycle, may stall on conflicts)
//
// Banking: Element-interleaved for RVV semantics
//   - Bank select = addr[5:3] (word within 64B cache line)
//   - 8 banks × 2KB each = 16KB total

`timescale 1ns/1ps
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off PINCONNECTEMPTY */
/* verilator lint_off UNOPTFLAT */

module rv64g_l1_banked_arrays #(
    parameter integer SETS   = 32,
    parameter integer WAYS   = 8,
    parameter integer TAG_W  = 53,
    parameter integer INDEX_W = 5,
    parameter integer NUM_BANKS = 8,
    parameter integer NUM_LANES = 8
) (
    input               clk_i,
    input               rst_ni,
    input               invalidate_all_i,

    // =========================================================================
    // Scalar Port (Port A) - Single request interface
    // =========================================================================
    input               scalar_req_i,
    input               scalar_we_i,
    input               scalar_tag_we_i,     // Broadcast tag/state write to ALL banks
    input               scalar_tag_broadcast_i, // Broadcast tag/state to ALL banks
    input  [INDEX_W-1:0] scalar_index_i,
    input  [2:0]        scalar_word_i,       // Word within line (also bank select)
    input  [2:0]        scalar_way_i,
    input  [7:0]        scalar_be_i,
    input  [63:0]       scalar_wdata_i,
    input  [TAG_W-1:0]  scalar_tag_i,
    input  [1:0]        scalar_state_i,

    // Scalar outputs (from targeted bank)
    output [63:0]       scalar_rdata_o,
    output [TAG_W-1:0]  scalar_tag_o,
    output [1:0]        scalar_state_o,
    output [WAYS*64-1:0]   scalar_rdata_way_o,  // All ways for hit detection
    output [WAYS*TAG_W-1:0] scalar_tag_way_o,
    output [WAYS*2-1:0]    scalar_state_way_o,

    // =========================================================================
    // Vector Port (Port B) - Multi-request interface via crossbar
    // =========================================================================
    input               vlsu_req_i,
    input  [NUM_LANES-1:0] vlsu_lane_valid_i,
    input  [NUM_LANES-1:0] vlsu_lane_we_i,
    input  [NUM_LANES*64-1:0] vlsu_lane_addr_i,
    input  [NUM_LANES*64-1:0] vlsu_lane_wdata_i,
    input  [NUM_LANES*8-1:0]  vlsu_lane_be_i,
    input  [NUM_LANES*3-1:0]  vlsu_lane_way_i,
    input  [NUM_LANES*TAG_W-1:0] vlsu_lane_tag_i,
    input  [NUM_LANES*2-1:0]  vlsu_lane_state_i,

    // Vector outputs
    output              vlsu_ready_o,
    output              vlsu_done_o,
    output [NUM_LANES-1:0] vlsu_lane_done_o,

    // Per-bank vector read data (routed back via bank_src_lane)
    output [NUM_BANKS*64-1:0]    vec_bank_rdata_o,
    output [NUM_BANKS*WAYS*64-1:0] vec_bank_rdata_way_o,
    output [NUM_BANKS*WAYS*TAG_W-1:0] vec_bank_tag_way_o,
    output [NUM_BANKS*WAYS*2-1:0] vec_bank_state_way_o,
    output [NUM_BANKS*3-1:0]     vec_bank_src_lane_o
);

    // =========================================================================
    // Crossbar instantiation
    // =========================================================================
    wire [NUM_BANKS-1:0]           xbar_bank_req;
    wire [NUM_BANKS-1:0]           xbar_bank_we;
    wire [NUM_BANKS*INDEX_W-1:0]   xbar_bank_index;
    wire [NUM_BANKS*3-1:0]         xbar_bank_word;
    wire [NUM_BANKS*3-1:0]         xbar_bank_way;
    wire [NUM_BANKS*8-1:0]         xbar_bank_be;
    wire [NUM_BANKS*64-1:0]        xbar_bank_wdata;
    wire [NUM_BANKS*TAG_W-1:0]     xbar_bank_tag;
    wire [NUM_BANKS*2-1:0]         xbar_bank_state;
    wire [NUM_BANKS*3-1:0]         xbar_bank_src_lane;

    rv64g_l1_crossbar #(
        .NUM_LANES(NUM_LANES),
        .NUM_BANKS(NUM_BANKS),
        .TAG_W(TAG_W),
        .INDEX_W(INDEX_W)
    ) u_crossbar (
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),
        .vlsu_req_i         (vlsu_req_i),
        .vlsu_lane_valid_i  (vlsu_lane_valid_i),
        .vlsu_lane_we_i     (vlsu_lane_we_i),
        .vlsu_lane_addr_i   (vlsu_lane_addr_i),
        .vlsu_lane_wdata_i  (vlsu_lane_wdata_i),
        .vlsu_lane_be_i     (vlsu_lane_be_i),
        .vlsu_lane_way_i    (vlsu_lane_way_i),
        .vlsu_lane_tag_i    (vlsu_lane_tag_i),
        .vlsu_lane_state_i  (vlsu_lane_state_i),
        .vlsu_ready_o       (vlsu_ready_o),
        .vlsu_done_o        (vlsu_done_o),
        .vlsu_lane_done_o   (vlsu_lane_done_o),
        .bank_req_o         (xbar_bank_req),
        .bank_we_o          (xbar_bank_we),
        .bank_index_o       (xbar_bank_index),
        .bank_word_o        (xbar_bank_word),
        .bank_way_o         (xbar_bank_way),
        .bank_be_o          (xbar_bank_be),
        .bank_wdata_o       (xbar_bank_wdata),
        .bank_tag_o         (xbar_bank_tag),
        .bank_state_o       (xbar_bank_state),
        .bank_src_lane_o    (xbar_bank_src_lane)
    );

    assign vec_bank_src_lane_o = xbar_bank_src_lane;

    // =========================================================================
    // Scalar bank decoding
    // =========================================================================
    wire [2:0] scalar_bank_sel = scalar_word_i;  // Bank = word within line

    // One-hot scalar bank enable for DATA writes (target bank only)
    wire [NUM_BANKS-1:0] scalar_bank_en;
    genvar sb;
    generate
        for (sb = 0; sb < NUM_BANKS; sb = sb + 1) begin : g_scalar_bank_en
            assign scalar_bank_en[sb] = scalar_req_i && (scalar_bank_sel == sb[2:0]);
        end
    endgenerate
    
    // Tag/state broadcast: when scalar_tag_we_i is set, ALL banks get tag write
    // This is handled by routing scalar_tag_we_i directly to all bank arbiters
    // even when scalar_bank_en is 0, via a separate path

    // =========================================================================
    // Per-bank arbiters and banks
    // =========================================================================
    // Arbiter outputs (to banks)
    wire [NUM_BANKS-1:0]           arb_bank_req;
    wire [NUM_BANKS-1:0]           arb_bank_we;
    wire [NUM_BANKS-1:0]           arb_bank_tag_we;  // Tag/state write enable
    wire [NUM_BANKS*INDEX_W-1:0]   arb_bank_index;
    wire [NUM_BANKS*3-1:0]         arb_bank_word;
    wire [NUM_BANKS*3-1:0]         arb_bank_way;
    wire [NUM_BANKS*8-1:0]         arb_bank_be;
    wire [NUM_BANKS*64-1:0]        arb_bank_wdata;
    wire [NUM_BANKS*TAG_W-1:0]     arb_bank_tag;
    wire [NUM_BANKS*2-1:0]         arb_bank_state;

    // Bank outputs
    wire [NUM_BANKS*WAYS*64-1:0]   bank_rdata_way;
    wire [NUM_BANKS*WAYS*TAG_W-1:0] bank_tag_way;
    wire [NUM_BANKS*WAYS*2-1:0]    bank_state_way;
    wire [NUM_BANKS*64-1:0]        bank_rdata_sel;
    wire [NUM_BANKS*TAG_W-1:0]     bank_tag_sel;
    wire [NUM_BANKS*2-1:0]         bank_state_sel;

    genvar bk;
    generate
        for (bk = 0; bk < NUM_BANKS; bk = bk + 1) begin : g_bank

            // Arbiter: scalar vs vector for this bank
            rv64g_l1_bank_arbiter #(
                .TAG_W(TAG_W),
                .INDEX_W(INDEX_W)
            ) u_arbiter (
                // Scalar port
                .scalar_req_i   (scalar_bank_en[bk]),
                .scalar_we_i    (scalar_we_i),
                .scalar_tag_we_i(scalar_tag_we_i),
                .scalar_tag_broadcast_i(scalar_tag_broadcast_i),
                .scalar_index_i (scalar_index_i),
                .scalar_word_i  (scalar_word_i),
                .scalar_way_i   (scalar_way_i),
                .scalar_be_i    (scalar_be_i),
                .scalar_wdata_i (scalar_wdata_i),
                .scalar_tag_i   (scalar_tag_i),
                .scalar_state_i (scalar_state_i),

                // Vector port (from crossbar)
                .vec_req_i      (xbar_bank_req[bk]),
                .vec_we_i       (xbar_bank_we[bk]),
                .vec_index_i    (xbar_bank_index[(bk+1)*INDEX_W-1 -: INDEX_W]),
                .vec_word_i     (xbar_bank_word[(bk+1)*3-1 -: 3]),
                .vec_way_i      (xbar_bank_way[(bk+1)*3-1 -: 3]),
                .vec_be_i       (xbar_bank_be[(bk+1)*8-1 -: 8]),
                .vec_wdata_i    (xbar_bank_wdata[(bk+1)*64-1 -: 64]),
                .vec_tag_i      (xbar_bank_tag[(bk+1)*TAG_W-1 -: TAG_W]),
                .vec_state_i    (xbar_bank_state[(bk+1)*2-1 -: 2]),

                // Arbitrated output
                .bank_req_o     (arb_bank_req[bk]),
                .bank_we_o      (arb_bank_we[bk]),
                .bank_tag_we_o  (arb_bank_tag_we[bk]),
                .bank_index_o   (arb_bank_index[(bk+1)*INDEX_W-1 -: INDEX_W]),
                .bank_word_o    (arb_bank_word[(bk+1)*3-1 -: 3]),
                .bank_way_o     (arb_bank_way[(bk+1)*3-1 -: 3]),
                .bank_be_o      (arb_bank_be[(bk+1)*8-1 -: 8]),
                .bank_wdata_o   (arb_bank_wdata[(bk+1)*64-1 -: 64]),
                .bank_tag_o     (arb_bank_tag[(bk+1)*TAG_W-1 -: TAG_W]),
                .bank_state_o   (arb_bank_state[(bk+1)*2-1 -: 2]),

                .scalar_stall_o (), // Scalar never stalls
                .vec_stall_o    ()  // Vector stall handled by crossbar FSM
            );

            // SRAM Bank
            rv64g_l1_sram_bank #(
                .SETS(SETS),
                .WAYS(WAYS),
                .TAG_W(TAG_W),
                .INDEX_W(INDEX_W)
            ) u_bank (
                .clk_i            (clk_i),
                .rst_ni           (rst_ni),
                .invalidate_all_i (invalidate_all_i),

                .req_i            (arb_bank_req[bk]),
                .we_i             (arb_bank_we[bk]),
                .tag_we_i         (arb_bank_tag_we[bk]),
                .index_i          (arb_bank_index[(bk+1)*INDEX_W-1 -: INDEX_W]),
                .word_i           (arb_bank_word[(bk+1)*3-1 -: 3]),
                .way_i            (arb_bank_way[(bk+1)*3-1 -: 3]),
                .be_i             (arb_bank_be[(bk+1)*8-1 -: 8]),
                .wdata_i          (arb_bank_wdata[(bk+1)*64-1 -: 64]),
                .tag_i            (arb_bank_tag[(bk+1)*TAG_W-1 -: TAG_W]),
                .state_i          (arb_bank_state[(bk+1)*2-1 -: 2]),

                .rdata_way_o      (bank_rdata_way[(bk+1)*WAYS*64-1 -: WAYS*64]),
                .tag_way_o        (bank_tag_way[(bk+1)*WAYS*TAG_W-1 -: WAYS*TAG_W]),
                .state_way_o      (bank_state_way[(bk+1)*WAYS*2-1 -: WAYS*2]),
                .rdata_sel_o      (bank_rdata_sel[(bk+1)*64-1 -: 64]),
                .tag_sel_o        (bank_tag_sel[(bk+1)*TAG_W-1 -: TAG_W]),
                .state_sel_o      (bank_state_sel[(bk+1)*2-1 -: 2])
            );
        end
    endgenerate

    // =========================================================================
    // Scalar output muxing (from targeted bank)
    // =========================================================================
    reg [63:0]          scalar_rdata_r;
    reg [TAG_W-1:0]     scalar_tag_r;
    reg [1:0]           scalar_state_r;
    reg [WAYS*64-1:0]   scalar_rdata_way_r;
    reg [WAYS*TAG_W-1:0] scalar_tag_way_r;
    reg [WAYS*2-1:0]    scalar_state_way_r;

    integer s;
    always @(*) begin
        scalar_rdata_r     = 64'b0;
        scalar_tag_r       = {TAG_W{1'b0}};
        scalar_state_r     = 2'b0;
        scalar_rdata_way_r = {(WAYS*64){1'b0}};
        scalar_tag_way_r   = {(WAYS*TAG_W){1'b0}};
        scalar_state_way_r = {(WAYS*2){1'b0}};

        for (s = 0; s < NUM_BANKS; s = s + 1) begin
            if (scalar_bank_sel == s[2:0]) begin
                scalar_rdata_r     = bank_rdata_sel[(s+1)*64-1 -: 64];
                scalar_tag_r       = bank_tag_sel[(s+1)*TAG_W-1 -: TAG_W];
                scalar_state_r     = bank_state_sel[(s+1)*2-1 -: 2];
                scalar_rdata_way_r = bank_rdata_way[(s+1)*WAYS*64-1 -: WAYS*64];
                scalar_tag_way_r   = bank_tag_way[(s+1)*WAYS*TAG_W-1 -: WAYS*TAG_W];
                scalar_state_way_r = bank_state_way[(s+1)*WAYS*2-1 -: WAYS*2];
            end
        end
    end

    assign scalar_rdata_o     = scalar_rdata_r;
    assign scalar_tag_o       = scalar_tag_r;
    assign scalar_state_o     = scalar_state_r;
    assign scalar_rdata_way_o = scalar_rdata_way_r;
    assign scalar_tag_way_o   = scalar_tag_way_r;
    assign scalar_state_way_o = scalar_state_way_r;

    // =========================================================================
    // Vector output passthrough (per-bank data)
    // =========================================================================
    assign vec_bank_rdata_o     = bank_rdata_sel;
    assign vec_bank_rdata_way_o = bank_rdata_way;
    assign vec_bank_tag_way_o   = bank_tag_way;
    assign vec_bank_state_way_o = bank_state_way;

endmodule
