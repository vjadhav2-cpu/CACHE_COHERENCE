// rv64g_l1_vlsu_hit_detect.v - Per-lane hit detection for Vector LSU
// Part of Phase 4: Tag Array & Coherence
//
// For each VLSU lane, check if the accessed address hits in the cache.
// Uses per-bank tag/state outputs from the banked arrays.
//
// Hit logic per lane:
//   1. Extract tag from lane address
//   2. Compare against all ways in the target bank
//   3. Check state is valid (not MESI_N)
//   4. Output hit signal and hit way

`timescale 1ns/1ps
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */

module rv64g_l1_vlsu_hit_detect #(
    parameter integer NUM_LANES  = 8,
    parameter integer NUM_BANKS  = 8,
    parameter integer WAYS       = 8,
    parameter integer TAG_W      = 53,
    parameter integer INDEX_W    = 5
) (
    // Per-lane addresses (to extract tags)
    input  [NUM_LANES*64-1:0]      lane_addr_i,
    input  [NUM_LANES-1:0]         lane_valid_i,

    // Per-bank tag/state arrays from banked_arrays
    // Each bank outputs WAYS tags and WAYS states for the accessed index
    input  [NUM_BANKS*WAYS*TAG_W-1:0] bank_tag_way_i,
    input  [NUM_BANKS*WAYS*2-1:0]     bank_state_way_i,

    // Which lane is accessing which bank (from crossbar)
    input  [NUM_BANKS*3-1:0]       bank_src_lane_i,
    input  [NUM_BANKS-1:0]         bank_active_i,

    // Per-lane hit results
    output [NUM_LANES-1:0]         lane_hit_o,
    output [NUM_LANES*3-1:0]       lane_hit_way_o,
    output [NUM_LANES*2-1:0]       lane_state_o,    // State of hit way (or Invalid if miss)

    // Aggregated miss information
    output                         any_miss_o,      // At least one valid lane missed
    output [NUM_LANES-1:0]         lane_miss_o      // Per-lane miss signals
);

    // =========================================================================
    // Address Decode - Extract tag from each lane's address
    // =========================================================================
    localparam LINE_OFF_W = 6;  // addr[5:0] = byte within 64B line

    wire [TAG_W-1:0] lane_tag [0:NUM_LANES-1];
    wire [2:0]       lane_bank [0:NUM_LANES-1];

    genvar ln;
    generate
        for (ln = 0; ln < NUM_LANES; ln = ln + 1) begin : g_lane_decode
            wire [63:0] addr = lane_addr_i[(ln+1)*64-1 -: 64];
            assign lane_tag[ln]  = addr[63 : INDEX_W + LINE_OFF_W];  // addr[63:11]
            assign lane_bank[ln] = addr[5:3];  // Bank = word within line
        end
    endgenerate

    // =========================================================================
    // Per-Lane Hit Detection
    // =========================================================================
    // For each lane, look up the bank it's targeting and compare tags

    reg [NUM_LANES-1:0]     hit_r;
    reg [NUM_LANES*3-1:0]   hit_way_r;
    reg [NUM_LANES*2-1:0]   state_r;

    // Expand tag/state arrays for easier indexing
    wire [TAG_W-1:0] bank_tags  [0:NUM_BANKS-1][0:WAYS-1];
    wire [1:0]       bank_states[0:NUM_BANKS-1][0:WAYS-1];

    genvar bn, wy;
    generate
        for (bn = 0; bn < NUM_BANKS; bn = bn + 1) begin : g_bank_expand
            for (wy = 0; wy < WAYS; wy = wy + 1) begin : g_way_expand
                assign bank_tags[bn][wy]   = bank_tag_way_i[(bn*WAYS + wy + 1)*TAG_W - 1 -: TAG_W];
                assign bank_states[bn][wy] = bank_state_way_i[(bn*WAYS + wy + 1)*2 - 1 -: 2];
            end
        end
    endgenerate

    // Per-lane way match signals (generated combinationally)
    wire [WAYS-1:0] way_match [0:NUM_LANES-1];
    
    generate
        for (ln = 0; ln < NUM_LANES; ln = ln + 1) begin : g_lane_match
            genvar wm;
            for (wm = 0; wm < WAYS; wm = wm + 1) begin : g_way_match
                assign way_match[ln][wm] = (bank_states[lane_bank[ln]][wm] != MESI_N) &&
                                           (bank_tags[lane_bank[ln]][wm] == lane_tag[ln]);
            end
        end
    endgenerate

    // Priority encoder to find first matching way
    integer l;
    always @(*) begin
        hit_r     = {NUM_LANES{1'b0}};
        hit_way_r = {(NUM_LANES*3){1'b0}};
        state_r   = {(NUM_LANES*2){1'b0}};

        for (l = 0; l < NUM_LANES; l = l + 1) begin
            if (lane_valid_i[l]) begin
                // Check each way in priority order
                if (way_match[l][0]) begin
                    hit_r[l] = 1'b1;
                    hit_way_r[(l+1)*3-1 -: 3] = 3'd0;
                    state_r[(l+1)*2-1 -: 2] = bank_states[lane_bank[l]][0];
                end else if (way_match[l][1]) begin
                    hit_r[l] = 1'b1;
                    hit_way_r[(l+1)*3-1 -: 3] = 3'd1;
                    state_r[(l+1)*2-1 -: 2] = bank_states[lane_bank[l]][1];
                end else if (way_match[l][2]) begin
                    hit_r[l] = 1'b1;
                    hit_way_r[(l+1)*3-1 -: 3] = 3'd2;
                    state_r[(l+1)*2-1 -: 2] = bank_states[lane_bank[l]][2];
                end else if (way_match[l][3]) begin
                    hit_r[l] = 1'b1;
                    hit_way_r[(l+1)*3-1 -: 3] = 3'd3;
                    state_r[(l+1)*2-1 -: 2] = bank_states[lane_bank[l]][3];
                end else if (way_match[l][4]) begin
                    hit_r[l] = 1'b1;
                    hit_way_r[(l+1)*3-1 -: 3] = 3'd4;
                    state_r[(l+1)*2-1 -: 2] = bank_states[lane_bank[l]][4];
                end else if (way_match[l][5]) begin
                    hit_r[l] = 1'b1;
                    hit_way_r[(l+1)*3-1 -: 3] = 3'd5;
                    state_r[(l+1)*2-1 -: 2] = bank_states[lane_bank[l]][5];
                end else if (way_match[l][6]) begin
                    hit_r[l] = 1'b1;
                    hit_way_r[(l+1)*3-1 -: 3] = 3'd6;
                    state_r[(l+1)*2-1 -: 2] = bank_states[lane_bank[l]][6];
                end else if (way_match[l][7]) begin
                    hit_r[l] = 1'b1;
                    hit_way_r[(l+1)*3-1 -: 3] = 3'd7;
                    state_r[(l+1)*2-1 -: 2] = bank_states[lane_bank[l]][7];
                end
                // else: miss - hit_r[l] stays 0, way/state stay 0
            end
        end
    end

    assign lane_hit_o     = hit_r;
    assign lane_hit_way_o = hit_way_r;
    assign lane_state_o   = state_r;

    // =========================================================================
    // Miss Aggregation
    // =========================================================================
    assign lane_miss_o = lane_valid_i & ~hit_r;
    assign any_miss_o  = |(lane_valid_i & ~hit_r);

endmodule
