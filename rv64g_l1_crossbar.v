// rv64g_l1_crossbar.v - 8x8 crossbar for Vector LSU to bank routing
// Part of Phase 3: The Crossbar ("Steering Wheel")
//
// Routes 8 VLSU lane requests to 8 banks based on element-to-bank mapping
// Includes conflict detection and multi-cycle FSM for bank conflicts
//
// Conflict handling:
//   - Cycle 1: Service non-conflicting requests
//   - Cycle 2+: Service remaining conflicting requests
//   - Repeat until all lanes served
//
// Element-interleaved: lane i naturally maps to bank i for unit-stride
// For strided/indexed: bank = addr[5:3] (word within cache line)

`timescale 1ns/1ps
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */

module rv64g_l1_crossbar #(
    parameter integer NUM_LANES = 8,
    parameter integer NUM_BANKS = 8,
    parameter integer TAG_W     = 53,
    parameter integer INDEX_W   = 5
) (
    input               clk_i,
    input               rst_ni,

    // VLSU interface - 8 lanes
    input               vlsu_req_i,                         // Vector request valid
    input  [NUM_LANES-1:0] vlsu_lane_valid_i,               // Per-lane valid (masked elements)
    input  [NUM_LANES-1:0] vlsu_lane_we_i,                  // Per-lane write enable
    input  [NUM_LANES*64-1:0] vlsu_lane_addr_i,             // Per-lane address (flat)
    input  [NUM_LANES*64-1:0] vlsu_lane_wdata_i,            // Per-lane write data (flat)
    input  [NUM_LANES*8-1:0]  vlsu_lane_be_i,               // Per-lane byte enables (flat)
    input  [NUM_LANES*3-1:0]  vlsu_lane_way_i,              // Per-lane way select (flat)
    input  [NUM_LANES*TAG_W-1:0] vlsu_lane_tag_i,           // Per-lane tag (flat)
    input  [NUM_LANES*2-1:0]  vlsu_lane_state_i,            // Per-lane state (flat)

    // Crossbar ready/done
    output              vlsu_ready_o,                       // Can accept new request
    output              vlsu_done_o,                        // All lanes served
    output [NUM_LANES-1:0] vlsu_lane_done_o,                // Per-lane completion

    // Output to banks (active requests this cycle)
    output [NUM_BANKS-1:0]  bank_req_o,                     // Per-bank request valid
    output [NUM_BANKS-1:0]  bank_we_o,                      // Per-bank write enable
    output [NUM_BANKS*INDEX_W-1:0] bank_index_o,            // Per-bank index (flat)
    output [NUM_BANKS*3-1:0] bank_word_o,                   // Per-bank word offset (flat)
    output [NUM_BANKS*3-1:0] bank_way_o,                    // Per-bank way select (flat)
    output [NUM_BANKS*8-1:0] bank_be_o,                     // Per-bank byte enables (flat)
    output [NUM_BANKS*64-1:0] bank_wdata_o,                 // Per-bank write data (flat)
    output [NUM_BANKS*TAG_W-1:0] bank_tag_o,                // Per-bank tag (flat)
    output [NUM_BANKS*2-1:0] bank_state_o,                  // Per-bank state (flat)

    // Lane-to-bank mapping for response routing
    output [NUM_BANKS*3-1:0] bank_src_lane_o                // Which lane each bank is serving
);

    // =========================================================================
    // FSM States
    // =========================================================================
    localparam [1:0] S_IDLE     = 2'b00;  // Waiting for request
    localparam [1:0] S_FIRST    = 2'b01;  // Service first group (non-conflicting)
    localparam [1:0] S_CONFLICT = 2'b10;  // Service conflicting leftovers

    reg [1:0] state_q, state_n;

    // =========================================================================
    // Request Tracking
    // =========================================================================
    // Track which lanes still need service
    reg [NUM_LANES-1:0] pending_q, pending_n;

    // Latched request data (held across conflict cycles)
    reg [NUM_LANES-1:0]          latch_valid_q;
    reg [NUM_LANES-1:0]          latch_we_q;
    reg [NUM_LANES*64-1:0]       latch_addr_q;
    reg [NUM_LANES*64-1:0]       latch_wdata_q;
    reg [NUM_LANES*8-1:0]        latch_be_q;
    reg [NUM_LANES*3-1:0]        latch_way_q;
    reg [NUM_LANES*TAG_W-1:0]    latch_tag_q;
    reg [NUM_LANES*2-1:0]        latch_state_q;

    // =========================================================================
    // Bank Selection per Lane
    // =========================================================================
    // Extract bank select from address: addr[5:3] = word within 64B line
    wire [2:0] lane_bank [0:NUM_LANES-1];
    wire [INDEX_W-1:0] lane_index [0:NUM_LANES-1];
    wire [2:0] lane_word [0:NUM_LANES-1];

    genvar ln;
    generate
        for (ln = 0; ln < NUM_LANES; ln = ln + 1) begin : g_lane_decode
            wire [63:0] addr = latch_addr_q[(ln+1)*64-1 : ln*64];
            assign lane_bank[ln]  = addr[5:3];   // Bank = word offset within line
            assign lane_index[ln] = addr[10:6];  // Set index
            assign lane_word[ln]  = addr[5:3];   // Word within line (same as bank for unit stride)
        end
    endgenerate

    // =========================================================================
    // Conflict Detection
    // =========================================================================
    // For each bank, find all lanes targeting it
    wire [NUM_LANES-1:0] bank_requesters [0:NUM_BANKS-1];

    genvar bk, la;
    generate
        for (bk = 0; bk < NUM_BANKS; bk = bk + 1) begin : g_bank_req
            for (la = 0; la < NUM_LANES; la = la + 1) begin : g_lane_chk
                assign bank_requesters[bk][la] = pending_q[la] && (lane_bank[la] == bk[2:0]);
            end
        end
    endgenerate

    // =========================================================================
    // Grant Logic - First requester wins (priority encoder per bank)
    // =========================================================================
    wire [NUM_LANES-1:0] lane_grant;      // Which lanes get serviced this cycle
    wire [2:0] bank_winner [0:NUM_BANKS-1]; // Which lane wins each bank
    wire [NUM_BANKS-1:0] bank_active;     // Which banks are active this cycle

    // Priority encoder for each bank - lowest lane index wins
    generate
        for (bk = 0; bk < NUM_BANKS; bk = bk + 1) begin : g_bank_arb
            reg [2:0] winner;
            reg       has_req;
            integer i;

            always @(*) begin
                winner  = 3'd0;
                has_req = 1'b0;
                for (i = 0; i < NUM_LANES; i = i + 1) begin
                    if (bank_requesters[bk][i] && !has_req) begin
                        winner  = i[2:0];
                        has_req = 1'b1;
                    end
                end
            end

            assign bank_winner[bk] = winner;
            assign bank_active[bk] = |bank_requesters[bk];
        end
    endgenerate

    // Compute which lanes are granted (won their target bank)
    generate
        for (la = 0; la < NUM_LANES; la = la + 1) begin : g_lane_grant
            wire [2:0] target_bank = lane_bank[la];
            assign lane_grant[la] = pending_q[la] && (bank_winner[target_bank] == la[2:0]);
        end
    endgenerate

    // =========================================================================
    // Output Steering
    // =========================================================================
    reg [NUM_BANKS-1:0]           bank_req_r;
    reg [NUM_BANKS-1:0]           bank_we_r;
    reg [NUM_BANKS*INDEX_W-1:0]   bank_index_r;
    reg [NUM_BANKS*3-1:0]         bank_word_r;
    reg [NUM_BANKS*3-1:0]         bank_way_r;
    reg [NUM_BANKS*8-1:0]         bank_be_r;
    reg [NUM_BANKS*64-1:0]        bank_wdata_r;
    reg [NUM_BANKS*TAG_W-1:0]     bank_tag_r;
    reg [NUM_BANKS*2-1:0]         bank_state_r;
    reg [NUM_BANKS*3-1:0]         bank_src_lane_r;

    integer b;
    always @(*) begin
        bank_req_r      = {NUM_BANKS{1'b0}};
        bank_we_r       = {NUM_BANKS{1'b0}};
        bank_index_r    = {(NUM_BANKS*INDEX_W){1'b0}};
        bank_word_r     = {(NUM_BANKS*3){1'b0}};
        bank_way_r      = {(NUM_BANKS*3){1'b0}};
        bank_be_r       = {(NUM_BANKS*8){1'b0}};
        bank_wdata_r    = {(NUM_BANKS*64){1'b0}};
        bank_tag_r      = {(NUM_BANKS*TAG_W){1'b0}};
        bank_state_r    = {(NUM_BANKS*2){1'b0}};
        bank_src_lane_r = {(NUM_BANKS*3){1'b0}};

        for (b = 0; b < NUM_BANKS; b = b + 1) begin
            if (bank_active[b] && (state_q == S_FIRST || state_q == S_CONFLICT)) begin
                bank_req_r[b] = 1'b1;
                bank_src_lane_r[(b+1)*3-1 -: 3] = bank_winner[b];

                // Route data from winning lane to this bank
                bank_we_r[b]                       = latch_we_q[bank_winner[b]];
                bank_index_r[(b+1)*INDEX_W-1 -: INDEX_W] = lane_index[bank_winner[b]];
                bank_word_r[(b+1)*3-1 -: 3]        = lane_word[bank_winner[b]];
                bank_way_r[(b+1)*3-1 -: 3]         = latch_way_q[(bank_winner[b]+1)*3-1 -: 3];
                bank_be_r[(b+1)*8-1 -: 8]          = latch_be_q[(bank_winner[b]+1)*8-1 -: 8];
                bank_wdata_r[(b+1)*64-1 -: 64]     = latch_wdata_q[(bank_winner[b]+1)*64-1 -: 64];
                bank_tag_r[(b+1)*TAG_W-1 -: TAG_W] = latch_tag_q[(bank_winner[b]+1)*TAG_W-1 -: TAG_W];
                bank_state_r[(b+1)*2-1 -: 2]       = latch_state_q[(bank_winner[b]+1)*2-1 -: 2];
            end
        end
    end

    assign bank_req_o      = bank_req_r;
    assign bank_we_o       = bank_we_r;
    assign bank_index_o    = bank_index_r;
    assign bank_word_o     = bank_word_r;
    assign bank_way_o      = bank_way_r;
    assign bank_be_o       = bank_be_r;
    assign bank_wdata_o    = bank_wdata_r;
    assign bank_tag_o      = bank_tag_r;
    assign bank_state_o    = bank_state_r;
    assign bank_src_lane_o = bank_src_lane_r;

    // =========================================================================
    // Completion Tracking
    // =========================================================================
    reg [NUM_LANES-1:0] done_q, done_n;

    assign vlsu_lane_done_o = done_q;
    // Done when all requested lanes have been serviced (compare next-state done with latched valid)
    assign vlsu_done_o      = (state_n == S_IDLE) && (done_n == latch_valid_q) && |latch_valid_q;
    assign vlsu_ready_o     = (state_q == S_IDLE);

    // =========================================================================
    // FSM and State Update
    // =========================================================================
    always @(*) begin
        state_n   = state_q;
        pending_n = pending_q;
        done_n    = done_q;

        case (state_q)
            S_IDLE: begin
                if (vlsu_req_i) begin
                    state_n   = S_FIRST;
                    pending_n = vlsu_lane_valid_i;
                    done_n    = {NUM_LANES{1'b0}};
                end
            end

            S_FIRST, S_CONFLICT: begin
                // Update pending: clear lanes that got serviced
                pending_n = pending_q & ~lane_grant;
                done_n    = done_q | lane_grant;

                // Check if all lanes done
                if ((pending_q & ~lane_grant) == {NUM_LANES{1'b0}}) begin
                    state_n = S_IDLE;
                end else begin
                    state_n = S_CONFLICT;
                end
            end

            default: state_n = S_IDLE;
        endcase
    end

    // =========================================================================
    // Sequential Logic
    // =========================================================================
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q       <= S_IDLE;
            pending_q     <= {NUM_LANES{1'b0}};
            done_q        <= {NUM_LANES{1'b0}};
            latch_valid_q <= {NUM_LANES{1'b0}};
            latch_we_q    <= {NUM_LANES{1'b0}};
            latch_addr_q  <= {(NUM_LANES*64){1'b0}};
            latch_wdata_q <= {(NUM_LANES*64){1'b0}};
            latch_be_q    <= {(NUM_LANES*8){1'b0}};
            latch_way_q   <= {(NUM_LANES*3){1'b0}};
            latch_tag_q   <= {(NUM_LANES*TAG_W){1'b0}};
            latch_state_q <= {(NUM_LANES*2){1'b0}};
        end else begin
            state_q   <= state_n;
            pending_q <= pending_n;
            done_q    <= done_n;

            // Latch request on new transaction
            if (state_q == S_IDLE && vlsu_req_i) begin
                latch_valid_q <= vlsu_lane_valid_i;
                latch_we_q    <= vlsu_lane_we_i;
                latch_addr_q  <= vlsu_lane_addr_i;
                latch_wdata_q <= vlsu_lane_wdata_i;
                latch_be_q    <= vlsu_lane_be_i;
                latch_way_q   <= vlsu_lane_way_i;
                latch_tag_q   <= vlsu_lane_tag_i;
                latch_state_q <= vlsu_lane_state_i;
            end
        end
    end

endmodule
