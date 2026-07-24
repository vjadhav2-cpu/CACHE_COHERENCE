// rv64g_l1_vlsu_miss_handler.v - Multi-miss handler for Vector LSU
// Part of Phase 5: Miss Handling (MSHRs)
//
// When a vector operation has multiple lane misses, this module:
// 1. Captures all unique cache lines that need refilling
// 2. Issues refill requests one at a time (blocking style)
// 3. Signals completion when all lines are filled
// 4. Allows vector operation replay
//
// Design: Simple blocking approach - gather unique miss addresses,
// issue refills sequentially, then signal ready for replay.

`timescale 1ns/1ps
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */

module rv64g_l1_vlsu_miss_handler #(
    parameter integer NUM_LANES  = 8,
    parameter integer TAG_W      = 53,
    parameter integer INDEX_W    = 5,
    parameter integer MAX_MISSES = 8   // Max unique lines per vector op
) (
    input              clk_i,
    input              rst_ni,

    // Interface from VLSU hit detection
    input                          vlsu_req_i,        // Vector operation active
    input  [NUM_LANES-1:0]         lane_miss_i,       // Per-lane miss signals
    input  [NUM_LANES*64-1:0]      lane_addr_i,       // Per-lane addresses
    input                          any_miss_i,        // At least one miss

    // Refill interface to main cache controller
    output reg                     refill_req_o,      // Request a refill
    output reg [63:0]              refill_addr_o,     // Address to refill
    input                          refill_done_i,     // Refill completed

    // Status outputs
    output reg                     busy_o,            // Handler is processing misses
    output reg                     ready_for_replay_o,// All misses resolved, replay
    output wire [3:0]              miss_count_o       // Number of unique misses
);

    // =========================================================================
    // FSM States
    // =========================================================================
    localparam [2:0] S_IDLE         = 3'd0,  // Waiting for vector miss
                     S_CAPTURE      = 3'd1,  // Capturing unique miss addresses
                     S_REFILL_REQ   = 3'd2,  // Issuing refill request
                     S_REFILL_WAIT  = 3'd3,  // Waiting for refill completion
                     S_REPLAY_READY = 3'd4;  // All done, ready for replay

    reg [2:0] state_q, state_n;

    // =========================================================================
    // Miss Address Storage
    // =========================================================================
    // Store unique cache line addresses (aligned to 64B)
    // Only need tag + index, not full address
    localparam LINE_ADDR_W = TAG_W + INDEX_W;  // 53 + 5 = 58 bits

    reg [LINE_ADDR_W-1:0] miss_addr_q [0:MAX_MISSES-1];
    reg [MAX_MISSES-1:0]  miss_valid_q;
    reg [3:0]             miss_count_q;
    reg [3:0]             refill_idx_q;  // Which miss we're currently refilling

    // =========================================================================
    // Address Extraction
    // =========================================================================
    // Extract cache line address (tag + index) from full address
    function [LINE_ADDR_W-1:0] get_line_addr;
        input [63:0] addr;
        begin
            // Line address = addr[63:6] = tag + index
            get_line_addr = addr[63:6];
        end
    endfunction

    // =========================================================================
    // Unique Address Detection
    // =========================================================================
    // Check if a line address is already in our miss list
    function is_duplicate;
        input [LINE_ADDR_W-1:0] addr;
        input [MAX_MISSES-1:0]  valid;
        input [LINE_ADDR_W-1:0] addr0, addr1, addr2, addr3;
        input [LINE_ADDR_W-1:0] addr4, addr5, addr6, addr7;
        begin
            is_duplicate = (valid[0] && (addr == addr0)) ||
                           (valid[1] && (addr == addr1)) ||
                           (valid[2] && (addr == addr2)) ||
                           (valid[3] && (addr == addr3)) ||
                           (valid[4] && (addr == addr4)) ||
                           (valid[5] && (addr == addr5)) ||
                           (valid[6] && (addr == addr6)) ||
                           (valid[7] && (addr == addr7));
        end
    endfunction

    // =========================================================================
    // Capture Logic - Gather unique miss addresses
    // =========================================================================
    // Sample miss signals on transition INTO capture state (while still in IDLE)
    // This avoids the circular dependency where busy_o gates the hit detection
    reg [MAX_MISSES-1:0]  miss_valid_n;
    reg [LINE_ADDR_W-1:0] miss_addr_n [0:MAX_MISSES-1];
    reg [3:0]             miss_count_n;
    reg [3:0]             refill_idx_n;

    integer ln;
    reg [LINE_ADDR_W-1:0] lane_line_addr;
    reg                   is_dup;

    // Determine if we're about to enter capture state (still in IDLE, about to transition)
    wire entering_capture = (state_q == S_IDLE) && vlsu_req_i && any_miss_i;

    always @(*) begin
        // Defaults - hold current values
        miss_valid_n = miss_valid_q;
        miss_count_n = miss_count_q;
        for (ln = 0; ln < MAX_MISSES; ln = ln + 1) begin
            miss_addr_n[ln] = miss_addr_q[ln];
        end
        
        // Initialize to avoid latches
        lane_line_addr = {LINE_ADDR_W{1'b0}};
        is_dup = 1'b0;

        // Capture phase: scan all lanes and add unique misses
        // IMPORTANT: Capture happens on the transition INTO S_CAPTURE (while still in S_IDLE)
        // This ensures we sample the miss signals BEFORE busy_o goes high
        if (entering_capture) begin
            // Clear for fresh capture
            miss_valid_n = {MAX_MISSES{1'b0}};
            miss_count_n = 4'd0;

            for (ln = 0; ln < NUM_LANES; ln = ln + 1) begin
                if (lane_miss_i[ln]) begin
                    lane_line_addr = get_line_addr(lane_addr_i[(ln+1)*64-1 -: 64]);
                    
                    // Check if this line is already captured
                    is_dup = is_duplicate(lane_line_addr, miss_valid_n,
                                          miss_addr_n[0], miss_addr_n[1],
                                          miss_addr_n[2], miss_addr_n[3],
                                          miss_addr_n[4], miss_addr_n[5],
                                          miss_addr_n[6], miss_addr_n[7]);
                    
                    if (!is_dup && (miss_count_n < MAX_MISSES)) begin
                        miss_addr_n[miss_count_n] = lane_line_addr;
                        miss_valid_n[miss_count_n] = 1'b1;
                        miss_count_n = miss_count_n + 1;
                    end
                end
            end
        end
    end

    // =========================================================================
    // FSM Next-State Logic
    // =========================================================================
    always @(*) begin
        state_n = state_q;
        refill_idx_n = refill_idx_q;
        refill_req_o = 1'b0;
        refill_addr_o = 64'd0;
        busy_o = 1'b0;
        ready_for_replay_o = 1'b0;

        case (state_q)
            S_IDLE: begin
                if (vlsu_req_i && any_miss_i) begin
                    state_n = S_CAPTURE;
                end
            end

            S_CAPTURE: begin
                busy_o = 1'b1;
                // Capture happened in previous cycle (on transition from IDLE)
                // Now move to refill if there are misses
                if (miss_count_q > 0) begin
                    state_n = S_REFILL_REQ;
                    refill_idx_n = 4'd0;
                end else begin
                    // No actual misses (shouldn't happen)
                    state_n = S_IDLE;
                end
            end

            S_REFILL_REQ: begin
                busy_o = 1'b1;
                // Issue refill request for current miss
                refill_req_o = 1'b1;
                // Reconstruct full address from line address (append 6'b0 for byte offset)
                refill_addr_o = {miss_addr_q[refill_idx_q], 6'b0};
                state_n = S_REFILL_WAIT;
            end

            S_REFILL_WAIT: begin
                busy_o = 1'b1;
                // Keep address valid while waiting
                refill_addr_o = {miss_addr_q[refill_idx_q], 6'b0};
                
                if (refill_done_i) begin
                    if (refill_idx_q + 1 < miss_count_q) begin
                        // More misses to refill
                        refill_idx_n = refill_idx_q + 1;
                        state_n = S_REFILL_REQ;
                    end else begin
                        // All done
                        state_n = S_REPLAY_READY;
                    end
                end
            end

            S_REPLAY_READY: begin
                ready_for_replay_o = 1'b1;
                // Wait for VLSU to acknowledge replay
                if (!vlsu_req_i) begin
                    state_n = S_IDLE;
                end
            end

            default: state_n = S_IDLE;
        endcase
    end

    // =========================================================================
    // Sequential Logic
    // =========================================================================
    integer idx;
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= S_IDLE;
            miss_valid_q <= {MAX_MISSES{1'b0}};
            miss_count_q <= 4'd0;
            refill_idx_q <= 4'd0;
            for (idx = 0; idx < MAX_MISSES; idx = idx + 1) begin
                miss_addr_q[idx] <= {LINE_ADDR_W{1'b0}};
            end
        end else begin
            state_q <= state_n;
            miss_valid_q <= miss_valid_n;
            miss_count_q <= miss_count_n;
            refill_idx_q <= refill_idx_n;
            // Capture miss addresses on transition INTO capture state
            if (entering_capture) begin
                for (idx = 0; idx < MAX_MISSES; idx = idx + 1) begin
                    miss_addr_q[idx] <= miss_addr_n[idx];
                end
            end
        end
    end

    // Output miss count
    assign miss_count_o = miss_count_q;

endmodule
