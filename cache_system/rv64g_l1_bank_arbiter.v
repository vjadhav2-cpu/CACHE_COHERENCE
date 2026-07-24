// rv64g_l1_bank_arbiter.v - 2:1 arbiter per bank (Scalar vs Vector lane)
// Part of Phase 2: Multi-Port Interface
//
// Fixed priority: Scalar > Vector to prevent OS starvation
// Combinational arbitration with registered outputs optional
//
// When both scalar and vector request the same bank:
//   - Scalar wins, vector stalls (vec_stall_o asserted)
//   - Next cycle vector can retry

`timescale 1ns/1ps
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNOPTFLAT */

module rv64g_l1_bank_arbiter #(
    parameter integer TAG_W   = 53,
    parameter integer INDEX_W = 5
) (
    // Scalar port (Port A) - higher priority
    input               scalar_req_i,
    input               scalar_we_i,
    input               scalar_tag_we_i,  // Tag/state write enable (per-bank or broadcast)
    input               scalar_tag_broadcast_i, // Broadcast tag/state to all banks
    input  [INDEX_W-1:0] scalar_index_i,
    input  [2:0]        scalar_word_i,
    input  [2:0]        scalar_way_i,
    input  [7:0]        scalar_be_i,
    input  [63:0]       scalar_wdata_i,
    input  [TAG_W-1:0]  scalar_tag_i,
    input  [1:0]        scalar_state_i,

    // Vector port (Port B) - lower priority
    input               vec_req_i,
    input               vec_we_i,
    input  [INDEX_W-1:0] vec_index_i,
    input  [2:0]        vec_word_i,
    input  [2:0]        vec_way_i,
    input  [7:0]        vec_be_i,
    input  [63:0]       vec_wdata_i,
    input  [TAG_W-1:0]  vec_tag_i,
    input  [1:0]        vec_state_i,

    // Arbitrated output to bank
    output              bank_req_o,
    output              bank_we_o,
    output              bank_tag_we_o,    // Tag/state write enable to bank
    output [INDEX_W-1:0] bank_index_o,
    output [2:0]        bank_word_o,
    output [2:0]        bank_way_o,
    output [7:0]        bank_be_o,
    output [63:0]       bank_wdata_o,
    output [TAG_W-1:0]  bank_tag_o,
    output [1:0]        bank_state_o,

    // Stall signals
    output              scalar_stall_o,  // Scalar blocked (never asserted - highest priority)
    output              vec_stall_o      // Vector blocked by scalar
);

    // =========================================================================
    // Arbitration Logic - Fixed Priority (Scalar > Vector)
    // =========================================================================
    // Scalar always wins when both request same bank
    // Tag broadcast also takes priority (happens during refill completion)
    wire scalar_grant;
    wire vec_grant;
    wire tag_broadcast;

    // Tag broadcast is a scalar operation that writes tag/state to ALL banks
    // It should take priority even when this isn't the data target bank
    assign tag_broadcast = scalar_tag_broadcast_i;
    assign scalar_grant = scalar_req_i || tag_broadcast;
    assign vec_grant    = vec_req_i && !scalar_grant;

    // =========================================================================
    // Output Muxing
    // =========================================================================
    // Select scalar if scalar has grant, else vector
    wire sel_scalar;
    assign sel_scalar = scalar_grant;

    // For tag broadcast (not data target), we don't write data
    wire scalar_data_write = scalar_req_i && scalar_we_i;

    assign bank_req_o    = scalar_grant || vec_grant;
    assign bank_we_o     = sel_scalar ? scalar_data_write : vec_we_i;
    assign bank_tag_we_o = sel_scalar ? scalar_tag_we_i  : 1'b0;  // Vector doesn't use broadcast
    assign bank_index_o  = sel_scalar ? scalar_index_i   : vec_index_i;
    assign bank_word_o   = sel_scalar ? scalar_word_i    : vec_word_i;
    assign bank_way_o   = sel_scalar ? scalar_way_i   : vec_way_i;
    assign bank_be_o    = sel_scalar ? scalar_be_i    : vec_be_i;
    assign bank_wdata_o = sel_scalar ? scalar_wdata_i : vec_wdata_i;
    assign bank_tag_o   = sel_scalar ? scalar_tag_i   : vec_tag_i;
    assign bank_state_o = sel_scalar ? scalar_state_i : vec_state_i;

    // =========================================================================
    // Stall Signals
    // =========================================================================
    // Scalar never stalls (highest priority)
    assign scalar_stall_o = 1'b0;

    // Vector stalls when scalar is also requesting (including broadcast)
    assign vec_stall_o = vec_req_i && scalar_grant;

endmodule
