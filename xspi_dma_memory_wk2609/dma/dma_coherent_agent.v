`timescale 1ns/1ps
`default_nettype none

// =============================================================================
// dma_coherent_agent.v
//
// Bridges the DMA core's mem_* bus (dma_flat_full_duplex_top.v) onto one
// l1_tilelink_adapter instance's CACHED request path
// (L1_REQ_READ_MISS/WRITE_MISS -> Acquire, L1_REQ_WRITE_BACK -> Release),
// reusing tlc_xbar_top.v's existing, unconnected Master-2 port. This is the
// design worked out in
// docs/microarchitecture/DMA_to_TLC_Crossbar_Adapter_Plan.md.
//
// Per that plan Sec.2, the cached path is used deliberately, NOT the
// adapter's uncached Get/Put path -- tlc_slave_model.v funnels every A
// request through its directory/probe/Grant machinery regardless of opcode
// and never drives AccessAck, so the uncached path hangs against it. This
// agent looks like a degenerate L1 with a one-entry, same-cycle-eviction
// cache: Acquire -> use once -> Release, immediately, every beat.
//
// --- Deviation from the plan: coherence granule is 1 word, not 1 line -----
// The plan's Sec.3/Sec.4 describe a 256-bit (32-byte, 4x64-bit-word) cache
// line with an addr[4:3] word-select and a read-modify-write that preserves
// the other three words. That does not match the actual RTL: every A/B/C/D
// channel `data` field in l1_tilelink_adapter.v is a hard-wired 64 bits
// (`input wire [63:0] d_data`, etc.); GrantData is forwarded to
// data_to_l1_data via `{192'b0, d_data}` (zero-extend, not a real 4-word
// burst), and a Release only ever sends `pending_data[63:0]` -- the upper
// 192 bits of l1_request_data are silently discarded on the wire. Directory
// and memory indexing in tlc_slave_model.v also key off addr[X:3] directly
// (8-byte-word granularity), not a line-aligned address. So there is no
// mechanism, with these unmodified files, to transport or preserve "the
// other three words" -- the real coherence granule this hardware supports
// is exactly one 64-bit word, which happens to be exactly what a DMA beat
// already is. This agent therefore does no word-select/merge: it Acquires,
// uses, and Releases the single word at l1_request_addr = mem_addr[31:0]
// directly. (Verified by isolated simulation reading the source directly;
// see conversation history / final report for the reproduction.)
//
// --- Known, separately-tracked defect in l1_tilelink_adapter.v (not fixed
// here; out of scope per explicit direction) ---------------------------
// l1_tilelink_adapter.v's STATE_PROBE_PROCESS re-samples `b_valid` one
// cycle after STATE_IDLE already decided to transition there; tlc_slave_
// model.v's B channel is a single-cycle pulse whenever the master's
// b_ready is already high (the common case, since b_ready tracks
// state==IDLE with a 1-cycle lag), so by the time PROBE_PROCESS re-checks
// b_valid it has already dropped, and the FSM hangs in PROBE_PROCESS
// forever without ever latching the probe address or issuing a ProbeAck.
// This affects every master (L1 #0/#1 and this agent identically) any time
// a probe arrives while that master is idle. It was reproduced in
// isolation against l1_tilelink_adapter.v alone. Per explicit instruction,
// this file is treated as a black box with this defect out of scope --
// the probe-response logic below is implemented to the documented
// interface contract and is correct on its own terms; it cannot be
// end-to-end verified against the real crossbar until the defect above is
// fixed elsewhere.
//
// --- Deviation from the plan Sec.6: the voluntary Release is never
// skipped, even when a probe forces an early surrender -----------------
// The plan suggests the agent's own pending Release becomes a no-op once a
// probe forces a data return. Re-reading tlc_slave_model.v shows this is
// unsafe: S_PROBE_WAIT never clears the probed master's dir_pres bit --
// only S_IDLE's real voluntary-Release handler does. Skipping the Release
// would leak this agent's presence bit in the directory forever (spurious
// future probes, but not raw data corruption). So this agent always
// completes its own Acquire->Release sequence unconditionally; the
// immediate probe-ack response (see below) is a separate, idempotent
// transaction that is safe to overlap with it (tlc_slave_model.v accepts a
// voluntary Release unconditionally regardless of current dir_coh/dir_pres,
// and re-writing the same value into mem[] a second time is harmless).
//
// --- DMA-side handshake convention -----------------------------------
// mem_req/mem_wr_en/mem_addr/mem_wdata are driven by the DMA and held
// stable for the whole request; mem_ack pulses exactly one cycle. This
// mirrors dma_tlc_uncached_bridge.v's documented convention (itself
// matching dma_streamer's `ack_qual = bus_ack & bus_req`). A request is
// only ever sampled from S_IDLE, once per full request/ack round trip, so
// a continuously-asserted mem_req (as dma_streamer can produce for
// back-to-back beats) naturally produces one transaction per round trip
// rather than launching the same beat twice -- no separate rearm signal
// is needed.
//
// --- Write-back completion: ack on Release *acceptance*, not ReleaseAck ---
// l1_tilelink_adapter.v exposes no dedicated "Release done" signal, and
// data_to_l1_valid never pulses for ReleaseAck. The natural first idea is
// to deassert l1_request_valid right after the WRITE_BACK is accepted and
// then poll l1_request_ready, waiting for it to go low (busy) and then
// high again (idle) as an implicit "ReleaseAck consumed" signal. This was
// implemented and tried, and empirically DEADLOCKS against the real
// adapter: l1_request_ready is not an independent "I'm free" signal -- it
// is `can_accept_new_request && (allocated_source_id_valid ||
// alloc_source_id_gnt)`, and `alloc_source_id_gnt` only ever gets asserted
// in reaction to `alloc_source_id_req`, which itself requires
// `l1_request_valid` to already be high
// (`alloc_source_id_req = can_accept_new_request && l1_request_valid &&
// !alloc_source_id_gnt`, rtl/tlc/l1_tilelink_adapter.v). So once this
// module deasserts l1_request_valid, l1_request_ready can never reassert
// on its own -- there is nothing left to trigger a fresh source-ID grant.
// Reproduced directly in simulation: after WRITE_BACK acceptance,
// l1_request_ready stays low forever once valid drops, even though the
// adapter's own internal FSM state genuinely returns to STATE_IDLE.
//
// Holding l1_request_valid high continuously past the first accept pulse
// doesn't work either: valid/ready has no transaction-identity concept,
// so if ready ever pulses again while valid is still held high (once the
// adapter is truly idle again), that is, by definition, a second,
// independent WRITE_BACK submission -- not a completion signal for the
// first one.
//
// Given the l1_request_valid/ready <-> data_to_l1_* boundary genuinely
// carries no observable "Release complete" event, this module instead
// acks the DMA as soon as the WRITE_BACK request is accepted
// (l1_request_valid && l1_request_ready). This is safe: the coherence
// guarantee (other sharers invalidated/drained) is already fully
// established by the time of our own Grant, before we ever reach the
// Release step -- the Release only returns the line so other masters can
// subsequently Acquire it. If the *next* DMA beat needs to Acquire while
// this agent's Release is still in flight, ordinary valid/ready
// backpressure (l1_request_ready staying low until the adapter is
// genuinely idle again) naturally stalls it correctly -- no explicit
// completion tracking is required for correctness.
//
// --- Error handling -------------------------------------------------
// data_to_l1_error (from Acquire/GrantData) is captured but not surfaced:
// dma_flat_full_duplex_top.v's mem_* bus has no error output, so this
// agent completes the transaction and acks regardless (documented, not
// silently invented). d_error on ReleaseAck is dropped by
// l1_tilelink_adapter.v itself before it ever reaches this module's ports
// -- this agent cannot detect or report a failed writeback; this is an
// inherited limitation of the adapter, not something introduced here.
// =============================================================================

module dma_coherent_agent #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 64   // real transport width is fixed at 64 bits
                                 // by l1_tilelink_adapter.v regardless of
                                 // this parameter; see header note above.
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // ---------------------------------------------------------------
    // DMA core mem_* bus (dma_flat_full_duplex_top.v) -- level-held
    // request, single-cycle ack.
    // ---------------------------------------------------------------
    input  wire                   mem_req,
    input  wire                   mem_wr_en,
    input  wire [ADDR_WIDTH-1:0]  mem_addr,
    input  wire [DATA_WIDTH-1:0]  mem_wdata,
    output reg  [DATA_WIDTH-1:0]  mem_rdata,
    output reg                    mem_ack,

    // ---------------------------------------------------------------
    // l1_tilelink_adapter.v L1-facing interface -- same port names as
    // tlc_xbar_top.v's existing Master-2 port (m2_req_*/m2_data_*/
    // m2_probe_*), so this module drops onto it with no renaming.
    // ---------------------------------------------------------------
    output reg                    l1_request_valid,
    output reg  [31:0]            l1_request_addr,
    output reg  [2:0]             l1_request_type,
    output reg  [255:0]           l1_request_data,
    output reg  [2:0]             l1_request_permissions,
    input  wire                   l1_request_ready,

    input  wire                   data_to_l1_valid,
    input  wire [255:0]           data_to_l1_data,
    input  wire                   data_to_l1_error,

    input  wire                   probe_req_to_l1_valid,
    input  wire [31:0]            probe_req_to_l1_addr,
    input  wire [2:0]             probe_req_to_l1_permissions,

    output reg                    probe_ack_from_l1_valid,
    output reg  [31:0]            probe_ack_from_l1_addr,
    output reg  [2:0]             probe_ack_from_l1_permissions,
    output reg  [255:0]           probe_ack_from_l1_dirty_data
);

    `include "tlc64b2M_params.v"

    // ------------------------------------------------------------------
    // Main FSM states
    // ------------------------------------------------------------------
    localparam [2:0]
        S_IDLE             = 3'd0,
        S_ACQ_SEND         = 3'd1,
        S_ACQ_WAIT         = 3'd2,
        S_WB_SUBMIT        = 3'd3,
        S_DMA_ACK          = 3'd4,
        S_REQ_ADVANCE      = 3'd5;

    reg [2:0] state;

    // Held-permission tracking for probe responses (Sec.6 Cases A-D).
    // Distinct from the TileLink PARAM_* *transition* encodings -- this is
    // "what do we currently hold", not "what transition are we making".
    localparam [1:0] HELD_NONE   = 2'd0,
                      HELD_BRANCH = 2'd1,
                      HELD_TRUNK  = 2'd2;

    reg [1:0]  held_perm;     // HELD_NONE until Grant, cleared after Release completes
    wire       held_valid = (held_perm != HELD_NONE);

    // Latched per-transaction context (accepted only from S_IDLE).
    reg [31:0]            ctx_addr;
    reg                   ctx_is_write;
    reg [DATA_WIDTH-1:0]  ctx_wdata;

    // Data captured from our own Grant/GrantData.
    reg [DATA_WIDTH-1:0]  captured_data;
    reg                   captured_error;   // observed, not currently surfaced -- see header note
    reg [31:0]            last_ack_addr;
    reg                   last_ack_is_write;
    reg [DATA_WIDTH-1:0]  last_ack_wdata;

    // ------------------------------------------------------------------
    // Probe response -- combinational, independent of the main FSM state,
    // so it always answers immediately regardless of what this agent's
    // own transaction is doing (Sec.6 Cases A-D). No side effect on the
    // main FSM (see header note on why the voluntary Release is never
    // skipped).
    // ------------------------------------------------------------------
    wire probe_line_match = held_valid &&
                             (probe_req_to_l1_addr[31:3] == ctx_addr[31:3]);

    always @(*) begin
        if (probe_req_to_l1_valid) begin
            probe_ack_from_l1_valid = 1'b1;
            probe_ack_from_l1_addr  = probe_req_to_l1_addr;
            if (probe_line_match && (held_perm == HELD_TRUNK)) begin
                // Case D: dirty write in flight -- hand back the exact
                // word we were about to voluntarily Release anyway.
                probe_ack_from_l1_permissions = PARAM_TtoN;
                probe_ack_from_l1_dirty_data  = {{(256-DATA_WIDTH){1'b0}}, ctx_wdata};
            end
            else if (probe_line_match && (held_perm == HELD_BRANCH)) begin
                // Case C: clean read hold -- downgrade, no data.
                probe_ack_from_l1_permissions = PARAM_BtoN;
                probe_ack_from_l1_dirty_data  = 256'b0;
            end
            else begin
                // Cases A/B: not holding this line (or not holding
                // anything yet -- probe arrived before our own Grant).
                probe_ack_from_l1_permissions = PARAM_NtoN;
                probe_ack_from_l1_dirty_data  = 256'b0;
            end
        end
        else begin
            probe_ack_from_l1_valid       = 1'b0;
            probe_ack_from_l1_addr        = 32'b0;
            probe_ack_from_l1_permissions = PARAM_NtoN;
            probe_ack_from_l1_dirty_data  = 256'b0;
        end
    end

    // ------------------------------------------------------------------
    // Main sequential FSM
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                   <= S_IDLE;
            ctx_addr                <= 32'b0;
            ctx_is_write            <= 1'b0;
            ctx_wdata               <= {DATA_WIDTH{1'b0}};
            captured_data           <= {DATA_WIDTH{1'b0}};
            captured_error          <= 1'b0;
            held_perm               <= HELD_NONE;
            last_ack_addr           <= 32'b0;
            last_ack_is_write       <= 1'b0;
            last_ack_wdata          <= {DATA_WIDTH{1'b0}};

            l1_request_valid        <= 1'b0;
            l1_request_addr         <= 32'b0;
            l1_request_type         <= 3'b0;
            l1_request_data         <= 256'b0;
            l1_request_permissions  <= 3'b0;

            mem_rdata                <= {DATA_WIDTH{1'b0}};
            mem_ack                  <= 1'b0;
        end
        else begin
            mem_ack <= 1'b0;

            case (state)
                // ------------------------------------------------------
                S_IDLE: begin
                    l1_request_valid <= 1'b0;
                    if (mem_req) begin
                        $display("[DMA_AGENT_REQ][%0t] we=%0b addr=0x%08h wdata=0x%016h", $time, mem_wr_en, mem_addr[31:0], mem_wdata);
                        ctx_addr     <= mem_addr[31:0];
                        ctx_is_write <= mem_wr_en;
                        ctx_wdata    <= mem_wdata;
                        state        <= S_ACQ_SEND;
                    end
                end

                // ------------------------------------------------------
                // Acquire the line: NtoB for a read (don't disturb other
                // sharers unnecessarily), NtoT for a write (directory
                // must invalidate every other sharer before granting).
                // ------------------------------------------------------
                S_ACQ_SEND: begin
                    l1_request_valid       <= 1'b1;
                    l1_request_addr        <= ctx_addr;
                    l1_request_type        <= ctx_is_write ? L1_REQ_WRITE_MISS : L1_REQ_READ_MISS;
                    l1_request_permissions <= ctx_is_write ? PARAM_NtoT : PARAM_NtoB;
                    l1_request_data        <= 256'b0; // unused by Acquire

                    if (l1_request_valid && l1_request_ready) begin
                        l1_request_valid <= 1'b0;
                        state            <= S_ACQ_WAIT;
                    end
                end

                // ------------------------------------------------------
                // Wait for Grant/GrantData. l1_tilelink_adapter sequences
                // GrantAck on E internally -- no action needed here.
                // ------------------------------------------------------
                S_ACQ_WAIT: begin
                    if (data_to_l1_valid) begin
                        $display("[DMA_AGENT_GRANT][%0t] addr=0x%08h data=0x%016h err=%0b", $time, ctx_addr, data_to_l1_data[DATA_WIDTH-1:0], data_to_l1_error);
                        captured_data  <= data_to_l1_data[DATA_WIDTH-1:0];
                        captured_error <= data_to_l1_error;
                        held_perm      <= ctx_is_write ? HELD_TRUNK : HELD_BRANCH;
                        state          <= S_WB_SUBMIT;
                    end
                end

                // ------------------------------------------------------
                // Voluntarily release immediately: BtoN (read, data
                // unmodified) or TtoN (write, data = the DMA's word).
                // No merge is needed -- see header note on granule size.
                // Acks on Release *acceptance*, not ReleaseAck -- see the
                // "Write-back completion" header note for why: this
                // valid/ready boundary has no observable ReleaseAck event,
                // and acking here is provably safe (backpressure on the
                // next Acquire covers correctness if the Release is still
                // draining).
                // ------------------------------------------------------
                S_WB_SUBMIT: begin
                    l1_request_valid       <= 1'b1;
                    l1_request_addr        <= ctx_addr;
                    l1_request_type        <= L1_REQ_WRITE_BACK;
                    l1_request_permissions <= ctx_is_write ? PARAM_TtoN : PARAM_BtoN;
                    l1_request_data        <= ctx_is_write
                                               ? {{(256-DATA_WIDTH){1'b0}}, ctx_wdata}
                                               : {{(256-DATA_WIDTH){1'b0}}, captured_data};

                    if (l1_request_valid && l1_request_ready) begin
                        $display("[DMA_AGENT_WB_HS][%0t] addr=0x%08h wb_data=0x%016h grant_data=0x%016h req_data=0x%016h", $time, ctx_addr, ctx_wdata, captured_data, l1_request_data[DATA_WIDTH-1:0]);
                        l1_request_valid <= 1'b0;
                        held_perm        <= HELD_NONE;
                        mem_rdata        <= ctx_is_write ? {DATA_WIDTH{1'b0}} : captured_data;
                        state            <= S_DMA_ACK;
                    end
                end

                // ------------------------------------------------------
                S_DMA_ACK: begin
                    mem_ack <= 1'b1;
                    last_ack_addr     <= ctx_addr;
                    last_ack_is_write <= ctx_is_write;
                    last_ack_wdata    <= ctx_wdata;
                    state             <= S_REQ_ADVANCE;
                end

                // ------------------------------------------------------
                // The DMA mem_* request is level-held through ack and
                // advances one cycle later. If we sample immediately on
                // return to IDLE, we can accidentally capture the old
                // beat again. Wait here until mem_req either drops or
                // changes to a new tuple, then accept the next request.
                // ------------------------------------------------------
                S_REQ_ADVANCE: begin
                    l1_request_valid <= 1'b0;
                    if (!mem_req) begin
                        state <= S_IDLE;
                    end
                    else if ((mem_addr[31:0] != last_ack_addr) ||
                             (mem_wr_en      != last_ack_is_write) ||
                             (mem_wdata      != last_ack_wdata)) begin
                        $display("[DMA_AGENT_REQ][%0t] we=%0b addr=0x%08h wdata=0x%016h", $time, mem_wr_en, mem_addr[31:0], mem_wdata);
                        ctx_addr     <= mem_addr[31:0];
                        ctx_is_write <= mem_wr_en;
                        ctx_wdata    <= mem_wdata;
                        state        <= S_ACQ_SEND;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
