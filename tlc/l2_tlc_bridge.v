`timescale 1ns/1ps
`default_nettype none

// Bridges rv64g_l2_cache.v's memory-side TL-UH port (mem_a_*_o / mem_d_*_i,
// see cache_system/rv64g_l2_cache.v:58-78) onto one l1_tilelink_adapter
// instance. The original implementation used only the adapter's uncached
// Get/Put path. This file is now the first step toward a coherent L2/TLC
// integration:
//   - refill reads are issued as coherent read misses so the TLC-side
//     directory can start tracking L2 presence
//   - probe requests are surfaced and explicitly flagged as needing a real
//     snoop hook into L2, instead of being silently tied off
//
// This file alone cannot complete the full coherency fix. A correct probe
// response still requires a new snoop/invalidate path into rv64g_l2_cache /
// rv64g_l2_fsm.v, which is intentionally left as follow-up work rather than
// being faked here.
//
// Why this still cannot be a straight port hookup (see
// docs/l2_tlc_xbar_integration_plan.md for the full writeup):
// L2 does real 8-beat, 64-byte-line bursts on its mem port -- one Get
// covering all 8 read words, or one PutFullData streaming all 8 write
// words (rv64g_l2_fsm.v ST_MEM_READ / ST_MEM_WRITE) -- but
// l1_tilelink_adapter.v only ever moves one 64-bit beat per request.
// This bridge therefore keeps the documented v1 simplification: decompose
// each L2 line access into 8 sequential single-beat requests through the
// adapter, then reassemble the D-channel side back into the 8-beat handshake
// rv64g_l2_fsm.v expects.
//
// Transitional limitations:
//   - reads now use the coherent Acquire/Grant path
//   - writes still use the uncached Put/AccessAck path for now, because the
//     current l1_tilelink_adapter.v interface does not expose a bridge-visible
//     completion pulse for ReleaseAck
//   - incoming probes can now be observed here, but they cannot yet trigger a
//     real invalidate/writeback inside L2 without additional cache_system/ RTL
//
// Depends on a fix to l1_tilelink_adapter.v's STATE_UNCACHED_RSP_WAIT:
// originally only AccessAckData (read) pulsed data_to_l1_valid, leaving
// uncached writes with no observable completion signal at all (confirmed by
// simulation, not just inspection -- see commit history). Plain AccessAck now
// pulses it too, with data_to_l1_data driven to zero.
module l2_tlc_bridge #(
    parameter ADDR_W = 64,
    parameter DATA_W = 64
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // rv64g_l2_cache.v memory-side port. A = requests from L2, D = responses to L2.
    input  wire [2:0]            mem_a_opcode_i,
    input  wire [ADDR_W-1:0]     mem_a_address_i,
    input  wire [DATA_W-1:0]     mem_a_data_i,
    input  wire                  mem_a_valid_i,
    output wire                  mem_a_ready_o,

    output wire [2:0]            mem_d_opcode_o,
    output wire [DATA_W-1:0]     mem_d_data_o,
    output wire                  mem_d_valid_o,
    input  wire                  mem_d_ready_i,

    // Snoop handshake into rv64g_l2_cache.v / rv64g_l2_fsm.v.
    output reg                   snoop_req_valid_o,
    output reg  [ADDR_W-1:0]     snoop_req_addr_o,
    output reg  [2:0]            snoop_req_permissions_o,
    input  wire                  snoop_req_ready_i,

    input  wire                  snoop_rsp_valid_i,
    input  wire                  snoop_rsp_has_data_i,
    input  wire [DATA_W-1:0]     snoop_rsp_data_i,
    input  wire [2:0]            snoop_rsp_permissions_i,
    output wire                  snoop_rsp_ready_o,

    // l1_tilelink_adapter.v L1-facing interface -- same port shape as
    // tlc_xbar_fabric_top.v's mX_req_*/mX_data_* master port, so this
    // bridge drops onto any free master slot (e.g. m0) unchanged.
    output reg                    l1_request_valid,
    output reg  [31:0]            l1_request_addr,
    output reg  [2:0]             l1_request_type,
    output reg  [255:0]           l1_request_data,
    output reg  [2:0]             l1_request_permissions,
    input  wire                   l1_request_ready,

    input  wire                   data_to_l1_valid,
    input  wire [255:0]           data_to_l1_data,
    input  wire                   data_to_l1_error,

    // Probe side-band from tlc_xbar_fabric_top / l1_tilelink_adapter.
    input  wire                   probe_req_to_l1_valid,
    input  wire [31:0]            probe_req_to_l1_addr,
    input  wire [2:0]             probe_req_to_l1_permissions,

    output reg                    probe_ack_from_l1_valid,
    output reg  [31:0]            probe_ack_from_l1_addr,
    output reg  [2:0]             probe_ack_from_l1_permissions,
    output reg  [255:0]           probe_ack_from_l1_dirty_data
);

    `include "tlc64b2M_params.v"

    localparam [2:0] S_ACCEPT = 3'd0,  // mem_a_ready_o high; take one beat from L2
                     S_L1_REQ = 3'd1,  // drive l1_request_* until l1_request_ready
                     S_L1_RESP= 3'd2,  // wait for this beat's adapter completion
                     S_D_BEAT = 3'd3;  // hand a D-channel beat back to L2

    reg [2:0] state;
    reg [2:0] beat_cnt;
    reg       is_write;
    reg [ADDR_W-1:0] base_addr;
    reg [DATA_W-1:0] cur_wdata;
    reg [DATA_W-1:0] cur_rdata;

    reg [31:0] probe_addr_q;
    reg       probe_req_accepted_q;
    reg       probe_waiting_rsp_q;
    reg       probe_ack_pending_q;

    assign mem_a_ready_o = (state == S_ACCEPT);

    // Combinational, state-derived D-channel outputs -- deliberately not a
    // registered "assert then cancel if ready" pair. rv64g_l2_fsm.v (and
    // this testbench) hold mem_d_ready_o high continuously through
    // ST_MEM_READ/ST_MEM_WRITE/ST_MEM_RESP, so a registered mem_d_valid_o
    // that both sets to 1 and checks mem_d_ready_i for cancellation in the
    // same state-entry cycle never actually produces a visible 1: both the
    // set and the ready-driven clear happen in the same clock edge's NBA
    // update. Deriving mem_d_valid_o straight from `state` guarantees it is
    // asserted for a full cycle once S_D_BEAT is entered, regardless of
    // when ready arrives (confirmed by simulation -- the registered form
    // hung waiting for a mem_d_valid_o pulse that was never externally
    // observable).
    assign mem_d_valid_o  = (state == S_D_BEAT);
    assign mem_d_opcode_o = is_write ? D_OPCODE_ACCESS_ACK : D_OPCODE_ACCESS_ACK_DATA;
    assign mem_d_data_o   = is_write ? {DATA_W{1'b0}} : cur_rdata;

    wire [31:0] beat_addr = base_addr[31:0] + {26'b0, beat_cnt, 3'b0};
    assign snoop_rsp_ready_o = probe_waiting_rsp_q;

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (rst_n && state == S_ACCEPT && mem_a_valid_i && beat_cnt == 3'd0 &&
            mem_a_address_i[ADDR_W-1:32] != {(ADDR_W-32){1'b0}}) begin
            $error("[L2_BRIDGE_ASSERT] mem_a_address_i upper bits nonzero -- 32-bit l1_request_addr would silently truncate 0x%016h", mem_a_address_i);
        end

    end
`endif

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                         <= S_ACCEPT;
            beat_cnt                      <= 3'd0;
            is_write                      <= 1'b0;
            base_addr                     <= {ADDR_W{1'b0}};
            cur_wdata                     <= {DATA_W{1'b0}};
            cur_rdata                     <= {DATA_W{1'b0}};
            probe_addr_q                  <= 32'b0;
            probe_req_accepted_q          <= 1'b0;
            probe_waiting_rsp_q           <= 1'b0;
            probe_ack_pending_q           <= 1'b0;

            l1_request_valid              <= 1'b0;
            l1_request_addr               <= 32'b0;
            l1_request_type               <= 3'b0;
            l1_request_data               <= 256'b0;
            l1_request_permissions        <= 3'b0;

            probe_ack_from_l1_valid       <= 1'b0;
            probe_ack_from_l1_addr        <= 32'b0;
            probe_ack_from_l1_permissions <= PARAM_NtoN;
            probe_ack_from_l1_dirty_data  <= 256'b0;
            snoop_req_valid_o             <= 1'b0;
            snoop_req_addr_o              <= {ADDR_W{1'b0}};
            snoop_req_permissions_o       <= PARAM_NtoN;
        end else begin
            l1_request_valid <= 1'b0;
            probe_ack_from_l1_valid <= probe_ack_pending_q;
            snoop_req_valid_o <= snoop_req_valid_o;
            snoop_req_addr_o <= snoop_req_addr_o;
            snoop_req_permissions_o <= snoop_req_permissions_o;

            if (probe_ack_pending_q && !probe_req_to_l1_valid) begin
                probe_ack_pending_q <= 1'b0;
                probe_ack_from_l1_valid <= 1'b0;
            end

            if (probe_req_to_l1_valid && !probe_req_accepted_q && !probe_waiting_rsp_q) begin
                snoop_req_valid_o       <= 1'b1;
                snoop_req_addr_o        <= {{(ADDR_W-32){1'b0}}, probe_req_to_l1_addr};
                snoop_req_permissions_o <= probe_req_to_l1_permissions;
                probe_addr_q            <= probe_req_to_l1_addr;
                if (snoop_req_ready_i) begin
                    probe_req_accepted_q <= 1'b1;
                    probe_waiting_rsp_q  <= 1'b1;
                    snoop_req_valid_o    <= 1'b0;
                end
            end else if (snoop_req_valid_o && snoop_req_ready_i) begin
                probe_req_accepted_q <= 1'b1;
                probe_waiting_rsp_q  <= 1'b1;
                snoop_req_valid_o    <= 1'b0;
            end

            if (!probe_req_to_l1_valid && !probe_waiting_rsp_q) begin
                probe_req_accepted_q <= 1'b0;
            end

            if (probe_waiting_rsp_q && snoop_rsp_valid_i) begin
                probe_ack_pending_q           <= 1'b1;
                probe_ack_from_l1_valid       <= 1'b1;
                probe_ack_from_l1_addr        <= probe_addr_q;
                probe_ack_from_l1_permissions <= snoop_rsp_permissions_i;
                probe_ack_from_l1_dirty_data  <= snoop_rsp_has_data_i ? {{(256-DATA_W){1'b0}}, snoop_rsp_data_i} : 256'b0;
                probe_waiting_rsp_q           <= 1'b0;
            end

            case (state)
                S_ACCEPT: begin
                    if (mem_a_valid_i) begin
                        if (beat_cnt == 3'd0) begin
                            base_addr <= mem_a_address_i;
                            is_write  <= (mem_a_opcode_i == A_OPCODE_PUT_FULL_DATA);
                        end
                        cur_wdata <= mem_a_data_i;
                        state     <= S_L1_REQ;
                    end
                end

                S_L1_REQ: begin
                    l1_request_valid       <= 1'b1;
                    l1_request_type        <= is_write ? L1_REQ_UNCACHED_WRITE : L1_REQ_READ_MISS;
                    l1_request_addr        <= (beat_cnt == 3'd0) ? base_addr[31:0] : beat_addr;
                    l1_request_data        <= {{(256-DATA_W){1'b0}}, cur_wdata};
                    l1_request_permissions <= is_write ? 3'b0 : PARAM_NtoB;
                    if (l1_request_ready) begin
                        l1_request_valid <= 1'b0;
                        state            <= S_L1_RESP;
                    end
                end

                // For writes we still rely on the uncached AccessAck pulse.
                // For reads the coherent path returns GrantData, which the
                // adapter also reflects as data_to_l1_valid/data_to_l1_data.
                S_L1_RESP: begin
                    if (data_to_l1_valid) begin
                        if (is_write) begin
                            if (beat_cnt == 3'd7) begin
                                state <= S_D_BEAT; // final AccessAck to L2
                            end else begin
                                beat_cnt <= beat_cnt + 3'd1;
                                state    <= S_ACCEPT;
                            end
                        end else begin
                            cur_rdata <= data_to_l1_data[DATA_W-1:0];
                            state     <= S_D_BEAT;
                        end
                    end
                end

                S_D_BEAT: begin
                    if (mem_d_ready_i) begin
                        if (is_write) begin
                            beat_cnt <= 3'd0;
                            state    <= S_ACCEPT; // whole write transaction done
                        end else if (beat_cnt == 3'd7) begin
                            beat_cnt <= 3'd0;
                            state    <= S_ACCEPT; // whole read transaction done
                        end else begin
                            beat_cnt <= beat_cnt + 3'd1;
                            state    <= S_L1_REQ; // next read beat
                        end
                    end
                end

                default: begin
                    state <= S_ACCEPT;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
