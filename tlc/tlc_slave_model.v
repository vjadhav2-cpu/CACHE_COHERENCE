// =============================================================================
// tlc_slave_model.v  —  Behavioral TileLink-C slave for crossbar testing
// =============================================================================
//
// Implements a minimal coherent manager:
//   • Direct-mapped directory  (DIR_DEPTH entries)
//   • Behavioral memory array  (MEM_LINES × DATA_W words)
//   • Full 5-channel TL-C state machine (A/B/C/D/E)
//
// Designed to plug directly into the tl_xbar_nm slave ports (single arbitrated
// channel per slave, NOT the original l2_tilelink_adapter's 4-master packed bus).
//
// ─── Source / sink width convention ─────────────────────────────────────────
//   a_source / c_source  : M_SOURCE_W bits — {master_id, orig_source}
//                          The crossbar prepends master_id before forwarding.
//   d_source             : M_SOURCE_W bits — echoed verbatim from A/C source.
//                          The crossbar uses upper MST_ID_W bits to route D back.
//   b_dest               : MST_ID_W bits — which master the crossbar should probe.
//   d_sink / e_sink      : SINK_W bits — slave-allocated IDs; the crossbar
//                          prepends slave_id externally for E-channel routing.
//
// ─── FSM summary ─────────────────────────────────────────────────────────────
//   S_IDLE          Accept A (Acquire) or C (Release/ReleaseData)
//   S_DIR_LOOKUP    1-cycle directory lookup; compute which masters to probe
//   S_PROBE_SEND    Drive B channel probe to one master (lowest pending bit)
//   S_PROBE_WAIT    Wait for ProbeAck/ProbeAckData on C channel
//   S_GRANT_SEND    Drive D channel GrantData (or Grant for AcquirePerm)
//   S_GRANTACK_WAIT Wait for E channel GrantAck; update directory ownership
//   S_RELEASEACK    Drive D channel ReleaseAck
// =============================================================================
`timescale 1ns/10ps
/* verilator lint_off UNUSEDSIGNAL */

module tlc_slave_model #(
    parameter integer SLAVE_ID   = 0,
    parameter integer ADDR_W     = 32,
    parameter integer DATA_W     = 64,
    parameter integer SOURCE_W   = 4,    // original source width  (for B.source field)
    parameter integer M_SOURCE_W = 6,    // extended source = SOURCE_W + MST_ID_W
    parameter integer SINK_W     = 4,    // slave-allocated sink IDs
    parameter integer N_MASTERS  = 3,
    parameter integer MST_ID_W   = 2,    // $clog2(N_MASTERS)
    parameter integer MEM_LINES  = 256,  // behavioral memory depth (words)
    parameter integer DIR_DEPTH  = 32    // directory entries
) (
    input  wire clk,
    input  wire rst_n,

    // ── A channel: crossbar → slave ──────────────────────────────────────
    input  wire                    a_valid,
    output wire                    a_ready,
    input  wire [2:0]              a_opcode,
    input  wire [2:0]              a_param,
    input  wire [3:0]              a_size,
    input  wire [M_SOURCE_W-1:0]   a_source,
    input  wire [ADDR_W-1:0]       a_address,
    input  wire [7:0]              a_mask,
    input  wire [DATA_W-1:0]       a_data,
    input  wire                    a_corrupt,

    // ── B channel: slave → crossbar (b_dest selects target master) ───────
    output reg                     b_valid,
    input  wire                    b_ready,
    output reg  [2:0]              b_opcode,
    output reg  [2:0]              b_param,
    output reg  [3:0]              b_size,
    output reg  [SOURCE_W-1:0]     b_source,
    output reg  [ADDR_W-1:0]       b_address,
    output reg  [7:0]              b_mask,
    output reg  [DATA_W-1:0]       b_data,
    output reg                     b_corrupt,
    output reg  [MST_ID_W-1:0]     b_dest,

    // ── C channel: crossbar → slave ──────────────────────────────────────
    input  wire                    c_valid,
    output wire                    c_ready,
    input  wire [2:0]              c_opcode,
    input  wire [2:0]              c_param,
    input  wire [3:0]              c_size,
    input  wire [M_SOURCE_W-1:0]   c_source,
    input  wire [ADDR_W-1:0]       c_address,
    input  wire [DATA_W-1:0]       c_data,
    input  wire                    c_corrupt,

    // ── D channel: slave → crossbar (d_source echoes A/C extended source) ─
    output reg                     d_valid,
    input  wire                    d_ready,
    output reg  [2:0]              d_opcode,
    output reg  [1:0]              d_param,
    output reg  [3:0]              d_size,
    output reg  [M_SOURCE_W-1:0]   d_source,
    output reg  [SINK_W-1:0]       d_sink,
    output reg                     d_denied,
    output reg  [DATA_W-1:0]       d_data,
    output reg                     d_corrupt,

    // ── E channel: crossbar → slave (slave_id prefix already stripped) ────
    input  wire                    e_valid,
    output wire                    e_ready,
    input  wire [SINK_W-1:0]       e_sink
);

    // ── Combinational ready signals ──────────────────────────────────────
    // These MUST be combinational so they reflect the slave's true capacity
    // to accept this cycle.  Registered ready signals create a 1-cycle
    // window where a master can mistakenly think its request was accepted
    // after the slave has already transitioned.
    //
    // (state variable is declared below; we forward-reference it.)

    // ── TL-C opcode constants ─────────────────────────────────────────────
    localparam [2:0] TL_A_ACQUIRE_BLOCK  = 3'd6;
    localparam [2:0] TL_A_ACQUIRE_PERM   = 3'd7;
    localparam [2:0] TL_B_PROBE          = 3'd6;
    localparam [2:0] TL_C_PROBE_ACK      = 3'd4;
    localparam [2:0] TL_C_PROBE_ACK_DATA = 3'd5;
    localparam [2:0] TL_C_RELEASE        = 3'd6;
    localparam [2:0] TL_C_RELEASE_DATA   = 3'd7;
    localparam [2:0] TL_D_GRANT          = 3'd4;
    localparam [2:0] TL_D_GRANT_DATA     = 3'd5;
    localparam [2:0] TL_D_RELEASE_ACK    = 3'd6;

    // Coherence state encoding
    localparam [1:0] COH_I = 2'b00;  // Invalid
    localparam [1:0] COH_B = 2'b01;  // Branch (shared, read-only copies exist)
    localparam [1:0] COH_T = 2'b10;  // Trunk  (one exclusive / dirty owner)

    // Acquire param encoding (matches TileLink NtoB/NtoT/BtoT = 0/1/2)
    localparam [2:0] PARAM_NtoB = 3'd0;
    localparam [2:0] PARAM_NtoT = 3'd1;
    localparam [2:0] PARAM_BtoT = 3'd2;

    // FSM state encoding
    localparam [2:0] S_IDLE          = 3'd0;
    localparam [2:0] S_DIR_LOOKUP    = 3'd1;
    localparam [2:0] S_PROBE_SEND    = 3'd2;
    localparam [2:0] S_PROBE_WAIT    = 3'd3;
    localparam [2:0] S_GRANT_SEND    = 3'd4;
    localparam [2:0] S_GRANTACK_WAIT = 3'd5;
    localparam [2:0] S_RELEASEACK    = 3'd6;

    reg [2:0] state;

    // ── Combinational ready signals ──────────────────────────────────────
    // Must reflect the slave's true acceptance capacity THIS cycle to avoid
    // a 1-cycle window where a master sees ready=1 even though the slave
    // has already transitioned away from accepting that channel.
    assign a_ready = (state == S_IDLE);
    assign c_ready = (state == S_IDLE) || (state == S_PROBE_WAIT);
    assign e_ready = (state == S_GRANTACK_WAIT);

    // ── Address-index helper widths ───────────────────────────────────────
    localparam DIR_IDX_W = $clog2(DIR_DEPTH);   // 5 for DIR_DEPTH=32
    localparam MEM_IDX_W = $clog2(MEM_LINES);   // 8 for MEM_LINES=256
    // Directory tag occupies top bits above the index and 3-bit byte-offset
    localparam DIR_TAG_W = ADDR_W - DIR_IDX_W - 3;  // 32-5-3 = 24

    // ── Pending-transaction registers ─────────────────────────────────────
    reg [M_SOURCE_W-1:0]  req_source;     // extended source saved from A/C
    reg [ADDR_W-1:0]      req_addr;       // address of current transaction
    reg [2:0]             req_opcode;     // A.opcode or C.opcode
    reg [2:0]             req_param;      // A.param (NtoB/NtoT/BtoT)
    reg [3:0]             req_size;
    reg                   req_is_release; // 1 = voluntary Release, not Acquire

    // Probe bookkeeping: bit-per-master; 1 = still needs a probe sent
    reg [N_MASTERS-1:0]   probe_pending;

    // Sink ID allocator (simple rolling counter)
    reg [SINK_W-1:0]      sink_cnt;

    // ── Directory: direct-mapped on addr[DIR_IDX_W+2 : 3] ────────────────
    reg                    dir_v    [0:DIR_DEPTH-1];
    reg [DIR_TAG_W-1:0]    dir_tag  [0:DIR_DEPTH-1];
    reg [1:0]              dir_coh  [0:DIR_DEPTH-1];
    reg [N_MASTERS-1:0]    dir_pres [0:DIR_DEPTH-1];

    // ── Behavioral memory: one DATA_W word per line ───────────────────────
    reg [DATA_W-1:0] mem [0:MEM_LINES-1];

    // ── Combinational helpers derived from req_addr ───────────────────────
    wire [DIR_IDX_W-1:0] req_dir_idx = req_addr[DIR_IDX_W+2 : 3];
    wire [DIR_TAG_W-1:0] req_dir_tag = req_addr[ADDR_W-1    : DIR_IDX_W+3];
    wire [MEM_IDX_W-1:0] req_mem_idx = req_addr[MEM_IDX_W+2 : 3];

    wire dir_hit = dir_v[req_dir_idx] && (dir_tag[req_dir_idx] == req_dir_tag);

    // Requester's master_id (upper MST_ID_W bits of extended source)
    wire [MST_ID_W-1:0]   req_mst_id = req_source[M_SOURCE_W-1 : SOURCE_W];

    // One-hot mask for the requester (used to exclude from probes)
    wire [N_MASTERS-1:0]  req_mst_mask = {{(N_MASTERS-1){1'b0}}, 1'b1} << req_mst_id;

    // Combinational probe set (computed from directory state):
    //   NtoT / BtoT → invalidate all other sharers
    //   NtoB        → downgrade T holder only (if exclusive copy exists)
    wire [N_MASTERS-1:0] dir_pres_now  = dir_hit ? dir_pres[req_dir_idx] : {N_MASTERS{1'b0}};
    wire [1:0]           dir_coh_now   = dir_hit ? dir_coh [req_dir_idx] : COH_I;

    wire [N_MASTERS-1:0] probes_excl   = dir_pres_now & ~req_mst_mask; // for excl acquire
    wire [N_MASTERS-1:0] probes_shared = (dir_coh_now == COH_T) ? probes_excl : {N_MASTERS{1'b0}};
    wire [N_MASTERS-1:0] probes_needed = ((req_param == PARAM_NtoT) || (req_param == PARAM_BtoT))
                                         ? probes_excl : probes_shared;

    // Priority-encode probe_pending → index of lowest-set master to probe next
    wire [N_MASTERS-1:0] probe_next_oh; // one-hot: lowest set bit of probe_pending
    genvar gp;
    generate
        for (gp = 0; gp < N_MASTERS; gp = gp + 1) begin : g_probe_pri
            if (gp == 0)
                assign probe_next_oh[0] = probe_pending[0];
            else
                assign probe_next_oh[gp] = probe_pending[gp] && !(|(probe_pending[gp-1:0]));
        end
    endgenerate

    // Binary encode one-hot probe_next_oh → b_dest
    integer k;
    reg [MST_ID_W-1:0] probe_next_idx;
    always @(*) begin
        probe_next_idx = {MST_ID_W{1'b0}};
        for (k = 0; k < N_MASTERS; k = k + 1)
            if (probe_next_oh[k])
                probe_next_idx = k[MST_ID_W-1:0];
    end

    // ── Integer loop variable for reset ───────────────────────────────────
    integer ri;

    // ─────────────────────────────────────────────────────────────────────
    // Main FSM
    // ─────────────────────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            req_source     <= {M_SOURCE_W{1'b0}};
            req_addr       <= {ADDR_W{1'b0}};
            req_opcode     <= 3'b0;
            req_param      <= 3'b0;
            req_size       <= 4'b0;
            req_is_release <= 1'b0;
            probe_pending  <= {N_MASTERS{1'b0}};
            sink_cnt       <= {SINK_W{1'b0}};

            // Output resets (a_ready/c_ready/e_ready are combinational)
            b_valid  <= 1'b0;  b_opcode <= 3'b0;    b_param <= 3'b0;
            b_size   <= 4'b0;  b_source <= {SOURCE_W{1'b0}};
            b_address<= {ADDR_W{1'b0}};
            b_mask   <= 8'hFF; b_data   <= {DATA_W{1'b0}};
            b_corrupt<= 1'b0;  b_dest   <= {MST_ID_W{1'b0}};
            d_valid  <= 1'b0;  d_opcode <= 3'b0;    d_param  <= 2'b0;
            d_size   <= 4'b0;  d_source <= {M_SOURCE_W{1'b0}};
            d_sink   <= {SINK_W{1'b0}};
            d_denied <= 1'b0;  d_data   <= {DATA_W{1'b0}};  d_corrupt <= 1'b0;

            for (ri = 0; ri < DIR_DEPTH; ri = ri + 1) begin
                dir_v[ri]    <= 1'b0;
                dir_tag[ri]  <= {DIR_TAG_W{1'b0}};
                dir_coh[ri]  <= COH_I;
                dir_pres[ri] <= {N_MASTERS{1'b0}};
            end
        end
        else begin
            // ── Default: deassert one-shot signals ────────────────────────
            // (a_ready/c_ready/e_ready are now combinational outputs.)
            b_valid <= 1'b0;
            d_valid <= 1'b0;

            case (state)

                // ─────────────────────────────────────────────────────────
                // IDLE: ready to accept an Acquire (A) or a Release (C)
                // ─────────────────────────────────────────────────────────
                S_IDLE: begin
                    if (a_valid) begin
                        // Acquire request
                        req_source     <= a_source;
                        req_addr       <= a_address;
                        req_opcode     <= a_opcode;
                        req_param      <= a_param;
                        req_size       <= a_size;
                        req_is_release <= 1'b0;
                        state          <= S_DIR_LOOKUP;
                    end
                    else if (c_valid &&
                             (c_opcode == TL_C_RELEASE || c_opcode == TL_C_RELEASE_DATA)) begin
                        // Voluntary eviction (Release / ReleaseData)
                        req_source     <= c_source;
                        req_addr       <= c_address;
                        req_size       <= c_size;
                        req_is_release <= 1'b1;

                        // Store dirty data into the behavioral memory immediately
                        if (c_opcode == TL_C_RELEASE_DATA)
                            mem[c_address[MEM_IDX_W+2 : 3]] <= c_data;

                        // Remove this master from the directory's presence bitmask.
                        // (The releasing master is identified by c_source's upper bits.)
                        if (dir_v[c_address[DIR_IDX_W+2:3]] &&
                            dir_tag[c_address[DIR_IDX_W+2:3]] ==
                            c_address[ADDR_W-1 : DIR_IDX_W+3]) begin
                            dir_pres[c_address[DIR_IDX_W+2:3]] <=
                                dir_pres[c_address[DIR_IDX_W+2:3]] &
                                ~({{(N_MASTERS-1){1'b0}}, 1'b1} << c_source[M_SOURCE_W-1:SOURCE_W]);
                            if ((dir_pres[c_address[DIR_IDX_W+2:3]] &
                                 ~({{(N_MASTERS-1){1'b0}}, 1'b1} << c_source[M_SOURCE_W-1:SOURCE_W])) ==
                                {N_MASTERS{1'b0}}) begin
                                dir_v[c_address[DIR_IDX_W+2:3]]   <= 1'b0;
                                dir_tag[c_address[DIR_IDX_W+2:3]] <= {DIR_TAG_W{1'b0}};
                                dir_coh[c_address[DIR_IDX_W+2:3]]  <= COH_I;
                            end
                        end

                        state <= S_RELEASEACK;
                    end
                end

                // ─────────────────────────────────────────────────────────
                // DIR_LOOKUP: single-cycle lookup; compute probe_pending
                // ─────────────────────────────────────────────────────────
                S_DIR_LOOKUP: begin
                    // probes_needed is combinatorially driven from req_addr
                    // (wires at module scope); simply register the result.
                    probe_pending <= probes_needed;
                    state         <= (probes_needed != {N_MASTERS{1'b0}})
                                     ? S_PROBE_SEND : S_GRANT_SEND;
                end

                // ─────────────────────────────────────────────────────────
                // PROBE_SEND: assert B channel to the next pending master
                // ─────────────────────────────────────────────────────────
                S_PROBE_SEND: begin
                    b_valid   <= 1'b1;
                    b_opcode  <= TL_B_PROBE;
                    b_param   <= 3'd1;            // toN — invalidate the copy
                    b_size    <= req_size;
                    b_source  <= {SOURCE_W{1'b0}}; // probe source (can be 0 for simple model)
                    b_address <= req_addr;
                    b_mask    <= 8'hFF;
                    b_data    <= {DATA_W{1'b0}};
                    b_corrupt <= 1'b0;
                    b_dest    <= probe_next_idx;  // crossbar routes this B to the target master

                    if (b_ready) begin
                        // Handshake complete: clear this master's pending bit
                        probe_pending[probe_next_idx] <= 1'b0;
                        b_valid <= 1'b0;
                        state   <= S_PROBE_WAIT;
                    end
                end

                // ─────────────────────────────────────────────────────────
                // PROBE_WAIT: wait for ProbeAck / ProbeAckData on C channel
                // ─────────────────────────────────────────────────────────
                S_PROBE_WAIT: begin
                    if (c_valid &&
                        (c_opcode == TL_C_PROBE_ACK || c_opcode == TL_C_PROBE_ACK_DATA)) begin
                        // If the master returned dirty data, write it to memory
                        if (c_opcode == TL_C_PROBE_ACK_DATA)
                            mem[req_mem_idx] <= c_data;

                        // Are there more masters to probe?
                        // Note: probe_pending was cleared in PROBE_SEND for the
                        // master whose ack we just received.  Non-zero ⟹ more probes.
                        if (probe_pending != {N_MASTERS{1'b0}})
                            state <= S_PROBE_SEND;
                        else
                            state <= S_GRANT_SEND;
                    end
                end

                // ─────────────────────────────────────────────────────────
                // GRANT_SEND: issue D channel Grant or GrantData
                //   First cycle: assert d_valid and load outputs.
                //   Hold until d_ready (crossbar accepted the D beat).
                // ─────────────────────────────────────────────────────────
                S_GRANT_SEND: begin
                    if (!d_valid) begin
                        // Allocate a fresh sink ID
                        d_sink   <= sink_cnt;
                        sink_cnt <= sink_cnt + 1;

                        d_valid   <= 1'b1;
                        // AcquirePerm (permission-only upgrade) → Grant (no data)
                        // AcquireBlock → GrantData (return cache-line data)
                        d_opcode  <= (req_opcode == TL_A_ACQUIRE_PERM)
                                     ? TL_D_GRANT : TL_D_GRANT_DATA;
                        // d_param: capability being granted
                        //   NtoB → 2'b01 (ToT not appropriate; grant ToB)
                        //   NtoT/BtoT → 2'b00 (ToT)
                        d_param   <= (req_param == PARAM_NtoB) ? 2'b01 : 2'b00;
                        d_size    <= req_size;
                        d_source  <= req_source;  // echo extended source for D routing
                        d_denied  <= 1'b0;
                        d_data    <= (req_opcode == TL_A_ACQUIRE_PERM)
                                     ? {DATA_W{1'b0}} : mem[req_mem_idx];
                        d_corrupt <= 1'b0;

                        // Update directory to reflect new ownership immediately.
                        // (Formal update waits for GrantAck, but this model accepts
                        //  the simplified behavior for testing.)
                        dir_v  [req_dir_idx] <= 1'b1;
                        dir_tag[req_dir_idx] <= req_dir_tag;
                        if (req_param == PARAM_NtoB) begin
                            // Shared: add requester to presence bitmask
                            dir_coh [req_dir_idx] <= COH_B;
                            dir_pres[req_dir_idx] <=
                                (dir_hit ? dir_pres[req_dir_idx] : {N_MASTERS{1'b0}})
                                | req_mst_mask;
                        end
                        else begin
                            // Exclusive: only the requester holds the line
                            dir_coh [req_dir_idx] <= COH_T;
                            dir_pres[req_dir_idx] <= req_mst_mask;
                        end
                    end
                    else if (d_ready) begin
                        // D handshake complete
                        d_valid <= 1'b0;
                        state   <= S_GRANTACK_WAIT;
                    end
                end

                // ─────────────────────────────────────────────────────────
                // GRANTACK_WAIT: wait for E channel GrantAck from master
                // ─────────────────────────────────────────────────────────
                S_GRANTACK_WAIT: begin
                    if (e_valid) begin
                        state <= S_IDLE;
                    end
                end

                // ─────────────────────────────────────────────────────────
                // RELEASEACK: send D channel ReleaseAck
                // ─────────────────────────────────────────────────────────
                S_RELEASEACK: begin
                    if (!d_valid) begin
                        d_valid   <= 1'b1;
                        d_opcode  <= TL_D_RELEASE_ACK;
                        d_param   <= 2'b00;
                        d_size    <= req_size;
                        d_source  <= req_source;  // echo C's extended source for routing
                        d_sink    <= {SINK_W{1'b0}};
                        d_denied  <= 1'b0;
                        d_data    <= {DATA_W{1'b0}};
                        d_corrupt <= 1'b0;
                    end
                    else if (d_ready) begin
                        d_valid <= 1'b0;
                        state   <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
