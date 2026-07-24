// =============================================================================
// tlc_xbar_top.v  —  3-master × 2-slave TileLink-C crossbar test system
// =============================================================================
//
// Instantiates:
//   • 3 × l1_tilelink_adapter  (masters 0, 1, 2 — one L1 cache controller each)
//   • tl_xbar_nm               (3M × 2S coherent crossbar)
//   • 2 × tlc_slave_model      (behavioral TL-C slaves with directory + memory)
//
// Address map
//   Slave 0:  0x0000_0000 – 0x7FFF_FFFF  (addr[31] = 0, lower 2 GB)
//   Slave 1:  0x8000_0000 – 0xFFFF_FFFF  (addr[31] = 1, upper 2 GB)
//
// ── Interface-width notes ────────────────────────────────────────────────────
//
//   SOURCE_W = 4, SINK_W = 4, N_MASTERS = 3, N_SLAVES = 2
//   MST_ID_W = 2  (clog2(3))   → M_SOURCE_W = 6
//   SLV_ID_W = 1  (clog2(2))   → M_SINK_W   = 5
//
//   The crossbar extends D.sink to M_SINK_W = 5 bits before forwarding to
//   masters.  l1_tilelink_adapter expects only 4 bits (d_sink[3:0]) and echoes
//   it on E (e_sink[3:0]).  The top level bridges this gap:
//     1. mXX_d_slv_id  captures bit [4] of the 5-bit D sink when a Grant/
//        GrantData arrives.
//     2. When the adapter fires E, the top level prepends the stored slave_id
//        to form the full 5-bit mst_e_sink_i that the crossbar needs for E
//        channel routing.
//
//   Probe side-band
//     l1_tilelink_adapter needs probe_req_to_l1 to advance its FSM from
//     STATE_PROBE_PROCESS → STATE_PROBE_WAIT.  We derive it directly from
//     the crossbar's mst_b_valid_o / mst_b_address_o / mst_b_param_o.
//     The probe_ack_from_l1_* inputs are stubbed: NtoN (no copy) after a
//     2-cycle pipeline from B channel acceptance, keeping things simple for
//     crossbar routing tests.  Replace these stubs with actual L1 cache
//     controller connections for full-system verification.
//
// ── Top-level I/O ────────────────────────────────────────────────────────────
//   Exposes the L1 cache-controller-side interface of each l1_tilelink_adapter
//   so a testbench or external stimulus can drive read-miss / write-miss /
//   write-back / uncached requests and observe data responses.
// =============================================================================
`timescale 1ns/10ps

module tlc_xbar_top (
    input  wire clk,
    input  wire rst_n,

    // ── L1 Cache Controller — Master 0 ────────────────────────────────────
    input  wire          m0_req_valid,
    input  wire [31:0]   m0_req_addr,
    input  wire [2:0]    m0_req_type,        // 0=ReadMiss 1=WriteMiss 2=WriteBack 3/4=Uncached
    input  wire [255:0]  m0_req_data,        // write-back data (4 × 64-bit beats)
    input  wire [2:0]    m0_req_permissions, // NtoB / NtoT / BtoT / TtoN / BtoN
    output wire          m0_req_ready,

    output wire          m0_data_valid,
    output wire [255:0]  m0_data,
    output wire          m0_data_error,

    output wire          m0_probe_req_valid,
    output wire [31:0]   m0_probe_req_addr,
    output wire [2:0]    m0_probe_req_permissions,

    input  wire          m0_probe_ack_valid,
    input  wire [31:0]   m0_probe_ack_addr,
    input  wire [2:0]    m0_probe_ack_permissions,
    input  wire [255:0]  m0_probe_ack_dirty_data,

    // ── L1 Cache Controller — Master 1 ────────────────────────────────────
    input  wire          m1_req_valid,
    input  wire [31:0]   m1_req_addr,
    input  wire [2:0]    m1_req_type,
    input  wire [255:0]  m1_req_data,
    input  wire [2:0]    m1_req_permissions,
    output wire          m1_req_ready,

    output wire          m1_data_valid,
    output wire [255:0]  m1_data,
    output wire          m1_data_error,

    output wire          m1_probe_req_valid,
    output wire [31:0]   m1_probe_req_addr,
    output wire [2:0]    m1_probe_req_permissions,

    input  wire          m1_probe_ack_valid,
    input  wire [31:0]   m1_probe_ack_addr,
    input  wire [2:0]    m1_probe_ack_permissions,
    input  wire [255:0]  m1_probe_ack_dirty_data,

    // ── L1 Cache Controller — Master 2 ────────────────────────────────────
    input  wire          m2_req_valid,
    input  wire [31:0]   m2_req_addr,
    input  wire [2:0]    m2_req_type,
    input  wire [255:0]  m2_req_data,
    input  wire [2:0]    m2_req_permissions,
    output wire          m2_req_ready,

    output wire          m2_data_valid,
    output wire [255:0]  m2_data,
    output wire          m2_data_error,

    output wire          m2_probe_req_valid,
    output wire [31:0]   m2_probe_req_addr,
    output wire [2:0]    m2_probe_req_permissions,

    input  wire          m2_probe_ack_valid,
    input  wire [31:0]   m2_probe_ack_addr,
    input  wire [2:0]    m2_probe_ack_permissions,
    input  wire [255:0]  m2_probe_ack_dirty_data
);

    // ═══════════════════════════════════════════════════════════════════════
    // Parameters
    // ═══════════════════════════════════════════════════════════════════════
    localparam ADDR_W     = 32;
    localparam DATA_W     = 64;
    localparam SOURCE_W   = 4;   // l1_tilelink_adapter uses 4-bit source/sink
    localparam SINK_W     = 4;
    localparam N_MASTERS  = 3;
    localparam N_SLAVES   = 2;
    // Derived (must match tl_xbar_nm defaults — do not change independently)
    localparam MST_ID_W   = 2;   // $clog2(3)
    localparam SLV_ID_W   = 1;   // $clog2(2)
    localparam M_SOURCE_W = SOURCE_W + MST_ID_W;  // 6
    localparam M_SINK_W   = SINK_W   + SLV_ID_W;  // 5

    // D opcode values (must match params.vh / tlc64b2M_params.v)
    localparam D_GRANT      = 3'd4;
    localparam D_GRANT_DATA = 3'd5;

    // ═══════════════════════════════════════════════════════════════════════
    // TileLink channel wires between l1_tilelink_adapters and tl_xbar_nm
    // ═══════════════════════════════════════════════════════════════════════

    // ── A channel (master → crossbar) ─────────────────────────────────────
    wire [N_MASTERS-1:0]          mst_a_valid;
    wire [N_MASTERS-1:0]          mst_a_ready;
    wire [N_MASTERS*3-1:0]        mst_a_opcode;
    wire [N_MASTERS*3-1:0]        mst_a_param;
    wire [N_MASTERS*4-1:0]        mst_a_size;
    wire [N_MASTERS*SOURCE_W-1:0] mst_a_source;
    wire [N_MASTERS*ADDR_W-1:0]   mst_a_address;
    wire [N_MASTERS*8-1:0]        mst_a_mask;
    wire [N_MASTERS*DATA_W-1:0]   mst_a_data;

    // ── B channel (crossbar → master) ─────────────────────────────────────
    wire [N_MASTERS-1:0]          mst_b_valid;
    wire [N_MASTERS-1:0]          mst_b_ready;
    wire [N_MASTERS*3-1:0]        mst_b_opcode;
    wire [N_MASTERS*3-1:0]        mst_b_param;
    wire [N_MASTERS*4-1:0]        mst_b_size;
    wire [N_MASTERS*SOURCE_W-1:0] mst_b_source;
    wire [N_MASTERS*ADDR_W-1:0]   mst_b_address;
    wire [N_MASTERS*8-1:0]        mst_b_mask;
    wire [N_MASTERS*DATA_W-1:0]   mst_b_data;

    // ── C channel (master → crossbar) ─────────────────────────────────────
    wire [N_MASTERS-1:0]          mst_c_valid;
    wire [N_MASTERS-1:0]          mst_c_ready;
    wire [N_MASTERS*3-1:0]        mst_c_opcode;
    wire [N_MASTERS*3-1:0]        mst_c_param;
    wire [N_MASTERS*4-1:0]        mst_c_size;
    wire [N_MASTERS*SOURCE_W-1:0] mst_c_source;
    wire [N_MASTERS*ADDR_W-1:0]   mst_c_address;
    wire [N_MASTERS*DATA_W-1:0]   mst_c_data;

    // ── D channel (crossbar → master)  ─ sink is M_SINK_W bits ───────────
    wire [N_MASTERS-1:0]          mst_d_valid;
    wire [N_MASTERS-1:0]          mst_d_ready;
    wire [N_MASTERS*3-1:0]        mst_d_opcode;
    wire [N_MASTERS*SOURCE_W-1:0] mst_d_source; // orig source (4 bits) returned
    wire [N_MASTERS*M_SINK_W-1:0] mst_d_sink;   // extended sink: {slave_id, orig_sink}
    wire [N_MASTERS*DATA_W-1:0]   mst_d_data;
    wire [N_MASTERS-1:0]          mst_d_denied;
    wire [N_MASTERS-1:0]          mst_d_error;  // alias d_corrupt for adapter

    // ── E channel (master → crossbar) — sink is M_SINK_W bits ────────────
    wire [N_MASTERS-1:0]          mst_e_valid;
    wire [N_MASTERS-1:0]          mst_e_ready;
    wire [N_MASTERS*M_SINK_W-1:0] mst_e_sink;   // extended: {slave_id, orig_sink}

    // ═══════════════════════════════════════════════════════════════════════
    // Sink-width bridge: capture slave_id from D, prepend to E
    //
    //   l1_tilelink_adapter drives e_sink[3:0] (SINK_W bits).
    //   tl_xbar_nm expects mst_e_sink_i to be M_SINK_W = 5 bits.
    //   We capture the upper slave_id bit from D channel when Grant/GrantData
    //   arrives and prepend it when forwarding the adapter's 4-bit E sink.
    // ═══════════════════════════════════════════════════════════════════════

    // Per-master: 4-bit d_sink and 4-bit e_sink as seen by the adapters
    wire [SOURCE_W-1:0]  m0_d_sink_4, m1_d_sink_4, m2_d_sink_4;
    wire [SOURCE_W-1:0]  m0_e_sink_4, m1_e_sink_4, m2_e_sink_4;

    // Per-master: stored slave_id bit (upper bit of M_SINK_W D sink)
    reg  m0_d_slv_id, m1_d_slv_id, m2_d_slv_id;

    // Extract the 4-bit slices the adapters see (lower SINK_W bits of ext sink)
    assign m0_d_sink_4 = mst_d_sink[0*M_SINK_W +: SINK_W];  // [3:0] of 5-bit
    assign m1_d_sink_4 = mst_d_sink[1*M_SINK_W +: SINK_W];
    assign m2_d_sink_4 = mst_d_sink[2*M_SINK_W +: SINK_W];

    // Capture slave_id when D arrives with Grant or GrantData
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            m0_d_slv_id <= 1'b0;
        else if (mst_d_valid[0] && mst_d_ready[0] &&
                 (mst_d_opcode[0*3 +: 3] == D_GRANT ||
                  mst_d_opcode[0*3 +: 3] == D_GRANT_DATA))
            m0_d_slv_id <= mst_d_sink[0*M_SINK_W + SINK_W];  // bit [4]
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            m1_d_slv_id <= 1'b0;
        else if (mst_d_valid[1] && mst_d_ready[1] &&
                 (mst_d_opcode[1*3 +: 3] == D_GRANT ||
                  mst_d_opcode[1*3 +: 3] == D_GRANT_DATA))
            m1_d_slv_id <= mst_d_sink[1*M_SINK_W + SINK_W];
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            m2_d_slv_id <= 1'b0;
        else if (mst_d_valid[2] && mst_d_ready[2] &&
                 (mst_d_opcode[2*3 +: 3] == D_GRANT ||
                  mst_d_opcode[2*3 +: 3] == D_GRANT_DATA))
            m2_d_slv_id <= mst_d_sink[2*M_SINK_W + SINK_W];
    end

    // Build the full M_SINK_W E sink that the crossbar needs
    assign mst_e_sink[0*M_SINK_W +: M_SINK_W] = {m0_d_slv_id, m0_e_sink_4};
    assign mst_e_sink[1*M_SINK_W +: M_SINK_W] = {m1_d_slv_id, m1_e_sink_4};
    assign mst_e_sink[2*M_SINK_W +: M_SINK_W] = {m2_d_slv_id, m2_e_sink_4};

    // ═══════════════════════════════════════════════════════════════════════
    // Probe side-band bridge
    //
    //   l1_tilelink_adapter has three INPUTS that an external manager drives:
    //     probe_req_to_l1_valid / addr / permissions
    //   In our crossbar design probes arrive on B channel.  We latch the
    //   accepted B probe and hold it on the side-band until the matching
    //   probe ack arrives so the adapter can progress through
    //   STATE_PROBE_PROCESS → STATE_PROBE_WAIT → STATE_PROBE_ACK_SEND.
    // ═══════════════════════════════════════════════════════════════════════
    reg        m0_probe_req_valid_r, m1_probe_req_valid_r, m2_probe_req_valid_r;
    reg [31:0] m0_probe_req_addr_r,   m1_probe_req_addr_r,   m2_probe_req_addr_r;
    reg [2:0]  m0_probe_req_permissions_r, m1_probe_req_permissions_r, m2_probe_req_permissions_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m0_probe_req_valid_r       <= 1'b0;
            m0_probe_req_addr_r        <= 32'b0;
            m0_probe_req_permissions_r <= 3'b0;
        end else begin
            if (mst_b_valid[0] && mst_b_ready[0]) begin
                m0_probe_req_valid_r       <= 1'b1;
                m0_probe_req_addr_r        <= mst_b_address[0*ADDR_W +: ADDR_W];
                m0_probe_req_permissions_r <= mst_b_param[0*3 +: 3];
            end else if (m0_probe_ack_valid) begin
                m0_probe_req_valid_r <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m1_probe_req_valid_r       <= 1'b0;
            m1_probe_req_addr_r        <= 32'b0;
            m1_probe_req_permissions_r <= 3'b0;
        end else begin
            if (mst_b_valid[1] && mst_b_ready[1]) begin
                m1_probe_req_valid_r       <= 1'b1;
                m1_probe_req_addr_r        <= mst_b_address[1*ADDR_W +: ADDR_W];
                m1_probe_req_permissions_r <= mst_b_param[1*3 +: 3];
            end else if (m1_probe_ack_valid) begin
                m1_probe_req_valid_r <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m2_probe_req_valid_r       <= 1'b0;
            m2_probe_req_addr_r        <= 32'b0;
            m2_probe_req_permissions_r <= 3'b0;
        end else begin
            if (mst_b_valid[2] && mst_b_ready[2]) begin
                m2_probe_req_valid_r       <= 1'b1;
                m2_probe_req_addr_r        <= mst_b_address[2*ADDR_W +: ADDR_W];
                m2_probe_req_permissions_r <= mst_b_param[2*3 +: 3];
            end else if (m2_probe_ack_valid) begin
                m2_probe_req_valid_r <= 1'b0;
            end
        end
    end

    assign m0_probe_req_valid       = m0_probe_req_valid_r;
    assign m0_probe_req_addr        = m0_probe_req_addr_r;
    assign m0_probe_req_permissions = m0_probe_req_permissions_r;

    assign m1_probe_req_valid       = m1_probe_req_valid_r;
    assign m1_probe_req_addr        = m1_probe_req_addr_r;
    assign m1_probe_req_permissions = m1_probe_req_permissions_r;

    assign m2_probe_req_valid       = m2_probe_req_valid_r;
    assign m2_probe_req_addr        = m2_probe_req_addr_r;
    assign m2_probe_req_permissions = m2_probe_req_permissions_r;

    // ═══════════════════════════════════════════════════════════════════════
    // l1_tilelink_adapter — Master 0
    // ═══════════════════════════════════════════════════════════════════════
    l1_tilelink_adapter u_l1_m0 (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .l1_id                      (2'd0),

        // L1 cache-controller side
        .l1_request_valid           (m0_req_valid),
        .l1_request_addr            (m0_req_addr),
        .l1_request_type            (m0_req_type),
        .l1_request_data            (m0_req_data),
        .l1_request_permissions     (m0_req_permissions),
        .l1_request_ready           (m0_req_ready),
        .data_to_l1_valid           (m0_data_valid),
        .data_to_l1_data            (m0_data),
        .data_to_l1_error           (m0_data_error),

        // Probe request from manager (relayed from B channel)
        .probe_req_to_l1_valid      (m0_probe_req_valid),
        .probe_req_to_l1_addr       (m0_probe_req_addr),
        .probe_req_to_l1_permissions(m0_probe_req_permissions),

        // Probe acknowledgement from L1 cache controller (or stub)
        .probe_ack_from_l1_valid    (m0_probe_ack_valid),
        .probe_ack_from_l1_addr     (m0_probe_ack_addr),
        .probe_ack_from_l1_permissions(m0_probe_ack_permissions),
        .probe_ack_from_l1_dirty_data(m0_probe_ack_dirty_data),

        // TileLink A
        .a_valid                    (mst_a_valid[0]),
        .a_opcode                   (mst_a_opcode[0*3 +: 3]),
        .a_param                    (mst_a_param[0*3 +: 3]),
        .a_size                     (mst_a_size[0*4 +: 4]),
        .a_source                   (mst_a_source[0*SOURCE_W +: SOURCE_W]),
        .a_address                  (mst_a_address[0*ADDR_W +: ADDR_W]),
        .a_data                     (mst_a_data[0*DATA_W +: DATA_W]),
        .a_mask                     (mst_a_mask[0*8 +: 8]),
        .a_ready                    (mst_a_ready[0]),

        // TileLink B
        .b_valid                    (mst_b_valid[0]),
        .b_opcode                   (mst_b_opcode[0*3 +: 3]),
        .b_param                    (mst_b_param[0*3 +: 3]),
        .b_size                     (mst_b_size[0*4 +: 4]),
        .b_source                   (mst_b_source[0*SOURCE_W +: SOURCE_W]),
        .b_address                  (mst_b_address[0*ADDR_W +: ADDR_W]),
        .b_data                     (mst_b_data[0*DATA_W +: DATA_W]),
        .b_mask                     (mst_b_mask[0*8 +: 8]),
        .b_ready                    (mst_b_ready[0]),

        // TileLink C
        .c_valid                    (mst_c_valid[0]),
        .c_opcode                   (mst_c_opcode[0*3 +: 3]),
        .c_param                    (mst_c_param[0*3 +: 3]),
        .c_size                     (mst_c_size[0*4 +: 4]),
        .c_source                   (mst_c_source[0*SOURCE_W +: SOURCE_W]),
        .c_address                  (mst_c_address[0*ADDR_W +: ADDR_W]),
        .c_data                     (mst_c_data[0*DATA_W +: DATA_W]),
        .c_error                    (),           // output not used by crossbar
        .c_ready                    (mst_c_ready[0]),

        // TileLink D  (adapter sees 4-bit sink; we give it the lower SINK_W bits)
        .d_valid                    (mst_d_valid[0]),
        .d_opcode                   (mst_d_opcode[0*3 +: 3]),
        .d_param                    (/* unused */),
        .d_size                     (/* unused */),
        .d_source                   (mst_d_source[0*SOURCE_W +: SOURCE_W]),
        .d_sink                     (m0_d_sink_4),
        .d_data                     (mst_d_data[0*DATA_W +: DATA_W]),
        .d_error                    (mst_d_denied[0]),
        .d_ready                    (mst_d_ready[0]),

        // TileLink E  (adapter outputs 4-bit e_sink; top level extends to 5 bits)
        .e_valid                    (mst_e_valid[0]),
        .e_sink                     (m0_e_sink_4),
        .e_ready                    (mst_e_ready[0])
    );

    // ═══════════════════════════════════════════════════════════════════════
    // l1_tilelink_adapter — Master 1
    // ═══════════════════════════════════════════════════════════════════════
    l1_tilelink_adapter u_l1_m1 (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .l1_id                      (2'd1),

        .l1_request_valid           (m1_req_valid),
        .l1_request_addr            (m1_req_addr),
        .l1_request_type            (m1_req_type),
        .l1_request_data            (m1_req_data),
        .l1_request_permissions     (m1_req_permissions),
        .l1_request_ready           (m1_req_ready),
        .data_to_l1_valid           (m1_data_valid),
        .data_to_l1_data            (m1_data),
        .data_to_l1_error           (m1_data_error),

        .probe_req_to_l1_valid      (m1_probe_req_valid),
        .probe_req_to_l1_addr       (m1_probe_req_addr),
        .probe_req_to_l1_permissions(m1_probe_req_permissions),

        .probe_ack_from_l1_valid    (m1_probe_ack_valid),
        .probe_ack_from_l1_addr     (m1_probe_ack_addr),
        .probe_ack_from_l1_permissions(m1_probe_ack_permissions),
        .probe_ack_from_l1_dirty_data(m1_probe_ack_dirty_data),

        .a_valid                    (mst_a_valid[1]),
        .a_opcode                   (mst_a_opcode[1*3 +: 3]),
        .a_param                    (mst_a_param[1*3 +: 3]),
        .a_size                     (mst_a_size[1*4 +: 4]),
        .a_source                   (mst_a_source[1*SOURCE_W +: SOURCE_W]),
        .a_address                  (mst_a_address[1*ADDR_W +: ADDR_W]),
        .a_data                     (mst_a_data[1*DATA_W +: DATA_W]),
        .a_mask                     (mst_a_mask[1*8 +: 8]),
        .a_ready                    (mst_a_ready[1]),

        .b_valid                    (mst_b_valid[1]),
        .b_opcode                   (mst_b_opcode[1*3 +: 3]),
        .b_param                    (mst_b_param[1*3 +: 3]),
        .b_size                     (mst_b_size[1*4 +: 4]),
        .b_source                   (mst_b_source[1*SOURCE_W +: SOURCE_W]),
        .b_address                  (mst_b_address[1*ADDR_W +: ADDR_W]),
        .b_data                     (mst_b_data[1*DATA_W +: DATA_W]),
        .b_mask                     (mst_b_mask[1*8 +: 8]),
        .b_ready                    (mst_b_ready[1]),

        .c_valid                    (mst_c_valid[1]),
        .c_opcode                   (mst_c_opcode[1*3 +: 3]),
        .c_param                    (mst_c_param[1*3 +: 3]),
        .c_size                     (mst_c_size[1*4 +: 4]),
        .c_source                   (mst_c_source[1*SOURCE_W +: SOURCE_W]),
        .c_address                  (mst_c_address[1*ADDR_W +: ADDR_W]),
        .c_data                     (mst_c_data[1*DATA_W +: DATA_W]),
        .c_error                    (),           // output not used by crossbar
        .c_ready                    (mst_c_ready[1]),

        .d_valid                    (mst_d_valid[1]),
        .d_opcode                   (mst_d_opcode[1*3 +: 3]),
        .d_param                    (/* unused */),
        .d_size                     (/* unused */),
        .d_source                   (mst_d_source[1*SOURCE_W +: SOURCE_W]),
        .d_sink                     (m1_d_sink_4),
        .d_data                     (mst_d_data[1*DATA_W +: DATA_W]),
        .d_error                    (mst_d_denied[1]),
        .d_ready                    (mst_d_ready[1]),

        .e_valid                    (mst_e_valid[1]),
        .e_sink                     (m1_e_sink_4),
        .e_ready                    (mst_e_ready[1])
    );

    // ═══════════════════════════════════════════════════════════════════════
    // l1_tilelink_adapter — Master 2
    // ═══════════════════════════════════════════════════════════════════════
    l1_tilelink_adapter u_l1_m2 (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .l1_id                      (2'd2),

        .l1_request_valid           (m2_req_valid),
        .l1_request_addr            (m2_req_addr),
        .l1_request_type            (m2_req_type),
        .l1_request_data            (m2_req_data),
        .l1_request_permissions     (m2_req_permissions),
        .l1_request_ready           (m2_req_ready),
        .data_to_l1_valid           (m2_data_valid),
        .data_to_l1_data            (m2_data),
        .data_to_l1_error           (m2_data_error),

        .probe_req_to_l1_valid      (m2_probe_req_valid),
        .probe_req_to_l1_addr       (m2_probe_req_addr),
        .probe_req_to_l1_permissions(m2_probe_req_permissions),

        .probe_ack_from_l1_valid    (m2_probe_ack_valid),
        .probe_ack_from_l1_addr     (m2_probe_ack_addr),
        .probe_ack_from_l1_permissions(m2_probe_ack_permissions),
        .probe_ack_from_l1_dirty_data(m2_probe_ack_dirty_data),

        .a_valid                    (mst_a_valid[2]),
        .a_opcode                   (mst_a_opcode[2*3 +: 3]),
        .a_param                    (mst_a_param[2*3 +: 3]),
        .a_size                     (mst_a_size[2*4 +: 4]),
        .a_source                   (mst_a_source[2*SOURCE_W +: SOURCE_W]),
        .a_address                  (mst_a_address[2*ADDR_W +: ADDR_W]),
        .a_data                     (mst_a_data[2*DATA_W +: DATA_W]),
        .a_mask                     (mst_a_mask[2*8 +: 8]),
        .a_ready                    (mst_a_ready[2]),

        .b_valid                    (mst_b_valid[2]),
        .b_opcode                   (mst_b_opcode[2*3 +: 3]),
        .b_param                    (mst_b_param[2*3 +: 3]),
        .b_size                     (mst_b_size[2*4 +: 4]),
        .b_source                   (mst_b_source[2*SOURCE_W +: SOURCE_W]),
        .b_address                  (mst_b_address[2*ADDR_W +: ADDR_W]),
        .b_data                     (mst_b_data[2*DATA_W +: DATA_W]),
        .b_mask                     (mst_b_mask[2*8 +: 8]),
        .b_ready                    (mst_b_ready[2]),

        .c_valid                    (mst_c_valid[2]),
        .c_opcode                   (mst_c_opcode[2*3 +: 3]),
        .c_param                    (mst_c_param[2*3 +: 3]),
        .c_size                     (mst_c_size[2*4 +: 4]),
        .c_source                   (mst_c_source[2*SOURCE_W +: SOURCE_W]),
        .c_address                  (mst_c_address[2*ADDR_W +: ADDR_W]),
        .c_data                     (mst_c_data[2*DATA_W +: DATA_W]),
        .c_error                    (),           // output not used by crossbar
        .c_ready                    (mst_c_ready[2]),

        .d_valid                    (mst_d_valid[2]),
        .d_opcode                   (mst_d_opcode[2*3 +: 3]),
        .d_param                    (/* unused */),
        .d_size                     (/* unused */),
        .d_source                   (mst_d_source[2*SOURCE_W +: SOURCE_W]),
        .d_sink                     (m2_d_sink_4),
        .d_data                     (mst_d_data[2*DATA_W +: DATA_W]),
        .d_error                    (mst_d_denied[2]),
        .d_ready                    (mst_d_ready[2]),

        .e_valid                    (mst_e_valid[2]),
        .e_sink                     (m2_e_sink_4),
        .e_ready                    (mst_e_ready[2])
    );

    // ═══════════════════════════════════════════════════════════════════════
    // tl_xbar_nm — 3-master × 2-slave TileLink-C crossbar
    // ═══════════════════════════════════════════════════════════════════════
    //
    // Address map encoded as flattened {slave1_param, slave0_param}:
    //   slave 0: base=0x0000_0000  mask=0x8000_0000  → addr[31]==0
    //   slave 1: base=0x8000_0000  mask=0x8000_0000  → addr[31]==1
    //
    // The corrupt signals are not used by l1_tilelink_adapter; we tie them 0.

    // Slave-side wires
    wire [1:0]                     slv_a_valid,  slv_a_ready;
    wire [2*3-1:0]                 slv_a_opcode, slv_a_param;
    wire [2*4-1:0]                 slv_a_size;
    wire [2*M_SOURCE_W-1:0]        slv_a_source;
    wire [2*ADDR_W-1:0]            slv_a_address;
    wire [2*8-1:0]                 slv_a_mask;
    wire [2*DATA_W-1:0]            slv_a_data;

    wire [1:0]                     slv_b_valid,  slv_b_ready;
    wire [2*3-1:0]                 slv_b_opcode, slv_b_param;
    wire [2*4-1:0]                 slv_b_size;
    wire [2*SOURCE_W-1:0]          slv_b_source;
    wire [2*ADDR_W-1:0]            slv_b_address;
    wire [2*8-1:0]                 slv_b_mask;
    wire [2*DATA_W-1:0]            slv_b_data;
    wire [2*MST_ID_W-1:0]          slv_b_dest;

    wire [1:0]                     slv_c_valid,  slv_c_ready;
    wire [2*3-1:0]                 slv_c_opcode, slv_c_param;
    wire [2*4-1:0]                 slv_c_size;
    wire [2*M_SOURCE_W-1:0]        slv_c_source;
    wire [2*ADDR_W-1:0]            slv_c_address;
    wire [2*DATA_W-1:0]            slv_c_data;

    wire [1:0]                     slv_d_valid,  slv_d_ready;
    wire [2*3-1:0]                 slv_d_opcode;
    wire [2*2-1:0]                 slv_d_param;
    wire [2*4-1:0]                 slv_d_size;
    wire [2*M_SOURCE_W-1:0]        slv_d_source;
    wire [2*SINK_W-1:0]            slv_d_sink;
    wire [1:0]                     slv_d_denied;
    wire [2*DATA_W-1:0]            slv_d_data;

    wire [1:0]                     slv_e_valid,  slv_e_ready;
    wire [2*SINK_W-1:0]            slv_e_sink;

    tl_xbar_nm #(
        .N_MASTERS       (3),
        .N_SLAVES        (2),
        .DATA_W          (DATA_W),
        .ADDR_W          (ADDR_W),
        .SOURCE_W        (SOURCE_W),
        .SINK_W          (SINK_W),
        // Address map: {slave1_mask, slave0_mask} and {slave1_base, slave0_base}
        .SLAVE_ADDR_BASE ({32'h8000_0000, 32'h0000_0000}),
        .SLAVE_ADDR_MASK ({32'h8000_0000, 32'h8000_0000})
    ) u_xbar (
        .clk             (clk),
        .rst_n           (rst_n),

        // Master A
        .mst_a_valid_i   (mst_a_valid),
        .mst_a_ready_o   (mst_a_ready),
        .mst_a_opcode_i  (mst_a_opcode),
        .mst_a_param_i   (mst_a_param),
        .mst_a_size_i    (mst_a_size),
        .mst_a_source_i  (mst_a_source),
        .mst_a_address_i (mst_a_address),
        .mst_a_mask_i    (mst_a_mask),
        .mst_a_data_i    (mst_a_data),
        .mst_a_corrupt_i (3'b000),

        // Master B
        .mst_b_valid_o   (mst_b_valid),
        .mst_b_ready_i   (mst_b_ready),
        .mst_b_opcode_o  (mst_b_opcode),
        .mst_b_param_o   (mst_b_param),
        .mst_b_size_o    (mst_b_size),
        .mst_b_source_o  (mst_b_source),
        .mst_b_address_o (mst_b_address),
        .mst_b_mask_o    (mst_b_mask),
        .mst_b_data_o    (mst_b_data),
        .mst_b_corrupt_o (/* unused */),

        // Master C
        .mst_c_valid_i   (mst_c_valid),
        .mst_c_ready_o   (mst_c_ready),
        .mst_c_opcode_i  (mst_c_opcode),
        .mst_c_param_i   (mst_c_param),
        .mst_c_size_i    (mst_c_size),
        .mst_c_source_i  (mst_c_source),
        .mst_c_address_i (mst_c_address),
        .mst_c_data_i    (mst_c_data),
        .mst_c_corrupt_i (3'b000),

        // Master D
        .mst_d_valid_o   (mst_d_valid),
        .mst_d_ready_i   (mst_d_ready),
        .mst_d_opcode_o  (mst_d_opcode),
        .mst_d_param_o   (/* unused */),
        .mst_d_size_o    (/* unused */),
        .mst_d_source_o  (mst_d_source),
        .mst_d_sink_o    (mst_d_sink),
        .mst_d_denied_o  (mst_d_denied),
        .mst_d_data_o    (mst_d_data),
        .mst_d_corrupt_o (/* unused */),

        // Master E
        .mst_e_valid_i   (mst_e_valid),
        .mst_e_ready_o   (mst_e_ready),
        .mst_e_sink_i    (mst_e_sink),

        // Slave A
        .slv_a_valid_o   (slv_a_valid),
        .slv_a_ready_i   (slv_a_ready),
        .slv_a_opcode_o  (slv_a_opcode),
        .slv_a_param_o   (slv_a_param),
        .slv_a_size_o    (slv_a_size),
        .slv_a_source_o  (slv_a_source),
        .slv_a_address_o (slv_a_address),
        .slv_a_mask_o    (slv_a_mask),
        .slv_a_data_o    (slv_a_data),
        .slv_a_corrupt_o (/* unused */),

        // Slave B
        .slv_b_valid_i   (slv_b_valid),
        .slv_b_ready_o   (slv_b_ready),
        .slv_b_opcode_i  (slv_b_opcode),
        .slv_b_param_i   (slv_b_param),
        .slv_b_size_i    (slv_b_size),
        .slv_b_source_i  (slv_b_source),
        .slv_b_address_i (slv_b_address),
        .slv_b_mask_i    (slv_b_mask),
        .slv_b_data_i    (slv_b_data),
        .slv_b_corrupt_i (2'b00),
        .slv_b_dest_i    (slv_b_dest),

        // Slave C
        .slv_c_valid_o   (slv_c_valid),
        .slv_c_ready_i   (slv_c_ready),
        .slv_c_opcode_o  (slv_c_opcode),
        .slv_c_param_o   (slv_c_param),
        .slv_c_size_o    (slv_c_size),
        .slv_c_source_o  (slv_c_source),
        .slv_c_address_o (slv_c_address),
        .slv_c_data_o    (slv_c_data),
        .slv_c_corrupt_o (/* unused */),

        // Slave D
        .slv_d_valid_i   (slv_d_valid),
        .slv_d_ready_o   (slv_d_ready),
        .slv_d_opcode_i  (slv_d_opcode),
        .slv_d_param_i   (slv_d_param),
        .slv_d_size_i    (slv_d_size),
        .slv_d_source_i  (slv_d_source),
        .slv_d_sink_i    (slv_d_sink),
        .slv_d_denied_i  (slv_d_denied),
        .slv_d_data_i    (slv_d_data),
        .slv_d_corrupt_i (2'b00),

        // Slave E
        .slv_e_valid_o   (slv_e_valid),
        .slv_e_ready_i   (slv_e_ready),
        .slv_e_sink_o    (slv_e_sink)
    );

    // ═══════════════════════════════════════════════════════════════════════
    // tlc_slave_model — Slave 0  (lower 2 GB)
    // ═══════════════════════════════════════════════════════════════════════
    tlc_slave_model #(
        .SLAVE_ID   (0),
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .SOURCE_W   (SOURCE_W),
        .M_SOURCE_W (M_SOURCE_W),
        .SINK_W     (SINK_W),
        .N_MASTERS  (N_MASTERS),
        .MST_ID_W   (MST_ID_W),
        .MEM_LINES  (256),
        .DIR_DEPTH  (32)
    ) u_slv0 (
        .clk       (clk),
        .rst_n     (rst_n),

        .a_valid   (slv_a_valid[0]),
        .a_ready   (slv_a_ready[0]),
        .a_opcode  (slv_a_opcode[0*3 +: 3]),
        .a_param   (slv_a_param[0*3 +: 3]),
        .a_size    (slv_a_size[0*4 +: 4]),
        .a_source  (slv_a_source[0*M_SOURCE_W +: M_SOURCE_W]),
        .a_address (slv_a_address[0*ADDR_W +: ADDR_W]),
        .a_mask    (slv_a_mask[0*8 +: 8]),
        .a_data    (slv_a_data[0*DATA_W +: DATA_W]),
        .a_corrupt (1'b0),

        .b_valid   (slv_b_valid[0]),
        .b_ready   (slv_b_ready[0]),
        .b_opcode  (slv_b_opcode[0*3 +: 3]),
        .b_param   (slv_b_param[0*3 +: 3]),
        .b_size    (slv_b_size[0*4 +: 4]),
        .b_source  (slv_b_source[0*SOURCE_W +: SOURCE_W]),
        .b_address (slv_b_address[0*ADDR_W +: ADDR_W]),
        .b_mask    (slv_b_mask[0*8 +: 8]),
        .b_data    (slv_b_data[0*DATA_W +: DATA_W]),
        .b_corrupt (/* unused output */),
        .b_dest    (slv_b_dest[0*MST_ID_W +: MST_ID_W]),

        .c_valid   (slv_c_valid[0]),
        .c_ready   (slv_c_ready[0]),
        .c_opcode  (slv_c_opcode[0*3 +: 3]),
        .c_param   (slv_c_param[0*3 +: 3]),
        .c_size    (slv_c_size[0*4 +: 4]),
        .c_source  (slv_c_source[0*M_SOURCE_W +: M_SOURCE_W]),
        .c_address (slv_c_address[0*ADDR_W +: ADDR_W]),
        .c_data    (slv_c_data[0*DATA_W +: DATA_W]),
        .c_corrupt (1'b0),

        .d_valid   (slv_d_valid[0]),
        .d_ready   (slv_d_ready[0]),
        .d_opcode  (slv_d_opcode[0*3 +: 3]),
        .d_param   (slv_d_param[0*2 +: 2]),
        .d_size    (slv_d_size[0*4 +: 4]),
        .d_source  (slv_d_source[0*M_SOURCE_W +: M_SOURCE_W]),
        .d_sink    (slv_d_sink[0*SINK_W +: SINK_W]),
        .d_denied  (slv_d_denied[0]),
        .d_data    (slv_d_data[0*DATA_W +: DATA_W]),
        .d_corrupt (/* unused output */),

        .e_valid   (slv_e_valid[0]),
        .e_ready   (slv_e_ready[0]),
        .e_sink    (slv_e_sink[0*SINK_W +: SINK_W])
    );

    // ═══════════════════════════════════════════════════════════════════════
    // tlc_slave_model — Slave 1  (upper 2 GB)
    // ═══════════════════════════════════════════════════════════════════════
    tlc_slave_model #(
        .SLAVE_ID   (1),
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .SOURCE_W   (SOURCE_W),
        .M_SOURCE_W (M_SOURCE_W),
        .SINK_W     (SINK_W),
        .N_MASTERS  (N_MASTERS),
        .MST_ID_W   (MST_ID_W),
        .MEM_LINES  (256),
        .DIR_DEPTH  (32)
    ) u_slv1 (
        .clk       (clk),
        .rst_n     (rst_n),

        .a_valid   (slv_a_valid[1]),
        .a_ready   (slv_a_ready[1]),
        .a_opcode  (slv_a_opcode[1*3 +: 3]),
        .a_param   (slv_a_param[1*3 +: 3]),
        .a_size    (slv_a_size[1*4 +: 4]),
        .a_source  (slv_a_source[1*M_SOURCE_W +: M_SOURCE_W]),
        .a_address (slv_a_address[1*ADDR_W +: ADDR_W]),
        .a_mask    (slv_a_mask[1*8 +: 8]),
        .a_data    (slv_a_data[1*DATA_W +: DATA_W]),
        .a_corrupt (1'b0),

        .b_valid   (slv_b_valid[1]),
        .b_ready   (slv_b_ready[1]),
        .b_opcode  (slv_b_opcode[1*3 +: 3]),
        .b_param   (slv_b_param[1*3 +: 3]),
        .b_size    (slv_b_size[1*4 +: 4]),
        .b_source  (slv_b_source[1*SOURCE_W +: SOURCE_W]),
        .b_address (slv_b_address[1*ADDR_W +: ADDR_W]),
        .b_mask    (slv_b_mask[1*8 +: 8]),
        .b_data    (slv_b_data[1*DATA_W +: DATA_W]),
        .b_corrupt (/* unused output */),
        .b_dest    (slv_b_dest[1*MST_ID_W +: MST_ID_W]),

        .c_valid   (slv_c_valid[1]),
        .c_ready   (slv_c_ready[1]),
        .c_opcode  (slv_c_opcode[1*3 +: 3]),
        .c_param   (slv_c_param[1*3 +: 3]),
        .c_size    (slv_c_size[1*4 +: 4]),
        .c_source  (slv_c_source[1*M_SOURCE_W +: M_SOURCE_W]),
        .c_address (slv_c_address[1*ADDR_W +: ADDR_W]),
        .c_data    (slv_c_data[1*DATA_W +: DATA_W]),
        .c_corrupt (1'b0),

        .d_valid   (slv_d_valid[1]),
        .d_ready   (slv_d_ready[1]),
        .d_opcode  (slv_d_opcode[1*3 +: 3]),
        .d_param   (slv_d_param[1*2 +: 2]),
        .d_size    (slv_d_size[1*4 +: 4]),
        .d_source  (slv_d_source[1*M_SOURCE_W +: M_SOURCE_W]),
        .d_sink    (slv_d_sink[1*SINK_W +: SINK_W]),
        .d_denied  (slv_d_denied[1]),
        .d_data    (slv_d_data[1*DATA_W +: DATA_W]),
        .d_corrupt (/* unused output */),

        .e_valid   (slv_e_valid[1]),
        .e_ready   (slv_e_ready[1]),
        .e_sink    (slv_e_sink[1*SINK_W +: SINK_W])
    );

endmodule
