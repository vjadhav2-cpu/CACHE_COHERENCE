// =============================================================================
// tlc_xbar_fabric_top.v  —  3-master × 2-slave TileLink-C fabric with
//                           externally connected slave managers
// =============================================================================
//
// This is a variant of tlc_xbar_top.v that keeps the existing master-side
// l1_tilelink_adapter integration, probe side-band bridge, and tl_xbar_nm
// wiring, but removes the embedded tlc_slave_model instances. Instead, it
// exposes the slave-side TL-C channel bundles so external slave managers can
// be connected and tested explicitly.
// =============================================================================
`timescale 1ns/10ps

module tlc_xbar_fabric_top #(
    parameter ADDR_W     = 32,
    parameter DATA_W     = 64,
    parameter SOURCE_W   = 4,
    parameter SINK_W     = 4,
    parameter N_MASTERS  = 3,
    parameter N_SLAVES   = 2,
    parameter MST_ID_W   = 2,
    parameter SLV_ID_W   = 1,
    parameter M_SOURCE_W = SOURCE_W + MST_ID_W,
    parameter M_SINK_W   = SINK_W + SLV_ID_W
) (
    input  wire clk,
    input  wire rst_n,

    // ── L1 Cache Controller — Master 0 ────────────────────────────────────
    input  wire          m0_req_valid,
    input  wire [31:0]   m0_req_addr,
    input  wire [2:0]    m0_req_type,
    input  wire [255:0]  m0_req_data,
    input  wire [2:0]    m0_req_permissions,
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
    input  wire [255:0]  m2_probe_ack_dirty_data,

    // ── External Slave-Side TL-C Bundles ──────────────────────────────────
    // A: crossbar -> slave managers
    output wire [N_SLAVES-1:0]              slv_a_valid,
    input  wire [N_SLAVES-1:0]              slv_a_ready,
    output wire [N_SLAVES*3-1:0]            slv_a_opcode,
    output wire [N_SLAVES*3-1:0]            slv_a_param,
    output wire [N_SLAVES*4-1:0]            slv_a_size,
    output wire [N_SLAVES*M_SOURCE_W-1:0]   slv_a_source,
    output wire [N_SLAVES*ADDR_W-1:0]       slv_a_address,
    output wire [N_SLAVES*8-1:0]            slv_a_mask,
    output wire [N_SLAVES*DATA_W-1:0]       slv_a_data,

    // B: slave managers -> crossbar
    input  wire [N_SLAVES-1:0]              slv_b_valid,
    output wire [N_SLAVES-1:0]              slv_b_ready,
    input  wire [N_SLAVES*3-1:0]            slv_b_opcode,
    input  wire [N_SLAVES*3-1:0]            slv_b_param,
    input  wire [N_SLAVES*4-1:0]            slv_b_size,
    input  wire [N_SLAVES*SOURCE_W-1:0]     slv_b_source,
    input  wire [N_SLAVES*ADDR_W-1:0]       slv_b_address,
    input  wire [N_SLAVES*8-1:0]            slv_b_mask,
    input  wire [N_SLAVES*DATA_W-1:0]       slv_b_data,
    input  wire [N_SLAVES*MST_ID_W-1:0]     slv_b_dest,

    // C: crossbar -> slave managers
    output wire [N_SLAVES-1:0]              slv_c_valid,
    input  wire [N_SLAVES-1:0]              slv_c_ready,
    output wire [N_SLAVES*3-1:0]            slv_c_opcode,
    output wire [N_SLAVES*3-1:0]            slv_c_param,
    output wire [N_SLAVES*4-1:0]            slv_c_size,
    output wire [N_SLAVES*M_SOURCE_W-1:0]   slv_c_source,
    output wire [N_SLAVES*ADDR_W-1:0]       slv_c_address,
    output wire [N_SLAVES*DATA_W-1:0]       slv_c_data,

    // D: slave managers -> crossbar
    input  wire [N_SLAVES-1:0]              slv_d_valid,
    output wire [N_SLAVES-1:0]              slv_d_ready,
    input  wire [N_SLAVES*3-1:0]            slv_d_opcode,
    input  wire [N_SLAVES*2-1:0]            slv_d_param,
    input  wire [N_SLAVES*4-1:0]            slv_d_size,
    input  wire [N_SLAVES*M_SOURCE_W-1:0]   slv_d_source,
    input  wire [N_SLAVES*SINK_W-1:0]       slv_d_sink,
    input  wire [N_SLAVES-1:0]              slv_d_denied,
    input  wire [N_SLAVES*DATA_W-1:0]       slv_d_data,

    // E: crossbar -> slave managers
    output wire [N_SLAVES-1:0]              slv_e_valid,
    input  wire [N_SLAVES-1:0]              slv_e_ready,
    output wire [N_SLAVES*SINK_W-1:0]       slv_e_sink
);

    localparam D_GRANT      = 3'd4;
    localparam D_GRANT_DATA = 3'd5;

    // Master-side TileLink channel wires
    wire [N_MASTERS-1:0]          mst_a_valid;
    wire [N_MASTERS-1:0]          mst_a_ready;
    wire [N_MASTERS*3-1:0]        mst_a_opcode;
    wire [N_MASTERS*3-1:0]        mst_a_param;
    wire [N_MASTERS*4-1:0]        mst_a_size;
    wire [N_MASTERS*SOURCE_W-1:0] mst_a_source;
    wire [N_MASTERS*ADDR_W-1:0]   mst_a_address;
    wire [N_MASTERS*8-1:0]        mst_a_mask;
    wire [N_MASTERS*DATA_W-1:0]   mst_a_data;

    wire [N_MASTERS-1:0]          mst_b_valid;
    wire [N_MASTERS-1:0]          mst_b_ready;
    wire [N_MASTERS*3-1:0]        mst_b_opcode;
    wire [N_MASTERS*3-1:0]        mst_b_param;
    wire [N_MASTERS*4-1:0]        mst_b_size;
    wire [N_MASTERS*SOURCE_W-1:0] mst_b_source;
    wire [N_MASTERS*ADDR_W-1:0]   mst_b_address;
    wire [N_MASTERS*8-1:0]        mst_b_mask;
    wire [N_MASTERS*DATA_W-1:0]   mst_b_data;

    wire [N_MASTERS-1:0]          mst_c_valid;
    wire [N_MASTERS-1:0]          mst_c_ready;
    wire [N_MASTERS*3-1:0]        mst_c_opcode;
    wire [N_MASTERS*3-1:0]        mst_c_param;
    wire [N_MASTERS*4-1:0]        mst_c_size;
    wire [N_MASTERS*SOURCE_W-1:0] mst_c_source;
    wire [N_MASTERS*ADDR_W-1:0]   mst_c_address;
    wire [N_MASTERS*DATA_W-1:0]   mst_c_data;

    wire [N_MASTERS-1:0]          mst_d_valid;
    wire [N_MASTERS-1:0]          mst_d_ready;
    wire [N_MASTERS*3-1:0]        mst_d_opcode;
    wire [N_MASTERS*SOURCE_W-1:0] mst_d_source;
    wire [N_MASTERS*M_SINK_W-1:0] mst_d_sink;
    wire [N_MASTERS*DATA_W-1:0]   mst_d_data;
    wire [N_MASTERS-1:0]          mst_d_denied;

    wire [N_MASTERS-1:0]          mst_e_valid;
    wire [N_MASTERS-1:0]          mst_e_ready;
    wire [N_MASTERS*M_SINK_W-1:0] mst_e_sink;

    // Sink-width bridge: capture slave_id from D, prepend to E
    wire [SOURCE_W-1:0]  m0_d_sink_4, m1_d_sink_4, m2_d_sink_4;
    wire [SOURCE_W-1:0]  m0_e_sink_4, m1_e_sink_4, m2_e_sink_4;
    reg  m0_d_slv_id, m1_d_slv_id, m2_d_slv_id;

    assign m0_d_sink_4 = mst_d_sink[0*M_SINK_W +: SINK_W];
    assign m1_d_sink_4 = mst_d_sink[1*M_SINK_W +: SINK_W];
    assign m2_d_sink_4 = mst_d_sink[2*M_SINK_W +: SINK_W];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            m0_d_slv_id <= 1'b0;
        else if (mst_d_valid[0] && mst_d_ready[0] &&
                 (mst_d_opcode[0*3 +: 3] == D_GRANT ||
                  mst_d_opcode[0*3 +: 3] == D_GRANT_DATA))
            m0_d_slv_id <= mst_d_sink[0*M_SINK_W + SINK_W];
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

    assign mst_e_sink[0*M_SINK_W +: M_SINK_W] = {m0_d_slv_id, m0_e_sink_4};
    assign mst_e_sink[1*M_SINK_W +: M_SINK_W] = {m1_d_slv_id, m1_e_sink_4};
    assign mst_e_sink[2*M_SINK_W +: M_SINK_W] = {m2_d_slv_id, m2_e_sink_4};

    // Probe side-band bridge
    reg        m0_probe_req_valid_r, m1_probe_req_valid_r, m2_probe_req_valid_r;
    reg [31:0] m0_probe_req_addr_r,  m1_probe_req_addr_r,  m2_probe_req_addr_r;
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

    // Master adapters
    l1_tilelink_adapter u_l1_m0 (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .l1_id                      (2'd0),
        .l1_request_valid           (m0_req_valid),
        .l1_request_addr            (m0_req_addr),
        .l1_request_type            (m0_req_type),
        .l1_request_data            (m0_req_data),
        .l1_request_permissions     (m0_req_permissions),
        .l1_request_ready           (m0_req_ready),
        .data_to_l1_valid           (m0_data_valid),
        .data_to_l1_data            (m0_data),
        .data_to_l1_error           (m0_data_error),
        .probe_req_to_l1_valid      (m0_probe_req_valid),
        .probe_req_to_l1_addr       (m0_probe_req_addr),
        .probe_req_to_l1_permissions(m0_probe_req_permissions),
        .probe_ack_from_l1_valid    (m0_probe_ack_valid),
        .probe_ack_from_l1_addr     (m0_probe_ack_addr),
        .probe_ack_from_l1_permissions(m0_probe_ack_permissions),
        .probe_ack_from_l1_dirty_data(m0_probe_ack_dirty_data),
        .a_valid                    (mst_a_valid[0]),
        .a_opcode                   (mst_a_opcode[0*3 +: 3]),
        .a_param                    (mst_a_param[0*3 +: 3]),
        .a_size                     (mst_a_size[0*4 +: 4]),
        .a_source                   (mst_a_source[0*SOURCE_W +: SOURCE_W]),
        .a_address                  (mst_a_address[0*ADDR_W +: ADDR_W]),
        .a_data                     (mst_a_data[0*DATA_W +: DATA_W]),
        .a_mask                     (mst_a_mask[0*8 +: 8]),
        .a_ready                    (mst_a_ready[0]),
        .b_valid                    (mst_b_valid[0]),
        .b_opcode                   (mst_b_opcode[0*3 +: 3]),
        .b_param                    (mst_b_param[0*3 +: 3]),
        .b_size                     (mst_b_size[0*4 +: 4]),
        .b_source                   (mst_b_source[0*SOURCE_W +: SOURCE_W]),
        .b_address                  (mst_b_address[0*ADDR_W +: ADDR_W]),
        .b_data                     (mst_b_data[0*DATA_W +: DATA_W]),
        .b_mask                     (mst_b_mask[0*8 +: 8]),
        .b_ready                    (mst_b_ready[0]),
        .c_valid                    (mst_c_valid[0]),
        .c_opcode                   (mst_c_opcode[0*3 +: 3]),
        .c_param                    (mst_c_param[0*3 +: 3]),
        .c_size                     (mst_c_size[0*4 +: 4]),
        .c_source                   (mst_c_source[0*SOURCE_W +: SOURCE_W]),
        .c_address                  (mst_c_address[0*ADDR_W +: ADDR_W]),
        .c_data                     (mst_c_data[0*DATA_W +: DATA_W]),
        .c_error                    (),
        .c_ready                    (mst_c_ready[0]),
        .d_valid                    (mst_d_valid[0]),
        .d_opcode                   (mst_d_opcode[0*3 +: 3]),
        .d_param                    (/* unused */),
        .d_size                     (/* unused */),
        .d_source                   (mst_d_source[0*SOURCE_W +: SOURCE_W]),
        .d_sink                     (m0_d_sink_4),
        .d_data                     (mst_d_data[0*DATA_W +: DATA_W]),
        .d_error                    (mst_d_denied[0]),
        .d_ready                    (mst_d_ready[0]),
        .e_valid                    (mst_e_valid[0]),
        .e_sink                     (m0_e_sink_4),
        .e_ready                    (mst_e_ready[0])
    );

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
        .c_error                    (),
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
        .c_error                    (),
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

    // Crossbar
    tl_xbar_nm #(
        .N_MASTERS       (N_MASTERS),
        .N_SLAVES        (N_SLAVES),
        .DATA_W          (DATA_W),
        .ADDR_W          (ADDR_W),
        .SOURCE_W        (SOURCE_W),
        .SINK_W          (SINK_W),
        .SLAVE_ADDR_BASE ({32'h8000_0000, 32'h0000_0000}),
        .SLAVE_ADDR_MASK ({32'h8000_0000, 32'h8000_0000})
    ) u_xbar (
        .clk             (clk),
        .rst_n           (rst_n),
        .mst_a_valid_i   (mst_a_valid),
        .mst_a_ready_o   (mst_a_ready),
        .mst_a_opcode_i  (mst_a_opcode),
        .mst_a_param_i   (mst_a_param),
        .mst_a_size_i    (mst_a_size),
        .mst_a_source_i  (mst_a_source),
        .mst_a_address_i (mst_a_address),
        .mst_a_mask_i    (mst_a_mask),
        .mst_a_data_i    (mst_a_data),
        .mst_a_corrupt_i ({N_MASTERS{1'b0}}),
        .mst_b_valid_o   (mst_b_valid),
        .mst_b_ready_i   (mst_b_ready),
        .mst_b_opcode_o  (mst_b_opcode),
        .mst_b_param_o   (mst_b_param),
        .mst_b_size_o    (mst_b_size),
        .mst_b_source_o  (mst_b_source),
        .mst_b_address_o (mst_b_address),
        .mst_b_mask_o    (mst_b_mask),
        .mst_b_data_o    (mst_b_data),
        .mst_b_corrupt_o (),
        .mst_c_valid_i   (mst_c_valid),
        .mst_c_ready_o   (mst_c_ready),
        .mst_c_opcode_i  (mst_c_opcode),
        .mst_c_param_i   (mst_c_param),
        .mst_c_size_i    (mst_c_size),
        .mst_c_source_i  (mst_c_source),
        .mst_c_address_i (mst_c_address),
        .mst_c_data_i    (mst_c_data),
        .mst_c_corrupt_i ({N_MASTERS{1'b0}}),
        .mst_d_valid_o   (mst_d_valid),
        .mst_d_ready_i   (mst_d_ready),
        .mst_d_opcode_o  (mst_d_opcode),
        .mst_d_param_o   (),
        .mst_d_size_o    (),
        .mst_d_source_o  (mst_d_source),
        .mst_d_sink_o    (mst_d_sink),
        .mst_d_denied_o  (mst_d_denied),
        .mst_d_data_o    (mst_d_data),
        .mst_d_corrupt_o (),
        .mst_e_valid_i   (mst_e_valid),
        .mst_e_ready_o   (mst_e_ready),
        .mst_e_sink_i    (mst_e_sink),
        .slv_a_valid_o   (slv_a_valid),
        .slv_a_ready_i   (slv_a_ready),
        .slv_a_opcode_o  (slv_a_opcode),
        .slv_a_param_o   (slv_a_param),
        .slv_a_size_o    (slv_a_size),
        .slv_a_source_o  (slv_a_source),
        .slv_a_address_o (slv_a_address),
        .slv_a_mask_o    (slv_a_mask),
        .slv_a_data_o    (slv_a_data),
        .slv_a_corrupt_o (),
        .slv_b_valid_i   (slv_b_valid),
        .slv_b_ready_o   (slv_b_ready),
        .slv_b_opcode_i  (slv_b_opcode),
        .slv_b_param_i   (slv_b_param),
        .slv_b_size_i    (slv_b_size),
        .slv_b_source_i  (slv_b_source),
        .slv_b_address_i (slv_b_address),
        .slv_b_mask_i    (slv_b_mask),
        .slv_b_data_i    (slv_b_data),
        .slv_b_corrupt_i ({N_SLAVES{1'b0}}),
        .slv_b_dest_i    (slv_b_dest),
        .slv_c_valid_o   (slv_c_valid),
        .slv_c_ready_i   (slv_c_ready),
        .slv_c_opcode_o  (slv_c_opcode),
        .slv_c_param_o   (slv_c_param),
        .slv_c_size_o    (slv_c_size),
        .slv_c_source_o  (slv_c_source),
        .slv_c_address_o (slv_c_address),
        .slv_c_data_o    (slv_c_data),
        .slv_c_corrupt_o (),
        .slv_d_valid_i   (slv_d_valid),
        .slv_d_ready_o   (slv_d_ready),
        .slv_d_opcode_i  (slv_d_opcode),
        .slv_d_param_i   (slv_d_param),
        .slv_d_size_i    (slv_d_size),
        .slv_d_source_i  (slv_d_source),
        .slv_d_sink_i    (slv_d_sink),
        .slv_d_denied_i  (slv_d_denied),
        .slv_d_data_i    (slv_d_data),
        .slv_d_corrupt_i ({N_SLAVES{1'b0}}),
        .slv_e_valid_o   (slv_e_valid),
        .slv_e_ready_i   (slv_e_ready),
        .slv_e_sink_o    (slv_e_sink)
    );

endmodule
