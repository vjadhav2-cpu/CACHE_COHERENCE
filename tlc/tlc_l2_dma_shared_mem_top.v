`timescale 1ns/10ps
`default_nettype none

// tlc_l2_dma_shared_mem_top.v
//
// Variant of tlc_dma_system_with_wrapper_top.v that puts rv64g_l2_cache.v
// on master slot 0 (previously free and tied to 0 by every existing
// testbench) instead of leaving it unused, so L2 refill/writeback traffic
// and DMA traffic (still on master slot 2, unchanged) share the same
// tlc_xbar_fabric_top crossbar and the same slv0 memory manager/backend.
//
// See docs/l2_tlc_xbar_integration_plan.md for why this needs
// l2_tlc_bridge.v in between (L2 does real 8-beat bursts; the
// l1_tilelink_adapter path only moves one 64-bit beat per transaction)
// and for why this port is safe from ever being probed.
//
// L2's own upstream (L1-facing) TileLink-C interface is exposed directly
// at this top level (l2_tl_*) as the stimulus surface for a testbench --
// it is a private point-to-point link, not routed through the 32-bit
// tlc/ crossbar, so it keeps L2's native 64-bit ADDR_W/SOURCE_W untouched.
module tlc_l2_dma_shared_mem_top #(
    parameter ADDR_W     = 32,
    parameter DATA_W     = 64,
    parameter SOURCE_W   = 4,
    parameter SINK_W     = 4,
    parameter N_MASTERS  = 3,
    parameter N_SLAVES   = 2,
    parameter NUM_BANKS  = 4,
    parameter BANK_DEPTH = 262144,
    parameter RD_LATENCY = 1,
    parameter WR_LATENCY = 1,
    parameter MEM_INIT_FILE = "",

    // rv64g_l2_cache.v parameters (upstream side is private, not on the
    // shared 32-bit crossbar -- see header)
    parameter L2_CORES    = 4,
    parameter L2_WAYS     = 16,
    parameter L2_SETS     = 256,
    parameter L2_ADDR_W   = 64,
    parameter L2_SOURCE_W = 6,
    parameter L2_CID_W    = 2
) (
    input  wire          clk,
    input  wire          rst_n,

    // ── L2 cache upstream (L1-facing) interface -- testbench stimulus ──
    input  wire [2:0]                 l2_tl_a_opcode,
    input  wire [2:0]                 l2_tl_a_param,
    input  wire [L2_SOURCE_W-1:0]     l2_tl_a_source,
    input  wire [L2_ADDR_W-1:0]       l2_tl_a_address,
    input  wire                       l2_tl_a_valid,
    output wire                       l2_tl_a_ready,

    output wire [2:0]                 l2_tl_b_opcode,
    output wire [2:0]                 l2_tl_b_param,
    output wire [L2_ADDR_W-1:0]       l2_tl_b_address,
    output wire                       l2_tl_b_valid,
    input  wire                       l2_tl_b_ready,
    output wire [L2_CID_W-1:0]        l2_tl_b_dest,

    input  wire [2:0]                 l2_tl_c_opcode,
    input  wire [2:0]                 l2_tl_c_param,
    input  wire [L2_SOURCE_W-1:0]     l2_tl_c_source,
    input  wire [L2_ADDR_W-1:0]       l2_tl_c_address,
    input  wire [DATA_W-1:0]          l2_tl_c_data,
    input  wire                       l2_tl_c_valid,
    output wire                       l2_tl_c_ready,

    output wire [2:0]                 l2_tl_d_opcode,
    output wire [1:0]                 l2_tl_d_param,
    output wire [DATA_W-1:0]          l2_tl_d_data,
    output wire [L2_SOURCE_W-1:0]     l2_tl_d_source,
    output wire [1:0]                 l2_tl_d_sink,
    output wire                       l2_tl_d_valid,
    input  wire                       l2_tl_d_ready,

    input  wire                       l2_tl_e_valid,
    input  wire [1:0]                 l2_tl_e_sink,
    output wire                       l2_tl_e_ready,

    // ── Master 1 controller-side interface -- still free ───────────────
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

    // Real DMA top control / descriptor interface
    input  wire          dma_start,
    input  wire          dma_stream_type,
    input  wire [63:0]   dma_mem_base_addr,
    input  wire [63:0]   dma_peri_base_addr,
    input  wire [15:0]   dma_num_bytes,

    // Peripheral-side DMA bus
    output wire          peri_req,
    output wire          peri_wr_en,
    output wire [63:0]   peri_addr,
    output wire [63:0]   peri_wdata,
    input  wire [63:0]   peri_rdata,
    input  wire          peri_ack,

    // DMA status
    output wire          dma_busy,
    output wire          dma_done,

    // Optional counters for each slave memory backend
    output wire [31:0]   slv0_total_rd_count,
    output wire [31:0]   slv0_total_wr_count,
    output wire [63:0]   slv0_total_rd_latency_sum,
    output wire [31:0]   slv1_total_rd_count,
    output wire [31:0]   slv1_total_wr_count,
    output wire [63:0]   slv1_total_rd_latency_sum
);

    localparam MST_ID_W   = 2;
    localparam SLV_ID_W   = 1;
    localparam M_SOURCE_W = SOURCE_W + MST_ID_W;

    // ── Master 0: rv64g_l2_cache.v memory-side port -> l2_tlc_bridge -> m0 ──
    wire          m0_req_valid;
    wire [31:0]   m0_req_addr;
    wire [2:0]    m0_req_type;
    wire [255:0]  m0_req_data;
    wire [2:0]    m0_req_permissions;
    wire          m0_req_ready;
    wire          m0_data_valid;
    wire [255:0]  m0_data;
    wire          m0_data_error;
    wire          m0_probe_req_valid;
    wire [31:0]   m0_probe_req_addr;
    wire [2:0]    m0_probe_req_permissions;
    wire          m0_probe_ack_valid;
    wire [31:0]   m0_probe_ack_addr;
    wire [2:0]    m0_probe_ack_permissions;
    wire [255:0]  m0_probe_ack_dirty_data;

    wire [2:0]         l2_mem_a_opcode;
    wire [L2_ADDR_W-1:0] l2_mem_a_address;
    wire [DATA_W-1:0]  l2_mem_a_data;
    wire               l2_mem_a_valid;
    wire               l2_mem_a_ready;
    wire [2:0]         l2_mem_d_opcode;
    wire [DATA_W-1:0]  l2_mem_d_data;
    wire               l2_mem_d_valid;
    wire               l2_mem_d_ready;

    rv64g_l2_cache #(
        .CORES    (L2_CORES),
        .WAYS     (L2_WAYS),
        .SETS     (L2_SETS),
        .ADDR_W   (L2_ADDR_W),
        .DATA_W   (DATA_W),
        .SOURCE_W (L2_SOURCE_W),
        .CID_W    (L2_CID_W)
    ) u_l2 (
        .clk_i (clk),
        .rst_ni(rst_n),

        .tl_a_opcode_i (l2_tl_a_opcode),
        .tl_a_param_i  (l2_tl_a_param),
        .tl_a_source_i (l2_tl_a_source),
        .tl_a_address_i(l2_tl_a_address),
        .tl_a_valid_i  (l2_tl_a_valid),
        .tl_a_ready_o  (l2_tl_a_ready),

        .tl_b_opcode_o (l2_tl_b_opcode),
        .tl_b_param_o  (l2_tl_b_param),
        .tl_b_address_o(l2_tl_b_address),
        .tl_b_valid_o  (l2_tl_b_valid),
        .tl_b_ready_i  (l2_tl_b_ready),
        .tl_b_dest_o   (l2_tl_b_dest),

        .tl_c_opcode_i (l2_tl_c_opcode),
        .tl_c_param_i  (l2_tl_c_param),
        .tl_c_source_i (l2_tl_c_source),
        .tl_c_address_i(l2_tl_c_address),
        .tl_c_data_i   (l2_tl_c_data),
        .tl_c_valid_i  (l2_tl_c_valid),
        .tl_c_ready_o  (l2_tl_c_ready),

        .tl_d_opcode_o (l2_tl_d_opcode),
        .tl_d_param_o  (l2_tl_d_param),
        .tl_d_data_o   (l2_tl_d_data),
        .tl_d_source_o (l2_tl_d_source),
        .tl_d_sink_o   (l2_tl_d_sink),
        .tl_d_valid_o  (l2_tl_d_valid),
        .tl_d_ready_i  (l2_tl_d_ready),

        .tl_e_valid_i(l2_tl_e_valid),
        .tl_e_sink_i (l2_tl_e_sink),
        .tl_e_ready_o(l2_tl_e_ready),

        .mem_a_opcode_o(l2_mem_a_opcode),
        .mem_a_param_o (),
        .mem_a_size_o  (),
        .mem_a_source_o(),
        .mem_a_address_o(l2_mem_a_address),
        .mem_a_mask_o  (),
        .mem_a_data_o  (l2_mem_a_data),
        .mem_a_valid_o (l2_mem_a_valid),
        .mem_a_ready_i (l2_mem_a_ready),

        .mem_d_opcode_i(l2_mem_d_opcode),
        .mem_d_param_i (2'b0),
        .mem_d_size_i  (3'b0),
        .mem_d_source_i(4'b0),
        .mem_d_sink_i  (2'b0),
        .mem_d_denied_i(1'b0),
        .mem_d_data_i  (l2_mem_d_data),
        .mem_d_corrupt_i(1'b0),
        .mem_d_valid_i (l2_mem_d_valid),
        .mem_d_ready_o (l2_mem_d_ready)
    );

    l2_tlc_bridge #(
        .ADDR_W(L2_ADDR_W),
        .DATA_W(DATA_W)
    ) u_l2_bridge (
        .clk  (clk),
        .rst_n(rst_n),

        .mem_a_opcode_i(l2_mem_a_opcode),
        .mem_a_address_i(l2_mem_a_address),
        .mem_a_data_i  (l2_mem_a_data),
        .mem_a_valid_i (l2_mem_a_valid),
        .mem_a_ready_o (l2_mem_a_ready),

        .mem_d_opcode_o(l2_mem_d_opcode),
        .mem_d_data_o  (l2_mem_d_data),
        .mem_d_valid_o (l2_mem_d_valid),
        .mem_d_ready_i (l2_mem_d_ready),

        .l1_request_valid          (m0_req_valid),
        .l1_request_addr           (m0_req_addr),
        .l1_request_type           (m0_req_type),
        .l1_request_data           (m0_req_data),
        .l1_request_permissions    (m0_req_permissions),
        .l1_request_ready          (m0_req_ready),

        .data_to_l1_valid          (m0_data_valid),
        .data_to_l1_data           (m0_data),
        .data_to_l1_error          (m0_data_error),

        .probe_req_to_l1_valid      (m0_probe_req_valid),
        .probe_req_to_l1_addr       (m0_probe_req_addr),
        .probe_req_to_l1_permissions(m0_probe_req_permissions),

        .probe_ack_from_l1_valid      (m0_probe_ack_valid),
        .probe_ack_from_l1_addr       (m0_probe_ack_addr),
        .probe_ack_from_l1_permissions(m0_probe_ack_permissions),
        .probe_ack_from_l1_dirty_data (m0_probe_ack_dirty_data)
    );

    // ── Master 2: DMA (unchanged from tlc_dma_system_with_wrapper_top.v) ──
    wire          m2_req_valid;
    wire [31:0]   m2_req_addr;
    wire [2:0]    m2_req_type;
    wire [255:0]  m2_req_data;
    wire [2:0]    m2_req_permissions;
    wire          m2_req_ready;
    wire          m2_data_valid;
    wire [255:0]  m2_data;
    wire          m2_data_error;
    wire          m2_probe_req_valid;
    wire [31:0]   m2_probe_req_addr;
    wire [2:0]    m2_probe_req_permissions;
    wire          m2_probe_ack_valid;
    wire [31:0]   m2_probe_ack_addr;
    wire [2:0]    m2_probe_ack_permissions;
    wire [255:0]  m2_probe_ack_dirty_data;

    wire [N_SLAVES-1:0]            slv_a_valid,  slv_a_ready;
    wire [N_SLAVES*3-1:0]          slv_a_opcode, slv_a_param;
    wire [N_SLAVES*4-1:0]          slv_a_size;
    wire [N_SLAVES*M_SOURCE_W-1:0] slv_a_source;
    wire [N_SLAVES*ADDR_W-1:0]     slv_a_address;
    wire [N_SLAVES*8-1:0]          slv_a_mask;
    wire [N_SLAVES*DATA_W-1:0]     slv_a_data;

    wire [N_SLAVES-1:0]            slv_b_valid,  slv_b_ready;
    wire [N_SLAVES*3-1:0]          slv_b_opcode, slv_b_param;
    wire [N_SLAVES*4-1:0]          slv_b_size;
    wire [N_SLAVES*SOURCE_W-1:0]   slv_b_source;
    wire [N_SLAVES*ADDR_W-1:0]     slv_b_address;
    wire [N_SLAVES*8-1:0]          slv_b_mask;
    wire [N_SLAVES*DATA_W-1:0]     slv_b_data;
    wire [N_SLAVES*MST_ID_W-1:0]   slv_b_dest;

    wire [N_SLAVES-1:0]            slv_c_valid,  slv_c_ready;
    wire [N_SLAVES*3-1:0]          slv_c_opcode, slv_c_param;
    wire [N_SLAVES*4-1:0]          slv_c_size;
    wire [N_SLAVES*M_SOURCE_W-1:0] slv_c_source;
    wire [N_SLAVES*ADDR_W-1:0]     slv_c_address;
    wire [N_SLAVES*DATA_W-1:0]     slv_c_data;

    wire [N_SLAVES-1:0]            slv_d_valid,  slv_d_ready;
    wire [N_SLAVES*3-1:0]          slv_d_opcode;
    wire [N_SLAVES*2-1:0]          slv_d_param;
    wire [N_SLAVES*4-1:0]          slv_d_size;
    wire [N_SLAVES*M_SOURCE_W-1:0] slv_d_source;
    wire [N_SLAVES*SINK_W-1:0]     slv_d_sink;
    wire [N_SLAVES-1:0]            slv_d_denied;
    wire [N_SLAVES*DATA_W-1:0]     slv_d_data;

    wire [N_SLAVES-1:0]            slv_e_valid,  slv_e_ready;
    wire [N_SLAVES*SINK_W-1:0]     slv_e_sink;

    wire                            slv0_mem_req, slv0_mem_we, slv0_mem_rvalid, slv0_mem_ready;
    wire [ADDR_W-1:0]               slv0_mem_addr;
    wire [DATA_W-1:0]               slv0_mem_wdata, slv0_mem_rdata;
    wire [DATA_W/8-1:0]             slv0_mem_wmask;
    wire [SOURCE_W-1:0]             slv0_mem_rsource;

    wire                            slv1_mem_req, slv1_mem_we, slv1_mem_rvalid, slv1_mem_ready;
    wire [ADDR_W-1:0]               slv1_mem_addr;
    wire [DATA_W-1:0]               slv1_mem_wdata, slv1_mem_rdata;
    wire [DATA_W/8-1:0]             slv1_mem_wmask;
    wire [SOURCE_W-1:0]             slv1_mem_rsource;

    dma_tlc_master_wrapper #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (64),
        .BURST_MAX  (16),
        .FIFO_DEPTH (16)
    ) u_dma_master (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .dma_start                  (dma_start),
        .dma_stream_type            (dma_stream_type),
        .dma_mem_base_addr          (dma_mem_base_addr),
        .dma_peri_base_addr         (dma_peri_base_addr),
        .dma_num_bytes              (dma_num_bytes),
        .peri_req                   (peri_req),
        .peri_wr_en                 (peri_wr_en),
        .peri_addr                  (peri_addr),
        .peri_wdata                 (peri_wdata),
        .peri_rdata                 (peri_rdata),
        .peri_ack                   (peri_ack),
        .dma_busy                   (dma_busy),
        .dma_done                   (dma_done),
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
        .probe_ack_from_l1_dirty_data(m2_probe_ack_dirty_data)
    );

    tlc_xbar_fabric_top #(
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .SOURCE_W   (SOURCE_W),
        .SINK_W     (SINK_W),
        .N_MASTERS  (N_MASTERS),
        .N_SLAVES   (N_SLAVES),
        .MST_ID_W   (MST_ID_W),
        .SLV_ID_W   (SLV_ID_W)
    ) u_fabric (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .m0_req_valid               (m0_req_valid),
        .m0_req_addr                (m0_req_addr),
        .m0_req_type                (m0_req_type),
        .m0_req_data                (m0_req_data),
        .m0_req_permissions         (m0_req_permissions),
        .m0_req_ready               (m0_req_ready),
        .m0_data_valid              (m0_data_valid),
        .m0_data                    (m0_data),
        .m0_data_error              (m0_data_error),
        .m0_probe_req_valid         (m0_probe_req_valid),
        .m0_probe_req_addr          (m0_probe_req_addr),
        .m0_probe_req_permissions   (m0_probe_req_permissions),
        .m0_probe_ack_valid         (m0_probe_ack_valid),
        .m0_probe_ack_addr          (m0_probe_ack_addr),
        .m0_probe_ack_permissions   (m0_probe_ack_permissions),
        .m0_probe_ack_dirty_data    (m0_probe_ack_dirty_data),
        .m1_req_valid               (m1_req_valid),
        .m1_req_addr                (m1_req_addr),
        .m1_req_type                (m1_req_type),
        .m1_req_data                (m1_req_data),
        .m1_req_permissions         (m1_req_permissions),
        .m1_req_ready               (m1_req_ready),
        .m1_data_valid              (m1_data_valid),
        .m1_data                    (m1_data),
        .m1_data_error              (m1_data_error),
        .m1_probe_req_valid         (m1_probe_req_valid),
        .m1_probe_req_addr          (m1_probe_req_addr),
        .m1_probe_req_permissions   (m1_probe_req_permissions),
        .m1_probe_ack_valid         (m1_probe_ack_valid),
        .m1_probe_ack_addr          (m1_probe_ack_addr),
        .m1_probe_ack_permissions   (m1_probe_ack_permissions),
        .m1_probe_ack_dirty_data    (m1_probe_ack_dirty_data),
        .m2_req_valid               (m2_req_valid),
        .m2_req_addr                (m2_req_addr),
        .m2_req_type                (m2_req_type),
        .m2_req_data                (m2_req_data),
        .m2_req_permissions         (m2_req_permissions),
        .m2_req_ready               (m2_req_ready),
        .m2_data_valid              (m2_data_valid),
        .m2_data                    (m2_data),
        .m2_data_error              (m2_data_error),
        .m2_probe_req_valid         (m2_probe_req_valid),
        .m2_probe_req_addr          (m2_probe_req_addr),
        .m2_probe_req_permissions   (m2_probe_req_permissions),
        .m2_probe_ack_valid         (m2_probe_ack_valid),
        .m2_probe_ack_addr          (m2_probe_ack_addr),
        .m2_probe_ack_permissions   (m2_probe_ack_permissions),
        .m2_probe_ack_dirty_data    (m2_probe_ack_dirty_data),
        .slv_a_valid                (slv_a_valid),
        .slv_a_ready                (slv_a_ready),
        .slv_a_opcode               (slv_a_opcode),
        .slv_a_param                (slv_a_param),
        .slv_a_size                 (slv_a_size),
        .slv_a_source               (slv_a_source),
        .slv_a_address              (slv_a_address),
        .slv_a_mask                 (slv_a_mask),
        .slv_a_data                 (slv_a_data),
        .slv_b_valid                (slv_b_valid),
        .slv_b_ready                (slv_b_ready),
        .slv_b_opcode               (slv_b_opcode),
        .slv_b_param                (slv_b_param),
        .slv_b_size                 (slv_b_size),
        .slv_b_source               (slv_b_source),
        .slv_b_address              (slv_b_address),
        .slv_b_mask                 (slv_b_mask),
        .slv_b_data                 (slv_b_data),
        .slv_b_dest                 (slv_b_dest),
        .slv_c_valid                (slv_c_valid),
        .slv_c_ready                (slv_c_ready),
        .slv_c_opcode               (slv_c_opcode),
        .slv_c_param                (slv_c_param),
        .slv_c_size                 (slv_c_size),
        .slv_c_source               (slv_c_source),
        .slv_c_address              (slv_c_address),
        .slv_c_data                 (slv_c_data),
        .slv_d_valid                (slv_d_valid),
        .slv_d_ready                (slv_d_ready),
        .slv_d_opcode               (slv_d_opcode),
        .slv_d_param                (slv_d_param),
        .slv_d_size                 (slv_d_size),
        .slv_d_source               (slv_d_source),
        .slv_d_sink                 (slv_d_sink),
        .slv_d_denied               (slv_d_denied),
        .slv_d_data                 (slv_d_data),
        .slv_e_valid                (slv_e_valid),
        .slv_e_ready                (slv_e_ready),
        .slv_e_sink                 (slv_e_sink)
    );

    tlc_slave_mem_manager #(
        .SLAVE_ID   (0),
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .SOURCE_W   (SOURCE_W),
        .M_SOURCE_W (M_SOURCE_W),
        .SINK_W     (SINK_W),
        .N_MASTERS  (N_MASTERS),
        .MST_ID_W   (MST_ID_W),
        .DIR_DEPTH  (32)
    ) u_slv0_mgr (
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
        .b_corrupt (),
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
        .d_corrupt (),
        .e_valid   (slv_e_valid[0]),
        .e_ready   (slv_e_ready[0]),
        .e_sink    (slv_e_sink[0*SINK_W +: SINK_W]),
        .mem_req   (slv0_mem_req),
        .mem_we    (slv0_mem_we),
        .mem_addr  (slv0_mem_addr),
        .mem_wdata (slv0_mem_wdata),
        .mem_wmask (slv0_mem_wmask),
        .mem_rdata (slv0_mem_rdata),
        .mem_rvalid(slv0_mem_rvalid),
        .mem_ready (slv0_mem_ready)
    );

    raw_sram_ctrl_top #(
        .AW         (ADDR_W),
        .DW         (DATA_W),
        .AIW        (SOURCE_W),
        .NUM_BANKS  (NUM_BANKS),
        .BANK_DEPTH (BANK_DEPTH),
        .RD_LATENCY (RD_LATENCY),
        .WR_LATENCY (WR_LATENCY),
        .INIT_FILE  (MEM_INIT_FILE)
    ) u_slv0_mem (
        .clk                  (clk),
        .rst_n                (rst_n),
        .mem_req              (slv0_mem_req),
        .mem_ready            (slv0_mem_ready),
        .mem_we               (slv0_mem_we),
        .mem_addr             (slv0_mem_addr),
        .mem_wdata            (slv0_mem_wdata),
        .mem_wmask            (slv0_mem_wmask),
        .mem_source           ({SOURCE_W{1'b0}}),
        .mem_rvalid           (slv0_mem_rvalid),
        .mem_rdata            (slv0_mem_rdata),
        .mem_rsource          (slv0_mem_rsource),
        .total_rd_count       (slv0_total_rd_count),
        .total_wr_count       (slv0_total_wr_count),
        .total_rd_latency_sum (slv0_total_rd_latency_sum)
    );

    tlc_slave_mem_manager #(
        .SLAVE_ID   (1),
        .ADDR_W     (ADDR_W),
        .DATA_W     (DATA_W),
        .SOURCE_W   (SOURCE_W),
        .M_SOURCE_W (M_SOURCE_W),
        .SINK_W     (SINK_W),
        .N_MASTERS  (N_MASTERS),
        .MST_ID_W   (MST_ID_W),
        .DIR_DEPTH  (32)
    ) u_slv1_mgr (
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
        .b_corrupt (),
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
        .d_corrupt (),
        .e_valid   (slv_e_valid[1]),
        .e_ready   (slv_e_ready[1]),
        .e_sink    (slv_e_sink[1*SINK_W +: SINK_W]),
        .mem_req   (slv1_mem_req),
        .mem_we    (slv1_mem_we),
        .mem_addr  (slv1_mem_addr),
        .mem_wdata (slv1_mem_wdata),
        .mem_wmask (slv1_mem_wmask),
        .mem_rdata (slv1_mem_rdata),
        .mem_rvalid(slv1_mem_rvalid),
        .mem_ready (slv1_mem_ready)
    );

    raw_sram_ctrl_top #(
        .AW         (ADDR_W),
        .DW         (DATA_W),
        .AIW        (SOURCE_W),
        .NUM_BANKS  (NUM_BANKS),
        .BANK_DEPTH (BANK_DEPTH),
        .RD_LATENCY (RD_LATENCY),
        .WR_LATENCY (WR_LATENCY),
        .INIT_FILE  (MEM_INIT_FILE)
    ) u_slv1_mem (
        .clk                  (clk),
        .rst_n                (rst_n),
        .mem_req              (slv1_mem_req),
        .mem_ready            (slv1_mem_ready),
        .mem_we               (slv1_mem_we),
        .mem_addr             (slv1_mem_addr),
        .mem_wdata            (slv1_mem_wdata),
        .mem_wmask            (slv1_mem_wmask),
        .mem_source           ({SOURCE_W{1'b0}}),
        .mem_rvalid           (slv1_mem_rvalid),
        .mem_rdata            (slv1_mem_rdata),
        .mem_rsource          (slv1_mem_rsource),
        .total_rd_count       (slv1_total_rd_count),
        .total_wr_count       (slv1_total_wr_count),
        .total_rd_latency_sum (slv1_total_rd_latency_sum)
    );

endmodule

`default_nettype wire
