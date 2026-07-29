`timescale 1ns/10ps
`default_nettype none

// Standalone test of l2_tlc_bridge.v against the real crossbar + slave
// manager + memory backend (tl_xbar_nm via tlc_xbar_fabric_top,
// tlc_slave_mem_manager, raw_sram_ctrl_top), driving the mem_a_*/mem_d_*
// port directly the same way rv64g_l2_fsm.v's ST_MEM_WRITE/ST_MEM_READ
// states do (8-beat PutFullData burst, then a single Get expecting an
// 8-beat GrantData-equivalent response). This isolates and verifies the
// new bridge/8-beat-decomposition logic (docs/l2_tlc_xbar_integration_plan.md,
// Option A) independent of rv64g_l2_cache's own coherence-directory
// behavior, which tb_tlc_l2_dma_shared_mem.v exercises separately end to
// end through a real AcquireBlock.
module tb_l2_tlc_bridge_standalone;

    localparam integer CLK_HALF = 5;
    localparam ADDR_W = 64;
    localparam DATA_W = 64;

    reg clk;
    reg rst_n;

    always #CLK_HALF clk = ~clk;

    // ---- l2_tlc_bridge mem_a/mem_d stimulus (mimics rv64g_l2_fsm.v) ----
    reg  [2:0]        mem_a_opcode;
    reg  [ADDR_W-1:0] mem_a_address;
    reg  [DATA_W-1:0] mem_a_data;
    reg               mem_a_valid;
    wire              mem_a_ready;

    wire [2:0]        mem_d_opcode;
    wire [DATA_W-1:0] mem_d_data;
    wire              mem_d_valid;
    reg               mem_d_ready;

    wire                   m0_req_valid;
    wire [31:0]            m0_req_addr;
    wire [2:0]             m0_req_type;
    wire [255:0]           m0_req_data;
    wire [2:0]             m0_req_permissions;
    wire                   m0_req_ready;
    wire                   m0_data_valid;
    wire [255:0]           m0_data;
    wire                   m0_data_error;
    wire                   m0_probe_req_valid;
    wire [31:0]            m0_probe_req_addr;
    wire [2:0]             m0_probe_req_permissions;
    wire                   m0_probe_ack_valid;
    wire [31:0]            m0_probe_ack_addr;
    wire [2:0]             m0_probe_ack_permissions;
    wire [255:0]           m0_probe_ack_dirty_data;

    l2_tlc_bridge #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W)
    ) u_bridge (
        .clk  (clk),
        .rst_n(rst_n),

        .mem_a_opcode_i (mem_a_opcode),
        .mem_a_address_i(mem_a_address),
        .mem_a_data_i   (mem_a_data),
        .mem_a_valid_i  (mem_a_valid),
        .mem_a_ready_o  (mem_a_ready),

        .mem_d_opcode_o(mem_d_opcode),
        .mem_d_data_o  (mem_d_data),
        .mem_d_valid_o (mem_d_valid),
        .mem_d_ready_i (mem_d_ready),

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

    // ---- masters 1 and 2 tied off (not exercised by this test) ----
    localparam ADDR_W32 = 32;
    localparam DATA_WF  = 64;
    localparam SOURCE_W = 4;
    localparam SINK_W   = 4;
    localparam N_MASTERS = 3;
    localparam N_SLAVES  = 2;
    localparam MST_ID_W  = 2;
    localparam SLV_ID_W  = 1;
    localparam M_SOURCE_W = SOURCE_W + MST_ID_W;

    wire [N_SLAVES-1:0]            slv_a_valid,  slv_a_ready;
    wire [N_SLAVES*3-1:0]          slv_a_opcode, slv_a_param;
    wire [N_SLAVES*4-1:0]          slv_a_size;
    wire [N_SLAVES*M_SOURCE_W-1:0] slv_a_source;
    wire [N_SLAVES*ADDR_W32-1:0]   slv_a_address;
    wire [N_SLAVES*8-1:0]          slv_a_mask;
    wire [N_SLAVES*DATA_WF-1:0]    slv_a_data;

    wire [N_SLAVES-1:0]            slv_b_valid,  slv_b_ready;
    wire [N_SLAVES*3-1:0]          slv_b_opcode, slv_b_param;
    wire [N_SLAVES*4-1:0]          slv_b_size;
    wire [N_SLAVES*SOURCE_W-1:0]   slv_b_source;
    wire [N_SLAVES*ADDR_W32-1:0]   slv_b_address;
    wire [N_SLAVES*8-1:0]          slv_b_mask;
    wire [N_SLAVES*DATA_WF-1:0]    slv_b_data;
    wire [N_SLAVES*MST_ID_W-1:0]   slv_b_dest;

    wire [N_SLAVES-1:0]            slv_c_valid,  slv_c_ready;
    wire [N_SLAVES*3-1:0]          slv_c_opcode, slv_c_param;
    wire [N_SLAVES*4-1:0]          slv_c_size;
    wire [N_SLAVES*M_SOURCE_W-1:0] slv_c_source;
    wire [N_SLAVES*ADDR_W32-1:0]   slv_c_address;
    wire [N_SLAVES*DATA_WF-1:0]    slv_c_data;

    wire [N_SLAVES-1:0]            slv_d_valid,  slv_d_ready;
    wire [N_SLAVES*3-1:0]          slv_d_opcode;
    wire [N_SLAVES*2-1:0]          slv_d_param;
    wire [N_SLAVES*4-1:0]          slv_d_size;
    wire [N_SLAVES*M_SOURCE_W-1:0] slv_d_source;
    wire [N_SLAVES*SINK_W-1:0]     slv_d_sink;
    wire [N_SLAVES-1:0]            slv_d_denied;
    wire [N_SLAVES*DATA_WF-1:0]    slv_d_data;

    wire [N_SLAVES-1:0]            slv_e_valid,  slv_e_ready;
    wire [N_SLAVES*SINK_W-1:0]     slv_e_sink;

    wire                        slv0_mem_req, slv0_mem_we, slv0_mem_rvalid, slv0_mem_ready;
    wire [ADDR_W32-1:0]         slv0_mem_addr;
    wire [DATA_WF-1:0]          slv0_mem_wdata, slv0_mem_rdata;
    wire [DATA_WF/8-1:0]        slv0_mem_wmask;

    wire                        slv1_mem_req, slv1_mem_we, slv1_mem_rvalid, slv1_mem_ready;
    wire [ADDR_W32-1:0]         slv1_mem_addr;
    wire [DATA_WF-1:0]          slv1_mem_wdata, slv1_mem_rdata;
    wire [DATA_WF/8-1:0]        slv1_mem_wmask;

    tlc_xbar_fabric_top #(
        .ADDR_W    (ADDR_W32),
        .DATA_W    (DATA_WF),
        .SOURCE_W  (SOURCE_W),
        .SINK_W    (SINK_W),
        .N_MASTERS (N_MASTERS),
        .N_SLAVES  (N_SLAVES),
        .MST_ID_W  (MST_ID_W),
        .SLV_ID_W  (SLV_ID_W)
    ) u_fabric (
        .clk    (clk),
        .rst_n  (rst_n),

        .m0_req_valid              (m0_req_valid),
        .m0_req_addr               (m0_req_addr),
        .m0_req_type               (m0_req_type),
        .m0_req_data               (m0_req_data),
        .m0_req_permissions        (m0_req_permissions),
        .m0_req_ready              (m0_req_ready),
        .m0_data_valid             (m0_data_valid),
        .m0_data                   (m0_data),
        .m0_data_error             (m0_data_error),
        .m0_probe_req_valid        (m0_probe_req_valid),
        .m0_probe_req_addr         (m0_probe_req_addr),
        .m0_probe_req_permissions  (m0_probe_req_permissions),
        .m0_probe_ack_valid        (m0_probe_ack_valid),
        .m0_probe_ack_addr         (m0_probe_ack_addr),
        .m0_probe_ack_permissions  (m0_probe_ack_permissions),
        .m0_probe_ack_dirty_data   (m0_probe_ack_dirty_data),

        .m1_req_valid              (1'b0),
        .m1_req_addr               (32'b0),
        .m1_req_type               (3'b0),
        .m1_req_data               (256'b0),
        .m1_req_permissions        (3'b0),
        .m1_req_ready              (),
        .m1_data_valid             (),
        .m1_data                   (),
        .m1_data_error             (),
        .m1_probe_req_valid        (),
        .m1_probe_req_addr         (),
        .m1_probe_req_permissions  (),
        .m1_probe_ack_valid        (1'b0),
        .m1_probe_ack_addr         (32'b0),
        .m1_probe_ack_permissions  (3'b0),
        .m1_probe_ack_dirty_data   (256'b0),

        .m2_req_valid              (1'b0),
        .m2_req_addr               (32'b0),
        .m2_req_type               (3'b0),
        .m2_req_data               (256'b0),
        .m2_req_permissions        (3'b0),
        .m2_req_ready              (),
        .m2_data_valid             (),
        .m2_data                   (),
        .m2_data_error             (),
        .m2_probe_req_valid        (),
        .m2_probe_req_addr         (),
        .m2_probe_req_permissions  (),
        .m2_probe_ack_valid        (1'b0),
        .m2_probe_ack_addr         (32'b0),
        .m2_probe_ack_permissions  (3'b0),
        .m2_probe_ack_dirty_data   (256'b0),

        .slv_a_valid   (slv_a_valid),
        .slv_a_ready   (slv_a_ready),
        .slv_a_opcode  (slv_a_opcode),
        .slv_a_param   (slv_a_param),
        .slv_a_size    (slv_a_size),
        .slv_a_source  (slv_a_source),
        .slv_a_address (slv_a_address),
        .slv_a_mask    (slv_a_mask),
        .slv_a_data    (slv_a_data),
        .slv_b_valid   (slv_b_valid),
        .slv_b_ready   (slv_b_ready),
        .slv_b_opcode  (slv_b_opcode),
        .slv_b_param   (slv_b_param),
        .slv_b_size    (slv_b_size),
        .slv_b_source  (slv_b_source),
        .slv_b_address (slv_b_address),
        .slv_b_mask    (slv_b_mask),
        .slv_b_data    (slv_b_data),
        .slv_b_dest    (slv_b_dest),
        .slv_c_valid   (slv_c_valid),
        .slv_c_ready   (slv_c_ready),
        .slv_c_opcode  (slv_c_opcode),
        .slv_c_param   (slv_c_param),
        .slv_c_size    (slv_c_size),
        .slv_c_source  (slv_c_source),
        .slv_c_address (slv_c_address),
        .slv_c_data    (slv_c_data),
        .slv_d_valid   (slv_d_valid),
        .slv_d_ready   (slv_d_ready),
        .slv_d_opcode  (slv_d_opcode),
        .slv_d_param   (slv_d_param),
        .slv_d_size    (slv_d_size),
        .slv_d_source  (slv_d_source),
        .slv_d_sink    (slv_d_sink),
        .slv_d_denied  (slv_d_denied),
        .slv_d_data    (slv_d_data),
        .slv_e_valid   (slv_e_valid),
        .slv_e_ready   (slv_e_ready),
        .slv_e_sink    (slv_e_sink)
    );

    tlc_slave_mem_manager #(
        .SLAVE_ID  (0),
        .ADDR_W    (ADDR_W32),
        .DATA_W    (DATA_WF),
        .SOURCE_W  (SOURCE_W),
        .M_SOURCE_W(M_SOURCE_W),
        .SINK_W    (SINK_W),
        .N_MASTERS (N_MASTERS),
        .MST_ID_W  (MST_ID_W),
        .DIR_DEPTH (32)
    ) u_slv0_mgr (
        .clk       (clk),
        .rst_n     (rst_n),
        .a_valid   (slv_a_valid[0]),
        .a_ready   (slv_a_ready[0]),
        .a_opcode  (slv_a_opcode[0*3 +: 3]),
        .a_param   (slv_a_param[0*3 +: 3]),
        .a_size    (slv_a_size[0*4 +: 4]),
        .a_source  (slv_a_source[0*M_SOURCE_W +: M_SOURCE_W]),
        .a_address (slv_a_address[0*ADDR_W32 +: ADDR_W32]),
        .a_mask    (slv_a_mask[0*8 +: 8]),
        .a_data    (slv_a_data[0*DATA_WF +: DATA_WF]),
        .a_corrupt (1'b0),
        .b_valid   (slv_b_valid[0]),
        .b_ready   (slv_b_ready[0]),
        .b_opcode  (slv_b_opcode[0*3 +: 3]),
        .b_param   (slv_b_param[0*3 +: 3]),
        .b_size    (slv_b_size[0*4 +: 4]),
        .b_source  (slv_b_source[0*SOURCE_W +: SOURCE_W]),
        .b_address (slv_b_address[0*ADDR_W32 +: ADDR_W32]),
        .b_mask    (slv_b_mask[0*8 +: 8]),
        .b_data    (slv_b_data[0*DATA_WF +: DATA_WF]),
        .b_corrupt (),
        .b_dest    (slv_b_dest[0*MST_ID_W +: MST_ID_W]),
        .c_valid   (slv_c_valid[0]),
        .c_ready   (slv_c_ready[0]),
        .c_opcode  (slv_c_opcode[0*3 +: 3]),
        .c_param   (slv_c_param[0*3 +: 3]),
        .c_size    (slv_c_size[0*4 +: 4]),
        .c_source  (slv_c_source[0*M_SOURCE_W +: M_SOURCE_W]),
        .c_address (slv_c_address[0*ADDR_W32 +: ADDR_W32]),
        .c_data    (slv_c_data[0*DATA_WF +: DATA_WF]),
        .c_corrupt (1'b0),
        .d_valid   (slv_d_valid[0]),
        .d_ready   (slv_d_ready[0]),
        .d_opcode  (slv_d_opcode[0*3 +: 3]),
        .d_param   (slv_d_param[0*2 +: 2]),
        .d_size    (slv_d_size[0*4 +: 4]),
        .d_source  (slv_d_source[0*M_SOURCE_W +: M_SOURCE_W]),
        .d_sink    (slv_d_sink[0*SINK_W +: SINK_W]),
        .d_denied  (slv_d_denied[0]),
        .d_data    (slv_d_data[0*DATA_WF +: DATA_WF]),
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
        .AW        (ADDR_W32),
        .DW        (DATA_WF),
        .AIW       (SOURCE_W),
        .NUM_BANKS (4),
        .BANK_DEPTH(262144),
        .RD_LATENCY(1),
        .WR_LATENCY(1),
        .INIT_FILE ("")
    ) u_slv0_mem (
        .clk       (clk),
        .rst_n     (rst_n),
        .mem_req   (slv0_mem_req),
        .mem_ready (slv0_mem_ready),
        .mem_we    (slv0_mem_we),
        .mem_addr  (slv0_mem_addr),
        .mem_wdata (slv0_mem_wdata),
        .mem_wmask (slv0_mem_wmask),
        .mem_source({SOURCE_W{1'b0}}),
        .mem_rvalid(slv0_mem_rvalid),
        .mem_rdata (slv0_mem_rdata),
        .mem_rsource(),
        .total_rd_count(),
        .total_wr_count(),
        .total_rd_latency_sum()
    );

    tlc_slave_mem_manager #(
        .SLAVE_ID  (1),
        .ADDR_W    (ADDR_W32),
        .DATA_W    (DATA_WF),
        .SOURCE_W  (SOURCE_W),
        .M_SOURCE_W(M_SOURCE_W),
        .SINK_W    (SINK_W),
        .N_MASTERS (N_MASTERS),
        .MST_ID_W  (MST_ID_W),
        .DIR_DEPTH (32)
    ) u_slv1_mgr (
        .clk       (clk),
        .rst_n     (rst_n),
        .a_valid   (slv_a_valid[1]),
        .a_ready   (slv_a_ready[1]),
        .a_opcode  (slv_a_opcode[1*3 +: 3]),
        .a_param   (slv_a_param[1*3 +: 3]),
        .a_size    (slv_a_size[1*4 +: 4]),
        .a_source  (slv_a_source[1*M_SOURCE_W +: M_SOURCE_W]),
        .a_address (slv_a_address[1*ADDR_W32 +: ADDR_W32]),
        .a_mask    (slv_a_mask[1*8 +: 8]),
        .a_data    (slv_a_data[1*DATA_WF +: DATA_WF]),
        .a_corrupt (1'b0),
        .b_valid   (slv_b_valid[1]),
        .b_ready   (slv_b_ready[1]),
        .b_opcode  (slv_b_opcode[1*3 +: 3]),
        .b_param   (slv_b_param[1*3 +: 3]),
        .b_size    (slv_b_size[1*4 +: 4]),
        .b_source  (slv_b_source[1*SOURCE_W +: SOURCE_W]),
        .b_address (slv_b_address[1*ADDR_W32 +: ADDR_W32]),
        .b_mask    (slv_b_mask[1*8 +: 8]),
        .b_data    (slv_b_data[1*DATA_WF +: DATA_WF]),
        .b_corrupt (),
        .b_dest    (slv_b_dest[1*MST_ID_W +: MST_ID_W]),
        .c_valid   (slv_c_valid[1]),
        .c_ready   (slv_c_ready[1]),
        .c_opcode  (slv_c_opcode[1*3 +: 3]),
        .c_param   (slv_c_param[1*3 +: 3]),
        .c_size    (slv_c_size[1*4 +: 4]),
        .c_source  (slv_c_source[1*M_SOURCE_W +: M_SOURCE_W]),
        .c_address (slv_c_address[1*ADDR_W32 +: ADDR_W32]),
        .c_data    (slv_c_data[1*DATA_WF +: DATA_WF]),
        .c_corrupt (1'b0),
        .d_valid   (slv_d_valid[1]),
        .d_ready   (slv_d_ready[1]),
        .d_opcode  (slv_d_opcode[1*3 +: 3]),
        .d_param   (slv_d_param[1*2 +: 2]),
        .d_size    (slv_d_size[1*4 +: 4]),
        .d_source  (slv_d_source[1*M_SOURCE_W +: M_SOURCE_W]),
        .d_sink    (slv_d_sink[1*SINK_W +: SINK_W]),
        .d_denied  (slv_d_denied[1]),
        .d_data    (slv_d_data[1*DATA_WF +: DATA_WF]),
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
        .AW        (ADDR_W32),
        .DW        (DATA_WF),
        .AIW       (SOURCE_W),
        .NUM_BANKS (4),
        .BANK_DEPTH(262144),
        .RD_LATENCY(1),
        .WR_LATENCY(1),
        .INIT_FILE ("")
    ) u_slv1_mem (
        .clk       (clk),
        .rst_n     (rst_n),
        .mem_req   (slv1_mem_req),
        .mem_ready (slv1_mem_ready),
        .mem_we    (slv1_mem_we),
        .mem_addr  (slv1_mem_addr),
        .mem_wdata (slv1_mem_wdata),
        .mem_wmask (slv1_mem_wmask),
        .mem_source({SOURCE_W{1'b0}}),
        .mem_rvalid(slv1_mem_rvalid),
        .mem_rdata (slv1_mem_rdata),
        .mem_rsource(),
        .total_rd_count(),
        .total_wr_count(),
        .total_rd_latency_sum()
    );

    // ---- Test sequence ----
    localparam [ADDR_W-1:0] BASE_ADDR = 64'h0000_0000_0000_1000;
    reg [63:0] wr_pattern [0:7];
    reg [63:0] rd_capture [0:7];
    integer i;
    integer errors;

    // Settle helper: advance one clock edge and let NBA-driven
    // combinational outputs (mem_a_ready_o, mem_d_valid_o -- both `assign`
    // straight off a register that updates via NBA on this same edge)
    // resolve before anything checks them.
    task automatic settle;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    // mem_a_valid is held high continuously across the whole burst (never
    // dropped between beats) and only mem_a_data changes per accepted
    // beat -- matching how rv64g_l2_fsm.v's ST_MEM_WRITE actually drives
    // mem_a_valid_o (unconditionally 1 for the whole burst; only
    // data_word_sel_o/mem_a_data_o change per beat).
    //
    // Checks mem_a_ready BEFORE waiting for a new edge, not after: a
    // version that unconditionally called settle() once before its first
    // check ran chronically one edge late relative to the DUT's own
    // ready window (confirmed by simulation -- every call ended up
    // observing the *following* beat's ready pulse instead of its own,
    // which happened to look like success for beats 0-6 since there was
    // always a next pulse to catch, and then hung forever on the last
    // beat, which has none).
    task automatic write_beat(input [63:0] data);
        begin
            mem_a_data = data;
            while (!mem_a_ready) settle();
            settle(); // consume the accept edge before the next beat's check
        end
    endtask

    // Likewise, mem_d_ready is asserted once by the caller and held for
    // the whole multi-beat sequence; this task only samples mem_d_valid.
    task automatic wait_d_beat(output [63:0] data);
        begin
            while (!mem_d_valid) settle();
            data = mem_d_data;
            settle(); // consume the beat before the next one is checked
        end
    endtask

    initial begin
        clk = 0;
        rst_n = 0;
        mem_a_opcode  = 3'b0;
        mem_a_address = {ADDR_W{1'b0}};
        mem_a_data    = {DATA_W{1'b0}};
        mem_a_valid   = 1'b0;
        mem_d_ready   = 1'b0;
        errors = 0;

        for (i = 0; i < 8; i = i + 1) begin
            wr_pattern[i] = 64'hA5A5_0000_0000_0000 + i;
        end

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // --- 8-beat PutFullData write burst, mimicking ST_MEM_WRITE ---
        $display("[TB] Starting 8-beat write burst to addr 0x%016h", BASE_ADDR);
        mem_a_opcode  = 3'd0; // PutFullData
        mem_a_address = BASE_ADDR;
        mem_a_valid   = 1'b1;
        for (i = 0; i < 8; i = i + 1) begin
            write_beat(wr_pattern[i]);
        end
        mem_a_valid = 1'b0;

        // Final AccessAck for the whole write
        mem_d_ready = 1'b1;
        while (!mem_d_valid) settle();
        if (mem_d_opcode !== 3'd0) begin // D_OPCODE_ACCESS_ACK
            $display("[TB][FAIL] write completion opcode=%0d, expected AccessAck(0)", mem_d_opcode);
            errors = errors + 1;
        end
        settle();
        mem_d_ready = 1'b0;
        $display("[TB] Write burst complete");

        repeat (5) @(posedge clk);

        // --- Single Get, expecting 8-beat GrantData-equivalent response ---
        $display("[TB] Starting Get read-back from addr 0x%016h", BASE_ADDR);
        mem_a_opcode  = 3'd4; // Get
        mem_a_address = BASE_ADDR;
        mem_a_data    = {DATA_W{1'b0}};
        mem_a_valid   = 1'b1;
        while (!mem_a_ready) settle();
        settle();
        mem_a_valid = 1'b0;

        mem_d_ready = 1'b1;
        for (i = 0; i < 8; i = i + 1) begin
            wait_d_beat(rd_capture[i]);
            if (mem_d_opcode !== 3'd1) begin // D_OPCODE_ACCESS_ACK_DATA
                $display("[TB][FAIL] beat %0d opcode=%0d, expected AccessAckData(1)", i, mem_d_opcode);
                errors = errors + 1;
            end
        end
        mem_d_ready = 1'b0;

        for (i = 0; i < 8; i = i + 1) begin
            if (rd_capture[i] !== wr_pattern[i]) begin
                $display("[TB][FAIL] beat %0d mismatch: wrote 0x%016h read 0x%016h", i, wr_pattern[i], rd_capture[i]);
                errors = errors + 1;
            end else begin
                $display("[TB][PASS] beat %0d matched: 0x%016h", i, rd_capture[i]);
            end
        end

        if (errors == 0) begin
            $display("[TB] ALL CHECKS PASSED");
        end else begin
            $display("[TB] %0d CHECK(S) FAILED", errors);
        end

        $finish;
    end

    initial begin
        #200000;
        $display("[TB][FAIL] TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
