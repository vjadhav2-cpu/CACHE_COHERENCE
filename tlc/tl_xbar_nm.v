// tl_xbar_nm.v — TileLink-C parametric N-master × M-slave crossbar
//
// Supports all five TL-C channels (A, B, C, D, E) across an arbitrary
// number of masters (e.g. L1 caches) and slaves (e.g. L2 caches or
// memory-mapped peripherals), with address-based slave routing.
//
// ──────────────────────────────────────────────────────────────────────
// ROUTING SUMMARY
// ──────────────────────────────────────────────────────────────────────
//   A (master→slave) : per-slave round-robin arbiter, address-decoded
//   B (slave→master) : per-master round-robin arbiter, slv_b_dest_i field
//   C (master→slave) : per-slave round-robin arbiter, address-decoded
//   D (slave→master) : per-master round-robin arbiter, master_id in source
//   E (master→slave) : per-slave round-robin arbiter, slave_id in sink
//
// ──────────────────────────────────────────────────────────────────────
// SOURCE / SINK EXTENSION CONVENTION
// ──────────────────────────────────────────────────────────────────────
//   Extended source (slaves see on A/C):
//       bits [M_SOURCE_W-1 : SOURCE_W]  = master_id
//       bits [SOURCE_W-1   : 0        ]  = original source from master
//   Slaves MUST echo the full M_SOURCE_W-bit extended source on D.
//
//   Extended sink (masters see on D/E):
//       bits [M_SINK_W-1 : SINK_W]  = slave_id
//       bits [SINK_W-1   : 0     ]  = original sink from slave
//   Masters MUST echo the full M_SINK_W-bit extended sink on E.
//
// ──────────────────────────────────────────────────────────────────────
// ADDRESS MAP
// ──────────────────────────────────────────────────────────────────────
//   Slave j is selected when:
//       (address & SLAVE_ADDR_MASK[j]) == SLAVE_ADDR_BASE[j]
//   Both are flattened N_SLAVES*ADDR_W-bit parameters.
//   Last-matching slave wins (allows overlapping ranges with priority).
//
`timescale 1ns/10ps
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM  */

module tl_xbar_nm #(
    parameter integer N_MASTERS  = 3,
    parameter integer N_SLAVES   = 2,
    parameter integer DATA_W     = 64,
    parameter integer ADDR_W     = 64,
    parameter integer SOURCE_W   = 4,
    parameter integer SINK_W     = 4,

    // Flattened address map (N_SLAVES entries, each ADDR_W wide).
    // Slave j occupies index [j*ADDR_W +: ADDR_W].
    parameter [N_SLAVES*ADDR_W-1:0] SLAVE_ADDR_BASE = {N_SLAVES*ADDR_W{1'b0}},
    parameter [N_SLAVES*ADDR_W-1:0] SLAVE_ADDR_MASK = {N_SLAVES*ADDR_W{1'b0}},

    // Derived — do not override at instantiation
    parameter integer MST_ID_W   = (N_MASTERS > 1) ? $clog2(N_MASTERS) : 1,
    parameter integer SLV_ID_W   = (N_SLAVES  > 1) ? $clog2(N_SLAVES)  : 1,
    parameter integer M_SOURCE_W = SOURCE_W + MST_ID_W,   // width seen by slaves on A/C/D source
    parameter integer M_SINK_W   = SINK_W   + SLV_ID_W    // width seen by masters on D/E sink
) (
    input  wire clk,
    input  wire rst_n,

    // ================================================================
    // MASTER INTERFACES — [N_MASTERS] flattened arrays
    //   Master index i occupies bit range [i*W +: W] in every bus.
    // ================================================================

    // A : master → crossbar
    input  wire [N_MASTERS-1:0]           mst_a_valid_i,
    output wire [N_MASTERS-1:0]           mst_a_ready_o,
    input  wire [N_MASTERS*3-1:0]         mst_a_opcode_i,
    input  wire [N_MASTERS*3-1:0]         mst_a_param_i,
    input  wire [N_MASTERS*4-1:0]         mst_a_size_i,
    input  wire [N_MASTERS*SOURCE_W-1:0]  mst_a_source_i,
    input  wire [N_MASTERS*ADDR_W-1:0]    mst_a_address_i,
    input  wire [N_MASTERS*8-1:0]         mst_a_mask_i,
    input  wire [N_MASTERS*DATA_W-1:0]    mst_a_data_i,
    input  wire [N_MASTERS-1:0]           mst_a_corrupt_i,

    // B : crossbar → master
    output wire [N_MASTERS-1:0]           mst_b_valid_o,
    input  wire [N_MASTERS-1:0]           mst_b_ready_i,
    output wire [N_MASTERS*3-1:0]         mst_b_opcode_o,
    output wire [N_MASTERS*3-1:0]         mst_b_param_o,
    output wire [N_MASTERS*4-1:0]         mst_b_size_o,
    output wire [N_MASTERS*SOURCE_W-1:0]  mst_b_source_o,
    output wire [N_MASTERS*ADDR_W-1:0]    mst_b_address_o,
    output wire [N_MASTERS*8-1:0]         mst_b_mask_o,
    output wire [N_MASTERS*DATA_W-1:0]    mst_b_data_o,
    output wire [N_MASTERS-1:0]           mst_b_corrupt_o,

    // C : master → crossbar
    input  wire [N_MASTERS-1:0]           mst_c_valid_i,
    output wire [N_MASTERS-1:0]           mst_c_ready_o,
    input  wire [N_MASTERS*3-1:0]         mst_c_opcode_i,
    input  wire [N_MASTERS*3-1:0]         mst_c_param_i,
    input  wire [N_MASTERS*4-1:0]         mst_c_size_i,
    input  wire [N_MASTERS*SOURCE_W-1:0]  mst_c_source_i,
    input  wire [N_MASTERS*ADDR_W-1:0]    mst_c_address_i,
    input  wire [N_MASTERS*DATA_W-1:0]    mst_c_data_i,
    input  wire [N_MASTERS-1:0]           mst_c_corrupt_i,

    // D : crossbar → master   (sink is extended: {slave_id, orig_sink})
    output wire [N_MASTERS-1:0]           mst_d_valid_o,
    input  wire [N_MASTERS-1:0]           mst_d_ready_i,
    output wire [N_MASTERS*3-1:0]         mst_d_opcode_o,
    output wire [N_MASTERS*2-1:0]         mst_d_param_o,
    output wire [N_MASTERS*4-1:0]         mst_d_size_o,
    output wire [N_MASTERS*SOURCE_W-1:0]  mst_d_source_o,
    output wire [N_MASTERS*M_SINK_W-1:0]  mst_d_sink_o,
    output wire [N_MASTERS-1:0]           mst_d_denied_o,
    output wire [N_MASTERS*DATA_W-1:0]    mst_d_data_o,
    output wire [N_MASTERS-1:0]           mst_d_corrupt_o,

    // E : master → crossbar   (sink is extended: {slave_id, orig_sink})
    input  wire [N_MASTERS-1:0]           mst_e_valid_i,
    output wire [N_MASTERS-1:0]           mst_e_ready_o,
    input  wire [N_MASTERS*M_SINK_W-1:0]  mst_e_sink_i,

    // ================================================================
    // SLAVE INTERFACES — [N_SLAVES] flattened arrays
    //   Slave index j occupies bit range [j*W +: W] in every bus.
    // ================================================================

    // A : crossbar → slave   (source is extended: {master_id, orig_source})
    output wire [N_SLAVES-1:0]            slv_a_valid_o,
    input  wire [N_SLAVES-1:0]            slv_a_ready_i,
    output wire [N_SLAVES*3-1:0]          slv_a_opcode_o,
    output wire [N_SLAVES*3-1:0]          slv_a_param_o,
    output wire [N_SLAVES*4-1:0]          slv_a_size_o,
    output wire [N_SLAVES*M_SOURCE_W-1:0] slv_a_source_o,
    output wire [N_SLAVES*ADDR_W-1:0]     slv_a_address_o,
    output wire [N_SLAVES*8-1:0]          slv_a_mask_o,
    output wire [N_SLAVES*DATA_W-1:0]     slv_a_data_o,
    output wire [N_SLAVES-1:0]            slv_a_corrupt_o,

    // B : slave → crossbar   (slave supplies slv_b_dest_i: target master index)
    input  wire [N_SLAVES-1:0]            slv_b_valid_i,
    output wire [N_SLAVES-1:0]            slv_b_ready_o,
    input  wire [N_SLAVES*3-1:0]          slv_b_opcode_i,
    input  wire [N_SLAVES*3-1:0]          slv_b_param_i,
    input  wire [N_SLAVES*4-1:0]          slv_b_size_i,
    input  wire [N_SLAVES*SOURCE_W-1:0]   slv_b_source_i,
    input  wire [N_SLAVES*ADDR_W-1:0]     slv_b_address_i,
    input  wire [N_SLAVES*8-1:0]          slv_b_mask_i,
    input  wire [N_SLAVES*DATA_W-1:0]     slv_b_data_i,
    input  wire [N_SLAVES-1:0]            slv_b_corrupt_i,
    input  wire [N_SLAVES*MST_ID_W-1:0]   slv_b_dest_i,    // which master to probe

    // C : crossbar → slave   (source is extended: {master_id, orig_source})
    output wire [N_SLAVES-1:0]            slv_c_valid_o,
    input  wire [N_SLAVES-1:0]            slv_c_ready_i,
    output wire [N_SLAVES*3-1:0]          slv_c_opcode_o,
    output wire [N_SLAVES*3-1:0]          slv_c_param_o,
    output wire [N_SLAVES*4-1:0]          slv_c_size_o,
    output wire [N_SLAVES*M_SOURCE_W-1:0] slv_c_source_o,
    output wire [N_SLAVES*ADDR_W-1:0]     slv_c_address_o,
    output wire [N_SLAVES*DATA_W-1:0]     slv_c_data_o,
    output wire [N_SLAVES-1:0]            slv_c_corrupt_o,

    // D : slave → crossbar   (source MUST be the extended source echoed from A/C)
    input  wire [N_SLAVES-1:0]            slv_d_valid_i,
    output wire [N_SLAVES-1:0]            slv_d_ready_o,
    input  wire [N_SLAVES*3-1:0]          slv_d_opcode_i,
    input  wire [N_SLAVES*2-1:0]          slv_d_param_i,
    input  wire [N_SLAVES*4-1:0]          slv_d_size_i,
    input  wire [N_SLAVES*M_SOURCE_W-1:0] slv_d_source_i,  // extended, echoed from A/C
    input  wire [N_SLAVES*SINK_W-1:0]     slv_d_sink_i,
    input  wire [N_SLAVES-1:0]            slv_d_denied_i,
    input  wire [N_SLAVES*DATA_W-1:0]     slv_d_data_i,
    input  wire [N_SLAVES-1:0]            slv_d_corrupt_i,

    // E : crossbar → slave   (orig_sink stripped of slave_id)
    output wire [N_SLAVES-1:0]            slv_e_valid_o,
    input  wire [N_SLAVES-1:0]            slv_e_ready_i,
    output wire [N_SLAVES*SINK_W-1:0]     slv_e_sink_o
);

    // ================================================================
    // Packed bundle widths (for tl_arbiter DATA_W parameter)
    // ================================================================
    // A bundle: {master_id, opcode, param, size, source, address, mask, data, corrupt}
    localparam A_PACKED_W = MST_ID_W + 3 + 3 + 4 + SOURCE_W + ADDR_W + 8 + DATA_W + 1;
    // C bundle: {master_id, opcode, param, size, source, address, data, corrupt}
    localparam C_PACKED_W = MST_ID_W + 3 + 3 + 4 + SOURCE_W + ADDR_W + DATA_W + 1;
    // B bundle: {opcode, param, size, source, address, mask, data, corrupt}
    localparam B_PACKED_W = 3 + 3 + 4 + SOURCE_W + ADDR_W + 8 + DATA_W + 1;
    // D bundle: {opcode, param, size, orig_source, ext_sink={slave_id,orig_sink}, denied, data, corrupt}
    localparam D_PACKED_W = 3 + 2 + 4 + SOURCE_W + M_SINK_W + 1 + DATA_W + 1;
    // E bundle: {orig_sink}  (slave_id consumed by routing logic, not passed to slave)
    localparam E_PACKED_W = SINK_W;

    // ================================================================
    // Ready-bus collections
    //   a_arb_ready_bus[slv * N_MASTERS + mst] = arb_a[slv].ready_o[mst]
    //   b_arb_ready_bus[mst * N_SLAVES  + slv] = arb_b[mst].ready_o[slv]
    //   (similarly for c, d, e)
    // ================================================================
    wire [N_SLAVES *N_MASTERS-1:0] a_arb_rdy_bus;
    wire [N_SLAVES *N_MASTERS-1:0] c_arb_rdy_bus;
    wire [N_SLAVES *N_MASTERS-1:0] e_arb_rdy_bus;
    wire [N_MASTERS*N_SLAVES -1:0] b_arb_rdy_bus;
    wire [N_MASTERS*N_SLAVES -1:0] d_arb_rdy_bus;

    // ================================================================
    // ADDRESS DECODE
    // For each master i, determine which slave each A/C address maps to.
    // Last-matching slave wins (enables default/fallback slaves).
    // ================================================================
    wire [N_MASTERS*SLV_ID_W-1:0] a_slave_sel;
    wire [N_MASTERS*SLV_ID_W-1:0] c_slave_sel;

    genvar gmi_dec;
    generate
        for (gmi_dec = 0; gmi_dec < N_MASTERS; gmi_dec = gmi_dec + 1) begin : g_addr_dec
            wire [ADDR_W-1:0] a_addr_w = mst_a_address_i[gmi_dec*ADDR_W +: ADDR_W];
            wire [ADDR_W-1:0] c_addr_w = mst_c_address_i[gmi_dec*ADDR_W +: ADDR_W];
            reg [SLV_ID_W-1:0] a_sel_r;
            reg [SLV_ID_W-1:0] c_sel_r;
            integer ds;
            always @(*) begin
                a_sel_r = {SLV_ID_W{1'b0}};
                c_sel_r = {SLV_ID_W{1'b0}};
                for (ds = 0; ds < N_SLAVES; ds = ds + 1) begin
                    if ((a_addr_w & SLAVE_ADDR_MASK[ds*ADDR_W +: ADDR_W]) ==
                                    SLAVE_ADDR_BASE[ds*ADDR_W +: ADDR_W])
                        a_sel_r = ds[SLV_ID_W-1:0];
                    if ((c_addr_w & SLAVE_ADDR_MASK[ds*ADDR_W +: ADDR_W]) ==
                                    SLAVE_ADDR_BASE[ds*ADDR_W +: ADDR_W])
                        c_sel_r = ds[SLV_ID_W-1:0];
                end
            end
            assign a_slave_sel[gmi_dec*SLV_ID_W +: SLV_ID_W] = a_sel_r;
            assign c_slave_sel[gmi_dec*SLV_ID_W +: SLV_ID_W] = c_sel_r;
        end
    endgenerate

    // ================================================================
    // CHANNEL A — per-slave arbiter (N_MASTERS inputs each)
    // ================================================================
    genvar gsi_a;
    generate
        for (gsi_a = 0; gsi_a < N_SLAVES; gsi_a = gsi_a + 1) begin : g_slv_a

            wire [N_MASTERS-1:0]            a_vf;    // filtered valid
            wire [N_MASTERS*A_PACKED_W-1:0] a_din;   // packed input data
            wire [N_MASTERS-1:0]            a_rdy;   // per-master ready from arbiter
            wire [A_PACKED_W-1:0]           a_dout;  // arbiter winner data

            genvar gmi_a;
            for (gmi_a = 0; gmi_a < N_MASTERS; gmi_a = gmi_a + 1) begin : g_mst_a_filt
                assign a_vf[gmi_a] = mst_a_valid_i[gmi_a] &
                    (a_slave_sel[gmi_a*SLV_ID_W +: SLV_ID_W] == gsi_a[SLV_ID_W-1:0]);

                assign a_din[gmi_a*A_PACKED_W +: A_PACKED_W] = {
                    gmi_a[MST_ID_W-1:0],
                    mst_a_opcode_i [gmi_a*3       +: 3],
                    mst_a_param_i  [gmi_a*3       +: 3],
                    mst_a_size_i   [gmi_a*4       +: 4],
                    mst_a_source_i [gmi_a*SOURCE_W +: SOURCE_W],
                    mst_a_address_i[gmi_a*ADDR_W  +: ADDR_W],
                    mst_a_mask_i   [gmi_a*8       +: 8],
                    mst_a_data_i   [gmi_a*DATA_W  +: DATA_W],
                    mst_a_corrupt_i[gmi_a]
                };

                assign a_arb_rdy_bus[gsi_a*N_MASTERS + gmi_a] = a_rdy[gmi_a];
            end

            tl_arbiter #(.N(N_MASTERS), .DATA_W(A_PACKED_W)) arb_a (
                .clk    (clk),          .rst_n  (rst_n),
                .valid_i(a_vf),         .ready_o(a_rdy),
                .data_i (a_din),
                .valid_o(slv_a_valid_o[gsi_a]),
                .ready_i(slv_a_ready_i[gsi_a]),
                .data_o (a_dout)
            );

            // Unpack and extend source: {master_id, orig_source}
            wire [MST_ID_W-1:0] a_mid;
            wire [SOURCE_W-1:0] a_src;
            assign {
                a_mid,
                slv_a_opcode_o [gsi_a*3        +: 3],
                slv_a_param_o  [gsi_a*3        +: 3],
                slv_a_size_o   [gsi_a*4        +: 4],
                a_src,
                slv_a_address_o[gsi_a*ADDR_W   +: ADDR_W],
                slv_a_mask_o   [gsi_a*8        +: 8],
                slv_a_data_o   [gsi_a*DATA_W   +: DATA_W],
                slv_a_corrupt_o[gsi_a]
            } = a_dout;
            assign slv_a_source_o[gsi_a*M_SOURCE_W +: M_SOURCE_W] = {a_mid, a_src};
        end
    endgenerate

    // Collect mst_a_ready_o: OR over all slave arbiters
    genvar gmi_ar;
    generate
        for (gmi_ar = 0; gmi_ar < N_MASTERS; gmi_ar = gmi_ar + 1) begin : g_mst_a_rdy
            wire [N_SLAVES-1:0] bits;
            genvar gsi_ar;
            for (gsi_ar = 0; gsi_ar < N_SLAVES; gsi_ar = gsi_ar + 1) begin : g_ar_bits
                assign bits[gsi_ar] = a_arb_rdy_bus[gsi_ar*N_MASTERS + gmi_ar];
            end
            assign mst_a_ready_o[gmi_ar] = |bits;
        end
    endgenerate

    // ================================================================
    // CHANNEL C — per-slave arbiter (same structure as A, no mask field)
    // ================================================================
    genvar gsi_c;
    generate
        for (gsi_c = 0; gsi_c < N_SLAVES; gsi_c = gsi_c + 1) begin : g_slv_c

            wire [N_MASTERS-1:0]            c_vf;
            wire [N_MASTERS*C_PACKED_W-1:0] c_din;
            wire [N_MASTERS-1:0]            c_rdy;
            wire [C_PACKED_W-1:0]           c_dout;

            genvar gmi_c;
            for (gmi_c = 0; gmi_c < N_MASTERS; gmi_c = gmi_c + 1) begin : g_mst_c_filt
                assign c_vf[gmi_c] = mst_c_valid_i[gmi_c] &
                    (c_slave_sel[gmi_c*SLV_ID_W +: SLV_ID_W] == gsi_c[SLV_ID_W-1:0]);

                assign c_din[gmi_c*C_PACKED_W +: C_PACKED_W] = {
                    gmi_c[MST_ID_W-1:0],
                    mst_c_opcode_i [gmi_c*3       +: 3],
                    mst_c_param_i  [gmi_c*3       +: 3],
                    mst_c_size_i   [gmi_c*4       +: 4],
                    mst_c_source_i [gmi_c*SOURCE_W +: SOURCE_W],
                    mst_c_address_i[gmi_c*ADDR_W  +: ADDR_W],
                    mst_c_data_i   [gmi_c*DATA_W  +: DATA_W],
                    mst_c_corrupt_i[gmi_c]
                };

                assign c_arb_rdy_bus[gsi_c*N_MASTERS + gmi_c] = c_rdy[gmi_c];
            end

            tl_arbiter #(.N(N_MASTERS), .DATA_W(C_PACKED_W)) arb_c (
                .clk    (clk),          .rst_n  (rst_n),
                .valid_i(c_vf),         .ready_o(c_rdy),
                .data_i (c_din),
                .valid_o(slv_c_valid_o[gsi_c]),
                .ready_i(slv_c_ready_i[gsi_c]),
                .data_o (c_dout)
            );

            wire [MST_ID_W-1:0] c_mid;
            wire [SOURCE_W-1:0] c_src;
            assign {
                c_mid,
                slv_c_opcode_o [gsi_c*3        +: 3],
                slv_c_param_o  [gsi_c*3        +: 3],
                slv_c_size_o   [gsi_c*4        +: 4],
                c_src,
                slv_c_address_o[gsi_c*ADDR_W   +: ADDR_W],
                slv_c_data_o   [gsi_c*DATA_W   +: DATA_W],
                slv_c_corrupt_o[gsi_c]
            } = c_dout;
            assign slv_c_source_o[gsi_c*M_SOURCE_W +: M_SOURCE_W] = {c_mid, c_src};
        end
    endgenerate

    genvar gmi_cr;
    generate
        for (gmi_cr = 0; gmi_cr < N_MASTERS; gmi_cr = gmi_cr + 1) begin : g_mst_c_rdy
            wire [N_SLAVES-1:0] bits;
            genvar gsi_cr;
            for (gsi_cr = 0; gsi_cr < N_SLAVES; gsi_cr = gsi_cr + 1) begin : g_cr_bits
                assign bits[gsi_cr] = c_arb_rdy_bus[gsi_cr*N_MASTERS + gmi_cr];
            end
            assign mst_c_ready_o[gmi_cr] = |bits;
        end
    endgenerate

    // ================================================================
    // CHANNEL E — per-slave arbiter (route by slave_id in extended sink)
    // ================================================================
    genvar gsi_e;
    generate
        for (gsi_e = 0; gsi_e < N_SLAVES; gsi_e = gsi_e + 1) begin : g_slv_e

            wire [N_MASTERS-1:0]            e_vf;
            wire [N_MASTERS*E_PACKED_W-1:0] e_din;
            wire [N_MASTERS-1:0]            e_rdy;
            wire [E_PACKED_W-1:0]           e_dout;

            genvar gmi_e;
            for (gmi_e = 0; gmi_e < N_MASTERS; gmi_e = gmi_e + 1) begin : g_mst_e_filt
                // slave_id sits in the upper SLV_ID_W bits of the extended sink
                assign e_vf[gmi_e] = mst_e_valid_i[gmi_e] &
                    (mst_e_sink_i[gmi_e*M_SINK_W + SINK_W +: SLV_ID_W] == gsi_e[SLV_ID_W-1:0]);

                // Strip slave_id; only forward orig_sink to the slave
                assign e_din[gmi_e*E_PACKED_W +: E_PACKED_W] =
                    mst_e_sink_i[gmi_e*M_SINK_W +: SINK_W];

                assign e_arb_rdy_bus[gsi_e*N_MASTERS + gmi_e] = e_rdy[gmi_e];
            end

            tl_arbiter #(.N(N_MASTERS), .DATA_W(E_PACKED_W)) arb_e (
                .clk    (clk),          .rst_n  (rst_n),
                .valid_i(e_vf),         .ready_o(e_rdy),
                .data_i (e_din),
                .valid_o(slv_e_valid_o[gsi_e]),
                .ready_i(slv_e_ready_i[gsi_e]),
                .data_o (e_dout)
            );

            assign slv_e_sink_o[gsi_e*SINK_W +: SINK_W] = e_dout;
        end
    endgenerate

    genvar gmi_er;
    generate
        for (gmi_er = 0; gmi_er < N_MASTERS; gmi_er = gmi_er + 1) begin : g_mst_e_rdy
            wire [N_SLAVES-1:0] bits;
            genvar gsi_er;
            for (gsi_er = 0; gsi_er < N_SLAVES; gsi_er = gsi_er + 1) begin : g_er_bits
                assign bits[gsi_er] = e_arb_rdy_bus[gsi_er*N_MASTERS + gmi_er];
            end
            assign mst_e_ready_o[gmi_er] = |bits;
        end
    endgenerate

    // ================================================================
    // CHANNEL B — per-master arbiter (N_SLAVES inputs each)
    // Slave drives slv_b_dest_i to indicate target master.
    // ================================================================
    genvar gmi_b;
    generate
        for (gmi_b = 0; gmi_b < N_MASTERS; gmi_b = gmi_b + 1) begin : g_mst_b

            wire [N_SLAVES-1:0]            b_vf;
            wire [N_SLAVES*B_PACKED_W-1:0] b_din;
            wire [N_SLAVES-1:0]            b_rdy;
            wire [B_PACKED_W-1:0]          b_dout;

            genvar gsi_b;
            for (gsi_b = 0; gsi_b < N_SLAVES; gsi_b = gsi_b + 1) begin : g_slv_b_filt
                assign b_vf[gsi_b] = slv_b_valid_i[gsi_b] &
                    (slv_b_dest_i[gsi_b*MST_ID_W +: MST_ID_W] == gmi_b[MST_ID_W-1:0]);

                assign b_din[gsi_b*B_PACKED_W +: B_PACKED_W] = {
                    slv_b_opcode_i [gsi_b*3       +: 3],
                    slv_b_param_i  [gsi_b*3       +: 3],
                    slv_b_size_i   [gsi_b*4       +: 4],
                    slv_b_source_i [gsi_b*SOURCE_W +: SOURCE_W],
                    slv_b_address_i[gsi_b*ADDR_W  +: ADDR_W],
                    slv_b_mask_i   [gsi_b*8       +: 8],
                    slv_b_data_i   [gsi_b*DATA_W  +: DATA_W],
                    slv_b_corrupt_i[gsi_b]
                };

                assign b_arb_rdy_bus[gmi_b*N_SLAVES + gsi_b] = b_rdy[gsi_b];
            end

            tl_arbiter #(.N(N_SLAVES), .DATA_W(B_PACKED_W)) arb_b (
                .clk    (clk),          .rst_n  (rst_n),
                .valid_i(b_vf),         .ready_o(b_rdy),
                .data_i (b_din),
                .valid_o(mst_b_valid_o[gmi_b]),
                .ready_i(mst_b_ready_i[gmi_b]),
                .data_o (b_dout)
            );

            assign {
                mst_b_opcode_o [gmi_b*3       +: 3],
                mst_b_param_o  [gmi_b*3       +: 3],
                mst_b_size_o   [gmi_b*4       +: 4],
                mst_b_source_o [gmi_b*SOURCE_W +: SOURCE_W],
                mst_b_address_o[gmi_b*ADDR_W  +: ADDR_W],
                mst_b_mask_o   [gmi_b*8       +: 8],
                mst_b_data_o   [gmi_b*DATA_W  +: DATA_W],
                mst_b_corrupt_o[gmi_b]
            } = b_dout;
        end
    endgenerate

    // Collect slv_b_ready_o: OR over all master arbiters
    genvar gsi_br;
    generate
        for (gsi_br = 0; gsi_br < N_SLAVES; gsi_br = gsi_br + 1) begin : g_slv_b_rdy
            wire [N_MASTERS-1:0] bits;
            genvar gmi_br;
            for (gmi_br = 0; gmi_br < N_MASTERS; gmi_br = gmi_br + 1) begin : g_br_bits
                assign bits[gmi_br] = b_arb_rdy_bus[gmi_br*N_SLAVES + gsi_br];
            end
            assign slv_b_ready_o[gsi_br] = |bits;
        end
    endgenerate

    // ================================================================
    // CHANNEL D — per-master arbiter (N_SLAVES inputs each)
    // master_id is extracted from upper bits of extended source.
    // slave_id is prepended to orig_sink to form extended sink for E.
    // ================================================================
    genvar gmi_d;
    generate
        for (gmi_d = 0; gmi_d < N_MASTERS; gmi_d = gmi_d + 1) begin : g_mst_d

            wire [N_SLAVES-1:0]            d_vf;
            wire [N_SLAVES*D_PACKED_W-1:0] d_din;
            wire [N_SLAVES-1:0]            d_rdy;
            wire [D_PACKED_W-1:0]          d_dout;

            genvar gsi_d;
            for (gsi_d = 0; gsi_d < N_SLAVES; gsi_d = gsi_d + 1) begin : g_slv_d_filt
                // master_id lives in upper MST_ID_W bits of extended source
                assign d_vf[gsi_d] = slv_d_valid_i[gsi_d] &
                    (slv_d_source_i[gsi_d*M_SOURCE_W + SOURCE_W +: MST_ID_W] == gmi_d[MST_ID_W-1:0]);

                // Pack: orig_source from lower SOURCE_W; build ext_sink = {slave_id, orig_sink}
                assign d_din[gsi_d*D_PACKED_W +: D_PACKED_W] = {
                    slv_d_opcode_i [gsi_d*3         +: 3],
                    slv_d_param_i  [gsi_d*2         +: 2],
                    slv_d_size_i   [gsi_d*4         +: 4],
                    slv_d_source_i [gsi_d*M_SOURCE_W +: SOURCE_W],  // orig_source (lower bits)
                    gsi_d[SLV_ID_W-1:0],                             // slave_id prefix for E routing
                    slv_d_sink_i   [gsi_d*SINK_W    +: SINK_W],     // orig_sink from slave
                    slv_d_denied_i [gsi_d],
                    slv_d_data_i   [gsi_d*DATA_W    +: DATA_W],
                    slv_d_corrupt_i[gsi_d]
                };

                assign d_arb_rdy_bus[gmi_d*N_SLAVES + gsi_d] = d_rdy[gsi_d];
            end

            tl_arbiter #(.N(N_SLAVES), .DATA_W(D_PACKED_W)) arb_d (
                .clk    (clk),          .rst_n  (rst_n),
                .valid_i(d_vf),         .ready_o(d_rdy),
                .data_i (d_din),
                .valid_o(mst_d_valid_o[gmi_d]),
                .ready_i(mst_d_ready_i[gmi_d]),
                .data_o (d_dout)
            );

            // ext_sink = {slave_id, orig_sink} flows to master as M_SINK_W-bit sink
            assign {
                mst_d_opcode_o [gmi_d*3         +: 3],
                mst_d_param_o  [gmi_d*2         +: 2],
                mst_d_size_o   [gmi_d*4         +: 4],
                mst_d_source_o [gmi_d*SOURCE_W  +: SOURCE_W],
                mst_d_sink_o   [gmi_d*M_SINK_W  +: M_SINK_W],
                mst_d_denied_o [gmi_d],
                mst_d_data_o   [gmi_d*DATA_W    +: DATA_W],
                mst_d_corrupt_o[gmi_d]
            } = d_dout;
        end
    endgenerate

    // Collect slv_d_ready_o: OR over all master arbiters
    genvar gsi_dr;
    generate
        for (gsi_dr = 0; gsi_dr < N_SLAVES; gsi_dr = gsi_dr + 1) begin : g_slv_d_rdy
            wire [N_MASTERS-1:0] bits;
            genvar gmi_dr;
            for (gmi_dr = 0; gmi_dr < N_MASTERS; gmi_dr = gmi_dr + 1) begin : g_dr_bits
                assign bits[gmi_dr] = d_arb_rdy_bus[gmi_dr*N_SLAVES + gsi_dr];
            end
            assign slv_d_ready_o[gsi_dr] = |bits;
        end
    endgenerate

endmodule
