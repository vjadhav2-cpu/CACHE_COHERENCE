`timescale 1ns/1ps

module rv64g_cache_system #(
    parameter CORES = 4,
    parameter ADDR_W = 64,
    parameter DATA_W = 64,
    parameter SOURCE_W = 4, // L1 Source Width
    parameter SINK_W = 4,
    parameter M_SOURCE_W = SOURCE_W + ((CORES > 1) ? $clog2(CORES) : 1) // L2 Source Width
) (
    input wire clk_i,
    input wire rst_ni,

    // CPU Interfaces (Array of ports)
    input  wire [CORES-1:0] cpu_req_i,
    input  wire [CORES-1:0] cpu_we_i,
    input  wire [CORES*8-1:0] cpu_be_i,
    input  wire [CORES*64-1:0] cpu_addr_i,
    input  wire [CORES*64-1:0] cpu_wdata_i,
    // Atomic Inputs
    input  wire [CORES-1:0]    cpu_amo_i,
    input  wire [CORES-1:0]    cpu_lr_i,
    input  wire [CORES-1:0]    cpu_sc_i,
    input  wire [CORES*5-1:0]  cpu_amo_op_i,
    input  wire [CORES-1:0]    cpu_amo_word_i,
    
    output wire [CORES-1:0] cpu_gnt_o,
    output wire [CORES-1:0] cpu_rvalid_o,
    output wire [CORES*64-1:0] cpu_rdata_o,

    // Memory Interface (from L2)
    output wire [2:0]   mem_a_opcode_o,
    output wire [2:0]   mem_a_param_o,
    output wire [2:0]   mem_a_size_o,
    output wire [3:0]   mem_a_source_o,
    output wire [ADDR_W-1:0]  mem_a_address_o,
    output wire [7:0]   mem_a_mask_o,
    output wire [DATA_W-1:0]  mem_a_data_o,
    output wire         mem_a_valid_o,
    input  wire         mem_a_ready_i,

    input  wire [2:0]   mem_d_opcode_i,
    input  wire [1:0]   mem_d_param_i,
    input  wire [2:0]   mem_d_size_i,
    input  wire [3:0]   mem_d_source_i,
    input  wire [1:0]   mem_d_sink_i,
    input  wire         mem_d_denied_i,
    input  wire [DATA_W-1:0]  mem_d_data_i,
    input  wire         mem_d_corrupt_i,
    input  wire         mem_d_valid_i,
    output wire         mem_d_ready_o
);

    // Wires for L1 <-> Xbar
    wire [CORES-1:0]          l1_a_valid;
    wire [CORES-1:0]          l1_a_ready;
    wire [CORES*3-1:0]        l1_a_opcode;
    wire [CORES*3-1:0]        l1_a_param;
    wire [CORES*4-1:0]        l1_a_size;
    wire [CORES*SOURCE_W-1:0] l1_a_source;
    wire [CORES*ADDR_W-1:0]   l1_a_address;
    wire [CORES*8-1:0]        l1_a_mask;
    wire [CORES*DATA_W-1:0]   l1_a_data;
    wire [CORES-1:0]          l1_a_corrupt;

    wire [CORES-1:0]          l1_b_valid;
    wire [CORES-1:0]          l1_b_ready;
    wire [CORES*3-1:0]        l1_b_opcode;
    wire [CORES*3-1:0]        l1_b_param;
    wire [CORES*4-1:0]        l1_b_size;
    wire [CORES*SOURCE_W-1:0] l1_b_source;
    wire [CORES*ADDR_W-1:0]   l1_b_address;
    wire [CORES*8-1:0]        l1_b_mask;
    wire [CORES*DATA_W-1:0]   l1_b_data;
    wire [CORES-1:0]          l1_b_corrupt;

    wire [CORES-1:0]          l1_c_valid;
    wire [CORES-1:0]          l1_c_ready;
    wire [CORES*3-1:0]        l1_c_opcode;
    wire [CORES*3-1:0]        l1_c_param;
    wire [CORES*4-1:0]        l1_c_size;
    wire [CORES*SOURCE_W-1:0] l1_c_source;
    wire [CORES*ADDR_W-1:0]   l1_c_address;
    wire [CORES*DATA_W-1:0]   l1_c_data;
    wire [CORES-1:0]          l1_c_corrupt;

    wire [CORES-1:0]          l1_d_valid;
    wire [CORES-1:0]          l1_d_ready;
    wire [CORES*3-1:0]        l1_d_opcode;
    wire [CORES*3-1:0]        l1_d_param;
    wire [CORES*4-1:0]        l1_d_size;
    wire [CORES*SOURCE_W-1:0] l1_d_source;
    wire [CORES*SINK_W-1:0]   l1_d_sink;
    wire [CORES-1:0]          l1_d_denied;
    wire [CORES*DATA_W-1:0]   l1_d_data;
    wire [CORES-1:0]          l1_d_corrupt;
    
    wire [CORES-1:0]          unused_l1_d_param_msb;
    genvar k;
    generate
        for (k = 0; k < CORES; k = k + 1) begin : gen_unused_l1
            assign unused_l1_d_param_msb[k] = l1_d_param[k*3 + 2];
        end
    endgenerate

    wire [CORES-1:0]          l1_e_valid;
    wire [CORES-1:0]          l1_e_ready;
    wire [CORES*SINK_W-1:0]   l1_e_sink;

    // Wires for Xbar <-> L2
    localparam integer CID_W = (CORES > 1) ? $clog2(CORES) : 1;

    wire          l2_a_valid;
    wire          l2_a_ready;
    wire [2:0]    l2_a_opcode;
    wire [2:0]    l2_a_param;
    wire [M_SOURCE_W-1:0] l2_a_source;
    wire [ADDR_W-1:0]   l2_a_address;

    wire          l2_b_valid;
    wire          l2_b_ready;
    wire [2:0]    l2_b_opcode;
    wire [2:0]    l2_b_param;
    wire [3:0]    l2_b_size;
    wire [SOURCE_W-1:0] l2_b_source;
    wire [ADDR_W-1:0]   l2_b_address;
    wire [7:0]    l2_b_mask;
    wire [DATA_W-1:0]   l2_b_data;
    wire          l2_b_corrupt;
    wire [CID_W-1:0] l2_b_dest;

    wire          l2_c_valid;
    wire          l2_c_ready;
    wire [2:0]    l2_c_opcode;
    wire [2:0]    l2_c_param;
    wire [M_SOURCE_W-1:0] l2_c_source;
    wire [ADDR_W-1:0]   l2_c_address;
    wire [DATA_W-1:0]   l2_c_data;

    wire          l2_d_valid;
    wire          l2_d_ready;
    wire [2:0]    l2_d_opcode;
    wire [1:0]    l2_d_param;
    wire [3:0]    l2_d_size;
    wire [M_SOURCE_W-1:0] l2_d_source;
    wire [1:0]    l2_d_sink;
    wire          l2_d_denied;
    wire [DATA_W-1:0]   l2_d_data;
    wire          l2_d_corrupt;

    wire          l2_e_ready;
    wire          l2_e_valid;
    wire [SINK_W-1:0] l2_e_sink;

    // Unused Manager Interface Signals
    wire [3:0]    unused_l2_a_size;
    wire [7:0]    unused_l2_a_mask;
    wire [DATA_W-1:0] unused_l2_a_data;
    wire          unused_l2_a_corrupt;
    wire [3:0]    unused_l2_c_size;
    wire          unused_l2_c_corrupt;
    // (removed unused E wires)

    genvar i;
    generate
        for (i = 0; i < CORES; i = i + 1) begin : gen_l1
            rv64g_l1_dcache l1 (
                .clk_i(clk_i),
                .rst_ni(rst_ni),
                .invalidate_all_i(1'b0),
                
                .req_i(cpu_req_i[i]),
                .we_i(cpu_we_i[i]),
                .be_i(cpu_be_i[i*8 +: 8]),
                .addr_i(cpu_addr_i[i*64 +: 64]),
                .wdata_i(cpu_wdata_i[i*64 +: 64]),
                .amo_i(cpu_amo_i[i]),
                .lr_i(cpu_lr_i[i]),
                .sc_i(cpu_sc_i[i]),
                .amo_op_i(cpu_amo_op_i[i*5 +: 5]),
                .amo_word_i(cpu_amo_word_i[i]),
                .gnt_o(cpu_gnt_o[i]),
                .rvalid_o(cpu_rvalid_o[i]),
                .rdata_o(cpu_rdata_o[i*64 +: 64]),

                // TileLink Connections (Slice and Connect)
                .tl_a_valid_o(l1_a_valid[i]),
                .tl_a_ready_i(l1_a_ready[i]),
                .tl_a_opcode_o(l1_a_opcode[i*3 +: 3]),
                .tl_a_param_o(l1_a_param[i*3 +: 3]),
                .tl_a_size_o(l1_a_size[i*4 +: 4]),
                .tl_a_source_o(l1_a_source[i*SOURCE_W +: SOURCE_W]),
                .tl_a_address_o(l1_a_address[i*ADDR_W +: ADDR_W]),
                .tl_a_mask_o(l1_a_mask[i*8 +: 8]),
                .tl_a_data_o(l1_a_data[i*DATA_W +: DATA_W]),
                .tl_a_corrupt_o(l1_a_corrupt[i]),

                .tl_b_valid_i(l1_b_valid[i]),
                .tl_b_ready_o(l1_b_ready[i]),
                .tl_b_opcode_i(l1_b_opcode[i*3 +: 3]),
                .tl_b_param_i(l1_b_param[i*3 +: 3]),
                .tl_b_size_i(l1_b_size[i*4 +: 4]),
                .tl_b_source_i(l1_b_source[i*SOURCE_W +: SOURCE_W]),
                .tl_b_address_i(l1_b_address[i*ADDR_W +: ADDR_W]),
                .tl_b_mask_i(l1_b_mask[i*8 +: 8]),
                .tl_b_data_i(l1_b_data[i*DATA_W +: DATA_W]),
                .tl_b_corrupt_i(l1_b_corrupt[i]),

                .tl_c_valid_o(l1_c_valid[i]),
                .tl_c_ready_i(l1_c_ready[i]),
                .tl_c_opcode_o(l1_c_opcode[i*3 +: 3]),
                .tl_c_param_o(l1_c_param[i*3 +: 3]),
                .tl_c_size_o(l1_c_size[i*4 +: 4]),
                .tl_c_source_o(l1_c_source[i*SOURCE_W +: SOURCE_W]),
                .tl_c_address_o(l1_c_address[i*ADDR_W +: ADDR_W]),
                .tl_c_data_o(l1_c_data[i*DATA_W +: DATA_W]),
                .tl_c_corrupt_o(l1_c_corrupt[i]),

                .tl_d_valid_i(l1_d_valid[i]),
                .tl_d_ready_o(l1_d_ready[i]),
                .tl_d_opcode_i(l1_d_opcode[i*3 +: 3]),
                .tl_d_param_i(l1_d_param[i*3 +: 2]),
                .tl_d_size_i(l1_d_size[i*4 +: 4]),
                .tl_d_source_i(l1_d_source[i*SOURCE_W +: SOURCE_W]),
                .tl_d_sink_i(l1_d_sink[i*SINK_W +: SINK_W]),
                .tl_d_denied_i(l1_d_denied[i]),
                .tl_d_data_i(l1_d_data[i*DATA_W +: DATA_W]),
                .tl_d_corrupt_i(l1_d_corrupt[i]),

                .tl_e_valid_o(l1_e_valid[i]),
                .tl_e_ready_i(l1_e_ready[i]),
                .tl_e_sink_o(l1_e_sink[i*SINK_W +: SINK_W]),

                // VLSU ports (tied off - no vector unit connected yet)
                .vlsu_req_i(1'b0),
                .vlsu_lane_valid_i(8'b0),
                .vlsu_lane_we_i(8'b0),
                .vlsu_lane_addr_i(512'b0),
                .vlsu_lane_wdata_i(512'b0),
                .vlsu_lane_be_i(64'b0),
                .vlsu_ready_o(),
                .vlsu_done_o(),
                .vlsu_lane_done_o(),
                .vlsu_lane_hit_o(),
                .vlsu_lane_rdata_o()
            );
        end
    endgenerate

    tl_socket_m1 #(
        .N_CLIENTS(CORES),
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .SOURCE_W(SOURCE_W),
        .SINK_W(SINK_W)
    ) xbar (
        .clk(clk_i),
        .rst_n(rst_ni),
        
        // Client Interfaces
        .cli_a_valid_i(l1_a_valid),
        .cli_a_ready_o(l1_a_ready),
        .cli_a_opcode_i(l1_a_opcode),
        .cli_a_param_i(l1_a_param),
        .cli_a_size_i(l1_a_size),
        .cli_a_source_i(l1_a_source),
        .cli_a_address_i(l1_a_address),
        .cli_a_mask_i(l1_a_mask),
        .cli_a_data_i(l1_a_data),
        .cli_a_corrupt_i(l1_a_corrupt),

        .cli_b_valid_o(l1_b_valid),
        .cli_b_ready_i(l1_b_ready),
        .cli_b_opcode_o(l1_b_opcode),
        .cli_b_param_o(l1_b_param),
        .cli_b_size_o(l1_b_size),
        .cli_b_source_o(l1_b_source),
        .cli_b_address_o(l1_b_address),
        .cli_b_mask_o(l1_b_mask),
        .cli_b_data_o(l1_b_data),
        .cli_b_corrupt_o(l1_b_corrupt),

        .cli_c_valid_i(l1_c_valid),
        .cli_c_ready_o(l1_c_ready),
        .cli_c_opcode_i(l1_c_opcode),
        .cli_c_param_i(l1_c_param),
        .cli_c_size_i(l1_c_size),
        .cli_c_source_i(l1_c_source),
        .cli_c_address_i(l1_c_address),
        .cli_c_data_i(l1_c_data),
        .cli_c_corrupt_i(l1_c_corrupt),

        .cli_d_valid_o(l1_d_valid),
        .cli_d_ready_i(l1_d_ready),
        .cli_d_opcode_o(l1_d_opcode),
        .cli_d_param_o(l1_d_param),
        .cli_d_size_o(l1_d_size),
        .cli_d_source_o(l1_d_source),
        .cli_d_sink_o(l1_d_sink),
        .cli_d_denied_o(l1_d_denied),
        .cli_d_data_o(l1_d_data),
        .cli_d_corrupt_o(l1_d_corrupt),

        .cli_e_valid_i(l1_e_valid),
        .cli_e_ready_o(l1_e_ready),
        .cli_e_sink_i(l1_e_sink),

        // Manager Interface
        .mgr_a_valid_o(l2_a_valid),
        .mgr_a_ready_i(l2_a_ready),
        .mgr_a_opcode_o(l2_a_opcode),
        .mgr_a_param_o(l2_a_param),
        .mgr_a_size_o(unused_l2_a_size),
        .mgr_a_source_o(l2_a_source),
        .mgr_a_address_o(l2_a_address),
        .mgr_a_mask_o(unused_l2_a_mask),
        .mgr_a_data_o(unused_l2_a_data),
        .mgr_a_corrupt_o(unused_l2_a_corrupt),

        .mgr_b_valid_i(l2_b_valid),
        .mgr_b_ready_o(l2_b_ready),
        .mgr_b_opcode_i(l2_b_opcode),
        .mgr_b_param_i(l2_b_param),
        .mgr_b_size_i(l2_b_size),
        .mgr_b_source_i(l2_b_source),
        .mgr_b_address_i(l2_b_address),
        .mgr_b_mask_i(l2_b_mask),
        .mgr_b_data_i(l2_b_data),
        .mgr_b_corrupt_i(l2_b_corrupt),
        .mgr_b_dest_i(l2_b_dest),

        .mgr_c_valid_o(l2_c_valid),
        .mgr_c_ready_i(l2_c_ready),
        .mgr_c_opcode_o(l2_c_opcode),
        .mgr_c_param_o(l2_c_param),
        .mgr_c_size_o(unused_l2_c_size),
        .mgr_c_source_o(l2_c_source),
        .mgr_c_address_o(l2_c_address),
        .mgr_c_data_o(l2_c_data),
        .mgr_c_corrupt_o(unused_l2_c_corrupt),

        .mgr_d_valid_i(l2_d_valid),
        .mgr_d_ready_o(l2_d_ready),
        .mgr_d_opcode_i(l2_d_opcode),
        .mgr_d_param_i({1'b0, l2_d_param}),
        .mgr_d_size_i(l2_d_size),
        .mgr_d_source_i(l2_d_source),
        .mgr_d_sink_i({2'b00, l2_d_sink}),
        .mgr_d_denied_i(l2_d_denied),
        .mgr_d_data_i(l2_d_data),
        .mgr_d_corrupt_i(l2_d_corrupt),

        .mgr_e_valid_o(l2_e_valid),
        .mgr_e_ready_i(l2_e_ready),
        .mgr_e_sink_o(l2_e_sink)
    );

    // L2 Cache
    // Fix missing signals
    assign l2_b_size = 4'd6; // 64 bytes
    assign l2_b_source = 4'd0; // Dummy
    assign l2_b_mask = 8'hFF; // Full mask
    assign l2_b_data = 64'd0; // No data in probes usually
    assign l2_b_corrupt = 1'b0;

    assign l2_d_size = 4'd6; // 64 bytes
    assign l2_d_denied = 1'b0;
    assign l2_d_corrupt = 1'b0;

    rv64g_l2_cache #(
        .CORES(CORES),
        .SOURCE_W(M_SOURCE_W),
        .CID_W(CID_W)
    ) l2 (
        .clk_i(clk_i),
        .rst_ni(rst_ni),

        .tl_a_opcode_i(l2_a_opcode),
        .tl_a_param_i(l2_a_param),
        .tl_a_source_i(l2_a_source),
        .tl_a_address_i(l2_a_address),
        .tl_a_valid_i(l2_a_valid),
        .tl_a_ready_o(l2_a_ready),

        .tl_b_opcode_o(l2_b_opcode),
        .tl_b_param_o(l2_b_param),
        .tl_b_address_o(l2_b_address),
        .tl_b_valid_o(l2_b_valid),
        .tl_b_ready_i(l2_b_ready),
        .tl_b_dest_o(l2_b_dest),

        .tl_c_opcode_i(l2_c_opcode),
        .tl_c_param_i(l2_c_param),
        .tl_c_source_i(l2_c_source),
        .tl_c_address_i(l2_c_address),
        .tl_c_data_i(l2_c_data),
        .tl_c_valid_i(l2_c_valid),
        .tl_c_ready_o(l2_c_ready),

        .tl_d_opcode_o(l2_d_opcode),
        .tl_d_param_o(l2_d_param),
        .tl_d_data_o(l2_d_data),
        .tl_d_source_o(l2_d_source),
        .tl_d_sink_o(l2_d_sink),
        .tl_d_valid_o(l2_d_valid),
        .tl_d_ready_i(l2_d_ready),

        .tl_e_valid_i(l2_e_valid),
        .tl_e_sink_i(l2_e_sink[1:0]),
        .tl_e_ready_o(l2_e_ready),

        .mem_a_opcode_o(mem_a_opcode_o),
        .mem_a_param_o(mem_a_param_o),
        .mem_a_size_o(mem_a_size_o),
        .mem_a_source_o(mem_a_source_o),
        .mem_a_address_o(mem_a_address_o),
        .mem_a_mask_o(mem_a_mask_o),
        .mem_a_data_o(mem_a_data_o),
        .mem_a_valid_o(mem_a_valid_o),
        .mem_a_ready_i(mem_a_ready_i),

        .mem_d_opcode_i(mem_d_opcode_i),
        .mem_d_param_i(mem_d_param_i),
        .mem_d_size_i(mem_d_size_i),
        .mem_d_source_i(mem_d_source_i),
        .mem_d_sink_i(mem_d_sink_i),
        .mem_d_denied_i(mem_d_denied_i),
        .mem_d_data_i(mem_d_data_i),
        .mem_d_corrupt_i(mem_d_corrupt_i),
        .mem_d_valid_i(mem_d_valid_i),
        .mem_d_ready_o(mem_d_ready_o)
    );

endmodule
