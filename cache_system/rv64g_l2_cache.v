`timescale 1ns/1ps

/* verilator lint_off UNUSEDSIGNAL */

module rv64g_l2_cache #(
    parameter CORES = 4,
    parameter WAYS = 16,
    parameter SETS = 256,
    parameter ADDR_W = 64,
    parameter DATA_W = 64,
    parameter SOURCE_W = 6,
    parameter CID_W = 2
) (
    input wire clk_i,
    input wire rst_ni,

    // TileLink A Channel (Sink)
    input  wire [2:0]   tl_a_opcode_i,
    input  wire [2:0]   tl_a_param_i,
    input  wire [SOURCE_W-1:0]   tl_a_source_i,
    input  wire [ADDR_W-1:0]  tl_a_address_i,
    input  wire         tl_a_valid_i,
    output wire         tl_a_ready_o,

    // TileLink B Channel (Source) - Probes
    output wire [2:0]   tl_b_opcode_o,
    output wire [2:0]   tl_b_param_o,
    output wire [ADDR_W-1:0]  tl_b_address_o,
    output wire         tl_b_valid_o,
    input  wire         tl_b_ready_i,
    output wire [CID_W-1:0] tl_b_dest_o, // To Crossbar/Demux

    // TileLink C Channel (Sink) - Release/ProbeAck
    input  wire [2:0]   tl_c_opcode_i,
    input  wire [2:0]   tl_c_param_i,
    input  wire [SOURCE_W-1:0]   tl_c_source_i,
    input  wire [ADDR_W-1:0]  tl_c_address_i,
    input  wire [DATA_W-1:0]  tl_c_data_i,
    input  wire         tl_c_valid_i,
    output wire         tl_c_ready_o,

    // TileLink D Channel (Source) - Grants
    output wire [2:0]   tl_d_opcode_o,
    output wire [1:0]   tl_d_param_o,
    output wire [DATA_W-1:0]  tl_d_data_o,
    output wire [SOURCE_W-1:0]   tl_d_source_o,
    output wire [1:0]   tl_d_sink_o,
    output wire         tl_d_valid_o,
    input  wire         tl_d_ready_i,

    // TileLink E Channel (Sink) - GrantAck
    input  wire         tl_e_valid_i,
    input  wire [1:0]   tl_e_sink_i,
    output wire         tl_e_ready_o,

    // Memory Interface (TL-UH)
    // A Channel (Source)
    output wire [2:0]   mem_a_opcode_o,
    output wire [2:0]   mem_a_param_o,
    output wire [2:0]   mem_a_size_o,
    output wire [3:0]   mem_a_source_o,
    output wire [ADDR_W-1:0]  mem_a_address_o,
    output wire [7:0]   mem_a_mask_o,
    output wire [DATA_W-1:0]  mem_a_data_o,
    output wire         mem_a_valid_o,
    input  wire         mem_a_ready_i,

    // D Channel (Sink)
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

    // Internal Signals
    
    // Directory Signals
    wire [7:0]   dir_rd_set;
    wire [WAYS-1:0]          dir_rd_valid;
    wire [WAYS*CORES-1:0]    dir_rd_sharers;
    wire [WAYS-1:0]          dir_rd_owner_valid;
    wire [WAYS*CID_W-1:0] dir_rd_owner_id;
    wire [WAYS-1:0]          dir_rd_dirty;

    wire          dir_we;
    wire [7:0]    dir_wr_set;
    wire [3:0]    dir_wr_way;
    wire          dir_wr_valid;
    wire [CORES-1:0] dir_wr_sharers;
    wire          dir_wr_owner_valid;
    wire [CID_W-1:0] dir_wr_owner_id;
    wire          dir_wr_dirty;

    // Tag/Data Array Signals
    wire [WAYS*50-1:0] tag_way_flat;
    wire          tag_we;
    wire [7:0]    tag_set;
    wire [3:0]    tag_way;
    wire [49:0]   tag_wdata;
    
    wire [3:0]    data_way;
    wire [2:0]    data_word_sel;
    wire          data_we;
    wire [63:0]   data_wdata;
    wire [63:0]   data_rdata;
    
    // Unused Array/MSHR Outputs
    wire [7:0]          unused_data_set;
    wire [49:0]         unused_tag_selected;
    wire [WAYS*64-1:0]  unused_rdata_way_flat;
    wire                unused_mshr_alloc_ready;
    wire [ADDR_W-1:0]   unused_mshr_addr;
    wire [SOURCE_W-1:0] unused_mshr_source;
    wire [2:0]          unused_mshr_type;

    // MSHR Signals
    wire          mshr_alloc;
    wire          mshr_dealloc;
    wire [CORES-1:0] mshr_set_probes;
    wire          mshr_probe_ack;
    wire [CID_W-1:0] mshr_probe_ack_id;
    wire [CORES-1:0] mshr_pending_probes;
    wire          mshr_busy;
    
    // ---------------------------------------------------------
    // L2 FSM
    // ---------------------------------------------------------
    rv64g_l2_fsm #(
        .CORES(CORES),
        .WAYS(WAYS),
        .SETS(SETS),
        .SOURCE_W(SOURCE_W),
        .CID_W(CID_W)
    ) fsm (
        .clk_i(clk_i),
        .rst_ni(rst_ni),

        // TileLink A
        .a_opcode_i(tl_a_opcode_i),
        .a_param_i(tl_a_param_i),
        .a_source_i(tl_a_source_i),
        .a_address_i(tl_a_address_i),
        .a_valid_i(tl_a_valid_i),
        .a_ready_o(tl_a_ready_o),

        // TileLink B
        .b_opcode_o(tl_b_opcode_o),
        .b_param_o(tl_b_param_o),
        .b_address_o(tl_b_address_o),
        .b_valid_o(tl_b_valid_o),
        .b_ready_i(tl_b_ready_i),
        .b_dest_o(tl_b_dest_o),

        // TileLink C
        .c_opcode_i(tl_c_opcode_i),
        .c_param_i(tl_c_param_i),
        .c_source_i(tl_c_source_i),
        .c_address_i(tl_c_address_i),
        .c_data_i(tl_c_data_i),
        .c_valid_i(tl_c_valid_i),
        .c_ready_o(tl_c_ready_o),

        // TileLink D
        .d_opcode_o(tl_d_opcode_o),
        .d_param_o(tl_d_param_o),
        .d_data_o(tl_d_data_o),
        .d_source_o(tl_d_source_o),
        .d_sink_o(tl_d_sink_o),
        .d_valid_o(tl_d_valid_o),
        .d_ready_i(tl_d_ready_i),

        // TileLink E
        .e_valid_i(tl_e_valid_i),
        .e_sink_i(tl_e_sink_i),
        .e_ready_o(tl_e_ready_o),

        // Memory Interface
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
        .mem_d_ready_o(mem_d_ready_o),

        // Directory Interface
        .dir_rd_set_o(dir_rd_set),
        .dir_rd_valid_i(dir_rd_valid),
        .dir_rd_sharers_i(dir_rd_sharers),
        .dir_rd_owner_valid_i(dir_rd_owner_valid),
        .dir_rd_owner_id_i(dir_rd_owner_id),
        .dir_rd_dirty_i(dir_rd_dirty),

        .dir_we_o(dir_we),
        .dir_wr_set_o(dir_wr_set),
        .dir_wr_way_o(dir_wr_way),
        .dir_wr_valid_o(dir_wr_valid),
        .dir_wr_sharers_o(dir_wr_sharers),
        .dir_wr_owner_valid_o(dir_wr_owner_valid),
        .dir_wr_owner_id_o(dir_wr_owner_id),
        .dir_wr_dirty_o(dir_wr_dirty),

        // Tag/Data Interface
        .tag_way_flat_i(tag_way_flat),
        .tag_we_o(tag_we),
        .tag_set_o(tag_set),
        .tag_way_o(tag_way),
        .tag_wdata_o(tag_wdata),

        .data_set_o(unused_data_set),
        .data_way_o(data_way),
        .data_word_sel_o(data_word_sel),
        .data_we_o(data_we),
        .data_wdata_o(data_wdata),
        .data_rdata_i(data_rdata),

        // MSHR Interface
        .mshr_alloc_o(mshr_alloc),
        .mshr_dealloc_o(mshr_dealloc),
        .mshr_set_probes_o(mshr_set_probes),
        .mshr_probe_ack_o(mshr_probe_ack),
        .mshr_probe_ack_id_o(mshr_probe_ack_id),
        .mshr_pending_probes_i(mshr_pending_probes),
        .mshr_busy_i(mshr_busy)
    );

    // ---------------------------------------------------------
    // L2 Directory
    // ---------------------------------------------------------
    rv64g_l2_directory #(
        .SETS(SETS),
        .WAYS(WAYS),
        .CORES(CORES)
    ) directory (
        .clk(clk_i),
        .rst_n(rst_ni),
        .rd_set_i(dir_rd_set),
        .rd_valid_o(dir_rd_valid),
        .rd_sharers_o(dir_rd_sharers),
        .rd_owner_valid_o(dir_rd_owner_valid),
        .rd_owner_id_o(dir_rd_owner_id),
        .rd_dirty_o(dir_rd_dirty),
        .we_i(dir_we),
        .wr_set_i(dir_wr_set),
        .wr_way_i(dir_wr_way),
        .wr_valid_i(dir_wr_valid),
        .wr_sharers_i(dir_wr_sharers),
        .wr_owner_valid_i(dir_wr_owner_valid),
        .wr_owner_id_i(dir_wr_owner_id),
        .wr_dirty_i(dir_wr_dirty)
    );

    // ---------------------------------------------------------
    // L2 Arrays (Tags + Data)
    // ---------------------------------------------------------
    // Note: Arrays module has combined interface for Tag and Data?
    // Let's check rv64g_l2_arrays.v ports.
    // It has: index_i, word_sel_i, way_sel_i, write_en_i, be_i, tag_in_i, wdata_i
    // It seems it shares address/control for both Tag and Data?
    // But FSM drives them separately: tag_set_o, tag_way_o vs data_set_o, data_way_o.
    // And tag_we_o vs data_we_o.
    
    // If rv64g_l2_arrays.v has shared control, we have a problem if FSM tries to access them differently.
    // FSM usually accesses them together or separately.
    // Let's check rv64g_l2_arrays.v again.
    
    // It has ONE set of controls: index_i, word_sel_i, way_sel_i, write_en_i.
    // This implies simultaneous access or shared port.
    // But FSM has separate outputs.
    // We might need to OR them or Multiplex them?
    // Or maybe rv64g_l2_arrays.v needs to be split or modified?
    // Or we instantiate two copies? No, they are physically different arrays (Tag vs Data).
    // rv64g_l2_arrays.v contains BOTH.
    
    // If FSM asserts tag_we and data_we at same time for same set/way, it works.
    // If FSM asserts tag_we for Set A and data_we for Set B, it fails.
    // FSM logic:
    // ST_UPDATE: tag_we=1 (if miss), dir_we=1.
    // ST_MEM_READ: data_we=1.
    // ST_MEM_WRITE: data read (implicit).
    
    // It seems FSM accesses them in different states usually.
    // But we need to mux the controls.
    
    wire [7:0] array_index;
    wire [2:0] array_word_sel;
    wire [3:0] array_way_sel;
    wire [7:0] array_be; // Always FF?
    
    // Priority Mux for Arrays
    // Data access is more frequent (bursts). Tag write is rare (miss).
    // But Tag Read is needed for lookup?
    // Wait, rv64g_l2_arrays.v provides `tag_way_flat_o` which is ALL tags for ALL ways in the set?
    // No, `tag_way_flat_o` depends on `index_i`.
    // So we need to drive `index_i` with `req_addr` during lookup.
    // FSM drives `tag_set_o` (which comes from req_addr).
    
    // So:
    // If tag_we, use tag_set/tag_way.
    // If data_we, use data_set/data_way.
    // If neither (Read), use... req_addr?
    // FSM drives `tag_set_o` continuously?
    // In ST_IDLE/RAM_WAIT, FSM drives `tag_set_o = req_addr_q[13:6]`.
    // So we can default to tag_set_o.
    
    // Fix UNOPTFLAT: Use tag_set directly as it is always req_addr_q[13:6]
    assign array_index = tag_set; 
    assign array_way_sel = (data_we) ? data_way : tag_way;
    assign array_word_sel = data_word_sel;
    assign array_be = 8'hFF; // Always full word write for now? FSM doesn't output mask for data array write?
    // FSM has `mem_a_mask_o` for Memory Interface, but for internal Data Array?
    // `data_we_o` is 1 bit. `data_wdata_o` is 64 bits.
    // We assume full 64-bit write to array.
    
    rv64g_l2_arrays arrays (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .index_i(array_index),
        .word_sel_i(array_word_sel),
        .way_sel_i(array_way_sel),
        .data_we_i(data_we),
        .tag_we_i(tag_we),
        .be_i(array_be),
        .tag_in_i(tag_wdata),
        .wdata_i(data_wdata),
        .rdata_selected_o(data_rdata),
        .tag_selected_o(unused_tag_selected),
        .rdata_way_flat_o(unused_rdata_way_flat), 
        .tag_way_flat_o(tag_way_flat)
    );

    // ---------------------------------------------------------
    // L2 MSHR
    // ---------------------------------------------------------
    rv64g_l2_mshr #(
        .ADDR_W(ADDR_W),
        .SOURCE_W(SOURCE_W),
        .TYPE_W(3),
        .CORES(CORES),
        .CID_W(CID_W)
    ) mshr (
        .clk(clk_i),
        .rst_n(rst_ni),
        .alloc_req_i(mshr_alloc),
        .alloc_addr_i(tl_a_address_i), // Alloc uses current A address
        .alloc_source_i(tl_a_source_i), // Alloc uses current A source
        .alloc_type_i(tl_a_opcode_i),   // Alloc uses current A opcode
        .alloc_ready_o(unused_mshr_alloc_ready),
        
        .dealloc_req_i(mshr_dealloc),
        .set_probes_i(mshr_set_probes != 0), // Trigger when mask is non-zero?
        // FSM asserts `mshr_set_probes_o` with the mask.
        // MSHR expects `set_probes_i` (strobe) and `probes_mask_i`.
        // FSM logic: `mshr_set_probes_o = probes_to_send`.
        // This is a level signal in ST_CHECK?
        // We need to be careful. MSHR `set_probes_i` loads the mask.
        // FSM should pulse it?
        // In `rv64g_l2_fsm.v`:
        // `mshr_set_probes_o` is driven in `ST_CHECK`.
        // It seems to be a level.
        // MSHR logic:
        // `if (set_probes_i) pending_probes_q <= probes_mask_i;`
        // If it's level, it will keep reloading.
        // But `pending_probes_q` is also cleared by `probe_ack_i`.
        // If we keep reloading, we might overwrite the cleared bits.
        // FSM stays in ST_CHECK until all probes sent?
        // No, FSM goes to ST_WAIT_ACK.
        // In ST_WAIT_ACK, `mshr_set_probes_o` is 0 (default).
        // So it should be fine.
        
        .probes_mask_i(mshr_set_probes),
        .probe_ack_i(mshr_probe_ack),
        .probe_ack_id_i(mshr_probe_ack_id),
        
        .valid_o(mshr_busy),
        .addr_o(unused_mshr_addr),
        .source_o(unused_mshr_source),
        .type_o(unused_mshr_type),
        .pending_probes_o(mshr_pending_probes)
    );

endmodule
