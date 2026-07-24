`timescale 1ns/1ps
`include "params.vh"

/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */

module rv64g_l2_fsm #(
    parameter CORES = 4,
    parameter WAYS = 16,
    parameter SETS = 256,
    parameter SOURCE_W = 6,
    parameter CID_W = 2
) (
    input wire clk_i,
    input wire rst_ni,

    // TileLink A Channel (Sink)
    input  wire [2:0]   a_opcode_i,
    input  wire [2:0]   a_param_i,
    input  wire [SOURCE_W-1:0]   a_source_i,
    input  wire [63:0]  a_address_i,
    input  wire         a_valid_i,
    output reg          a_ready_o,

    // TileLink B Channel (Source) - Probes
    output reg  [2:0]   b_opcode_o,
    output reg  [2:0]   b_param_o,
    output reg  [63:0]  b_address_o,
    output reg          b_valid_o,
    input  wire         b_ready_i,
    output reg  [CID_W-1:0] b_dest_o,

    // TileLink C Channel (Sink) - Release/ProbeAck
    input  wire [2:0]   c_opcode_i,
    input  wire [2:0]   c_param_i,
    input  wire [SOURCE_W-1:0]   c_source_i, // L1 Source ID + Core ID
    input  wire [63:0]  c_address_i,
    input  wire [63:0]  c_data_i,
    input  wire         c_valid_i,
    output reg          c_ready_o,

    // TileLink D Channel (Source) - Grants
    output reg  [2:0]   d_opcode_o,
    output reg  [1:0]   d_param_o,
    output reg  [63:0]  d_data_o,
    output reg  [SOURCE_W-1:0]   d_source_o,
    output reg  [1:0]   d_sink_o,
    output reg          d_valid_o,
    input  wire         d_ready_i,

    // TileLink E Channel (Sink) - GrantAck
    input  wire         e_valid_i,
    input  wire [1:0]   e_sink_i,
    output reg          e_ready_o,

    // Memory Interface (TL-UH)
    // A Channel (Source)
    output reg  [2:0]   mem_a_opcode_o,
    output reg  [2:0]   mem_a_param_o,
    output reg  [2:0]   mem_a_size_o,
    output reg  [3:0]   mem_a_source_o,
    output reg  [63:0]  mem_a_address_o,
    output reg  [7:0]   mem_a_mask_o,
    output reg  [63:0]  mem_a_data_o,
    output reg          mem_a_valid_o,
    input  wire         mem_a_ready_i,

    // D Channel (Sink)
    input  wire [2:0]   mem_d_opcode_i,
    input  wire [1:0]   mem_d_param_i,
    input  wire [2:0]   mem_d_size_i,
    input  wire [3:0]   mem_d_source_i,
    input  wire [1:0]   mem_d_sink_i,
    input  wire         mem_d_denied_i,
    input  wire [63:0]  mem_d_data_i,
    input  wire         mem_d_corrupt_i,
    input  wire         mem_d_valid_i,
    output reg          mem_d_ready_o,

    // Directory Read Interface
    output reg  [7:0]   dir_rd_set_o,
    input  wire [WAYS-1:0]          dir_rd_valid_i,
    input  wire [WAYS*CORES-1:0]    dir_rd_sharers_i,
    input  wire [WAYS-1:0]          dir_rd_owner_valid_i,
    input  wire [WAYS*CID_W-1:0] dir_rd_owner_id_i,
    input  wire [WAYS-1:0]          dir_rd_dirty_i,

    // Directory Write Interface
    output reg          dir_we_o,
    output reg  [7:0]   dir_wr_set_o,
    output reg  [3:0]   dir_wr_way_o,
    output reg          dir_wr_valid_o,
    output reg  [CORES-1:0] dir_wr_sharers_o,
    output reg          dir_wr_owner_valid_o,
    output reg  [CID_W-1:0] dir_wr_owner_id_o,
    output reg          dir_wr_dirty_o,

    // Tag Array Interface (Read/Write)
    input  wire [WAYS*50-1:0] tag_way_flat_i,
    output reg          tag_we_o,
    output reg  [7:0]   tag_set_o,
    output reg  [3:0]   tag_way_o,
    output reg  [49:0]  tag_wdata_o,

    // Data Array Interface (Read/Write)
    output reg  [7:0]   data_set_o,
    output reg  [3:0]   data_way_o,
    output reg  [2:0]   data_word_sel_o, // Added for Burst
    output reg          data_we_o,
    output reg  [63:0]  data_wdata_o,
    input  wire [63:0]  data_rdata_i,

    // MSHR Interface
    output reg          mshr_alloc_o,
    output reg          mshr_dealloc_o,
    output reg  [CORES-1:0] mshr_set_probes_o,
    output reg          mshr_probe_ack_o,
    output reg  [CID_W-1:0] mshr_probe_ack_id_o,
    input  wire [CORES-1:0] mshr_pending_probes_i,
    input  wire         mshr_busy_i
);

    // FSM States
    localparam ST_IDLE       = 4'd0;
    localparam ST_RAM_WAIT   = 4'd1; // Wait for RAM read
    localparam ST_CHECK      = 4'd2; // Check Hit/Miss, Send Probes
    localparam ST_WAIT_ACK   = 4'd3; // Wait for Probe Acks
    localparam ST_GRANT      = 4'd4; // Send Grant
    localparam ST_UPDATE     = 4'd5; // Write Directory
    localparam ST_COMPLETE   = 4'd6; // Done
    localparam ST_EVICT_WAIT = 4'd7; // Wait for Eviction Probes
    localparam ST_MEM_READ   = 4'd8; // Read from Memory (Refill)
    localparam ST_MEM_WRITE  = 4'd9; // Write to Memory (Writeback)
    localparam ST_MEM_RESP   = 4'd10; // Wait for Writeback Ack
    localparam ST_REL_DATA   = 4'd11; // Receive ReleaseData beats
    localparam ST_WAIT_E     = 4'd12; // Wait for GrantAck

reg [3:0] next_state, state_q;
    reg [2:0] burst_cnt; // Burst Counter for Memory Access
    reg [2:0] probe_data_cnt; // Counter for ProbeAckData beats
reg mem_req_sent_q; // Flag to track if Memory Request (Get) was sent
reg e_seen_q, e_seen_n;
reg [CID_W-1:0] probe_ack_id;
    reg [2:0] rel_data_cnt_q, rel_data_cnt_n;
    reg rel_has_data_q, rel_has_data_n;
    reg rel_buf_valid_q, rel_buf_valid_n;
    reg [63:0] rel_buf_data_q, rel_buf_data_n;
    reg rel_drop_q, rel_drop_n;

    // Latched Request
    reg [63:0] req_addr_q;
    reg [63:0] req_data_q; // Added for ReleaseData
    reg [2:0]  req_opcode_q;
    reg [2:0]  req_param_q;
    reg [SOURCE_W-1:0]  req_source_q;

wire [CID_W-1:0] req_core_id = req_source_q[SOURCE_W-1 -: CID_W];
wire [CID_W-1:0] c_core_id = c_source_i[SOURCE_W-1 -: CID_W];

    // Hit Logic
    reg hit;
    reg [3:0] hit_way;
    reg [CORES-1:0] hit_sharers;
    reg hit_owner_valid;
    reg [CID_W-1:0] hit_owner_id;
    reg hit_dirty;
    
    // Latched Hit Info (for ST_WAIT_ACK/ST_UPDATE)
    reg [3:0] latched_hit_way;
    reg       latched_hit;
    reg       processing_release; // Flag to indicate we are handling a Release

    // Tag Comparison
    integer w;
    reg [49:0] current_tag;
    reg [49:0] req_tag;
    
    always @* begin
        req_tag = req_addr_q[63:14]; // Assuming 64B lines, 256 sets -> 6+8=14 bits offset+index
        hit = 1'b0;
        hit_way = 4'd0;
        hit_sharers = {CORES{1'b0}};
        hit_owner_valid = 1'b0;
        hit_owner_id = 0;
        hit_dirty = 1'b0;

        for (w = 0; w < WAYS; w = w + 1) begin
            current_tag = tag_way_flat_i[w*50 +: 50];
            if (dir_rd_valid_i[w] && current_tag == req_tag) begin
                hit = 1'b1;
                hit_way = w[3:0];
                hit_sharers = dir_rd_sharers_i[w*CORES +: CORES];
                hit_owner_valid = dir_rd_owner_valid_i[w];
                hit_owner_id = dir_rd_owner_id_i[w*CID_W +: CID_W];
                hit_dirty = dir_rd_dirty_i[w];
            end
        end
    end
    
    // Latch Hit Info
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            latched_hit_way <= 4'd0;
            latched_hit <= 1'b0;
        end else if (state_q == ST_CHECK) begin
            latched_hit_way <= hit_way;
            latched_hit <= hit;
        end
    end

    // PLRU Instance
    wire [3:0] plru_victim_way;
    reg plru_access;
    reg [3:0] plru_used_way;
    
    rv64g_l2_plru plru (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        .set_i(req_addr_q[13:6]),
        .access_i(plru_access),
        .used_way_i(plru_used_way),
        .valid_i(dir_rd_valid_i), // Use current valid bits
        .victim_o(plru_victim_way)
    );

    // Victim Info
    reg [3:0] victim_way_q;
    reg victim_valid_q;
    reg [CORES-1:0] victim_sharers_q;
    reg victim_owner_valid_q;
    reg [CID_W-1:0] victim_owner_id_q;
    reg victim_dirty_q;
    reg [49:0] victim_tag_q;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            victim_way_q <= 4'd0;
            victim_valid_q <= 1'b0;
            victim_sharers_q <= 0;
            victim_owner_valid_q <= 0;
            victim_owner_id_q <= 0;
            victim_dirty_q <= 0;
            victim_tag_q <= 0;
        end else if (state_q == ST_CHECK && !hit) begin
            victim_way_q <= plru_victim_way;
            victim_valid_q <= dir_rd_valid_i[plru_victim_way];
            victim_sharers_q <= dir_rd_sharers_i[plru_victim_way*CORES +: CORES];
            victim_owner_valid_q <= dir_rd_owner_valid_i[plru_victim_way];
            victim_owner_id_q <= dir_rd_owner_id_i[plru_victim_way*CID_W +: CID_W];
            victim_dirty_q <= dir_rd_dirty_i[plru_victim_way];
            victim_tag_q <= tag_way_flat_i[plru_victim_way*50 +: 50];
        end else if (c_valid_i && c_opcode_i == C_PROBE_ACK_DATA && !latched_hit) begin
            victim_dirty_q <= 1'b1;
        end
    end

    // Probe Logic
    reg [CORES-1:0] probes_to_send;
    reg [CORES-1:0] probes_sent_q;
    
    // Calculate who to probe
    // If AcquireBlock (Read): Probe Owner (to get data/downgrade)
    // If AcquirePerm (Write): Probe Sharers + Owner (to invalidate)
    always @* begin
        probes_to_send = {CORES{1'b0}};
        if (hit) begin
            if (req_opcode_q == 3'd6) begin // AcquireBlock
                if (req_param_q == 3'd0) begin // NtoB (Read Shared)
                    // If owned, probe owner (to downgrade/flush)
                    if (hit_owner_valid) begin
                        probes_to_send[hit_owner_id] = 1'b1;
                    end
                end else begin // NtoT (Read Exclusive) or BtoT (Upgrade)
                    // Probe all sharers and owner (to invalidate)
                    probes_to_send = hit_sharers;
                    if (hit_owner_valid) begin
                        probes_to_send[hit_owner_id] = 1'b1;
                    end
                end
            end else if (req_opcode_q == 3'd7) begin // AcquirePerm
                // Probe all sharers and owner
                probes_to_send = hit_sharers;
                if (hit_owner_valid) begin
                    probes_to_send[hit_owner_id] = 1'b1;
                end
            end
            // Exclude requester
            probes_to_send[req_core_id] = 1'b0;
        end else begin
            // Miss - Check Eviction
            // Use PLRU victim
            if (dir_rd_valid_i[plru_victim_way]) begin
                // Evict: Probe all sharers and owner
                probes_to_send = dir_rd_sharers_i[plru_victim_way*CORES +: CORES];
                if (dir_rd_owner_valid_i[plru_victim_way]) begin
                    probes_to_send[dir_rd_owner_id_i[plru_victim_way*CID_W +: CID_W]] = 1'b1;
                end
            end
        end
    end

    // Helper to find next probe target
    reg [1:0] next_probe_target;
    reg       probe_needed;
    integer i;
    always @* begin
        next_probe_target = 0;
        probe_needed = 0;
        for (i = 0; i < CORES; i = i + 1) begin
            if (probes_to_send[i] && !probes_sent_q[i]) begin
                next_probe_target = i[1:0];
                probe_needed = 1;
            end
        end
    end



    // FSM Next State Logic (Moved to end)


    // C-Channel Handling
    // We need to arbitrate between FSM (Acquire) and C-Channel (Release/ProbeAck)
    // Priority:
    // 1. ProbeAck (Critical for FSM progress)
    // 2. Release (Important to free resources)
    // 3. Acquire (FSM)
    
    // However, FSM is stateful. C-Channel is transaction based.
    // If FSM is in ST_WAIT_ACK, it is waiting for ProbeAck.
    // ProbeAck comes on C.
    
    // Let's use C-Channel opcodes from params.vh

    reg c_handled;
    reg c_is_probe_ack;
    reg c_is_release;
    
    always @* begin
        c_is_probe_ack = (c_opcode_i == C_PROBE_ACK || c_opcode_i == C_PROBE_ACK_DATA);
        c_is_release   = (c_opcode_i == C_RELEASE   || c_opcode_i == C_RELEASE_DATA);
    end

    always @* begin
        next_state = state_q;
        a_ready_o = 1'b0;
        b_valid_o = 1'b0;
        b_opcode_o = 3'd6; // ProbeBlock
        b_param_o = 3'd0; // To N
        b_address_o = req_addr_q;
        b_dest_o = 0;
        d_valid_o = 1'b0;
        d_opcode_o = 3'd0;
        d_param_o = 2'd0;
        d_data_o = 64'd0;
        d_source_o = req_source_q;
        d_sink_o = 2'd0;
        
        dir_rd_set_o = req_addr_q[13:6];
        dir_we_o = 1'b0;
        dir_wr_set_o = req_addr_q[13:6];
        dir_wr_way_o = (latched_hit) ? latched_hit_way : victim_way_q;
        dir_wr_valid_o = 1'b0;
        dir_wr_sharers_o = {CORES{1'b0}};
        dir_wr_owner_valid_o = 1'b0;
        dir_wr_owner_id_o = 0;
        dir_wr_dirty_o = 1'b0;

        tag_we_o = 1'b0;
        tag_set_o = req_addr_q[13:6];
        tag_way_o = (latched_hit) ? latched_hit_way : victim_way_q;
        tag_wdata_o = req_addr_q[63:14];

        data_set_o = req_addr_q[13:6];
        data_way_o = (latched_hit) ? latched_hit_way : victim_way_q;
        data_word_sel_o = 3'd0;
        data_we_o = 1'b0;
        data_wdata_o = 64'd0;

        mshr_alloc_o = 1'b0;
        mshr_dealloc_o = 1'b0;
        mshr_set_probes_o = {CORES{1'b0}};
        mshr_probe_ack_o = 1'b0;
        mshr_probe_ack_id_o = 0;
        probe_ack_id = {CID_W{1'b0}};

        rel_data_cnt_n = rel_data_cnt_q;
        rel_has_data_n = rel_has_data_q;
        rel_buf_valid_n = rel_buf_valid_q;
        rel_buf_data_n = rel_buf_data_q;
        rel_drop_n = rel_drop_q;
        
        c_ready_o = 1'b0;
        e_ready_o = 1'b1;
        e_seen_n = e_seen_q;
        if (e_valid_i) begin
            e_seen_n = 1'b1;
        end
        
        plru_access = 1'b0;
        plru_used_way = 4'd0;

        // Memory Interface Defaults
        mem_a_opcode_o = 3'd0;
        mem_a_param_o = 3'd0;
        mem_a_size_o = 3'd6; // 64 Bytes
        mem_a_source_o = 4'd0;
        mem_a_address_o = 64'd0;
        mem_a_mask_o = 8'hFF;
        mem_a_data_o = 64'd0;
        mem_a_valid_o = 1'b0;
        mem_d_ready_o = 1'b0;

        // C-Channel Processing (High Priority for ProbeAck)
        if (c_valid_i) begin
            if (c_is_probe_ack) begin
                // Always accept ProbeAck
                c_ready_o = 1'b1;
                probe_ack_id = c_core_id;
                mshr_probe_ack_id_o = probe_ack_id;

                if (mshr_pending_probes_i[probe_ack_id]) begin
                    if (c_opcode_i == C_PROBE_ACK_DATA) begin
                        // Write Data
                        data_we_o = 1'b1;
                        data_way_o = (latched_hit) ? latched_hit_way : victim_way_q;
                        data_set_o = req_addr_q[13:6];
                        data_word_sel_o = probe_data_cnt; 
                        data_wdata_o = c_data_i;

                        // Only signal MSHR on last beat
                        if (probe_data_cnt == 3'd7) begin
                            mshr_probe_ack_o = 1'b1;
                        end else begin
                            mshr_probe_ack_o = 1'b0;
                        end
                    end else begin
                        mshr_probe_ack_o = 1'b1;
                    end
                end else begin
                    // Unexpected ProbeAck; ignore for state updates
                    mshr_probe_ack_o = 1'b0;
                end
            end else if (c_is_release) begin
                // Handle Release
                // Needs Directory Write access.
                // Only process if FSM is not using Directory Write.
                // FSM uses Dir Write in ST_UPDATE.
                if (state_q != ST_UPDATE && state_q != ST_RAM_WAIT && state_q != ST_CHECK) begin
                    // We can process Release
                    // But we need to Read Directory first to find the way?
                    // Or does Release carry enough info?
                    // Release has Address. We need to lookup Way.
                    // This implies a Read-Modify-Write on Directory.
                    // This is complex to do in one cycle if we don't know the way.
                    // We need a Release FSM?
                    // Or we steal cycles.
                    
                    // For now, let's assume we can't handle Release in one cycle without knowing the way.
                    // We need to:
                    // 1. Read Directory (using c_address_i)
                    // 2. Find Way
                    // 3. Write Directory (clear sharer/owner)
                    // 4. Write Data (if ReleaseData)
                    // 5. Send ReleaseAck
                    
                    // This requires a state machine.
                    // Since we are single-threaded (blocking), we can only handle Release if FSM is IDLE?
                    // Or we can interrupt?
                    // If FSM is IDLE, we can use the main FSM states?
                    // Let's add ST_RELEASE states.
                end
            end
        end

        case (state_q)
            ST_IDLE: begin
                if (c_valid_i && c_is_release) begin
                    c_ready_o = 1'b1; // Accept Release (beat 0)
                    a_ready_o = 1'b0; // Block A handshake on same cycle
                    next_state = ST_RAM_WAIT;
                    rel_has_data_n = (c_opcode_i == C_RELEASE_DATA);
                    rel_buf_valid_n = (c_opcode_i == C_RELEASE_DATA);
                    rel_buf_data_n = c_data_i;
                    rel_data_cnt_n = 3'd0;
                    rel_drop_n = 1'b0;
                end else begin
                    a_ready_o = !mshr_busy_i;
                    if (a_valid_i && !mshr_busy_i) begin
                        mshr_alloc_o = 1'b1;
                        next_state = ST_RAM_WAIT;
                    end
                end
            end

            ST_RAM_WAIT: begin
                // Just wait for RAM access
                next_state = ST_CHECK;
            end

            ST_CHECK: begin
                if (processing_release) begin
                    // Release Handling
                    if (hit) begin
                        if (rel_has_data_q) begin
                            next_state = ST_REL_DATA;
                            rel_drop_n = 1'b0;
                        end else begin
                            next_state = ST_UPDATE;
                        end
                    end else begin
                        // Miss on Release? Drain data if present, then Ack.
                        if (rel_has_data_q) begin
                            next_state = ST_REL_DATA;
                            rel_drop_n = 1'b1;
                        end else begin
                            next_state = ST_GRANT; 
                        end
                    end
                end else begin
                    // Acquire Handling
                    // Check if we need to send probes (Hit or Eviction)
                    if (probes_to_send != 0) begin
                        if (probes_sent_q == 0) begin
                            mshr_set_probes_o = probes_to_send;
                        end
                        
                        if (probe_needed) begin
                            b_valid_o = 1'b1;
                            b_dest_o = next_probe_target;
                            
                            if (hit) begin
                                b_address_o = req_addr_q;
                                if (req_opcode_q == 3'd7) begin // AcquirePerm (Upgrade/Write)
                                    b_opcode_o = 3'd7; // B_PROBE_PERM
                                    b_param_o = 3'd1; // TtoN (Invalidate) - FIXED (Was 3'd0 TtoB)
                                end else begin // AcquireBlock
                                    b_opcode_o = 3'd6; // B_PROBE
                                    b_param_o = (req_param_q == 3'd0) ? 3'd0 : 3'd1; // NtoB->TtoB (downgrade), NtoT->TtoN (invalidate)
                                end
                            end else begin
                                // Eviction
                                b_address_o = {tag_way_flat_i[plru_victim_way*50 +: 50], req_addr_q[13:6], 6'b0};
                                b_opcode_o = 3'd6; // B_PROBE (Invalidate)
                                b_param_o = 3'd2; // To N (Invalidate)
                            end
                        end
                        
                        if (b_valid_o && b_ready_i) begin
                            if (probes_to_send == (probes_sent_q | (1 << next_probe_target))) begin
                                next_state = (hit) ? ST_WAIT_ACK : ST_EVICT_WAIT;
                            end
                        end
                    end else begin
                        // No probes needed
                        if (!hit) begin
                             // Miss -> Go to Memory Read
                             next_state = ST_MEM_READ;
                        end else begin
                             next_state = ST_GRANT;
                        end
                    end
                end
            end

            ST_WAIT_ACK: begin
                if (mshr_pending_probes_i == 0) begin
                    next_state = ST_GRANT;
                end
            end

            ST_EVICT_WAIT: begin
                if (mshr_pending_probes_i == 0) begin
                    // All probes acked. Victim is now Invalid (in L1s).
                    if (victim_dirty_q) begin
                        next_state = ST_MEM_WRITE;
                    end else begin
                        next_state = ST_MEM_READ;
                    end
                end
            end

            ST_MEM_READ: begin
                // Send Get
                if (!mem_req_sent_q) begin
                    mem_a_valid_o = 1'b1;
                    mem_a_opcode_o = 3'd4; // Get
                    mem_a_address_o = {req_addr_q[63:6], 6'b0}; // Align to line
                end
                
                // Receive Data
                mem_d_ready_o = 1'b1;
                if (mem_d_valid_i) begin
                    data_we_o = 1'b1;
                    data_way_o = victim_way_q;
                    data_set_o = req_addr_q[13:6];
                    data_word_sel_o = burst_cnt;
                    data_wdata_o = mem_d_data_i;
                    
                    if (burst_cnt == 3'd7) begin
                        next_state = ST_GRANT;
                    end
                end
            end

            ST_MEM_WRITE: begin
                // Send PutFullData
                mem_a_valid_o = 1'b1;
                mem_a_opcode_o = 3'd0; // PutFullData
                mem_a_address_o = {victim_tag_q, req_addr_q[13:6], 6'b0}; // Victim Address
                mem_a_data_o = data_rdata_i;
                
                // Read Data from Array
                data_set_o = req_addr_q[13:6];
                data_way_o = victim_way_q;
                tag_way_o = victim_way_q; // Fix: Ensure way_sel is correct for Read
                data_word_sel_o = burst_cnt; // Read current word
                
                if (mem_a_ready_i) begin
                    if (burst_cnt == 3'd7) begin
                        next_state = ST_MEM_RESP;
                    end
                end
            end
            
            ST_MEM_RESP: begin
                mem_d_ready_o = 1'b1;
                if (mem_d_valid_i) begin
                    // AccessAck
                    next_state = ST_MEM_READ; // Proceed to refill
                end
            end

            ST_REL_DATA: begin
                // Drain ReleaseData beats (beat 0 buffered in req_data_q)
                c_ready_o = !rel_buf_valid_q;

                if (rel_buf_valid_q) begin
                    if (!rel_drop_q) begin
                        data_we_o = 1'b1;
                        data_way_o = latched_hit_way;
                        data_set_o = req_addr_q[13:6];
                        data_word_sel_o = 3'd0;
                        data_wdata_o = rel_buf_data_q;
                    end
                    rel_buf_valid_n = 1'b0;
                    rel_data_cnt_n = 3'd1;
                end else if (c_valid_i && c_ready_o) begin
                    if (!rel_drop_q) begin
                        data_we_o = 1'b1;
                        data_way_o = latched_hit_way;
                        data_set_o = req_addr_q[13:6];
                        data_word_sel_o = rel_data_cnt_q;
                        data_wdata_o = c_data_i;
                    end

                    if (rel_data_cnt_q == 3'd7) begin
                        next_state = latched_hit ? ST_UPDATE : ST_GRANT;
                    end else begin
                        rel_data_cnt_n = rel_data_cnt_q + 3'd1;
                    end
                end
            end

            ST_GRANT: begin
                d_valid_o = 1'b1;
                if (processing_release) begin
                    // ReleaseAck
                    d_opcode_o = 3'd6; // ReleaseAck
                    d_param_o = 2'd0;
                    d_source_o = req_source_q;
                    d_data_o = 64'd0;
                    
                    if (d_ready_i) begin
                        next_state = ST_COMPLETE;
                    end
                end else begin
                    d_source_o = req_source_q;
                    
                    if (req_opcode_q == 3'd6) begin // AcquireBlock -> GrantData
                        d_opcode_o = 3'd5; 
                        d_param_o = (req_param_q == 3'd0) ? 2'd1 : 2'd0; // NtoB->toB, NtoT->toT 
                        
                        // Read Data
                        data_set_o = req_addr_q[13:6];
                        data_way_o = (latched_hit) ? latched_hit_way : victim_way_q;
                        tag_way_o = data_way_o; // Fix: Ensure way_sel is correct for Read
                        data_word_sel_o = burst_cnt;
                        d_data_o = data_rdata_i; 
                        
                        if (d_ready_i) begin
                            if (burst_cnt == 3'd7) begin
                                next_state = ST_UPDATE;
                            end
                        end
                    end else begin // AcquirePerm -> Grant
                        d_opcode_o = 3'd4; 
                        d_param_o = 2'd0; // toT (exclusive permission) 
                        d_data_o = 64'd0;
                        
                        if (d_ready_i) begin
                            next_state = ST_UPDATE;
                        end
                    end
                end
            end

            ST_UPDATE: begin
                // Update Directory
                dir_we_o = 1'b1;
                dir_wr_valid_o = 1'b1;
                
                // Update Tag if Miss (New Line)
                if (!latched_hit && !processing_release) begin
                    tag_we_o = 1'b1;
                end
                
                // Update PLRU on Acquire
                if (!processing_release) begin
                    plru_access = 1'b1;
                    plru_used_way = (latched_hit) ? latched_hit_way : victim_way_q;
                end
                
                if (processing_release) begin
                    // Release: Remove sharer
                    dir_wr_sharers_o = hit_sharers & ~(1 << req_core_id);
                    // If owner releasing, clear owner
                    if (hit_owner_valid && hit_owner_id == req_core_id) begin
                        dir_wr_owner_valid_o = 1'b0;
                        dir_wr_owner_id_o = 0;
                        dir_wr_dirty_o = 1'b0; // Clean now (data written if dirty)
                    end else begin
                        dir_wr_owner_valid_o = hit_owner_valid;
                        dir_wr_owner_id_o = hit_owner_id;
                        dir_wr_dirty_o = hit_dirty;
                    end
                    
                    next_state = ST_GRANT; // Send Ack after Update
                end else begin
                    // Acquire (Existing Logic)
                    if (req_opcode_q == 3'd6) begin // AcquireBlock
                        if (req_param_q == 3'd0) begin // NtoB (Read Shared)
                            if (hit_owner_valid) begin
                                // Owner was probed with TtoB (downgraded to shared)
                                // Both old owner and requester become sharers
                                dir_wr_sharers_o = (1 << req_core_id) | (1 << hit_owner_id);
                            end else begin
                                // No owner — add requester to existing sharers
                                dir_wr_sharers_o = hit_sharers | (1 << req_core_id);
                            end
                            dir_wr_owner_valid_o = 1'b0;
                            dir_wr_owner_id_o = {CID_W{1'b0}};
                            dir_wr_dirty_o = 1'b0; // Data written back via ProbeAckData
                        end else begin // NtoT (Read Exclusive) or BtoT (Upgrade)
                            dir_wr_sharers_o = (1 << req_core_id); // Only requester
                            dir_wr_owner_valid_o = 1'b1;
                            dir_wr_owner_id_o = req_core_id;
                            // FIX: NtoT grants exclusive but CLEAN - dirty only set on actual write
                            // AcquireBlock NtoT is read-exclusive, not write
                            dir_wr_dirty_o = 1'b0;
                        end
                    end else begin // AcquirePerm (Write)
                        dir_wr_sharers_o = (1 << req_core_id); // Only requester
                        dir_wr_owner_valid_o = 1'b1;
                        dir_wr_owner_id_o = req_core_id;
                        dir_wr_dirty_o = 1'b1;
                    end
                    next_state = ST_WAIT_E;
                end
            end

            ST_WAIT_E: begin
                // Wait for GrantAck before completing the transaction
                if (e_seen_q || e_valid_i) begin
                    next_state = ST_COMPLETE;
                end
            end

            ST_COMPLETE: begin
                mshr_dealloc_o = 1'b1;
                next_state = ST_IDLE;
            end
            default: begin
                next_state = ST_IDLE;
            end
        endcase
    end

    // FSM Next State Logic
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q <= ST_IDLE;
            req_addr_q <= 64'd0;
            req_opcode_q <= 3'd0;
            req_param_q <= 3'd0;
            req_source_q <= 6'd0;
            probes_sent_q <= {CORES{1'b0}};
            burst_cnt <= 3'd0;
            probe_data_cnt <= 3'd0;
            mem_req_sent_q <= 1'b0;
            e_seen_q <= 1'b0;
            rel_data_cnt_q <= 3'd0;
            rel_has_data_q <= 1'b0;
            rel_buf_valid_q <= 1'b0;
            rel_buf_data_q <= 64'd0;
            rel_drop_q <= 1'b0;
            processing_release <= 1'b0;
        end else begin
            state_q <= next_state;
            e_seen_q <= e_seen_n;
            rel_data_cnt_q <= rel_data_cnt_n;
            rel_has_data_q <= rel_has_data_n;
            rel_buf_valid_q <= rel_buf_valid_n;
            rel_buf_data_q <= rel_buf_data_n;
            rel_drop_q <= rel_drop_n;

            // Probe Data Counter Logic
            if (c_valid_i && c_ready_o && c_opcode_i == C_PROBE_ACK_DATA &&
                mshr_pending_probes_i[c_core_id]) begin
                probe_data_cnt <= probe_data_cnt + 1;
            end else if (state_q == ST_IDLE) begin
                probe_data_cnt <= 3'd0;
            end

            // Burst Counter Logic
            if (state_q == ST_MEM_READ) begin
                if (mem_d_valid_i && mem_d_ready_o) begin
                    burst_cnt <= burst_cnt + 1;
                end
                // Track if Get sent
                if (mem_a_valid_o && mem_a_ready_i) begin
                    mem_req_sent_q <= 1'b1;
                end
            end else if (state_q == ST_MEM_WRITE) begin
                if (mem_a_valid_o && mem_a_ready_i) begin
                    burst_cnt <= burst_cnt + 1;
                end
            end else if (state_q == ST_GRANT) begin
                if (d_valid_o && d_ready_i) begin
                    burst_cnt <= burst_cnt + 1;
                end
            end else begin
                burst_cnt <= 3'd0;
                mem_req_sent_q <= 1'b0;
            end

            // Latch request
            if (state_q == ST_IDLE) begin
                if (c_valid_i && c_is_release) begin
                    req_addr_q <= c_address_i;
                    req_data_q <= c_data_i;
                    req_opcode_q <= c_opcode_i;
                    req_param_q <= c_param_i;
                    req_source_q <= c_source_i;
                    processing_release <= 1'b1;
                    e_seen_q <= 1'b0;
                end else if (a_valid_i && a_ready_o) begin
                    req_addr_q <= a_address_i;
                    req_opcode_q <= a_opcode_i;
                    req_param_q <= a_param_i;
                    req_source_q <= a_source_i;
                    processing_release <= 1'b0;
                    e_seen_q <= 1'b0;
                end
            end

            // Reset probes sent mask when starting check
            if (state_q == ST_RAM_WAIT) begin
                probes_sent_q <= {CORES{1'b0}};
            end else if (state_q == ST_CHECK && b_valid_o && b_ready_i) begin
                 probes_sent_q[next_probe_target] <= 1'b1;
            end
        end
    end

endmodule
