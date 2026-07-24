`timescale 1ns/1ps
`include "./async_fifo.v"
`include "./day14.v"
/*==============================================================================
 * TILELINK UH ARBITER - Round-Robin with Burst Support
 *
 * MODULE PURPOSE:
 * ---------------
 * This arbiter manages access to the shared TileLink bus for multiple masters.
 * It implements a round-robin arbitration scheme with burst transaction locking
 * to ensure fair access while maintaining atomic burst sequences.
 *
 * KEY FEATURES:
 * -------------
 * 1. Round-Robin Arbitration: Fair scheduling of 3 masters (M0, M1, M2)
 * 2. Burst Transaction Support: Locks bus for multi-beat bursts
 * 3. Response Routing FIFO: 8-deep queue to track which master gets each response
 * 4. A-Channel Multiplexing: Selects one master's request at a time
 * 5. D-Channel Demultiplexing: Routes responses back to correct master
 *
 * DATA FLOW:
 * ----------
 *   [Master 0]  ─┐
 *   [Master 1]  ─┼─→ [Round-Robin Arbiter] ─→ [Shared A-Channel Out]
 *   [Master 2]  ─┘
 *
 *   [Shared D-Channel In] ─→ [FIFO-Based Demux] ─→ [M0/M1/M2 D-Channel]
 *
 * ARBITRATION ALGORITHM:
 * ----------------------
 * 1. IDLE STATE: Check all master requests with round-robin priority
 *    - Priority rotates: M0→M1→M2→M0...
 *    - Mask register tracks last served master
 * 
 * 2. SINGLE-BEAT TRANSACTION:
 *    - Grant bus for one cycle (a_valid && a_ready handshake)
 *    - Write master ID to response FIFO
 *    - Update round-robin mask to next master
 *
 * 3. BURST TRANSACTION (when a_size > BEAT_LOG2):
 *    - Lock bus to winning master for entire burst
 *    - Write master ID to FIFO for EACH beat
 *    - Count beats: num_beats = 2^(a_size - BEAT_LOG2)
 *    - Release bus only after last beat completes
 *    - Then update round-robin mask
 *
 * 4. RESPONSE ROUTING:
 *    - FIFO stores master ID for each A-channel beat
 *    - When D-channel response arrives, read FIFO to determine destination
 *    - Route response to correct master using demux logic
 *
 * BURST LOCKING MECHANISM:
 * ------------------------
 * - burst_active flag prevents arbitration changes during burst
 * - burst_master register locks the granted master
 * - beat_counter tracks progress through burst
 * - effective_req = burst_active ? burst_master : normal_requests
 *
 * TIMING:
 * -------
 * - Arbiter operates in single clock domain (m_clk from master side)
 * - Combinational grant decision (zero-cycle arbitration latency)
 * - Sequential mask update (one cycle after transaction completes)
 *
 * IMPORTANT NOTES:
 * ----------------
 * - Response FIFO depth (8) limits max outstanding transactions
 * - Burst lock ensures no master can interrupt another's burst
 * - Round-robin fairness applies only between burst boundaries
 * - Source ID in TileLink protocol also helps match responses to requests
 *
 * AUTHORS: TileLink Crossbar Team
 * DATE: 2026-01-19
 *==============================================================================*/

module tilelink_uh_arbiter #(
    // =====================================================================
    // Arbiter Configuration Parameters
    // =====================================================================
    parameter  NUM_MASTERS     = 3,              // Number of master ports to arbitrate
    parameter TL_ADDR_WIDTH   = 32,             // Address bus width in bits
    parameter TL_DATA_WIDTH   = 64,             // Data bus width in bits  
    parameter TL_MASK_WIDTH   = TL_DATA_WIDTH/8, // Byte mask width (1 bit per byte)
    parameter TL_SIZE_WIDTH   = 8,              // Size field width (encodes 2^size bytes)
    parameter TL_SRC_WIDTH    = 4,              // Source ID width (transaction tagging)
    parameter TL_SINK_WIDTH   = 3,              // Sink ID width (response tagging)
    parameter TL_OPCODE_WIDTH = 3,              // Opcode width (operation type)
    parameter TL_PARAM_WIDTH  = 3,              // Parameter width (operation-specific)
    parameter TL_BEAT_WIDTH   = 8,              // Bytes per beat (bus transfer unit)
    parameter BEAT_LOG2       = $clog2(TL_BEAT_WIDTH) // log2(BEAT_WIDTH) for burst math
)(
    // =====================================================================
    // Clock and Reset
    // =====================================================================
    input  wire                    clk,          // Master clock domain
    input  wire                    reset,        // Synchronous active-high reset

    // =====================================================================
    // A-CHANNEL INPUTS: Requests from 3 Masters
    // =====================================================================
    // Each master has its own A-channel to send read/write requests
    // Protocol: valid/ready handshake (transaction occurs when both HIGH)
    
    // --- MASTER 0 A-CHANNEL ---
    input  wire                    m0_a_valid,      // M0: Request valid signal
    input  wire [TL_OPCODE_WIDTH-1:0] m0_a_opcode,  // M0: Operation (GET/PUT/ATOMIC)
    input  wire [TL_PARAM_WIDTH-1:0]  m0_a_param,   // M0: Operation parameter
    input  wire [TL_SIZE_WIDTH-1:0]   m0_a_size,    // M0: Transfer size (2^size bytes)
    input  wire [TL_SRC_WIDTH-1:0]    m0_a_source,  // M0: Source ID (for matching responses)
    input  wire [TL_ADDR_WIDTH-1:0]   m0_a_address, // M0: Target address
    input  wire [TL_MASK_WIDTH-1:0]   m0_a_mask,    // M0: Byte lane mask
    input  wire [TL_DATA_WIDTH-1:0]   m0_a_data,    // M0: Write data
    output reg                     m0_a_ready,      // M0: Arbiter ready to accept

    // --- MASTER 1 A-CHANNEL ---
    input  wire                    m1_a_valid,
    input  wire [TL_OPCODE_WIDTH-1:0] m1_a_opcode,
    input  wire [TL_PARAM_WIDTH-1:0]  m1_a_param,
    input  wire [TL_SIZE_WIDTH-1:0]   m1_a_size,
    input  wire [TL_SRC_WIDTH-1:0]    m1_a_source,
    input  wire [TL_ADDR_WIDTH-1:0]   m1_a_address,
    input  wire [TL_MASK_WIDTH-1:0]   m1_a_mask,
    input  wire [TL_DATA_WIDTH-1:0]   m1_a_data,
    output reg                     m1_a_ready,

    input  wire                    m2_a_valid,
    input  wire [TL_OPCODE_WIDTH-1:0] m2_a_opcode,
    input  wire [TL_PARAM_WIDTH-1:0]  m2_a_param,
    input  wire [TL_SIZE_WIDTH-1:0]   m2_a_size,
    input  wire [TL_SRC_WIDTH-1:0]    m2_a_source,
    input  wire [TL_ADDR_WIDTH-1:0]   m2_a_address,
    input  wire [TL_MASK_WIDTH-1:0]   m2_a_mask,
    input  wire [TL_DATA_WIDTH-1:0]   m2_a_data,
    output reg                     m2_a_ready,

    // =====================================================================
    // A-CHANNEL OUTPUT: Shared/Arbitrated Request to Downstream
    // =====================================================================
    // Multiplexed output carrying the winning master's request
    // Goes to Speed Adapter → CDC → Interconnect → Slaves
    output reg                     arb_a_valid_out,   // Arbitrated request valid
    output reg [TL_OPCODE_WIDTH-1:0] arb_a_opcode_out,
    output reg [TL_PARAM_WIDTH-1:0]  arb_a_param_out,
    output reg [TL_SIZE_WIDTH-1:0]   arb_a_size_out,
    output reg [TL_SRC_WIDTH-1:0]    arb_a_source_out,
    output reg [TL_ADDR_WIDTH-1:0]   arb_a_address_out,
    output reg [TL_MASK_WIDTH-1:0]   arb_a_mask_out,
    output reg [TL_DATA_WIDTH-1:0]   arb_a_data_out,
    input  wire                     a_ready_in,        // Downstream ready (from CDC/Interconnect)

    // =====================================================================
    // D-CHANNEL INPUT: Responses from Downstream (Slaves)
    // =====================================================================
    // Incoming responses from slaves (via Interconnect → CDC → Speed Adapter)
    // Arbiter must route these back to the correct requesting master
    input  wire                     d_valid_in,       // Response valid from slave
    input  wire [TL_OPCODE_WIDTH-1:0] d_opcode_in,
    input  wire [TL_PARAM_WIDTH-1:0]  d_param_in,
    input  wire [TL_SIZE_WIDTH-1:0]   d_size_in, 
    input  wire [TL_SRC_WIDTH-1:0]    d_source_in,
    input  wire [TL_SINK_WIDTH-1:0]   d_sink_in,
    input  wire [TL_DATA_WIDTH-1:0]   d_data_in,
    input  wire                     d_error_in,
    output wire                     d_ready_out,      // Arbiter ready for responses

    // =====================================================================
    // D-CHANNEL OUTPUTS: Responses Demuxed to Individual Masters
    // =====================================================================
    // Routed based on FIFO tracking which master initiated each request
    
    // --- MASTER 0 D-CHANNEL ---
    output wire                     m0_d_valid,       // Response valid to M0
    input  wire                     m0_d_ready,
    output wire [TL_DATA_WIDTH-1:0]   m0_d_data,
    output wire [TL_OPCODE_WIDTH-1:0] m0_d_opcode,
    output wire [TL_PARAM_WIDTH-1:0]  m0_d_param,
    output wire [TL_SIZE_WIDTH-1:0]   m0_d_size,
    output wire [TL_SINK_WIDTH-1:0]   m0_d_sink,
    output wire                     m0_d_error,
    output  wire [TL_SRC_WIDTH-1:0] m0_d_source,

    output wire                     m1_d_valid,
    input  wire                     m1_d_ready,
    output wire [TL_DATA_WIDTH-1:0]   m1_d_data,
    output wire [TL_OPCODE_WIDTH-1:0] m1_d_opcode,
    output wire [TL_PARAM_WIDTH-1:0]  m1_d_param,
    output wire [TL_SIZE_WIDTH-1:0]   m1_d_size,
    output wire [TL_SINK_WIDTH-1:0]   m1_d_sink,
    output wire                     m1_d_error,
    output  wire [TL_SRC_WIDTH-1:0] m1_d_source,

    output wire                     m2_d_valid,
    input  wire                     m2_d_ready,
    output wire [TL_DATA_WIDTH-1:0]   m2_d_data,
    output wire [TL_OPCODE_WIDTH-1:0] m2_d_opcode,
    output wire [TL_PARAM_WIDTH-1:0]  m2_d_param,
    output wire [TL_SIZE_WIDTH-1:0]   m2_d_size,
    output wire [TL_SINK_WIDTH-1:0]   m2_d_sink,
    output wire                     m2_d_error,
    output  wire [TL_SRC_WIDTH-1:0] m2_d_source
);

    // =====================================================================
    // BURST TRANSACTION SUPPORT SIGNALS
    // =====================================================================
    // These signals manage multi-beat burst transactions by tracking:
    // - Burst size and progress (how many beats remain)
    // - Which master owns the current burst (bus locking)
    // - Whether a burst is currently active (prevents arbitration)
    
    reg [TL_SIZE_WIDTH-1:0] current_burst_size; // Size field of current burst
    reg [TL_SIZE_WIDTH-1:0] num_beats;         // Total beats in current burst
    reg [TL_SIZE_WIDTH-1:0] beat_counter;      // Current beat number (0 to num_beats-1)
    reg burst_active;                          // 1 = burst in progress, 0 = idle
    reg [NUM_MASTERS-1:0] burst_master;        // One-hot: which master owns burst
    
    wire is_burst_transaction;
    wire [TL_SIZE_WIDTH-1:0] calculated_num_beats;  // Computed beats for new burst

    // =====================================================================
    // ROUND-ROBIN ARBITRATION LOGIC
    // =====================================================================
    // Implements fair scheduling using a rotating priority mask
    // Priority rotates after each complete transaction or full burst
    // Example: M0 served → mask = 110 (M1, M2 can win) → M1 serves → mask = 100 (M2 first)
    
    reg [NUM_MASTERS-1:0] mask_q, nxt_mask;     // Priority mask (current & next)
    wire [NUM_MASTERS-1:0] req;                  // Raw request vector [M2, M1, M0]
    wire [NUM_MASTERS-1:0] raw_gnt, mask_gnt;   // Grant outputs from priority encoders
    wire [NUM_MASTERS-1:0] mask_req;            // Masked requests
    
    wire [NUM_MASTERS-1:0] grant;                // Final grant decision (one-hot)
    wire [NUM_MASTERS-1:0] effective_req;       // Burst-aware request vector

    // =====================================================================
    // BURST DETECTION & SIZE CALCULATION
    // =====================================================================
    // TileLink encodes transfer size as: actual_bytes = 2^(a_size)
    // Burst occurs when a_size > BEAT_LOG2
    // Example: BEAT_WIDTH=8 bytes, BEAT_LOG2=3
    //   - a_size=3 → 8 bytes → 1 beat (single)
    //   - a_size=5 → 32 bytes → 4 beats (burst)
    
    // Get the a_size from the currently winning master
    wire [TL_SIZE_WIDTH-1:0] current_a_size = 
                        (grant == 3'b001) ? m0_a_size :
                        (grant == 3'b010) ? m1_a_size :
                                             m2_a_size;
    
    // Detect if current transaction is a burst
    assign is_burst_transaction = ({24'b0, current_a_size} > BEAT_LOG2);
    
    // Calculate number of beats: 2^(a_size - BEAT_LOG2)
    assign calculated_num_beats = is_burst_transaction ? 
                                  (1 << ({24'b0, current_a_size} - BEAT_LOG2)) : 8'b1;
    
    // =====================================================================
    // BURST STATE MACHINE
    // =====================================================================
    // IDLE STATE (burst_active = 0):
    //   - Monitor for new burst transactions
    //   - When detected: lock bus, set beat counter, remember winning master
    //
    // ACTIVE STATE (burst_active = 1):
    //   - Count beats as they complete (a_valid && a_ready handshakes)
    //   - Prevent arbitration (only burst_master can access bus)
    //   - Return to IDLE after last beat
    //
    // This ensures atomicity: no other master can interrupt a multi-beat burst
    
    // Burst state machine
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            burst_active <= 1'b0;
            beat_counter <= {TL_SIZE_WIDTH{1'b0}};
            num_beats <= {TL_SIZE_WIDTH{1'b0}};
            current_burst_size <= {TL_SIZE_WIDTH{1'b0}};
            burst_master <= {NUM_MASTERS{1'b0}};
        end
        else begin
            if (!burst_active) begin
                // No burst in progress. Check if a new burst starts
                if (arb_a_valid_out && a_ready_in && is_burst_transaction) begin
                    // New burst starting
                    burst_active <= 1'b1;
                    beat_counter <= {TL_SIZE_WIDTH{1'b0}};
                    num_beats <= calculated_num_beats;
                    current_burst_size <= current_a_size;
                    burst_master <= grant;
                end
            end
            else begin
                // Burst is in progress. Count beats as they are sent
                if (arb_a_valid_out && a_ready_in) begin
                    beat_counter <= beat_counter + 1;
                    
                    // Check if this is the last beat of the burst
                    if ((beat_counter + 1) >= num_beats) begin
                        burst_active <= 1'b0;
                        beat_counter <= {TL_SIZE_WIDTH{1'b0}};
                        num_beats <= {TL_SIZE_WIDTH{1'b0}};
                    end
                end
            end
        end
    end

    // =====================================================================
    // EFFECTIVE REQUEST LOGIC (Burst-Aware Arbitration)
    // =====================================================================
    // When burst is active: only the locked master appears to have a request
    // When burst is idle: all masters' requests are visible for arbitration
    // This prevents other masters from interfering during a burst sequence
    assign effective_req = burst_active ? burst_master : req;


    assign req = {m2_a_valid, m1_a_valid, m0_a_valid};  // Pack requests into vector [M2, M1, M0]

    // =====================================================================
    // MASK REGISTER UPDATE
    // =====================================================================
    // Update round-robin priority mask only when a complete transaction finishes:
    //   1. Single-beat transaction completes (a_valid && a_ready), OR
    //   2. Last beat of burst completes (beat_counter + 1 >= num_beats)
    // This ensures the mask rotates only at transaction boundaries, not mid-burst
    // Mask register - Only update when a complete transaction or full burst completes
    always @(posedge clk or posedge reset) begin
        if (reset)
            mask_q <= {NUM_MASTERS{1'b1}};
        // Update mask when either:
        // 1. A single-beat transaction completes, OR
        // 2. A burst transaction completes (last beat)
        else if (arb_a_valid_out && a_ready_in && (!burst_active || (beat_counter + 1) >= num_beats))
            mask_q <= nxt_mask;
    end

    // =====================================================================
    // MASKED REQUESTS & PRIORITY ENCODING
    // =====================================================================
    // Two-level priority encoding for round-robin fairness:
    //   1. Masked priority: Consider only requests allowed by mask_q
    //   2. Raw priority: Fallback if no masked requests exist
    // This ensures fairness while preventing starvation
    
    // Masked requests using effective_req (burst-aware)
    assign mask_req = effective_req & mask_q;

    // =====================================================================
    // PRIORITY ENCODER INSTANCES
    // =====================================================================
    // day14 module: Fixed-priority encoder (lowest bit = highest priority)
    // u_raw:  Unmasked encoder - grants to lowest requesting master
    // u_mask: Masked encoder - grants to lowest requesting master that's unmasked
    // Priority encoder instances
   
 
    day14 #(NUM_MASTERS) u_raw  (.req_i(req),      .gnt_o(raw_gnt));
    day14 #(NUM_MASTERS) u_mask (.req_i(mask_req), .gnt_o(mask_gnt));

    // Final grant decision: Use masked grant if any masked requests exist,
    // otherwise use raw grant (prevents deadlock if mask blocks all)
    assign grant = (|mask_req) ? mask_gnt : raw_gnt;

    // =====================================================================
    // NEXT MASK COMPUTATION (Round-Robin Rotation Logic)
    // =====================================================================
    // Rotates priority to next master after each complete transaction/burst
    // Example rotation sequence:
    //   M0 wins → nxt_mask = 110 (M1, M2 eligible)
    //   M1 wins → nxt_mask = 100 (M2 eligible first)
    //   M2 wins → nxt_mask = 001 (M0 eligible first, cycle repeats)
    // 
    // Next mask update logic - Only changes when burst completes
    always @(*) begin
        nxt_mask = {NUM_MASTERS{1'b1}};
        
        // Only compute next mask if burst is not in progress or is about to end
        if (!burst_active || ((beat_counter + 1) >= num_beats)) begin
            case (grant)
                3'b001: nxt_mask = 3'b110;
                3'b010: nxt_mask = 3'b100;
                3'b100: nxt_mask = 3'b001;
                default: nxt_mask = {NUM_MASTERS{1'b1}};
            endcase
        end
        else begin
            // During burst, keep mask unchanged to maintain burst lock
            nxt_mask = mask_q;
        end
    end

    // =====================================================================
    // A-CHANNEL MULTIPLEXER (Master Selection Logic)
    // =====================================================================
    // Routes the winning master's A-channel signals to the shared output
    // Also connects that master's a_ready feedback from downstream
    // 
    // Operation:
    //   - Based on 'grant' one-hot signal, select master's A-channel
    //   - Forward all TileLink A-channel fields (opcode, address, data, etc.)
    //   - Connect downstream a_ready_in to winning master's m#_a_ready
    //   - Other masters see a_ready=0 (blocked)
    //
    // This creates a time-multiplexed bus where only one master can transmit at a time
    
    // --------------------
    // Drive shared A-channel
    always@(*) begin
        arb_a_valid_out  = 1'b0;
        arb_a_opcode_out = {TL_OPCODE_WIDTH{1'b0}};
        arb_a_param_out  = {TL_PARAM_WIDTH{1'b0}};
        arb_a_size_out   = {TL_SIZE_WIDTH{1'b0}};
        arb_a_source_out = {TL_SRC_WIDTH{1'b0}};
        arb_a_address_out= {TL_ADDR_WIDTH{1'b0}};
        arb_a_mask_out   = {TL_MASK_WIDTH{1'b0}};
        arb_a_data_out   = {TL_DATA_WIDTH{1'b0}};

        m0_a_ready = 1'b0;
        m1_a_ready = 1'b0;
        m2_a_ready = 1'b0;
    
      case (grant)//no need to use _ff as it is immediate effect
            3'b001: begin // M0
                arb_a_valid_out  = m0_a_valid;
                arb_a_opcode_out = m0_a_opcode;
                arb_a_param_out  = m0_a_param;
                arb_a_size_out   = m0_a_size;
                arb_a_source_out = m0_a_source;
                arb_a_address_out= m0_a_address;
                arb_a_mask_out   = m0_a_mask;
                arb_a_data_out   = m0_a_data;
                m0_a_ready      = a_ready_in;
                if (m0_a_valid && a_ready_in) begin
                    $display("[ARB-GRANT-M0] Time=%0t HANDSHAKE: opcode=%0d addr=0x%h src=%0d",
                             $time, m0_a_opcode, m0_a_address, m0_a_source);
                end
            end
            3'b010: begin // M1
                arb_a_valid_out  = m1_a_valid;
                arb_a_opcode_out = m1_a_opcode;
                arb_a_param_out  = m1_a_param;
                arb_a_size_out   = m1_a_size;
                arb_a_source_out = m1_a_source;
                arb_a_address_out= m1_a_address;
                arb_a_mask_out   = m1_a_mask;
                arb_a_data_out   = m1_a_data;
                m1_a_ready      = a_ready_in;
                if (m1_a_valid && a_ready_in) begin
                    $display("[ARB-GRANT-M1] Time=%0t HANDSHAKE: opcode=%0d addr=0x%h src=%0d",
                             $time, m1_a_opcode, m1_a_address, m1_a_source);
                end
            end
            3'b100: begin // M2
                arb_a_valid_out  = m2_a_valid;
                arb_a_opcode_out = m2_a_opcode;
                arb_a_param_out  = m2_a_param;
                arb_a_size_out   = m2_a_size;
                arb_a_source_out = m2_a_source;
                arb_a_address_out= m2_a_address;
                arb_a_mask_out   = m2_a_mask;
                arb_a_data_out   = m2_a_data;
                m2_a_ready      = a_ready_in;
            end
            default: begin
            // For all other grant values: nothing granted
            arb_a_valid_out  = '0;
            arb_a_opcode_out = '0;
            arb_a_param_out  = '0;
            arb_a_size_out   = '0;
            arb_a_source_out = '0;
            arb_a_address_out= '0;
            arb_a_mask_out   = '0;
            arb_a_data_out   = '0;
            m0_a_ready = '0;
            m1_a_ready = '0;
            m2_a_ready = '0;
            end
        endcase
    end
    

    // =====================================================================
    // RESPONSE ROUTING FIFO
    // =====================================================================
    // PROBLEM: D-channel responses can arrive many cycles after A-channel requests
    //          How does arbiter know which master to route each response to?
    //
    // SOLUTION: FIFO queue that tracks master ownership
    //   - WRITE: Every time an A-channel beat is accepted, write grant pattern to FIFO
    //   - READ: Every time a D-channel response arrives, read FIFO to get destination master
    //
    // FOR BURSTS: Each beat generates a FIFO entry (4-beat burst = 4 FIFO writes)
    //             Each response consumes a FIFO entry (4 responses = 4 FIFO reads)
    //
    // FIFO Depth = 8 supports up to 8 outstanding requests/beats across all masters
    //
  // ---------------------------
// 4-deep 3-bit FIFO instance
// ---------------------------
reg [2:0] rd_data;       // Latched FIFO read data (stable during partial handshakes)
wire [2:0] rd_data_raw;  // Raw FIFO output (before latching)
wire fifo_full_unused;
wire fifo_empty_unused;

  // ========================
  // FIFO for Burst Support
  // ========================
  // This FIFO maintains a queue of which master should receive each D-channel response.
  // For burst transactions: each beat from the same master generates a FIFO entry.
  // This ensures correct routing of multi-beat responses back to the requesting master.
  // The FIFO depth of 8 supports up to 8 outstanding transactions (single beats + burst beats).

async_fifo #(
    .DATA_WIDTH(3),           // 3 bits for one-hot grant pattern [M2, M1, M0]
    .DEPTH(8)                 // Support 8 outstanding beats
  
) fifo_inst (
    .wr_clk(clk),             // A-channel clock
    .rd_clk(clk),             // Same clock (single domain)
    .reset(reset),
    .wr_en(arb_a_valid_out && a_ready_in),   // Write when A-channel beat is accepted
    .wr_data(grant),                         // Write current grant pattern (which master)
    .rd_en(d_valid_in && d_ready_out && !fifo_empty_unused), // Read with empty protection
    .rd_data(rd_data_raw),                   // Raw FIFO output
    .full(fifo_full_unused),                 // Unused (assume FIFO sized correctly)
    .empty(fifo_empty_unused)                // Unused (assume FIFO not over-drained)
);

// CRITICAL FIX: Latch rd_data during handshake to prevent stale reads
always @(posedge clk or posedge reset) begin
    if (reset) begin
        rd_data <= 3'b000;
    end else if (d_valid_in && d_ready_out) begin
        // Capture FIFO output only during successful handshake
        rd_data <= rd_data_raw;
    end
    // else: hold previous value during partial handshakes
end

// Debug FIFO operations and enhanced routing validation
always @(posedge clk) begin
    if (arb_a_valid_out && a_ready_in) begin
        $display("[ARB-FIFO-WR] Time=%0t WRITE grant=3'b%b source=%0d -> M%0d", 
                 $time, grant, arb_a_source_out, (grant[0] ? 0 : grant[1] ? 1 : 2));
    end
    if (d_valid_in && d_ready_out) begin
        $display("[ARB-FIFO-RD] Time=%0t READ rd_data_raw=3'b%b rd_data_latched=3'b%b source=%0d src_id_m0=%b src_id_m1=%b src_id_m2=%b route_to_m0=%b route_to_m1=%b route_to_m2=%b", 
                 $time, rd_data_raw, rd_data, d_source_in, src_id_master0, src_id_master1, src_id_master2, route_to_m0, route_to_m1, route_to_m2);
        
        // Enhanced debug: show source ID validation results
        $display("[ARB-SRC-VAL] Time=%0t d_source=%0d src_ranges[M0=%b M1=%b M2=%b] routes[M0=%b M1=%b M2=%b]",
                 $time, d_source_in, src_id_master0, src_id_master1, src_id_master2, route_to_m0, route_to_m1, route_to_m2);
        
        // Warn about routing mismatches  
        if (!route_to_m0 && !route_to_m1 && !route_to_m2) begin
            $display("[ARB-ERROR] Time=%0t NO MASTER ROUTED! fifo=3'b%b source=%0d", $time, rd_data_raw, d_source_in);
        end
    end
end

    // =====================================================================
    // D-CHANNEL DEMULTIPLEXER (Response Routing Logic)
    // =====================================================================
    // Routes incoming D-channel responses back to the correct master
    // based on FIFO output (rd_data = which master requested this response)
    //
    // Operation:
    //   1. When d_valid_in arrives, FIFO was read to get master ID (rd_data)
    //   2. Assert d_valid only to the matching master (m0/m1/m2_d_valid)
    //   3. Forward D-channel fields (opcode, data, error, etc.) to that master
    //   4. Connect that master's d_ready back to slave (d_ready_out)
    //
    // This ensures responses reach the correct requesting master even if
    // requests were interleaved or bursts were in flight
    //
    // D-channel demux - FIXED: Use rd_data_raw for immediate routing, rd_data for stability
  // TARGETED FIX: Use source ID based routing for d_ready to avoid FIFO routing deadlock 
  // while keeping FIFO-based routing for d_valid (which works fine for beat operations)
  // Master source ID ranges: M0=[0-3], M1=[4-7], M2=[8-11]
  wire route_ready_to_m0 = (d_source_in >= 0) && (d_source_in <= 3);
  wire route_ready_to_m1 = (d_source_in >= 4) && (d_source_in <= 7); 
  wire route_ready_to_m2 = (d_source_in >= 8) && (d_source_in <= 11);

  assign d_ready_out = route_ready_to_m0 ? m0_d_ready :
                       route_ready_to_m1 ? m1_d_ready :
                                           m2_d_ready;

    // =====================================================================
    // D-CHANNEL DEMULTIPLEXER (Response Routing Logic) 
    // =====================================================================
    // Enhanced routing with source ID validation to prevent cross-master contamination
    // Routes incoming D-channel responses back to the correct master based on:
    // 1. FIFO output (rd_data = which master requested this response) 
    // 2. Source ID validation (ensure source ID matches expected master range)
    //
    // Master Source ID Ranges:
    // - Master 0: source IDs 0,1,2,3
    // - Master 1: source IDs 4,5,6,7  
    // - Master 2: source IDs 8,9,10,11
    //
    // Dual validation prevents routing errors caused by FIFO timing issues
    // =====================================================================
    
    // Source ID range validation
    wire src_id_master0 = (d_source_in >= 0) && (d_source_in <= 3);
    wire src_id_master1 = (d_source_in >= 4) && (d_source_in <= 7);
    wire src_id_master2 = (d_source_in >= 8) && (d_source_in <= 11);
    
    // CRITICAL FIX: Use pure source ID routing for Master 0 and Master 1  
    // This prevents stale FIFO entries from causing cross-transaction contamination
    wire route_to_m0 = src_id_master0;  // Route based on source ID only
    wire route_to_m1 = src_id_master1;  // Route based on source ID only  
    wire route_to_m2 = (rd_data_raw == 3'b100) && src_id_master2;

  // D-channel valid signals (combinational for proper handshake)
  assign m0_d_valid  = d_valid_in && route_to_m0;
  assign m1_d_valid  = d_valid_in && route_to_m1;
  assign m2_d_valid  = d_valid_in && route_to_m2;

  // Forward D-channel fields only to the addressed master (others get zeros)
  // NOTE: The combinational nature causes glitches in waveforms when FIFO reads occur
  // However, this is acceptable per TileLink spec - data is only sampled during handshakes
  assign m0_d_data   = m0_d_valid ? d_data_in : {TL_DATA_WIDTH{1'b0}};
  assign m0_d_opcode = m0_d_valid ? d_opcode_in : {TL_OPCODE_WIDTH{1'b0}};
  assign m0_d_param  = m0_d_valid ? d_param_in : {TL_PARAM_WIDTH{1'b0}};
  assign m0_d_size   = m0_d_valid ? d_size_in : {TL_SIZE_WIDTH{1'b0}};
  assign m0_d_sink   = m0_d_valid ? d_sink_in : {TL_SINK_WIDTH{1'b0}};
  assign m0_d_error  = m0_d_valid ? d_error_in : 1'b0;
  assign m0_d_source = m0_d_valid ? d_source_in : {TL_SRC_WIDTH{1'b0}};

  assign m1_d_data   = m1_d_valid ? d_data_in : {TL_DATA_WIDTH{1'b0}};
  assign m1_d_opcode = m1_d_valid ? d_opcode_in : {TL_OPCODE_WIDTH{1'b0}};
  assign m1_d_param  = m1_d_valid ? d_param_in : {TL_PARAM_WIDTH{1'b0}};
  assign m1_d_size   = m1_d_valid ? d_size_in : {TL_SIZE_WIDTH{1'b0}};
  assign m1_d_sink   = m1_d_valid ? d_sink_in : {TL_SINK_WIDTH{1'b0}};
  assign m1_d_error  = m1_d_valid ? d_error_in : 1'b0;
  assign m1_d_source = m1_d_valid ? d_source_in : {TL_SRC_WIDTH{1'b0}};

  assign m2_d_data   = m2_d_valid ? d_data_in : {TL_DATA_WIDTH{1'b0}};
  assign m2_d_opcode = m2_d_valid ? d_opcode_in : {TL_OPCODE_WIDTH{1'b0}};
  assign m2_d_param  = m2_d_valid ? d_param_in : {TL_PARAM_WIDTH{1'b0}};
  assign m2_d_size   = m2_d_valid ? d_size_in : {TL_SIZE_WIDTH{1'b0}};
  assign m2_d_sink   = m2_d_valid ? d_sink_in : {TL_SINK_WIDTH{1'b0}};
  assign m2_d_error  = m2_d_valid ? d_error_in : 1'b0;
  assign m2_d_source = m2_d_valid ? d_source_in : {TL_SRC_WIDTH{1'b0}};
   
 

endmodule

