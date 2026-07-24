/*==============================================================================
 * TILELINK UH DECODER - Address-Based Slave Selection
 *
 * MODULE PURPOSE:
 * ---------------
 * This module decodes incoming addresses from the master and determines which
 * slave should service the request. It implements a simple linear address
 * mapping scheme where each slave owns a contiguous address range.
 *
 * FUNCTIONALITY:
 * --------------
 * Input:  TileLink A-channel address
 * Output: Slave ID (0 to NUM_SLAVES-1) and valid signal
 *
 * ADDRESS MAPPING SCHEME:
 * -----------------------
 * Each slave gets DEPTH consecutive addresses:
 *   Slave 0: 0x000 to (1×DEPTH - 1)     [0x000 - 0x1FF for DEPTH=512]
 *   Slave 1: 1×DEPTH to (2×DEPTH - 1)   [0x200 - 0x3FF for DEPTH=512]
 *   Slave 2: 2×DEPTH to (3×DEPTH - 1)   [0x400 - 0x5FF for DEPTH=512]
 *   ...
 *   Slave N: N×DEPTH to ((N+1)×DEPTH - 1)
 *
 * Example (NUM_SLAVES=5, DEPTH=512):
 *   Address 0x100 → Slave 0
 *   Address 0x250 → Slave 1  
 *   Address 0x480 → Slave 2
 *   Address 0x600 → Slave 3
 *   Address 0x850 → Slave 4
 *   Address 0xA00 → INVALID (tl_out_valid=0)
 *
 * OPERATION:
 * ----------
 * 1. Wait for tl_in_a_valid (incoming request)
 * 2. Compare address against slave address ranges
 * 3. Output matching slave ID (0-4)
 * 4. Assert tl_out_valid if address is within valid range
 * 5. Clear outputs if no valid match or no request
 *
 * TIMING:
 * -------
 * - REGISTERED output: One-cycle delay from input to output
 * - Outputs hold stable until next valid request
 * - Synchronous reset clears decoder to slave 0, valid=0
 *
 * IMPORTANT NOTES:
 * ----------------
 * - Uses lower half of address bus [ADDR_WIDTH/2-1:0] for comparison
 *   (assumes upper bits are zeros or reserved)
 * - Priority encoding: if-else chain checks slaves 0→1→2→3→4 in order
 * - Invalid addresses set tl_out=0 and tl_out_valid=0 (default to slave 0 but invalid)
 * - For sparse address maps or non-linear ranges, this module needs modification
 *
 * AUTHORS: TileLink Crossbar Team
 * DATE: 2026-01-19
 *==============================================================================*/

/**********Decode*************/
module tilelink_uh_decoder #(
    parameter ADDR_WIDTH = 64,       // Full address width (only lower half used)
    parameter NUM_SLAVES = 5,        // Number of slaves to decode (max 5 in this implementation)
    parameter DEPTH      = 512       // Address range per slave (in memory locations)
) (
    input  wire                      clk,           // Clock for registered output
    input  wire                      rst,           // Synchronous active-high reset
    input  wire                      tl_in_a_valid, // Request valid signal from master
    input  wire [ADDR_WIDTH-1:0] tl_in_addr,       // Full address from master (only lower half used)
    output wire [$clog2(NUM_SLAVES)-1:0] tl_out,   // Decoded slave ID (e.g., 3 bits for 5 slaves)
    output wire                     tl_out_valid    // High if address matched a valid slave
);
    // Internal registers for decoded outputs
    reg [$clog2(NUM_SLAVES)-1:0] r_tl_out;          // Registered slave ID
    reg r_tl_out_valid;                             // Registered valid flag
    
    // Assign outputs from registers (one-cycle latency)
    assign tl_out = r_tl_out;
    assign tl_out_valid = r_tl_out_valid;
    
    // =====================================================================
    // ADDRESS DECODING LOGIC
    // =====================================================================
    // Sequential logic with if-else priority encoding
    // Checks address ranges from Slave 0 → Slave 4
    // Only lower half of address is used: [ADDR_WIDTH/2-1:0]
    // =====================================================================
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset: Default to slave 0, not valid
            r_tl_out       <= 0;
            r_tl_out_valid <= 0;
        end else begin
            if (tl_in_a_valid) begin
                // ========== SLAVE 0: Address 0 to (DEPTH-1) ==========
                if ((tl_in_addr[ADDR_WIDTH/2-1:0] < 1*DEPTH)) begin
                    r_tl_out       <= 0;
                    r_tl_out_valid <= 1;
                end 
                // ========== SLAVE 1: Address DEPTH to (2*DEPTH-1) ==========
                else if ((tl_in_addr[ADDR_WIDTH/2-1:0] >= 1*DEPTH) && (tl_in_addr[ADDR_WIDTH/2-1:0] < 2*DEPTH)) begin
                    r_tl_out       <= 1;
                    r_tl_out_valid <= 1;
                end 
                // ========== SLAVE 2: Address 2*DEPTH to (3*DEPTH-1) ==========
                else if ((tl_in_addr[ADDR_WIDTH/2-1:0] >= 2*DEPTH) && (tl_in_addr[ADDR_WIDTH/2-1:0] < 3*DEPTH)) begin
                    r_tl_out       <= 2;
                    r_tl_out_valid <= 1;
                end 
                // ========== SLAVE 3: Address 3*DEPTH to (4*DEPTH-1) ==========
                else if ((tl_in_addr[ADDR_WIDTH/2-1:0] >= 3*DEPTH) && (tl_in_addr[ADDR_WIDTH/2-1:0] < 4*DEPTH)) begin
                    r_tl_out       <= 3;
                    r_tl_out_valid <= 1;
                end 
                // ========== SLAVE 4: Address 4*DEPTH to (5*DEPTH-1) ==========
                else if ((tl_in_addr[ADDR_WIDTH/2-1:0] >= 4*DEPTH) && (tl_in_addr[ADDR_WIDTH/2-1:0] < 5*DEPTH)) begin
                    r_tl_out       <= 4;
                    r_tl_out_valid <= 1;
                end 
                // ========== INVALID ADDRESS: Outside all slave ranges ==========
                else begin
                    r_tl_out       <= 0;              // Default to slave 0 (but not valid)
                    r_tl_out_valid <= 0;              // Signal address decode failure
                end
            end else begin
                // No valid request: Clear outputs
                r_tl_out       <= 0;
                r_tl_out_valid <= 0;
            end
        end
    end
endmodule
