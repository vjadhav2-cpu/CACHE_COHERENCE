`timescale 1ns/10ps
// =============================================================================
// Project: TileLink Inclusive Directory Coherence (TIDC) System
// Module: Source ID Manager
// Description: Manages allocation and deallocation of source IDs for L1 adapters
// =============================================================================

module source_id_manager (
    input  wire                   clk,
    input  wire                   rst,
    
    // Source ID allocation interface
    input  wire                   alloc_req,
    output wire                   alloc_gnt,
    output wire [3:0]             alloc_source_id,
    
    // Source ID deallocation interface
    input  wire                   dealloc_req,
    input  wire [3:0]             dealloc_source_id
);

    // Maximum number of source IDs = 2^4 = 16
    localparam MAX_IDS = 16;
    
    // Bitmap to track which source IDs are in use (1 = in use, 0 = free)
    reg [MAX_IDS-1:0] source_id_in_use;
    
    // Next source ID to allocate
    reg [3:0] next_free_id;
    
    // Flag to indicate if any source IDs are available
    wire any_id_available;
    
    // Request and Grant handshaking
    reg alloc_gnt_r;
    
    // Source ID allocation logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            source_id_in_use <= {MAX_IDS{1'b0}}; // All IDs free after reset
            next_free_id <= 4'b0;   // Start allocating from ID 0
            alloc_gnt_r <= 1'b0;
        end
        else begin
            // Default value
            alloc_gnt_r <= 1'b0;
            
            // Handle deallocation requests first
            if (dealloc_req) begin
                source_id_in_use[dealloc_source_id] <= 1'b0; // Mark as free
            end
            
            // Handle allocation requests
            if (alloc_req && any_id_available) begin
                source_id_in_use[next_free_id] <= 1'b1; // Mark as in use
                alloc_gnt_r <= 1'b1;
                
                // Find the next free ID for future allocations (round-robin)
                find_next_free_id();
            end
        end
    end
    
    // Finding the next free ID using circular search
    task find_next_free_id;
        integer i;
        reg found;
    begin
        found = 1'b0;
        for (i = 1; i <= MAX_IDS; i = i + 1) begin
            // Calculate next ID with wraparound
            if (!found && !source_id_in_use[((next_free_id + i[3:0]) % MAX_IDS)]) begin
                next_free_id <= 4'(({28'b0, next_free_id} + {28'b0, i[3:0]}) % MAX_IDS);
                found = 1'b1;
            end
        end
    end
    endtask
    
    // Check if any ID is available
    assign any_id_available = (source_id_in_use != {MAX_IDS{1'b1}});
    
    // Output assignments
    assign alloc_gnt = alloc_gnt_r;
    assign alloc_source_id = next_free_id;

endmodule 