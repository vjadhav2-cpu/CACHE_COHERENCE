`timescale 1ns/1ps

/* verilator lint_off UNUSEDSIGNAL */

module rv64g_l2_directory #(
    parameter SETS = 256,
    parameter WAYS = 16,
    parameter CORES = 4
) (
    input  wire clk,
    input  wire rst_n,

    // Read Port (Read whole set)
    input  wire [$clog2(SETS)-1:0]  rd_set_i,
    output wire [WAYS-1:0]          rd_valid_o,
    output wire [WAYS*CORES-1:0]    rd_sharers_o,
    output wire [WAYS-1:0]          rd_owner_valid_o,
    output wire [WAYS*((CORES > 1) ? $clog2(CORES) : 1)-1:0] rd_owner_id_o,
    output wire [WAYS-1:0]          rd_dirty_o,

    // Write Port (Write specific way)
    input  wire                     we_i,
    input  wire [$clog2(SETS)-1:0]  wr_set_i,
    input  wire [$clog2(WAYS)-1:0]  wr_way_i,
    
    // Write Data
    input  wire                     wr_valid_i,
    input  wire [CORES-1:0]         wr_sharers_i,
    input  wire                     wr_owner_valid_i,
    input  wire [((CORES > 1) ? $clog2(CORES) : 1)-1:0] wr_owner_id_i,
    input  wire                     wr_dirty_i
);

    localparam OWNER_ID_W = (CORES > 1) ? $clog2(CORES) : 1;

    // Storage per way
    // We can store them as separate arrays or one big array.
    // Since we read all ways at once, separate arrays for each way might be easier to express 
    // or a single array of width (WAYS * BITS).
    
    // Let's use a single array of width WAYS * (Structure)
    // Structure: {dirty, owner_id, owner_valid, sharers, valid}
    localparam ENTRY_W = 1 + OWNER_ID_W + 1 + CORES + 1;
    
    reg [WAYS*ENTRY_W-1:0] ram [0:SETS-1];

    // Read Logic
    wire [WAYS*ENTRY_W-1:0] rd_data_raw;
    assign rd_data_raw = ram[rd_set_i];

    // Unpack Read Data
    genvar w;
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : unpack_read
            wire [ENTRY_W-1:0] entry = rd_data_raw[w*ENTRY_W +: ENTRY_W];
            
            assign rd_valid_o[w] = entry[0];
            assign rd_sharers_o[w*CORES +: CORES] = entry[1 +: CORES];
            assign rd_owner_valid_o[w] = entry[1+CORES];
            assign rd_owner_id_o[w*OWNER_ID_W +: OWNER_ID_W] = entry[1+CORES+1 +: OWNER_ID_W];
            assign rd_dirty_o[w] = entry[1+CORES+1+OWNER_ID_W];
        end
    endgenerate

    // Write Logic
    // Enforce Invariants:
    // 1. dirty -> owner_valid
    // 2. owner_valid -> sharers == 0
    
    wire                     safe_valid;
    wire [CORES-1:0]         safe_sharers;
    wire                     safe_owner_valid;
    wire [OWNER_ID_W-1:0]    safe_owner_id;
    wire                     safe_dirty;

    assign safe_valid = wr_valid_i;
    assign safe_dirty = wr_dirty_i;
    
    // Rule: dirty -> owner_valid
    assign safe_owner_valid = wr_dirty_i ? 1'b1 : wr_owner_valid_i;
    
    // Rule: owner_valid -> sharers == 0
    assign safe_sharers = safe_owner_valid ? {CORES{1'b0}} : wr_sharers_i;
    
    assign safe_owner_id = wr_owner_id_i;

    wire [ENTRY_W-1:0] wr_entry_packed;
    assign wr_entry_packed = {safe_dirty, safe_owner_id, safe_owner_valid, safe_sharers, safe_valid};

    // FIX: Added proper synchronous reset for ASIC compatibility
    // The initial block is kept for simulation but reset_n provides runtime reset
    integer i;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Asynchronous active-low reset - consistent with rest of design
            for (i = 0; i < SETS; i = i + 1) begin
                ram[i] <= {(WAYS*ENTRY_W){1'b0}};
            end
        end else if (we_i) begin
            ram[wr_set_i][wr_way_i*ENTRY_W +: ENTRY_W] <= wr_entry_packed;
        end
    end
    
    // Simulation-only initialization (non-synthesizable for ASIC, works for FPGA/sim)
    // synthesis translate_off
    initial begin
        for (i = 0; i < SETS; i = i + 1) begin
            ram[i] = 0;
        end
    end
    // synthesis translate_on

endmodule
