`timescale 1ns/1ps

module rv64g_l2_mshr #(
    parameter ADDR_W = 64,
    parameter SOURCE_W = 6, // 4 (L1 Source) + 2 (Client ID)
    parameter TYPE_W = 3,   // Opcode
    parameter CORES = 4,
    parameter CID_W = 2
) (
    input  wire clk,
    input  wire rst_n,

    // Allocation Interface
    input  wire                     alloc_req_i,
    input  wire [ADDR_W-1:0]        alloc_addr_i,
    input  wire [SOURCE_W-1:0]      alloc_source_i,
    input  wire [TYPE_W-1:0]        alloc_type_i,
    output wire                     alloc_ready_o, // 1 if MSHR is free

    // Deallocation Interface
    input  wire                     dealloc_req_i,

    // Probe Management
    // FSM sets the initial set of probes to wait for
    input  wire                     set_probes_i,
    input  wire [CORES-1:0]         probes_mask_i,
    
    // Probe Ack (clears bit for specific core)
    input  wire                     probe_ack_i,
    input  wire [CID_W-1:0] probe_ack_id_i,

    // Status Outputs
    output wire                     valid_o,
    output wire [ADDR_W-1:0]        addr_o,
    output wire [SOURCE_W-1:0]      source_o,
    output wire [TYPE_W-1:0]        type_o,
    output wire [CORES-1:0]         pending_probes_o
);

    reg                     valid_q;
    reg [ADDR_W-1:0]        addr_q;
    reg [SOURCE_W-1:0]      source_q;
    reg [TYPE_W-1:0]        type_q;
    reg [CORES-1:0]         pending_probes_q;

    assign alloc_ready_o = !valid_q;
    
    assign valid_o          = valid_q;
    assign addr_o           = addr_q;
    assign source_o         = source_q;
    assign type_o           = type_q;
    assign pending_probes_o = pending_probes_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_q <= 1'b0;
            addr_q <= {ADDR_W{1'b0}};
            source_q <= {SOURCE_W{1'b0}};
            type_q <= {TYPE_W{1'b0}};
            pending_probes_q <= {CORES{1'b0}};
        end else begin
            if (dealloc_req_i) begin
                valid_q <= 1'b0;
                pending_probes_q <= {CORES{1'b0}};
            end else if (alloc_req_i && !valid_q) begin
                valid_q <= 1'b1;
                addr_q <= alloc_addr_i;
                source_q <= alloc_source_i;
                type_q <= alloc_type_i;
                pending_probes_q <= {CORES{1'b0}}; // Initially 0, FSM sets it later
            end else begin
                // Probe Management
                if (set_probes_i && probe_ack_i) begin
                    pending_probes_q <= probes_mask_i & ~( {{(CORES-1){1'b0}}, 1'b1} << probe_ack_id_i );
                end else if (set_probes_i) begin
                    pending_probes_q <= probes_mask_i;
                end else if (probe_ack_i) begin
                    pending_probes_q[probe_ack_id_i] <= 1'b0;
                end
            end
        end
    end

endmodule
