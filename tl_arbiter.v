`timescale 1ns/1ps

module tl_arbiter #(
    parameter N = 4,
    parameter DATA_W = 100
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Inputs (N clients)
    input  wire [N-1:0]          valid_i,
    output reg  [N-1:0]          ready_o,
    input  wire [N*DATA_W-1:0]   data_i,

    // Output (1 sink)
    output reg                   valid_o,
    input  wire                  ready_i,
    output reg  [DATA_W-1:0]     data_o,

    // Burst lock: 1 = last (or only) beat, 0 = more beats follow
    input  wire                  last_i
);

    reg [N-1:0] mask;
    wire [N-1:0] masked_req = valid_i & mask;
    wire [N-1:0] raw_grant;
    wire [N-1:0] masked_grant;
    reg [N-1:0] arb_grant;

    // Burst lock state
    reg         locked_q;
    reg [N-1:0] locked_grant_q;

    // Effective grant: use locked grant during burst, arbitration result otherwise
    wire [N-1:0] grant = locked_q ? locked_grant_q : arb_grant;

    // Priority arbiters (fixed priority)
    // Find first set bit
    assign raw_grant[0] = valid_i[0];
    genvar i;
    generate
        for (i = 1; i < N; i = i + 1) begin : gen_raw_grant
            assign raw_grant[i] = valid_i[i] & ~|valid_i[i-1:0];
        end
    endgenerate

    assign masked_grant[0] = masked_req[0];
    generate
        for (i = 1; i < N; i = i + 1) begin : gen_masked_grant
            assign masked_grant[i] = masked_req[i] & ~|masked_req[i-1:0];
        end
    endgenerate

    // Final arbitration selection (before lock override)
    always @(*) begin
        if (|masked_req) begin
            arb_grant = masked_grant;
        end else begin
            arb_grant = raw_grant;
        end
    end

    // Output Muxing
    integer j;
    always @(*) begin
        data_o = {DATA_W{1'b0}};
        valid_o = 1'b0;
        ready_o = {N{1'b0}};
        
        for (j = 0; j < N; j = j + 1) begin
            if (grant[j]) begin
                data_o = data_i[j*DATA_W +: DATA_W];
                valid_o = valid_i[j];
                ready_o[j] = ready_i;
            end
        end
    end

    // Rotate priority and burst lock tracking
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mask <= {N{1'b1}};
            locked_q <= 1'b0;
            locked_grant_q <= {N{1'b0}};
        end else if (valid_o && ready_i) begin
            if (last_i) begin
                // Last beat (or single-beat): unlock and rotate priority
                locked_q <= 1'b0;
                if (locked_q)
                    mask <= ~(locked_grant_q | (locked_grant_q - 1'b1));
                else
                    mask <= ~(arb_grant | (arb_grant - 1'b1));
            end else if (!locked_q) begin
                // First beat of multi-beat burst: lock to current winner
                locked_q <= 1'b1;
                locked_grant_q <= arb_grant;
            end
            // If locked_q && !last_i: stay locked, no mask change
        end
    end

endmodule
