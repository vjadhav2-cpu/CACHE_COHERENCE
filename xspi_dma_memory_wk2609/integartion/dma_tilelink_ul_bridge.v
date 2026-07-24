`timescale 1ns/1ps
`default_nettype none
//// DMA TO TLUL BRIDGE
module dma_tilelink_ul_bridge #(
    parameter DATA_WIDTH   = 64,
    parameter ADDR_WIDTH   = 64,
    parameter SOURCE_WIDTH = 4,
    parameter SIZE_WIDTH   = 3,
    parameter OPCODE_WIDTH = 3,
    parameter BEAT_LOG2    = $clog2(DATA_WIDTH/8),
    parameter PARAM_WIDTH  = 3,
    parameter SINK_WIDTH   = 3
)(
    input  wire                   clk,
    input  wire                   rst_n,

    input  wire [ADDR_WIDTH-1:0]   dma_address_in,
    input  wire [DATA_WIDTH-1:0]   dma_data_in,
    input  wire [DATA_WIDTH/8-1:0] dma_mask_in,
    input  wire [SIZE_WIDTH-1:0]   dma_size_in,
    input  wire [SOURCE_WIDTH-1:0] dma_source_in,
    input  wire [OPCODE_WIDTH-1:0] dma_opcode_in,
    input  wire [PARAM_WIDTH-1:0]  dma_param_in,
    input  wire                    dma_valid_in,
    output reg                     dma_ready_out,

    output wire [ADDR_WIDTH-1:0]             tl_a_address_out,
    output wire [OPCODE_WIDTH-1:0]           tl_a_opcode_out,
    output wire [(DATA_WIDTH/8)-1:0]         tl_a_mask_out,
    output wire [DATA_WIDTH-1:0]             tl_a_data_out,
    output wire [SIZE_WIDTH-1:0]             tl_a_size_out,
    output wire                              tl_a_valid_out,
    output wire [SOURCE_WIDTH-1:0]           tl_a_source_out,
    output wire [PARAM_WIDTH-1:0]            tl_a_param_out,
    input  wire                              tl_a_ready_in,

    input  wire [DATA_WIDTH-1:0]   tl_d_data_in,
    input  wire                    tl_d_valid_in,
    input  wire [SOURCE_WIDTH-1:0] tl_d_source_in,
    input  wire [SIZE_WIDTH-1:0]   tl_d_size_in,
    input  wire [OPCODE_WIDTH-1:0] tl_d_opcode_in,
    input  wire                    tl_d_error_in,
    input  wire [SINK_WIDTH-1:0]   tl_d_sink_in,
    input  wire [PARAM_WIDTH-1:0]  tl_d_param_in,
    output wire                    tl_d_ready_out,

    output wire [DATA_WIDTH-1:0]   dma_data_out,
    output wire                    dma_valid_out,
    input  wire                    dma_ready_in
);

    localparam [1:0] BRIDGE_IDLE = 2'd0,
                     WAIT_A      = 2'd1,
                     WAIT_D      = 2'd2,
                     DONE        = 2'd3;

    reg [1:0] bridge_state;

    reg [ADDR_WIDTH-1:0]       r_tl_a_address_out;
    reg [OPCODE_WIDTH-1:0]     r_tl_a_opcode_out;
    reg [(DATA_WIDTH/8)-1:0]   r_tl_a_mask_out;
    reg [DATA_WIDTH-1:0]       r_tl_a_data_out;
    reg [SIZE_WIDTH-1:0]       r_tl_a_size_out;
    reg                        r_tl_a_valid_out;
    reg [SOURCE_WIDTH-1:0]     r_tl_a_source_out;
    reg [PARAM_WIDTH-1:0]      r_tl_a_param_out;

    reg [DATA_WIDTH-1:0]       r_dma_data_out;
    reg                        r_dma_valid_out;

    assign tl_a_address_out = r_tl_a_address_out;
    assign tl_a_opcode_out  = r_tl_a_opcode_out;
    assign tl_a_mask_out    = r_tl_a_mask_out;
    assign tl_a_data_out    = r_tl_a_data_out;
    assign tl_a_size_out    = r_tl_a_size_out;
    assign tl_a_valid_out   = r_tl_a_valid_out;
    assign tl_a_source_out  = r_tl_a_source_out;
    assign tl_a_param_out   = r_tl_a_param_out;

    assign dma_data_out = r_dma_data_out;
    assign dma_valid_out = r_dma_valid_out;

    assign tl_d_ready_out = (bridge_state == WAIT_D);

`ifndef SYNTHESIS
    reg [OPCODE_WIDTH-1:0] a_opcode_q;
    reg [ADDR_WIDTH-1:0]   a_addr_q;
    reg [DATA_WIDTH-1:0]   a_data_q;
    reg [(DATA_WIDTH/8)-1:0] a_mask_q;
    reg [SIZE_WIDTH-1:0]   a_size_q;
    reg [SOURCE_WIDTH-1:0] a_source_q;
    reg                    a_wait_q;

    reg [DATA_WIDTH-1:0] d_hold_q;
    reg                  d_wait_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_wait_q   <= 1'b0;
            a_opcode_q <= {OPCODE_WIDTH{1'b0}};
            a_addr_q   <= {ADDR_WIDTH{1'b0}};
            a_data_q   <= {DATA_WIDTH{1'b0}};
            a_mask_q   <= {(DATA_WIDTH/8){1'b0}};
            a_size_q   <= {SIZE_WIDTH{1'b0}};
            a_source_q <= {SOURCE_WIDTH{1'b0}};
            d_hold_q   <= {DATA_WIDTH{1'b0}};
            d_wait_q   <= 1'b0;
        end else begin
            if (tl_a_valid_out && !tl_a_ready_in) begin
                if (a_wait_q) begin
                    if ((tl_a_opcode_out  !== a_opcode_q) ||
                        (tl_a_address_out !== a_addr_q)   ||
                        (tl_a_data_out    !== a_data_q)   ||
                        (tl_a_mask_out    !== a_mask_q)   ||
                        (tl_a_size_out    !== a_size_q)   ||
                        (tl_a_source_out  !== a_source_q)) begin
                        $error("[BR_ASSERT] TL A payload changed while waiting for ready");
                    end
                end else begin
                    a_opcode_q <= tl_a_opcode_out;
                    a_addr_q   <= tl_a_address_out;
                    a_data_q   <= tl_a_data_out;
                    a_mask_q   <= tl_a_mask_out;
                    a_size_q   <= tl_a_size_out;
                    a_source_q <= tl_a_source_out;
                end
                a_wait_q <= 1'b1;
            end else begin
                a_wait_q <= 1'b0;
            end

            if (dma_valid_out && !dma_ready_in) begin
                if (d_wait_q) begin
                    if (dma_data_out !== d_hold_q)
                        $error("[BR_ASSERT] DMA response data changed while backpressured");
                end else begin
                    d_hold_q <= dma_data_out;
                end
                d_wait_q <= 1'b1;
            end else begin
                d_wait_q <= 1'b0;
            end

            if (rst_n && tl_d_valid_in && tl_d_ready_out && tl_d_error_in)
                $error("[BR_ASSERT] TL D error observed during handshake");
        end
    end
`endif

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bridge_state       <= BRIDGE_IDLE;
            dma_ready_out      <= 1'b0;
            r_tl_a_address_out <= {ADDR_WIDTH{1'b0}};
            r_tl_a_opcode_out  <= {OPCODE_WIDTH{1'b0}};
            r_tl_a_mask_out    <= {(DATA_WIDTH/8){1'b0}};
            r_tl_a_data_out    <= {DATA_WIDTH{1'b0}};
            r_tl_a_size_out    <= {SIZE_WIDTH{1'b0}};
            r_tl_a_valid_out   <= 1'b0;
            r_tl_a_source_out  <= {SOURCE_WIDTH{1'b0}};
            r_tl_a_param_out   <= {PARAM_WIDTH{1'b0}};
            r_dma_data_out     <= {DATA_WIDTH{1'b0}};
            r_dma_valid_out    <= 1'b0;
        end else begin
            dma_ready_out <= 1'b0;

            case (bridge_state)
                BRIDGE_IDLE: begin
                    dma_ready_out <= 1'b1;
                    r_tl_a_valid_out <= 1'b0;
                    r_dma_valid_out <= 1'b0;
                    if (dma_valid_in) begin
                        r_tl_a_address_out <= dma_address_in;
                        r_tl_a_data_out    <= dma_data_in;
                        r_tl_a_mask_out    <= dma_mask_in;
                        r_tl_a_size_out    <= dma_size_in;
                        r_tl_a_opcode_out  <= dma_opcode_in;
                        r_tl_a_source_out  <= dma_source_in;
                        r_tl_a_param_out   <= dma_param_in;
                        r_tl_a_valid_out   <= 1'b1;
                        bridge_state       <= WAIT_A;
                    end
                end

                WAIT_A: begin
                    r_tl_a_valid_out <= 1'b1;
                    if (tl_a_ready_in) begin
                        r_tl_a_valid_out <= 1'b0;
                        bridge_state <= WAIT_D;
                    end
                end

                WAIT_D: begin
                    if (tl_d_valid_in) begin
                        r_dma_data_out  <= tl_d_data_in;
                        r_dma_valid_out <= 1'b1;
                        bridge_state    <= DONE;
                    end
                end

                DONE: begin
                    r_dma_valid_out <= 1'b1;
                    if (dma_ready_in) begin
                        r_dma_valid_out <= 1'b0;
                        bridge_state <= BRIDGE_IDLE;
                    end
                end

                default: begin
                    bridge_state <= BRIDGE_IDLE;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
