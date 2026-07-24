`timescale 1ns/1ps
`default_nettype none

// Bridges the DMA core's mem_* bus (dma_flat_full_duplex_top.v) onto one
// l1_tilelink_adapter instance's uncached Get/Put path
// (STATE_UNCACHED_REQ_SEND -> STATE_UNCACHED_RSP_WAIT). Deliberately does
// NOT use the cached Acquire/Grant/Release path -- no line is ever held,
// so this bridge is never a legitimate probe target and drives no
// probe_ack_from_l1_* response. That is a documented v1 assumption: it
// holds only if the far-side directory (tlc_slave_model.v) grants uncached
// Get/Put requests without tracking this port as an owner. See
// docs/microarchitecture/DMA_to_TLC_Crossbar_Adapter_Plan.md Sec.2 for why
// tlc_slave_model.v does not do that today -- that is a separately-tracked
// fix, not this bridge's concern.
//as
module dma_tlc_uncached_bridge #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 64
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // DMA core mem_* bus (dma_flat_full_duplex_top.v) -- level-held
    // request, single-cycle ack. mem_req/mem_wr_en/mem_addr/mem_wdata are
    // driven by the DMA and held stable for the whole request; mem_ack
    // pulses exactly one cycle to signal completion (matches
    // dma_streamer's `ack_qual = bus_ack & bus_req`, sampled only while
    // its own STATE_REQ is held -- no separate accept/ready phase needed
    // on this leg).
    input  wire                   mem_req,
    input  wire                   mem_wr_en,
    input  wire [ADDR_WIDTH-1:0]  mem_addr,
    input  wire [DATA_WIDTH-1:0]  mem_wdata,
    output wire [DATA_WIDTH-1:0]  mem_rdata,
    output reg                    mem_ack,

    // l1_tilelink_adapter.v L1-facing interface (same port names as
    // tlc_xbar_top.v's existing, currently-unused Master-2 port
    // m2_req_*/m2_data_*, so this module drops onto it with no renaming).
    output reg                    l1_request_valid,
    output reg  [31:0]            l1_request_addr,
    output reg  [2:0]             l1_request_type,
    output reg  [255:0]           l1_request_data,
    output reg  [2:0]             l1_request_permissions,
    input  wire                   l1_request_ready,

    input  wire                   data_to_l1_valid,
    input  wire [255:0]           data_to_l1_data,
    input  wire                   data_to_l1_error
);

    `include "tlc64b2M_params.v"

    localparam [1:0] BRIDGE_IDLE       = 2'd0,
                      WAIT_L1_READY    = 2'd1,
                      WAIT_L1_RESPONSE = 2'd2,
                      DONE             = 2'd3;

    reg [1:0] bridge_state;
    reg       r_is_write;
    reg [DATA_WIDTH-1:0] r_mem_rdata;

    assign mem_rdata = r_mem_rdata;

`ifndef SYNTHESIS
    reg [2:0]   req_type_q;
    reg [31:0]  req_addr_q;
    reg [255:0] req_data_q;
    reg         req_wait_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_wait_q <= 1'b0;
            req_type_q <= 3'b0;
            req_addr_q <= 32'b0;
            req_data_q <= 256'b0;
        end else begin
            if (l1_request_valid && !l1_request_ready) begin
                if (req_wait_q) begin
                    if ((l1_request_type !== req_type_q) ||
                        (l1_request_addr !== req_addr_q) ||
                        (l1_request_data !== req_data_q)) begin
                        $error("[BR_ASSERT] l1_request payload changed while waiting for ready");
                    end
                end else begin
                    req_type_q <= l1_request_type;
                    req_addr_q <= l1_request_addr;
                    req_data_q <= l1_request_data;
                end
                req_wait_q <= 1'b1;
            end else begin
                req_wait_q <= 1'b0;
            end

            if (rst_n && data_to_l1_valid && data_to_l1_error)
                $error("[BR_ASSERT] TLC uncached response error observed");
        end
    end
`endif

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bridge_state            <= BRIDGE_IDLE;
            mem_ack                 <= 1'b0;
            r_is_write               <= 1'b0;
            r_mem_rdata              <= {DATA_WIDTH{1'b0}};
            l1_request_valid         <= 1'b0;
            l1_request_addr          <= 32'b0;
            l1_request_type          <= 3'b0;
            l1_request_data          <= 256'b0;
            l1_request_permissions   <= 3'b0;
        end else begin
            mem_ack          <= 1'b0;
            l1_request_valid <= 1'b0;

            case (bridge_state)
                BRIDGE_IDLE: begin
                    if (mem_req) begin
                        l1_request_addr        <= mem_addr[31:0];
                        l1_request_type        <= mem_wr_en ? L1_REQ_UNCACHED_WRITE
                                                             : L1_REQ_UNCACHED_READ;
                        l1_request_data        <= {{(256-DATA_WIDTH){1'b0}}, mem_wdata};
                        l1_request_permissions <= 3'b000;  // unused by the uncached path
                        l1_request_valid       <= 1'b1;
                        r_is_write              <= mem_wr_en;
                        bridge_state            <= WAIT_L1_READY;
                    end
                end

                WAIT_L1_READY: begin
                    l1_request_valid <= 1'b1;
                    if (l1_request_ready) begin
                        l1_request_valid <= 1'b0;
                        bridge_state     <= WAIT_L1_RESPONSE;
                    end
                end

                // Read completion is signaled by data_to_l1_valid
                // (AccessAckData). Write completion is NOT -- a plain
                // AccessAck never pulses data_to_l1_valid
                // (l1_tilelink_adapter.v STATE_UNCACHED_RSP_WAIT only sets
                // it for the *_ACK_DATA opcode). The only observable write
                // completion signal is the adapter's own FSM returning to
                // its idle state, which is exactly when l1_request_ready
                // reasserts. Polling that level (not an edge) is safe here
                // because this bridge is the sole driver of the adapter's
                // request port and never has more than one request
                // in flight.
                WAIT_L1_RESPONSE: begin
                    if (r_is_write) begin
                        if (l1_request_ready) begin
                            bridge_state <= DONE;
                        end
                    end else begin
                        if (data_to_l1_valid) begin
                            r_mem_rdata  <= data_to_l1_data[DATA_WIDTH-1:0];
                            bridge_state <= DONE;
                        end
                    end
                end

                DONE: begin
                    mem_ack      <= 1'b1;
                    bridge_state <= BRIDGE_IDLE;
                end

                default: begin
                    bridge_state <= BRIDGE_IDLE;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
