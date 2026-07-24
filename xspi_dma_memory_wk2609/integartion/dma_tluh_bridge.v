`timescale 1ns/1ps
`default_nettype none

// Simple single-beat bridge:
//   mem_* (generic DMA-style interface)
// <-> TileLink-UH-like A/D channels (master side)
//
// You can drive mem_* from a TB or another module, and connect the
// TL ports to a TB "fake slave" or a real TL slave block.

////BRIDGE BETWEEN DMA AND TLUH, WHICH TRANSLATES DMA SIGNALS TO TLUH SIGNALS 
module dma_tluh_bridge #(

    parameter DATA_WIDTH    = 64,
    parameter ADDR_WIDTH    = 64,

    // TL side
    parameter TL_ADDR_WIDTH   = ADDR_WIDTH,
    parameter TL_DATA_WIDTH   = DATA_WIDTH,
    parameter TL_STRB_WIDTH   = TL_DATA_WIDTH / 8,
    parameter TL_SOURCE_WIDTH = 3,
    parameter TL_SINK_WIDTH   = 3,
    parameter TL_OPCODE_WIDTH = 3,
    parameter TL_PARAM_WIDTH  = 3,
    parameter TL_SIZE_WIDTH   = 8
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // ---------------- mem_* side (master: e.g. DMA, CPU) ----------------
    input  wire                      mem_req,
    input  wire                      mem_wr_en,      // 1 = write, 0 = read
    input  wire [ADDR_WIDTH-1:0]     mem_addr,
    input  wire [DATA_WIDTH-1:0]     mem_wdata,
    output wire [DATA_WIDTH-1:0]     mem_rdata,
    output wire                      mem_ack,        // 1-cycle pulse when beat completes

    // ---------------- TL-UH-like side (bridge is master) -----------------
    // A-channel: bridge → slave (TB or real TL slave)
    output reg                        tl_a_valid,
    output reg  [TL_OPCODE_WIDTH-1:0] tl_a_opcode,
    output reg  [TL_PARAM_WIDTH-1:0]  tl_a_param,
    output reg  [TL_ADDR_WIDTH-1:0]   tl_a_address,
    output reg  [TL_SIZE_WIDTH-1:0]   tl_a_size,
    output reg  [TL_STRB_WIDTH-1:0]   tl_a_mask,
    output reg  [TL_DATA_WIDTH-1:0]   tl_a_data,
    output reg  [TL_SOURCE_WIDTH-1:0] tl_a_source,
    output reg                        tl_a_corrupt,
    input  wire                       tl_a_ready,

    // D-channel: slave → bridge
    input  wire                       tl_d_valid,
    output wire                       tl_d_ready,    // bridge always ready
    input  wire [TL_OPCODE_WIDTH-1:0] tl_d_opcode,
    input  wire [TL_PARAM_WIDTH-1:0]  tl_d_param,
    input  wire [TL_SIZE_WIDTH-1:0]   tl_d_size,
    input  wire [TL_SINK_WIDTH-1:0]   tl_d_sink,
    input  wire [TL_SOURCE_WIDTH-1:0] tl_d_source,
    input  wire [TL_DATA_WIDTH-1:0]   tl_d_data,
    input  wire                       tl_d_error

);

    // --------------------------------------------------------------------
    // Constants / encodings (match your TLUH spec)
    // --------------------------------------------------------------------
    localparam [TL_OPCODE_WIDTH-1:0] TL_PUTFULL = 3'b000; // write
    localparam [TL_OPCODE_WIDTH-1:0] TL_GET     = 3'b100; // read

    localparam [TL_SIZE_WIDTH-1:0] TL_SIZE_FULL =
        TL_SIZE_WIDTH'($clog2(TL_DATA_WIDTH/8));          // full beat

    // Ready only when waiting for a response beat.
    assign tl_d_ready = (state == ST_WAIT_D);

    // --------------------------------------------------------------------
    // Internal state
    // --------------------------------------------------------------------
    localparam ST_IDLE   = 2'd0;
    localparam ST_SEND_A = 2'd1;
    localparam ST_WAIT_D = 2'd2;

    reg [1:0]            state;
    reg                  lat_wr_en;
    reg [ADDR_WIDTH-1:0] lat_addr;
    reg [DATA_WIDTH-1:0] lat_wdata;

    reg [DATA_WIDTH-1:0] mem_rdata_r;
    reg                  mem_ack_r;

    // Extra internal debug probes
    reg [DATA_WIDTH-1:0] d_data_last;   // last D-channel data

    // Convenient internal “probe” wires (hierarchical)
    wire [1:0]            dbg_state       = state;
    wire                  dbg_is_write    = lat_wr_en;
    wire                  dbg_in_flight   = (state != ST_IDLE);
    wire [ADDR_WIDTH-1:0] dbg_addr        = lat_addr;
    wire [DATA_WIDTH-1:0] dbg_wdata       = lat_wdata;
    wire [DATA_WIDTH-1:0] dbg_d_data_last = d_data_last;

    assign mem_rdata = mem_rdata_r;
    assign mem_ack   = mem_ack_r;

`ifdef RTL_TRACE
    integer rtl_cyc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rtl_cyc <= 0;
        else rtl_cyc <= rtl_cyc + 1;
    end
`endif

    // --------------------------------------------------------------------
    // FSM: mem_* request -> TL A -> TL D -> mem_ack/mem_rdata
    // --------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;

            tl_a_valid   <= 1'b0;
            tl_a_opcode  <= {TL_OPCODE_WIDTH{1'b0}};
            tl_a_param   <= {TL_PARAM_WIDTH{1'b0}};
            tl_a_address <= {TL_ADDR_WIDTH{1'b0}};
            tl_a_size    <= {TL_SIZE_WIDTH{1'b0}};
            tl_a_mask    <= {TL_STRB_WIDTH{1'b0}};
            tl_a_data    <= {TL_DATA_WIDTH{1'b0}};
            tl_a_source  <= {TL_SOURCE_WIDTH{1'b0}};
            tl_a_corrupt <= 1'b0;

            lat_wr_en    <= 1'b0;
            lat_addr     <= {ADDR_WIDTH{1'b0}};
            lat_wdata    <= {DATA_WIDTH{1'b0}};

            mem_rdata_r  <= {DATA_WIDTH{1'b0}};
            mem_ack_r    <= 1'b0;

            d_data_last  <= {DATA_WIDTH{1'b0}};
        end
        else begin
            // default: ack is a single-cycle pulse
            mem_ack_r <= 1'b0;

            case (state)
                // --------------------------------------------------------
                // IDLE: wait for mem_* transaction from upstream master
                // --------------------------------------------------------
                ST_IDLE: begin
                    tl_a_valid <= 1'b0;

                    // Avoid re-latching the just-completed beat while mem_ack is
                    // still observed high for one cycle by upstream logic.
                    if (mem_req && !mem_ack_r) begin
`ifdef RTL_TRACE
                        $display("RTR C%0d BRG IDLE_LATCH mem_req=%0b wr=%0b mem_addr=%h mem_wdata=%h",
                                 rtl_cyc, mem_req, mem_wr_en, mem_addr, mem_wdata);
`endif
                        // Latch request
                        lat_wr_en <= mem_wr_en;
                        lat_addr  <= mem_addr;
                        lat_wdata <= mem_wdata;

                        // Drive A-channel
                        tl_a_valid   <= 1'b1;
                        tl_a_opcode  <= mem_wr_en ? TL_PUTFULL : TL_GET;
                        tl_a_param   <= {TL_PARAM_WIDTH{1'b0}};
                        tl_a_address <= mem_addr;
                        tl_a_size    <= TL_SIZE_FULL;
                        tl_a_mask    <= {TL_STRB_WIDTH{1'b1}}; // full beat
                        tl_a_data    <= mem_wdata;             // for writes
                        tl_a_source  <= {TL_SOURCE_WIDTH{1'b0}};
                        tl_a_corrupt <= 1'b0;

                        state        <= ST_SEND_A;
                    end
                end

                // --------------------------------------------------------
                // SEND_A: wait for TL slave to accept A-beat
                // --------------------------------------------------------
                ST_SEND_A: begin
                    if (tl_a_valid && tl_a_ready) begin
`ifdef RTL_TRACE
                        $display("RTR C%0d BRG A_ACCEPT op=%0h addr=%h data=%h lat_addr=%h",
                                 rtl_cyc, tl_a_opcode, tl_a_address, tl_a_data, lat_addr);
`endif
                        tl_a_valid <= 1'b0;
                        state <= ST_WAIT_D;
                    end
                end

                // --------------------------------------------------------
                // WAIT_D: wait for TL response; drive mem_* completion
                // --------------------------------------------------------
                ST_WAIT_D: begin
    if (tl_d_valid && tl_d_ready) begin
`ifdef RTL_TRACE
        $display("RTR C%0d BRG D_RECV lat_wr=%0b lat_addr=%h d_op=%0h d_data=%h -> mem_ack=1",
                 rtl_cyc, lat_wr_en, lat_addr, tl_d_opcode, tl_d_data);
`endif
        d_data_last <= tl_d_data;

        if (!lat_wr_en) begin
            // READ: capture returned data
            mem_rdata_r <= tl_d_data;
        end else begin
            // WRITE: optionally clear read data for clarity
            mem_rdata_r <= {DATA_WIDTH{1'b0}};
        end

        mem_ack_r <= 1'b1;
        state     <= ST_IDLE;
    end
end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
