// ============================================================================
// bridge_spi8_top
// ----------------------------------------------------------------------------
// - Connects dma_tilelink_ul_bridge (master TL-UL A/D) into tilelink_wrapper_top
// - A-channel: bridge -> wrapper a_*_in
// - D-channel: wrapper d_*_tb -> bridge D inputs
// - Exposes wrapper a_**_tb taps as top-level ports (per requirement)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
////This connects the DMA-side TL-UL bridge into tilelink_wrapper_top. 
//The bridge creates TL-UL A/D traffic, and the wrapper gives SPI-oriented 
//slave behavior plus TB visibility of A/D activity.

module bridge_spi8_top #(
    // Bridge / TL widths
    parameter DATA_WIDTH    = 64,
    parameter ADDR_WIDTH    = 64,
    parameter SOURCE_WIDTH  = 4,
    parameter SIZE_WIDTH    = 3,
    parameter OPCODE_WIDTH  = 3,
    parameter PARAM_WIDTH   = 3,
    parameter SINK_WIDTH    = 3,

    // Wrapper (slave memory) params
    parameter MEM_BASE_ADDR = 64'h0000_0000_0000_0000,
    parameter DEPTH         = 512
)(
    input  wire                          clk,
    input  wire                          rst,           // active-high

    // ---------------- DMA-side interface (exposed) ----------------
    input  wire [ADDR_WIDTH-1:0]         dma_address_in,
    input  wire [DATA_WIDTH-1:0]         dma_data_in,
    input  wire [DATA_WIDTH/8-1:0]       dma_mask_in,
    input  wire [SIZE_WIDTH-1:0]         dma_size_in,
    input  wire [SOURCE_WIDTH-1:0]       dma_source_in,
    input  wire [OPCODE_WIDTH-1:0]       dma_opcode_in,
    input  wire [PARAM_WIDTH-1:0]        dma_param_in,
    input  wire                          dma_valid_in,
    output wire                          dma_ready_out,

    output wire [DATA_WIDTH-1:0]         dma_data_out,
    output wire                          dma_valid_out,
    input  wire                          dma_ready_in,

    // -------- Expose wrapper's A-channel testbench taps ----------
    output wire                          a_valid_tb,
    output wire [OPCODE_WIDTH-1:0]       a_opcode_tb,
    output wire [PARAM_WIDTH-1:0]        a_param_tb,
    output wire [ADDR_WIDTH-1:0]         a_address_tb,
    output wire [SIZE_WIDTH-1:0]         a_size_tb,
    output wire [DATA_WIDTH/8-1:0]       a_mask_tb,
    output wire [DATA_WIDTH-1:0]         a_data_tb,
    output wire [SOURCE_WIDTH-1:0]       a_source_tb,
    output wire                          a_ready_tb,
    output wire                          spi_wr_evt_valid_tb,
    output wire [47:0]                   spi_wr_evt_addr_tb,
    output wire [63:0]                   spi_wr_evt_data_tb,
    output wire                          spi_rd_req_valid_tb,
    output wire [47:0]                   spi_rd_req_addr_tb,
    input  wire                          spi_rd_rsp_valid_tb,
    input  wire [63:0]                   spi_rd_rsp_data_tb
);

    // ===================== TL-UL wires (bridge side) ======================
    // A-channel (bridge -> wrapper a_*_in)
    wire [ADDR_WIDTH-1:0]     tl_a_address;
    wire [OPCODE_WIDTH-1:0]   tl_a_opcode;
    wire [DATA_WIDTH/8-1:0]   tl_a_mask;
    wire [DATA_WIDTH-1:0]     tl_a_data;
    wire [SIZE_WIDTH-1:0]     tl_a_size;
    wire                      tl_a_valid;
    wire [SOURCE_WIDTH-1:0]   tl_a_source;
    wire [PARAM_WIDTH-1:0]    tl_a_param;
    wire                      tl_a_ready;    // from wrapper (a_ready_tb)

    // D-channel (wrapper taps -> bridge)
    wire [DATA_WIDTH-1:0]     tl_d_data;
    wire                      tl_d_valid;
    wire [SOURCE_WIDTH-1:0]   tl_d_source;
    wire [SIZE_WIDTH-1:0]     tl_d_size;
    wire [OPCODE_WIDTH-1:0]   tl_d_opcode;
    wire                      tl_d_error;
    wire [SINK_WIDTH-1:0]     tl_d_sink;
    wire [PARAM_WIDTH-1:0]    tl_d_param;
    wire                      tl_d_ready;    // bridge drives

    // Wrapper also exposes D ready as a tap; capture but not used here
    //wire                      d_ready_tb_unused;

    // ======================= Bridge instance ==============================
    dma_tilelink_ul_bridge #(
        .DATA_WIDTH    (DATA_WIDTH),
        .ADDR_WIDTH    (ADDR_WIDTH),
        .SOURCE_WIDTH  (SOURCE_WIDTH),
        .SIZE_WIDTH    (SIZE_WIDTH),
        .OPCODE_WIDTH  (OPCODE_WIDTH),
        .PARAM_WIDTH   (PARAM_WIDTH),
        .SINK_WIDTH    (SINK_WIDTH)
    ) u_bridge (
        .clk                (clk),
        .rst_n              (~rst),

        // DMA request side
        .dma_address_in     (dma_address_in),
        .dma_data_in        (dma_data_in),
        .dma_mask_in        (dma_mask_in),
        .dma_size_in        (dma_size_in),
        .dma_source_in      (dma_source_in),
        .dma_opcode_in      (dma_opcode_in),
        .dma_param_in       (dma_param_in),
        .dma_valid_in       (dma_valid_in),
        .dma_ready_out      (dma_ready_out),

        // TL-UL A channel (master out)
        .tl_a_address_out   (tl_a_address),
        .tl_a_opcode_out    (tl_a_opcode),
        .tl_a_mask_out      (tl_a_mask),
        .tl_a_data_out      (tl_a_data),
        .tl_a_size_out      (tl_a_size),
        .tl_a_valid_out     (tl_a_valid),
        .tl_a_source_out    (tl_a_source),
        .tl_a_param_out     (tl_a_param),
        .tl_a_ready_in      (tl_a_ready),

        // TL-UL D channel (fabric -> bridge)
        .tl_d_data_in       (tl_d_data),
        .tl_d_valid_in      (tl_d_valid),
        .tl_d_source_in     (tl_d_source),
        .tl_d_size_in       (tl_d_size),
        .tl_d_opcode_in     (tl_d_opcode),
        .tl_d_error_in      (tl_d_error),
        .tl_d_sink_in       (tl_d_sink),
        .tl_d_param_in      (tl_d_param),
        .tl_d_ready_out     (tl_d_ready),

        // DMA response side
        .dma_data_out       (dma_data_out),
        .dma_valid_out      (dma_valid_out),
        .dma_ready_in       (dma_ready_in)
    );

    // ======================= Wrapper instance =============================
    // NOTE: force TL_* widths to match bridge by parameter tying.
    tilelink_wrapper_top #(
        .TL_ADDR_WIDTH    (ADDR_WIDTH),
        .TL_DATA_WIDTH    (DATA_WIDTH),
        .TL_SOURCE_WIDTH  (SOURCE_WIDTH),
        .TL_SINK_WIDTH    (SINK_WIDTH),
        .TL_OPCODE_WIDTH  (OPCODE_WIDTH),
        .TL_PARAM_WIDTH   (PARAM_WIDTH),
        .TL_SIZE_WIDTH    (SIZE_WIDTH),      // <— match bridge SIZE width
        .MEM_BASE_ADDR    (MEM_BASE_ADDR),
        .DEPTH            (DEPTH)
    ) u_wrapper (
        .clk           (clk),
        .rst           (rst),

        // Drive the wrapper (master) from the bridge's A channel
        .a_valid_in    (tl_a_valid),
        .a_opcode_in   (tl_a_opcode),
        .a_param_in    (tl_a_param),
        .a_address_in  (tl_a_address),
        .a_size_in     (tl_a_size),
        .a_mask_in     (tl_a_mask),
        .a_data_in     (tl_a_data),
        .a_source_in   (tl_a_source),

        // Expose A-channel taps to the top ports (as required)
        .a_valid_tb    (a_valid_tb),
        .a_opcode_tb   (a_opcode_tb),
        .a_param_tb    (a_param_tb),
        .a_address_tb  (a_address_tb),
        .a_size_tb     (a_size_tb),
        .a_mask_tb     (a_mask_tb),
        .a_data_tb     (a_data_tb),
        .a_source_tb   (a_source_tb),
        .a_ready_tb    (a_ready_tb),

        // D-channel taps from wrapper — feed the bridge D inputs
        .d_valid_tb    (tl_d_valid),
        .d_ready_tb    (dma_ready_in),
        .d_opcode_tb   (tl_d_opcode),
        .d_param_tb    (tl_d_param),
        .d_size_tb     (tl_d_size),
        .d_sink_tb     (tl_d_sink),
        .d_source_tb   (tl_d_source),
        .d_data_tb     (tl_d_data),
        .d_error_tb    (tl_d_error),
        .spi_wr_evt_valid_tb(spi_wr_evt_valid_tb),
        .spi_wr_evt_addr_tb(spi_wr_evt_addr_tb),
        .spi_wr_evt_data_tb(spi_wr_evt_data_tb),
        .spi_rd_req_valid_tb(spi_rd_req_valid_tb),
        .spi_rd_req_addr_tb(spi_rd_req_addr_tb),
        .spi_rd_rsp_valid_tb(spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data_tb(spi_rd_rsp_data_tb)
    );

    // The bridge expects TL A ready from the fabric; use wrapper's a_ready_tb tap.
    assign tl_a_ready = a_ready_tb;

endmodule

`default_nettype wire
