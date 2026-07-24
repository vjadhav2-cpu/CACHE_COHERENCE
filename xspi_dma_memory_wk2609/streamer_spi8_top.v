`timescale 1ns/1ps
`default_nettype none
`include "./tluh_bridge_into_wrapper_top.v"
`include "./bridge_spi8_top.v"

////This is the main dual-path integration around the DMA streamer. 
//It instantiates the actual DMA engine, sends the DMA memory path through the TLUH
 //memory wrapper, and sends the peripheral path through the TLUL/SPI bridge.

module streamer_spi8_top #(
    parameter DATA_WIDTH    = 64,
    parameter ADDR_WIDTH    = 64,
    parameter SOURCE_WIDTH  = 4,
    parameter SIZE_WIDTH    = 3,
    parameter OPCODE_WIDTH  = 3,
    parameter PARAM_WIDTH   = 3,
    parameter SINK_WIDTH    = 3,
    parameter BURST_MAX     = 16,
    parameter FIFO_DEPTH    = 16,
    parameter MEM_BASE_ADDR = 64'h0000_0000_0000_0000,
    parameter DEPTH         = 512
)(
    input  wire                          clk,
    input  wire                          rst,

    input  wire                          start,
    input  wire                          stream_type,
    input  wire [ADDR_WIDTH-1:0]         dma_mem_base_addr,
    input  wire [ADDR_WIDTH-1:0]         dma_peri_base_addr,
    input  wire [15:0]                   num_bytes,

    output wire                          busy,
    output wire                          done,

    // Memory-controller side (from TileLink-UH wrapper slave)
    output wire [ADDR_WIDTH-1:0]         memc_waddr,
    output wire                          memc_wen,
    output wire [DATA_WIDTH-1:0]         memc_wdata,
    output wire [ADDR_WIDTH-1:0]         memc_raddr,
    output wire                          memc_ren,
    input  wire [DATA_WIDTH-1:0]         memc_rdata,
    input  wire                          memc_write_done,
    input  wire                          memc_acc_done,

    // SPI/TLUL-side TB taps
    output wire                          spi_wr_evt_valid_tb,
    output wire [47:0]                   spi_wr_evt_addr_tb,
    output wire [63:0]                   spi_wr_evt_data_tb,
    output wire                          spi_rd_req_valid_tb,
    output wire [47:0]                   spi_rd_req_addr_tb,
    input  wire                          spi_rd_rsp_valid_tb,
    input  wire [63:0]                   spi_rd_rsp_data_tb
);

    wire                  peri_req;
    wire                  peri_wr_en;
    wire [ADDR_WIDTH-1:0] peri_addr;
    wire [DATA_WIDTH-1:0] peri_wdata;
    wire [DATA_WIDTH-1:0] peri_rdata;
    wire                  peri_ack;

    wire                  mem_req;
    wire                  mem_wr_en;
    wire [ADDR_WIDTH-1:0] mem_addr;
    wire [DATA_WIDTH-1:0] mem_wdata;
    wire [DATA_WIDTH-1:0] mem_rdata;
    wire                  mem_ack;

    wire [ADDR_WIDTH-1:0]   dma_address_in;
    wire [DATA_WIDTH-1:0]   dma_data_in;
    wire [DATA_WIDTH/8-1:0] dma_mask_in;
    wire [SIZE_WIDTH-1:0]   dma_size_in;
    wire [SOURCE_WIDTH-1:0] dma_source_in;
    wire [OPCODE_WIDTH-1:0] dma_opcode_in;
    wire [PARAM_WIDTH-1:0]  dma_param_in;
    wire                    dma_valid_in;
    wire                    dma_ready_out;

    wire [DATA_WIDTH-1:0]   dma_data_out;
    wire                    dma_valid_out;
    wire                    peri_rsp_ready_int;

    wire                    tlul_a_valid_tb_unused;
    wire [OPCODE_WIDTH-1:0] tlul_a_opcode_tb_unused;
    wire [PARAM_WIDTH-1:0]  tlul_a_param_tb_unused;
    wire [ADDR_WIDTH-1:0]   tlul_a_address_tb_unused;
    wire [SIZE_WIDTH-1:0]   tlul_a_size_tb_unused;
    wire [DATA_WIDTH/8-1:0] tlul_a_mask_tb_unused;
    wire [DATA_WIDTH-1:0]   tlul_a_data_tb_unused;
    wire [SOURCE_WIDTH-1:0] tlul_a_source_tb_unused;
    wire                    tlul_a_ready_tb_unused;

    dma_flat_full_duplex_top #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .BURST_MAX  (BURST_MAX),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) u_stream (
        .clk            (clk),
        .rst            (rst),
        .start          (start),
        .stream_type    (stream_type),
        .mem_base_addr  (dma_mem_base_addr),
        .peri_base_addr (dma_peri_base_addr),
        .num_bytes      (num_bytes),
        .mem_req        (mem_req),
        .mem_wr_en      (mem_wr_en),
        .mem_addr       (mem_addr),
        .mem_wdata      (mem_wdata),
        .mem_rdata      (mem_rdata),
        .mem_ack        (mem_ack),
        .peri_req       (peri_req),
        .peri_wr_en     (peri_wr_en),
        .peri_addr      (peri_addr),
        .peri_wdata     (peri_wdata),
        .peri_rdata     (peri_rdata),
        .peri_ack       (peri_ack),
        .busy           (busy),
        .done           (done)
    );

    tluh_bridge_into_wrapper_top #(
        .ADDR_WIDTH      (ADDR_WIDTH),
        .DATA_WIDTH      (DATA_WIDTH),
        .TL_ADDR_WIDTH   (ADDR_WIDTH),
        .TL_DATA_WIDTH   (DATA_WIDTH),
        .TL_STRB_WIDTH   (DATA_WIDTH/8),
        .TL_SOURCE_WIDTH (3),
        .TL_SINK_WIDTH   (3),
        .TL_OPCODE_WIDTH (OPCODE_WIDTH),
        .TL_PARAM_WIDTH  (PARAM_WIDTH),
        .TL_SIZE_WIDTH   (8)
    ) u_mem_path (
        .clk                 (clk),
        .rst                 (rst),
        .mem_req             (mem_req),
        .mem_wr_en           (mem_wr_en),
        .mem_addr            (mem_addr),
        .mem_wdata           (mem_wdata),
        .mem_rdata           (mem_rdata),
        .mem_ack             (mem_ack),
        .memc_waddr          (memc_waddr),
        .memc_wen            (memc_wen),
        .memc_wdata          (memc_wdata),
        .memc_raddr          (memc_raddr),
        .memc_ren            (memc_ren),
        .memc_rdata          (memc_rdata),
        .memc_write_done     (memc_write_done),
        .memc_acc_done       (memc_acc_done),
        .tb_override_a_valid (1'b0),
        .tb_override_a_opcode({OPCODE_WIDTH{1'b0}}),
        .tb_override_a_param ({PARAM_WIDTH{1'b0}}),
        .tb_override_a_address({ADDR_WIDTH{1'b0}}),
        .tb_override_a_size  (8'd0),
        .tb_override_a_mask  ({(DATA_WIDTH/8){1'b0}}),
        .tb_override_a_data  ({DATA_WIDTH{1'b0}}),
        .tb_override_a_source(3'd0),
        .tb_override_a_corrupt(1'b0),
        .tb_drive_d_ready    (1'b0),
        .tb_d_ready_override (1'b1)
    );

    assign peri_rsp_ready_int = 1'b1;

    bridge_spi8_top #(
        .DATA_WIDTH    (DATA_WIDTH),
        .ADDR_WIDTH    (ADDR_WIDTH),
        .SOURCE_WIDTH  (SOURCE_WIDTH),
        .SIZE_WIDTH    (SIZE_WIDTH),
        .OPCODE_WIDTH  (OPCODE_WIDTH),
        .PARAM_WIDTH   (PARAM_WIDTH),
        .SINK_WIDTH    (SINK_WIDTH),
        .MEM_BASE_ADDR (MEM_BASE_ADDR),
        .DEPTH         (DEPTH)
    ) u_bridge (
        .clk                (clk),
        .rst                (rst),
        .dma_address_in     (dma_address_in),
        .dma_data_in        (dma_data_in),
        .dma_mask_in        (dma_mask_in),
        .dma_size_in        (dma_size_in),
        .dma_source_in      (dma_source_in),
        .dma_opcode_in      (dma_opcode_in),
        .dma_param_in       (dma_param_in),
        .dma_valid_in       (dma_valid_in),
        .dma_ready_out      (dma_ready_out),
        .dma_data_out       (dma_data_out),
        .dma_valid_out      (dma_valid_out),
        .dma_ready_in       (peri_rsp_ready_int),
        .a_valid_tb         (tlul_a_valid_tb_unused),
        .a_opcode_tb        (tlul_a_opcode_tb_unused),
        .a_param_tb         (tlul_a_param_tb_unused),
        .a_address_tb       (tlul_a_address_tb_unused),
        .a_size_tb          (tlul_a_size_tb_unused),
        .a_mask_tb          (tlul_a_mask_tb_unused),
        .a_data_tb          (tlul_a_data_tb_unused),
        .a_source_tb        (tlul_a_source_tb_unused),
        .a_ready_tb         (tlul_a_ready_tb_unused),
        .spi_wr_evt_valid_tb(spi_wr_evt_valid_tb),
        .spi_wr_evt_addr_tb (spi_wr_evt_addr_tb),
        .spi_wr_evt_data_tb (spi_wr_evt_data_tb),
        .spi_rd_req_valid_tb(spi_rd_req_valid_tb),
        .spi_rd_req_addr_tb (spi_rd_req_addr_tb),
        .spi_rd_rsp_valid_tb(spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data_tb (spi_rd_rsp_data_tb)
    );

    localparam [SIZE_WIDTH-1:0]   TL_SIZE_8B  = 3'd3;
    localparam [OPCODE_WIDTH-1:0] TL_PUT_FULL = 3'd0;
    localparam [OPCODE_WIDTH-1:0] TL_GET      = 3'd4;

    assign dma_address_in = peri_addr;
    assign dma_data_in    = peri_wdata;
    assign dma_mask_in    = {(DATA_WIDTH/8){1'b1}};
    assign dma_size_in    = TL_SIZE_8B;
    assign dma_source_in  = {SOURCE_WIDTH{1'b0}};
    assign dma_param_in   = {PARAM_WIDTH{1'b0}};
    assign dma_opcode_in  = peri_wr_en ? TL_PUT_FULL : TL_GET;
    assign dma_valid_in   = peri_req;

    assign peri_ack       = dma_valid_out;
    assign peri_rdata     = dma_data_out;

endmodule

`default_nettype wire
