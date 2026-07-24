`timescale 1ns/1ps
`default_nettype none
`include "./streamer_spi8_top.v"
`include "./dma_memc_to_xbar_m0_adapter.v"

//This takes the integrated DMA/SPI streamer system and adapts its 
//internal memc_* memory-side transactions into master-0 TileLink transactions 
//for the larger crossbar. It is the glue from the standalone streamer world 
//into the external TileLink-UH crossbar.

module spi8_to_xbar_adapter #(
    parameter DATA_WIDTH    = 64,
    parameter ADDR_WIDTH    = 64,
    parameter SOURCE_WIDTH  = 4,
    parameter SIZE_WIDTH    = 3,
    parameter OPCODE_WIDTH  = 3,
    parameter PARAM_WIDTH   = 3,
    parameter SINK_WIDTH    = 3,
    parameter BURST_MAX     = 16,
    parameter ADAPTER_DISPATCH_IDLE_CYCLES = 32,
    parameter FIFO_DEPTH    = 16,
    parameter MEM_BASE_ADDR = 64'h0000_0000_0000_0000,
    parameter DEPTH         = 512
)(
    input  wire                          clk,
    input  wire                          rst,

    // Streamer descriptor/control
    input  wire                          start,
    input  wire                          stream_type,
    input  wire [ADDR_WIDTH-1:0]         dma_mem_base_addr,
    input  wire [ADDR_WIDTH-1:0]         dma_peri_base_addr,
    input  wire [15:0]                   num_bytes,

    // Streamer status
    output wire                          busy,
    output wire                          done,

    // SPI TB ports
    output wire                          spi_wr_evt_valid_tb,
    output wire [47:0]                   spi_wr_evt_addr_tb,
    output wire [63:0]                   spi_wr_evt_data_tb,
    output wire                          spi_rd_req_valid_tb,
    output wire [47:0]                   spi_rd_req_addr_tb,
    input  wire                          spi_rd_rsp_valid_tb,
    input  wire [63:0]                   spi_rd_rsp_data_tb,

    // Xbar M0 ports (to be connected when Xbar top is instantiated later)
    output wire                          a_valid_in0,
    output wire [OPCODE_WIDTH-1:0]       a_opcode_in0,
    output wire [PARAM_WIDTH-1:0]        a_param_in0,
    output wire [ADDR_WIDTH-1:0]         a_address_in0,
    output wire [7:0]                    a_size_in0,
    output wire [(DATA_WIDTH/8)-1:0]     a_mask_in0,
    output wire [DATA_WIDTH-1:0]         a_data_in0,
    output wire [2:0]                    a_source_in0,
    input  wire                          a_ready0_tb,

    input  wire [OPCODE_WIDTH-1:0]       d_opcode0_tb,
    input  wire [PARAM_WIDTH-1:0]        d_param0_tb,
    input  wire [7:0]                    d_size0_tb,
    input  wire [SINK_WIDTH-1:0]         d_sink0_tb,
    input  wire [2:0]                    d_source0_tb,
    input  wire [DATA_WIDTH-1:0]         d_data0_tb,
    input  wire                          d_valid0_tb,
    input  wire                          d_ready0_tb,
    input  wire                          d_error0_tb,

    // Adapter burst completion visibility
    output wire                          memc_write_burst_done,
    output wire                          memc_read_burst_done
);
    // Internal memc boundary between streamer and adapter.
    wire [ADDR_WIDTH-1:0] memc_waddr;
    wire                  memc_wen;
    wire [DATA_WIDTH-1:0] memc_wdata;
    wire [ADDR_WIDTH-1:0] memc_raddr;
    wire                  memc_ren;
    wire [DATA_WIDTH-1:0] memc_rdata;
    wire                  memc_write_done;
    wire                  memc_acc_done;

    streamer_spi8_top #(
        .DATA_WIDTH    (DATA_WIDTH),
        .ADDR_WIDTH    (ADDR_WIDTH),
        .SOURCE_WIDTH  (SOURCE_WIDTH),
        .SIZE_WIDTH    (SIZE_WIDTH),
        .OPCODE_WIDTH  (OPCODE_WIDTH),
        .PARAM_WIDTH   (PARAM_WIDTH),
        .SINK_WIDTH    (SINK_WIDTH),
        .BURST_MAX     (BURST_MAX),
        .FIFO_DEPTH    (FIFO_DEPTH),
        .MEM_BASE_ADDR (MEM_BASE_ADDR),
        .DEPTH         (DEPTH)
    ) u_streamer_spi8_top (
        .clk                 (clk),
        .rst                 (rst),
        .start               (start),
        .stream_type         (stream_type),
        .dma_mem_base_addr   (dma_mem_base_addr),
        .dma_peri_base_addr  (dma_peri_base_addr),
        .num_bytes           (num_bytes),
        .busy                (busy),
        .done                (done),
        .memc_waddr          (memc_waddr),
        .memc_wen            (memc_wen),
        .memc_wdata          (memc_wdata),
        .memc_raddr          (memc_raddr),
        .memc_ren            (memc_ren),
        .memc_rdata          (memc_rdata),
        .memc_write_done     (memc_write_done),
        .memc_acc_done       (memc_acc_done),
        .spi_wr_evt_valid_tb (spi_wr_evt_valid_tb),
        .spi_wr_evt_addr_tb  (spi_wr_evt_addr_tb),
        .spi_wr_evt_data_tb  (spi_wr_evt_data_tb),
        .spi_rd_req_valid_tb (spi_rd_req_valid_tb),
        .spi_rd_req_addr_tb  (spi_rd_req_addr_tb),
        .spi_rd_rsp_valid_tb (spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data_tb  (spi_rd_rsp_data_tb)
    );

    dma_memc_to_xbar_m0_adapter #(
        .ADDR_WIDTH      (ADDR_WIDTH),
        .DATA_WIDTH      (DATA_WIDTH),
        .TL_OPCODE_WIDTH (OPCODE_WIDTH),
        .TL_PARAM_WIDTH  (PARAM_WIDTH),
        .TL_SIZE_WIDTH   (8),
        .TL_SOURCE_WIDTH (3),
        .TL_SINK_WIDTH   (SINK_WIDTH),
        .MAX_BURST_BEATS (BURST_MAX),
        .DISPATCH_IDLE_CYCLES(ADAPTER_DISPATCH_IDLE_CYCLES)
    ) u_dma_memc_to_xbar_m0_adapter (
        .m_clk                 (clk),
        .reset_m               (rst),
        .memc_waddr            (memc_waddr),
        .memc_wen              (memc_wen),
        .memc_wdata            (memc_wdata),
        .memc_raddr            (memc_raddr),
        .memc_ren              (memc_ren),
        .memc_rdata            (memc_rdata),
        .memc_write_done       (memc_write_done),
        .memc_acc_done         (memc_acc_done),
        .memc_write_burst_done (memc_write_burst_done),
        .memc_read_burst_done  (memc_read_burst_done),
        .a_valid_in0           (a_valid_in0),
        .a_opcode_in0          (a_opcode_in0),
        .a_param_in0           (a_param_in0),
        .a_address_in0         (a_address_in0),
        .a_size_in0            (a_size_in0),
        .a_mask_in0            (a_mask_in0),
        .a_data_in0            (a_data_in0),
        .a_source_in0          (a_source_in0),
        .a_ready0_tb           (a_ready0_tb),
        .d_opcode0_tb          (d_opcode0_tb),
        .d_param0_tb           (d_param0_tb),
        .d_size0_tb            (d_size0_tb),
        .d_sink0_tb            (d_sink0_tb),
        .d_source0_tb          (d_source0_tb),
        .d_data0_tb            (d_data0_tb),
        .d_valid0_tb           (d_valid0_tb),
        .d_ready0_tb           (d_ready0_tb),
        .d_error0_tb           (d_error0_tb)
    );

endmodule

`default_nettype wire
