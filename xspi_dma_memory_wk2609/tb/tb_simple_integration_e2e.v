`timescale 1ns/1ps
`default_nettype none

`include "./tb_simple_integration.v"
`include "./spi_dma_xbar_memory_system.v"

module tb_simple_integration_e2e;
    localparam DATA_WIDTH   = 64;
    localparam ADDR_WIDTH   = 64;
    localparam SOURCE_WIDTH = 4;
    localparam SIZE_WIDTH   = 3;
    localparam OPCODE_WIDTH = 3;
    localparam PARAM_WIDTH  = 3;
    localparam SINK_WIDTH   = 3;
    localparam BURST_MAX    = 16;
    localparam FIFO_DEPTH   = 16;
    localparam DEPTH        = 512;
    localparam NUM_BANKS    = 4;
    localparam BANK_DEPTH   = 262144;
    localparam RD_LATENCY   = 1;
    localparam WR_LATENCY   = 1;

    wire                          m_clk;
    wire                          reset_m;
    wire                          start;
    wire                          stream_type;
    wire [ADDR_WIDTH-1:0]         dma_mem_base_addr;
    wire [ADDR_WIDTH-1:0]         dma_peri_base_addr;
    wire [15:0]                   num_bytes;
    wire                          spi_rd_rsp_valid_tb;
    wire [63:0]                   spi_rd_rsp_data_tb;

    wire                          busy;
    wire                          done;
    wire                          spi_wr_evt_valid_tb;
    wire [47:0]                   spi_wr_evt_addr_tb;
    wire [63:0]                   spi_wr_evt_data_tb;
    wire                          spi_rd_req_valid_tb;
    wire [47:0]                   spi_rd_req_addr_tb;
    wire                          memc_write_burst_done;
    wire                          memc_read_burst_done;
    wire [ADDR_WIDTH-1:0]         a_address0_tb;
    wire [DATA_WIDTH-1:0]         a_data0_tb;
    wire [OPCODE_WIDTH-1:0]       a_opcode0_tb;
    wire [PARAM_WIDTH-1:0]        a_param0_tb;
    wire [7:0]                    a_size0_tb;
    wire [7:0]                    a_mask0_tb;
    wire [2:0]                    a_source0_tb;
    wire                          a_valid0_tb;
    wire                          a_ready0_tb;
    wire [OPCODE_WIDTH-1:0]       d_opcode0_tb;
    wire [PARAM_WIDTH-1:0]        d_param0_tb;
    wire [7:0]                    d_size0_tb;
    wire [SINK_WIDTH-1:0]         d_sink0_tb;
    wire [2:0]                    d_source0_tb;
    wire [DATA_WIDTH-1:0]         d_data0_tb;
    wire                          d_valid0_tb;
    wire                          d_ready0_tb;
    wire                          d_error0_tb;
    wire [31:0]                   mem_total_rd_count;
    wire [31:0]                   mem_total_wr_count;

    tb_simple_integration #(
        .DATA_WIDTH   (DATA_WIDTH),
        .ADDR_WIDTH   (ADDR_WIDTH),
        .SOURCE_WIDTH (SOURCE_WIDTH),
        .SIZE_WIDTH   (SIZE_WIDTH),
        .OPCODE_WIDTH (OPCODE_WIDTH),
        .PARAM_WIDTH  (PARAM_WIDTH),
        .SINK_WIDTH   (SINK_WIDTH),
        .BURST_MAX    (BURST_MAX),
        .FIFO_DEPTH   (FIFO_DEPTH),
        .DEPTH        (DEPTH),
        .NUM_BANKS    (NUM_BANKS),
        .BANK_DEPTH   (BANK_DEPTH),
        .RD_LATENCY   (RD_LATENCY),
        .WR_LATENCY   (WR_LATENCY)
    ) u_tb (
        .m_clk                 (m_clk),
        .reset_m               (reset_m),
        .start                 (start),
        .stream_type           (stream_type),
        .dma_mem_base_addr     (dma_mem_base_addr),
        .dma_peri_base_addr    (dma_peri_base_addr),
        .num_bytes             (num_bytes),
        .spi_rd_rsp_valid_tb   (spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data_tb    (spi_rd_rsp_data_tb),
        .busy                  (busy),
        .done                  (done),
        .spi_wr_evt_valid_tb   (spi_wr_evt_valid_tb),
        .spi_wr_evt_addr_tb    (spi_wr_evt_addr_tb),
        .spi_wr_evt_data_tb    (spi_wr_evt_data_tb),
        .spi_rd_req_valid_tb   (spi_rd_req_valid_tb),
        .spi_rd_req_addr_tb    (spi_rd_req_addr_tb),
        .memc_write_burst_done (memc_write_burst_done),
        .memc_read_burst_done  (memc_read_burst_done),
        .a_address0_tb         (a_address0_tb),
        .a_data0_tb            (a_data0_tb),
        .a_opcode0_tb          (a_opcode0_tb),
        .a_param0_tb           (a_param0_tb),
        .a_size0_tb            (a_size0_tb),
        .a_mask0_tb            (a_mask0_tb),
        .a_source0_tb          (a_source0_tb),
        .a_valid0_tb           (a_valid0_tb),
        .a_ready0_tb           (a_ready0_tb),
        .d_opcode0_tb          (d_opcode0_tb),
        .d_param0_tb           (d_param0_tb),
        .d_size0_tb            (d_size0_tb),
        .d_sink0_tb            (d_sink0_tb),
        .d_source0_tb          (d_source0_tb),
        .d_data0_tb            (d_data0_tb),
        .d_valid0_tb           (d_valid0_tb),
        .d_ready0_tb           (d_ready0_tb),
        .d_error0_tb           (d_error0_tb),
        .mem_total_rd_count    (mem_total_rd_count),
        .mem_total_wr_count    (mem_total_wr_count)
    );

    spi_dma_xbar_memory_system #(
        .DATA_WIDTH   (DATA_WIDTH),
        .ADDR_WIDTH   (ADDR_WIDTH),
        .SOURCE_WIDTH (SOURCE_WIDTH),
        .SIZE_WIDTH   (SIZE_WIDTH),
        .OPCODE_WIDTH (OPCODE_WIDTH),
        .PARAM_WIDTH  (PARAM_WIDTH),
        .SINK_WIDTH   (SINK_WIDTH),
        .BURST_MAX    (BURST_MAX),
        .FIFO_DEPTH   (FIFO_DEPTH),
        .DEPTH        (DEPTH),
        .NUM_BANKS    (NUM_BANKS),
        .BANK_DEPTH   (BANK_DEPTH),
        .RD_LATENCY   (RD_LATENCY),
        .WR_LATENCY   (WR_LATENCY),
        .INIT_FILE    ("")
    ) dut (
        .m_clk                 (m_clk),
        .reset_m               (reset_m),
        .start                 (start),
        .stream_type           (stream_type),
        .dma_mem_base_addr     (dma_mem_base_addr),
        .dma_peri_base_addr    (dma_peri_base_addr),
        .num_bytes             (num_bytes),
        .busy                  (busy),
        .done                  (done),
        .spi_wr_evt_valid_tb   (spi_wr_evt_valid_tb),
        .spi_wr_evt_addr_tb    (spi_wr_evt_addr_tb),
        .spi_wr_evt_data_tb    (spi_wr_evt_data_tb),
        .spi_rd_req_valid_tb   (spi_rd_req_valid_tb),
        .spi_rd_req_addr_tb    (spi_rd_req_addr_tb),
        .spi_rd_rsp_valid_tb   (spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data_tb    (spi_rd_rsp_data_tb),
        .memc_write_burst_done (memc_write_burst_done),
        .memc_read_burst_done  (memc_read_burst_done),
        .a_address0_tb         (a_address0_tb),
        .a_data0_tb            (a_data0_tb),
        .a_opcode0_tb          (a_opcode0_tb),
        .a_param0_tb           (a_param0_tb),
        .a_size0_tb            (a_size0_tb),
        .a_mask0_tb            (a_mask0_tb),
        .a_source0_tb          (a_source0_tb),
        .a_valid0_tb           (a_valid0_tb),
        .a_ready0_tb           (a_ready0_tb),
        .d_opcode0_tb          (d_opcode0_tb),
        .d_param0_tb           (d_param0_tb),
        .d_size0_tb            (d_size0_tb),
        .d_sink0_tb            (d_sink0_tb),
        .d_source0_tb          (d_source0_tb),
        .d_data0_tb            (d_data0_tb),
        .d_valid0_tb           (d_valid0_tb),
        .d_ready0_tb           (d_ready0_tb),
        .d_error0_tb           (d_error0_tb),
        .mem_total_rd_count    (mem_total_rd_count),
        .mem_total_wr_count    (mem_total_wr_count)
    );
endmodule

`default_nettype wire
