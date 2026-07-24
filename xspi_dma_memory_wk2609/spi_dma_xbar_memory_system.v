`timescale 1ns/1ps
`default_nettype none
`include "./dma_spi8_xbar_system_top.v"
`include "./tl_sram_ctrl_top.v"
/*==============================================================================
 * SPI-DMA-CROSSBAR-MEMORY SYSTEM WRAPPER
 * 
 * This module integrates:
 * - DMA-to-Crossbar adapter (from dma_spi8_xbar_system_top)
 * - 3 Master-2 Slave Crossbar with CDC
 * - 4-Bank Memory Controller (tl_sram_ctrl_top)
 *==============================================================================*/

//This is the full system wrapper. It takes dma_spi8_xbar_system_top and directly 
//connects its exported slave-0 TileLink port into tl_sram_ctrl_top, 
//which is the memory controller / SRAM subsystem.

module spi_dma_xbar_memory_system #(
    parameter DATA_WIDTH   = 64,
    parameter ADDR_WIDTH   = 64,
    parameter SOURCE_WIDTH = 4,
    parameter SIZE_WIDTH   = 3,
    parameter OPCODE_WIDTH = 3,
    parameter PARAM_WIDTH  = 3,
    parameter SINK_WIDTH   = 3,
    parameter BURST_MAX    = 16,
    parameter FIFO_DEPTH   = 16,
    parameter DEPTH        = 512,
    parameter NUM_BANKS    = 4,
    parameter BANK_DEPTH   = 262144,
    parameter RD_LATENCY   = 1,
    parameter WR_LATENCY   = 1,
    parameter INIT_FILE    = ""
)(
    input  wire                          m_clk,
    input  wire                          reset_m,
    
    // DMA Control Interface
    input  wire                          start,
    input  wire                          stream_type,
    input  wire [ADDR_WIDTH-1:0]         dma_mem_base_addr,
    input  wire [ADDR_WIDTH-1:0]         dma_peri_base_addr,
    input  wire [15:0]                   num_bytes,
    output wire                          busy,
    output wire                          done,
    
    // SPI Interface
    output wire                          spi_wr_evt_valid_tb,
    output wire [47:0]                   spi_wr_evt_addr_tb,
    output wire [63:0]                   spi_wr_evt_data_tb,
    output wire                          spi_rd_req_valid_tb,
    output wire [47:0]                   spi_rd_req_addr_tb,
    input  wire                          spi_rd_rsp_valid_tb,
    input  wire [63:0]                   spi_rd_rsp_data_tb,
    
    // Debug - DMA Adapter
    output wire                          memc_write_burst_done,
    output wire                          memc_read_burst_done,
    
    // Debug - Crossbar Master 0
    output wire [ADDR_WIDTH-1:0]         a_address0_tb,
    output wire [DATA_WIDTH-1:0]         a_data0_tb,
    output wire [OPCODE_WIDTH-1:0]       a_opcode0_tb,
    output wire [PARAM_WIDTH-1:0]        a_param0_tb,
    output wire [7:0]                    a_size0_tb,
    output wire [7:0]                    a_mask0_tb,
    output wire [2:0]                    a_source0_tb,
    output wire                          a_valid0_tb,
    output wire                          a_ready0_tb,
    output wire [OPCODE_WIDTH-1:0]       d_opcode0_tb,
    output wire [PARAM_WIDTH-1:0]        d_param0_tb,
    output wire [7:0]                    d_size0_tb,
    output wire [SINK_WIDTH-1:0]         d_sink0_tb,
    output wire [2:0]                    d_source0_tb,
    output wire [DATA_WIDTH-1:0]         d_data0_tb,
    output wire                          d_valid0_tb,
    output wire                          d_ready0_tb,
    output wire                          d_error0_tb,
    
    // Debug - Memory
    output wire [31:0]                   mem_total_rd_count,
    output wire [31:0]                   mem_total_wr_count
);

    // =========================================================================
    // Local Parameters
    // =========================================================================
    localparam TL_STRB_WIDTH = DATA_WIDTH / 8;
    localparam TL_SIZE_WIDTH_XB = 8;

    // =========================================================================
    // Crossbar-to-Memory Interface (Slave 0)
    // =========================================================================
    wire                          slave0_a_valid;
    wire [OPCODE_WIDTH-1:0]       slave0_a_opcode;
    wire [PARAM_WIDTH-1:0]        slave0_a_param;
    wire [TL_SIZE_WIDTH_XB-1:0]   slave0_a_size;
    wire [SOURCE_WIDTH-1:0]       slave0_a_source;
    wire [ADDR_WIDTH-1:0]         slave0_a_address;
    wire [TL_STRB_WIDTH-1:0]      slave0_a_mask;
    wire [DATA_WIDTH-1:0]         slave0_a_data;
    wire                          slave0_a_corrupt;
    wire                          slave0_a_ready;
    wire                          slave0_d_ready;
    wire                          slave0_d_valid;
    wire [OPCODE_WIDTH-1:0]       slave0_d_opcode;
    wire [PARAM_WIDTH-1:0]        slave0_d_param;
    wire [TL_SIZE_WIDTH_XB-1:0]   slave0_d_size;
    wire [SOURCE_WIDTH-1:0]       slave0_d_source;
    wire [SINK_WIDTH-1:0]         slave0_d_sink;
    wire [DATA_WIDTH-1:0]         slave0_d_data;
    wire                          slave0_d_denied;
    wire                          slave0_d_corrupt;

    // =========================================================================
    // DMA + Crossbar System (Adapter + 3M2S Crossbar +CDC)
    // =========================================================================
    dma_spi8_xbar_system_top #(
        .DATA_WIDTH       (DATA_WIDTH),
        .ADDR_WIDTH       (ADDR_WIDTH),
        .SOURCE_WIDTH     (SOURCE_WIDTH),
        .SIZE_WIDTH       (SIZE_WIDTH),
        .OPCODE_WIDTH     (OPCODE_WIDTH),
        .PARAM_WIDTH      (PARAM_WIDTH),
        .SINK_WIDTH       (SINK_WIDTH),
        .BURST_MAX        (BURST_MAX),
        .FIFO_DEPTH       (FIFO_DEPTH)
    ) u_dma_xbar (
        .m_clk                   (m_clk),
        .reset_m                 (reset_m),
        
        // DMA Control
        .start                   (start),
        .stream_type             (stream_type),
        .dma_mem_base_addr       (dma_mem_base_addr),
        .dma_peri_base_addr      (dma_peri_base_addr),
        .num_bytes               (num_bytes),
        .busy                    (busy),
        .done                    (done),
        
        // SPI Interface
        .spi_wr_evt_valid_tb     (spi_wr_evt_valid_tb),
        .spi_wr_evt_addr_tb      (spi_wr_evt_addr_tb),
        .spi_wr_evt_data_tb      (spi_wr_evt_data_tb),
        .spi_rd_req_valid_tb     (spi_rd_req_valid_tb),
        .spi_rd_req_addr_tb      (spi_rd_req_addr_tb),
        .spi_rd_rsp_valid_tb     (spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data_tb      (spi_rd_rsp_data_tb),
        
        // Debug - Adapter
        .memc_write_burst_done   (memc_write_burst_done),
        .memc_read_burst_done    (memc_read_burst_done),
        
        // Debug - Master 0
        .a_address0_tb           (a_address0_tb),
        .a_data0_tb              (a_data0_tb),
        .a_opcode0_tb            (a_opcode0_tb),
        .a_param0_tb             (a_param0_tb),
        .a_size0_tb              (a_size0_tb),
        .a_mask0_tb              (a_mask0_tb),
        .a_source0_tb            (a_source0_tb),
        .a_valid0_tb             (a_valid0_tb),
        .a_ready0_tb             (a_ready0_tb),
        .d_opcode0_tb            (d_opcode0_tb),
        .d_param0_tb             (d_param0_tb),
        .d_size0_tb              (d_size0_tb),
        .d_sink0_tb              (d_sink0_tb),
        .d_source0_tb            (d_source0_tb),
        .d_data0_tb              (d_data0_tb),
        .d_valid0_tb             (d_valid0_tb),
        .d_ready0_tb             (d_ready0_tb),
        .d_error0_tb             (d_error0_tb),
        
        // Crossbar Slave 0 A-Channel Output
        .slave0_a_valid          (slave0_a_valid),
        .slave0_a_opcode         (slave0_a_opcode),
        .slave0_a_param          (slave0_a_param),
        .slave0_a_size           (slave0_a_size),
        .slave0_a_source         (slave0_a_source),
        .slave0_a_address        (slave0_a_address),
        .slave0_a_mask           (slave0_a_mask),
        .slave0_a_data           (slave0_a_data),
        .slave0_a_corrupt        (slave0_a_corrupt),
        .slave0_a_ready          (slave0_a_ready),
        
        // Crossbar Slave 0 D-Channel Input
        .slave0_d_ready          (slave0_d_ready),
        .slave0_d_valid          (slave0_d_valid),
        .slave0_d_opcode         (slave0_d_opcode),
        .slave0_d_param          (slave0_d_param),
        .slave0_d_size           (slave0_d_size),
        .slave0_d_source         (slave0_d_source),
        .slave0_d_sink           (slave0_d_sink),
        .slave0_d_data           (slave0_d_data),
        .slave0_d_denied         (slave0_d_denied),
        .slave0_d_corrupt        (slave0_d_corrupt),
        
        // Debug - Master 1 (unused)
        .a_address1_tb           (),
        .a_data1_tb              (),
        .a_opcode1_tb            (),
        .a_param1_tb             (),
        .a_size1_tb              (),
        .a_mask1_tb              (),
        .a_source1_tb            (),
        .a_valid1_tb             (),
        .a_ready1_tb             (),
        .d_opcode1_tb            (),
        .d_param1_tb             (),
        .d_size1_tb              (),
        .d_sink1_tb              (),
        .d_source1_tb            (),
        .d_data1_tb              (),
        .d_valid1_tb             (),
        .d_ready1_tb             (),
        .d_error1_tb             (),
        
        // Debug - Master 2 (unused)
        .a_address2_tb           (),
        .a_data2_tb              (),
        .a_opcode2_tb            (),
        .a_param2_tb             (),
        .a_size2_tb              (),
        .a_mask2_tb              (),
        .a_source2_tb            (),
        .a_valid2_tb             (),
        .a_ready2_tb             (),
        .d_opcode2_tb            (),
        .d_param2_tb             (),
        .d_size2_tb              (),
        .d_sink2_tb              (),
        .d_source2_tb            (),
        .d_data2_tb              (),
        .d_valid2_tb             (),
        .d_ready2_tb             (),
        .d_error2_tb             (),
        
        // CDC Debug (unused)
        .cdc_arb_a_valid         (),
        .cdc_arb_a_ready         (),
        .cdc_int_a_valid         (),
        .cdc_int_a_ready         (),
        .cdc_fifo_a_empty        (),
        .cdc_fifo_a_full         (),
        .cdc_slave0_a_ready      (),
        .cdc_slave1_a_ready      (),
        .cdc_slave_select        ()
    );

    // =========================================================================
    // TileLink SRAM Memory Controller (4 Banks)
    // =========================================================================
    tl_sram_ctrl_top #(
        .AW               (ADDR_WIDTH),
        .DW               (DATA_WIDTH),
        .AIW              (SOURCE_WIDTH),
        .SZW              (TL_SIZE_WIDTH_XB),
        .BANK_DEPTH       (BANK_DEPTH),
        .NUM_BANKS        (NUM_BANKS),
        .RD_LATENCY       (RD_LATENCY),
        .WR_LATENCY       (WR_LATENCY),
        .INIT_FILE        (INIT_FILE)
    ) u_memory (
        .clk              (m_clk),
        .rst_n            (~reset_m),
        
        // A-Channel Input
        .tl_a_valid       (slave0_a_valid),
        .tl_a_opcode      (slave0_a_opcode),
        .tl_a_param       (slave0_a_param),
        .tl_a_size        (slave0_a_size),
        .tl_a_source      (slave0_a_source),
        .tl_a_address     (slave0_a_address),
        .tl_a_mask        (slave0_a_mask),
        .tl_a_data        (slave0_a_data),
        .tl_a_corrupt     (slave0_a_corrupt),
        .tl_a_ready       (slave0_a_ready),
        
        // D-Channel Output
        .tl_d_ready       (slave0_d_ready),
        .tl_d_valid       (slave0_d_valid),
        .tl_d_opcode      (slave0_d_opcode),
        .tl_d_param       (slave0_d_param),
        .tl_d_size        (slave0_d_size),
        .tl_d_source      (slave0_d_source),
        .tl_d_sink        (slave0_d_sink),
        .tl_d_data        (slave0_d_data),
        .tl_d_denied      (slave0_d_denied),
        .tl_d_corrupt     (slave0_d_corrupt),
        
        // Debug
        .total_rd_count   (mem_total_rd_count),
        .total_wr_count   (mem_total_wr_count)
    );

endmodule

`default_nettype wire
