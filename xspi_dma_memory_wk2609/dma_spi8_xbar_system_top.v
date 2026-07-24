`timescale 1ns/1ps
`default_nettype none
`include "./spi8_to_xbar_adapter.v"
`include "./tilelink_uh_3M_2S.v"
module dma_spi8_xbar_system_top #(
    parameter DATA_WIDTH       = 64,
    parameter ADDR_WIDTH       = 64,
    parameter SOURCE_WIDTH     = 4,
    parameter SIZE_WIDTH       = 3,
    parameter OPCODE_WIDTH     = 3,
    parameter PARAM_WIDTH      = 3,
    parameter SINK_WIDTH       = 3,
    parameter BURST_MAX        = 16,
    parameter ADAPTER_DISPATCH_IDLE_CYCLES = 32,
    parameter FIFO_DEPTH       = 16,
    parameter MEM_BASE_ADDR    = 64'h0000_0000_0000_0000,
    parameter DEPTH            = 512,
    parameter TL_STRB_WIDTH    = DATA_WIDTH / 8,
    parameter TL_BEAT_WIDTH    = 8,
    parameter BEAT_LOG2        = $clog2(TL_BEAT_WIDTH),
    parameter TL_SIZE_WIDTH_XB = 8,
    parameter MAX_BURST_LENGTH = 16,
    parameter GRANT_FIFO_DEPTH = 8,
    parameter NUM_SLAVES       = 2,
    parameter INTERCONNECT_D_READY = 1'b1
)(
    input  wire                          m_clk,
    input  wire                          reset_m,

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

    // Adapter burst completion visibility
    output wire                          memc_write_burst_done,
    output wire                          memc_read_burst_done,

    // Xbar master 0 TB outputs
    output wire [ADDR_WIDTH-1:0]         a_address0_tb,
    output wire [DATA_WIDTH-1:0]         a_data0_tb,
    output wire [OPCODE_WIDTH-1:0]       a_opcode0_tb,
    output wire [PARAM_WIDTH-1:0]        a_param0_tb,
    output wire [TL_SIZE_WIDTH_XB-1:0]   a_size0_tb,
    output wire [TL_STRB_WIDTH-1:0]      a_mask0_tb,
    output wire [2:0]                    a_source0_tb,
    output wire                          a_valid0_tb,
    output wire                          a_ready0_tb,
    output wire [OPCODE_WIDTH-1:0]       d_opcode0_tb,
    output wire [PARAM_WIDTH-1:0]        d_param0_tb,
    output wire [TL_SIZE_WIDTH_XB-1:0]   d_size0_tb,
    output wire [SINK_WIDTH-1:0]         d_sink0_tb,
    output wire [2:0]                    d_source0_tb,
    output wire [DATA_WIDTH-1:0]         d_data0_tb,
    output wire                          d_valid0_tb,
    output wire                          d_ready0_tb,
    output wire                          d_error0_tb,

    // Xbar master 1 TB outputs
    output wire [ADDR_WIDTH-1:0]         a_address1_tb,
    output wire [DATA_WIDTH-1:0]         a_data1_tb,
    output wire [OPCODE_WIDTH-1:0]       a_opcode1_tb,
    output wire [PARAM_WIDTH-1:0]        a_param1_tb,
    output wire [TL_SIZE_WIDTH_XB-1:0]   a_size1_tb,
    output wire [TL_STRB_WIDTH-1:0]      a_mask1_tb,
    output wire [2:0]                    a_source1_tb,
    output wire                          a_valid1_tb,
    output wire                          a_ready1_tb,
    output wire [OPCODE_WIDTH-1:0]       d_opcode1_tb,
    output wire [PARAM_WIDTH-1:0]        d_param1_tb,
    output wire [TL_SIZE_WIDTH_XB-1:0]   d_size1_tb,
    output wire [SINK_WIDTH-1:0]         d_sink1_tb,
    output wire [2:0]                    d_source1_tb,
    output wire [DATA_WIDTH-1:0]         d_data1_tb,
    output wire                          d_valid1_tb,
    output wire                          d_ready1_tb,
    output wire                          d_error1_tb,

    // Xbar master 2 TB outputs
    output wire [ADDR_WIDTH-1:0]         a_address2_tb,
    output wire [DATA_WIDTH-1:0]         a_data2_tb,
    output wire [OPCODE_WIDTH-1:0]       a_opcode2_tb,
    output wire [PARAM_WIDTH-1:0]        a_param2_tb,
    output wire [TL_SIZE_WIDTH_XB-1:0]   a_size2_tb,
    output wire [TL_STRB_WIDTH-1:0]      a_mask2_tb,
    output wire [2:0]                    a_source2_tb,
    output wire                          a_valid2_tb,
    output wire                          a_ready2_tb,
    output wire [OPCODE_WIDTH-1:0]       d_opcode2_tb,
    output wire [PARAM_WIDTH-1:0]        d_param2_tb,
    output wire [TL_SIZE_WIDTH_XB-1:0]   d_size2_tb,
    output wire [SINK_WIDTH-1:0]         d_sink2_tb,
    output wire [2:0]                    d_source2_tb,
    output wire [DATA_WIDTH-1:0]         d_data2_tb,
    output wire                          d_valid2_tb,
    output wire                          d_ready2_tb,
    output wire                          d_error2_tb,

    // Xbar CDC debug outputs
    output wire                          cdc_arb_a_valid,
    output wire                          cdc_arb_a_ready,
    output wire                          cdc_int_a_valid,
    output wire                          cdc_int_a_ready,
    output wire                          cdc_fifo_a_empty,
    output wire                          cdc_fifo_a_full,
    output wire                          cdc_slave0_a_ready,
    output wire                          cdc_slave1_a_ready,
    output wire                          cdc_slave_select,

    // =====================================================================
    // Slave 0 Interface - External Memory Connection
    // =====================================================================
    // A-Channel Outputs (Request to Slave)
    output wire                          slave0_a_valid,
    output wire [OPCODE_WIDTH-1:0]       slave0_a_opcode,
    output wire [PARAM_WIDTH-1:0]        slave0_a_param,
    output wire [TL_SIZE_WIDTH_XB-1:0]   slave0_a_size,
    output wire [2:0]                    slave0_a_source,
    output wire [ADDR_WIDTH-1:0]         slave0_a_address,
    output wire [TL_STRB_WIDTH-1:0]      slave0_a_mask,
    output wire [DATA_WIDTH-1:0]         slave0_a_data,
    output wire                          slave0_a_corrupt,
    output wire                          slave0_d_ready,
    
    // D-Channel Inputs (Response from Slave)
    input  wire                          slave0_a_ready,
    input  wire                          slave0_d_valid,
    input  wire [OPCODE_WIDTH-1:0]       slave0_d_opcode,
    input  wire [1:0]                    slave0_d_param,      // 2-bit from tl_sram_ctrl
    input  wire [TL_SIZE_WIDTH_XB-1:0]   slave0_d_size,
    input  wire [2:0]                    slave0_d_source,
    input  wire                          slave0_d_sink,       // 1-bit from tl_sram_ctrl
    input  wire [DATA_WIDTH-1:0]         slave0_d_data,
    input  wire                          slave0_d_denied,
    input  wire                          slave0_d_corrupt
);

    // Adapter -> Xbar M0 request path
    wire                          m0_a_valid;
    wire [OPCODE_WIDTH-1:0]       m0_a_opcode;
    wire [PARAM_WIDTH-1:0]        m0_a_param;
    wire [ADDR_WIDTH-1:0]         m0_a_address;
    wire [TL_SIZE_WIDTH_XB-1:0]   m0_a_size;
    wire [TL_STRB_WIDTH-1:0]      m0_a_mask;
    wire [DATA_WIDTH-1:0]         m0_a_data;
    wire [2:0]                    m0_a_source;

    // Xbar -> Adapter M0 response/ready path
    wire                          m0_a_ready;
    wire [OPCODE_WIDTH-1:0]       m0_d_opcode;
    wire [PARAM_WIDTH-1:0]        m0_d_param;
    wire [TL_SIZE_WIDTH_XB-1:0]   m0_d_size;
    wire [SINK_WIDTH-1:0]         m0_d_sink;
    wire [2:0]                    m0_d_source;
    wire [DATA_WIDTH-1:0]         m0_d_data;
    wire                          m0_d_valid;
    wire                          m0_d_ready;
    wire                          m0_d_error;

    // M1/M2 tied to idle constants per requirement.
    localparam [OPCODE_WIDTH-1:0] OPC_ZERO = {OPCODE_WIDTH{1'b0}};
    localparam [PARAM_WIDTH-1:0]  PAR_ZERO = {PARAM_WIDTH{1'b0}};
    localparam [ADDR_WIDTH-1:0]   ADDR_ZERO = {ADDR_WIDTH{1'b0}};
    localparam [TL_SIZE_WIDTH_XB-1:0] SIZE_ZERO = {TL_SIZE_WIDTH_XB{1'b0}};
    localparam [TL_STRB_WIDTH-1:0] STRB_ZERO = {TL_STRB_WIDTH{1'b0}};
    localparam [DATA_WIDTH-1:0] DATA_ZERO = {DATA_WIDTH{1'b0}};
    localparam [2:0] SRC_ZERO = 3'd0;

    spi8_to_xbar_adapter #(
        .DATA_WIDTH    (DATA_WIDTH),
        .ADDR_WIDTH    (ADDR_WIDTH),
        .SOURCE_WIDTH  (SOURCE_WIDTH),
        .SIZE_WIDTH    (SIZE_WIDTH),
        .OPCODE_WIDTH  (OPCODE_WIDTH),
        .PARAM_WIDTH   (PARAM_WIDTH),
        .SINK_WIDTH    (SINK_WIDTH),
        .BURST_MAX     (BURST_MAX),
        .ADAPTER_DISPATCH_IDLE_CYCLES(ADAPTER_DISPATCH_IDLE_CYCLES),
        .FIFO_DEPTH    (FIFO_DEPTH),
        .MEM_BASE_ADDR (MEM_BASE_ADDR),
        .DEPTH         (DEPTH)
    ) u_spi8_to_xbar_adapter (
        .clk                 (m_clk),
        .rst                 (reset_m),
        .start               (start),
        .stream_type         (stream_type),
        .dma_mem_base_addr   (dma_mem_base_addr),
        .dma_peri_base_addr  (dma_peri_base_addr),
        .num_bytes           (num_bytes),
        .busy                (busy),
        .done                (done),
        .spi_wr_evt_valid_tb (spi_wr_evt_valid_tb),
        .spi_wr_evt_addr_tb  (spi_wr_evt_addr_tb),
        .spi_wr_evt_data_tb  (spi_wr_evt_data_tb),
        .spi_rd_req_valid_tb (spi_rd_req_valid_tb),
        .spi_rd_req_addr_tb  (spi_rd_req_addr_tb),
        .spi_rd_rsp_valid_tb (spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data_tb  (spi_rd_rsp_data_tb),
        .a_valid_in0         (m0_a_valid),
        .a_opcode_in0        (m0_a_opcode),
        .a_param_in0         (m0_a_param),
        .a_address_in0       (m0_a_address),
        .a_size_in0          (m0_a_size),
        .a_mask_in0          (m0_a_mask),
        .a_data_in0          (m0_a_data),
        .a_source_in0        (m0_a_source),
        .a_ready0_tb         (m0_a_ready),
        .d_opcode0_tb        (m0_d_opcode),
        .d_param0_tb         (m0_d_param),
        .d_size0_tb          (m0_d_size),
        .d_sink0_tb          (m0_d_sink),
        .d_source0_tb        (m0_d_source),
        .d_data0_tb          (m0_d_data),
        .d_valid0_tb         (m0_d_valid),
        .d_ready0_tb         (m0_d_ready),
        .d_error0_tb         (m0_d_error),
        .memc_write_burst_done(memc_write_burst_done),
        .memc_read_burst_done (memc_read_burst_done)
    );

    tilelink_uh_3M_2S #(
        .TL_ADDR_WIDTH        (ADDR_WIDTH),
        .TL_DATA_WIDTH        (DATA_WIDTH),
        .TL_STRB_WIDTH        (TL_STRB_WIDTH),
        .TL_BEAT_WIDTH        (TL_BEAT_WIDTH),
        .BEAT_LOG2            (BEAT_LOG2),
        .TL_SOURCE_WIDTH      (3),
        .TL_SINK_WIDTH        (SINK_WIDTH),
        .TL_OPCODE_WIDTH      (OPCODE_WIDTH),
        .TL_PARAM_WIDTH       (PARAM_WIDTH),
        .TL_SIZE_WIDTH        (TL_SIZE_WIDTH_XB),
        .MAX_BURST_LENGTH     (MAX_BURST_LENGTH),
        .GRANT_FIFO_DEPTH     (GRANT_FIFO_DEPTH),
        .MEM_BASE_ADDR        (MEM_BASE_ADDR),
        .DEPTH                (DEPTH),
        .FIFO_DEPTH           (8),
        .NUM_SLAVES           (NUM_SLAVES),
        .INTERCONNECT_D_READY (INTERCONNECT_D_READY)
    ) u_tilelink_uh_3M_2S (
        .m_clk          (m_clk),
        .s_clk          (m_clk),
        .reset_m        (reset_m),
        .reset_s        (reset_m),

        .a_valid_in0    (m0_a_valid),
        .a_opcode_in0   (m0_a_opcode),
        .a_param_in0    (m0_a_param),
        .a_address_in0  (m0_a_address),
        .a_size_in0     (m0_a_size),
        .a_mask_in0     (m0_a_mask),
        .a_data_in0     (m0_a_data),
        .a_source_in0   (m0_a_source),

        .a_valid_in1    (1'b0),
        .a_opcode_in1   (OPC_ZERO),
        .a_param_in1    (PAR_ZERO),
        .a_address_in1  (ADDR_ZERO),
        .a_size_in1     (SIZE_ZERO),
        .a_mask_in1     (STRB_ZERO),
        .a_data_in1     (DATA_ZERO),
        .a_source_in1   (SRC_ZERO),

        .a_valid_in2    (1'b0),
        .a_opcode_in2   (OPC_ZERO),
        .a_param_in2    (PAR_ZERO),
        .a_address_in2  (ADDR_ZERO),
        .a_size_in2     (SIZE_ZERO),
        .a_mask_in2     (STRB_ZERO),
        .a_data_in2     (DATA_ZERO),
        .a_source_in2   (SRC_ZERO),

        .a_address0_tb  (a_address0_tb),
        .a_data0_tb     (a_data0_tb),
        .a_opcode0_tb   (a_opcode0_tb),
        .a_param0_tb    (a_param0_tb),
        .a_size0_tb     (a_size0_tb),
        .a_mask0_tb     (a_mask0_tb),
        .a_source0_tb   (a_source0_tb),
        .a_valid0_tb    (a_valid0_tb),
        .a_ready0_tb    (a_ready0_tb),
        .d_opcode0_tb   (d_opcode0_tb),
        .d_param0_tb    (d_param0_tb),
        .d_size0_tb     (d_size0_tb),
        .d_sink0_tb     (d_sink0_tb),
        .d_source0_tb   (d_source0_tb),
        .d_data0_tb     (d_data0_tb),
        .d_valid0_tb    (d_valid0_tb),
        .d_ready0_tb    (d_ready0_tb),
        .d_error0_tb    (d_error0_tb),

        .a_address1_tb  (a_address1_tb),
        .a_data1_tb     (a_data1_tb),
        .a_opcode1_tb   (a_opcode1_tb),
        .a_param1_tb    (a_param1_tb),
        .a_size1_tb     (a_size1_tb),
        .a_mask1_tb     (a_mask1_tb),
        .a_source1_tb   (a_source1_tb),
        .a_valid1_tb    (a_valid1_tb),
        .a_ready1_tb    (a_ready1_tb),
        .d_opcode1_tb   (d_opcode1_tb),
        .d_param1_tb    (d_param1_tb),
        .d_size1_tb     (d_size1_tb),
        .d_sink1_tb     (d_sink1_tb),
        .d_source1_tb   (d_source1_tb),
        .d_data1_tb     (d_data1_tb),
        .d_valid1_tb    (d_valid1_tb),
        .d_ready1_tb    (d_ready1_tb),
        .d_error1_tb    (d_error1_tb),

        .a_address2_tb  (a_address2_tb),
        .a_data2_tb     (a_data2_tb),
        .a_opcode2_tb   (a_opcode2_tb),
        .a_param2_tb    (a_param2_tb),
        .a_size2_tb     (a_size2_tb),
        .a_mask2_tb     (a_mask2_tb),
        .a_source2_tb   (a_source2_tb),
        .a_valid2_tb    (a_valid2_tb),
        .a_ready2_tb    (a_ready2_tb),
        .d_opcode2_tb   (d_opcode2_tb),
        .d_param2_tb    (d_param2_tb),
        .d_size2_tb     (d_size2_tb),
        .d_sink2_tb     (d_sink2_tb),
        .d_source2_tb   (d_source2_tb),
        .d_data2_tb     (d_data2_tb),
        .d_valid2_tb    (d_valid2_tb),
        .d_ready2_tb    (d_ready2_tb),
        .d_error2_tb    (d_error2_tb),

        .cdc_arb_a_valid(cdc_arb_a_valid),
        .cdc_arb_a_ready(cdc_arb_a_ready),
        .cdc_int_a_valid(cdc_int_a_valid),
        .cdc_int_a_ready(cdc_int_a_ready),
        .cdc_fifo_a_empty(cdc_fifo_a_empty),
        .cdc_fifo_a_full(cdc_fifo_a_full),
        .cdc_slave0_a_ready(cdc_slave0_a_ready),
        .cdc_slave1_a_ready(cdc_slave1_a_ready),
        .cdc_slave_select(cdc_slave_select),

        // Slave 0 External Interface
        .slave0_a_valid(slave0_a_valid),
        .slave0_a_opcode(slave0_a_opcode),
        .slave0_a_param(slave0_a_param),
        .slave0_a_size(slave0_a_size),
        .slave0_a_source(slave0_a_source),
        .slave0_a_address(slave0_a_address),
        .slave0_a_mask(slave0_a_mask),
        .slave0_a_data(slave0_a_data),
        .slave0_a_corrupt(slave0_a_corrupt),
        .slave0_d_ready(slave0_d_ready),
        .slave0_a_ready(slave0_a_ready),
        .slave0_d_valid(slave0_d_valid),
        .slave0_d_opcode(slave0_d_opcode),
        .slave0_d_param(slave0_d_param),
        .slave0_d_size(slave0_d_size),
        .slave0_d_source(slave0_d_source),
        .slave0_d_sink(slave0_d_sink),
        .slave0_d_data(slave0_d_data),
        .slave0_d_denied(slave0_d_denied),
        .slave0_d_corrupt(slave0_d_corrupt),

        // Slave 1 - Tied off for now (can connect CLINT peripheral later)
        .slave1_a_ready(1'b0),
        .slave1_d_valid(1'b0),
        .slave1_d_opcode(3'b0),
        .slave1_d_param(2'b0),
        .slave1_d_size(8'b0),
        .slave1_d_source(3'b0),
        .slave1_d_sink(1'b0),
        .slave1_d_data(64'b0),
        .slave1_d_denied(1'b1),
        .slave1_d_corrupt(1'b0)
    );

    // Internal loopback from xbar M0 monitor outputs back into adapter response inputs.
    assign m0_a_ready = a_ready0_tb;
    assign m0_d_opcode = d_opcode0_tb;
    assign m0_d_param  = d_param0_tb;
    assign m0_d_size   = d_size0_tb;
    assign m0_d_sink   = d_sink0_tb;
    assign m0_d_source = d_source0_tb;
    assign m0_d_data   = d_data0_tb;
    assign m0_d_valid  = d_valid0_tb;
    assign m0_d_ready  = d_ready0_tb;
    assign m0_d_error  = d_error0_tb;

endmodule

`default_nettype wire
