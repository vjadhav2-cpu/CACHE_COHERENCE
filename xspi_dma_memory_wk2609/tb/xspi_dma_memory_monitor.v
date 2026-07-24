module xspi_dma_memory_monitor #(

    // Parameters
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
   
    input wire                          busy,
    input wire                          done,
    
    // SPI Interface
    input wire                          spi_wr_evt_valid_tb,
    input wire [47:0]                   spi_wr_evt_addr_tb,
    input wire [63:0]                   spi_wr_evt_data_tb,
    input wire                          spi_rd_req_valid_tb,
    input wire [47:0]                   spi_rd_req_addr_tb,
   // input  wire                          spi_rd_rsp_valid_tb,
    //input  wire [63:0]                   spi_rd_rsp_data_tb,
    
    // Debug - DMA Adapter
    input wire                          memc_write_burst_done,
    input wire                          memc_read_burst_done,
    
    // Debug - Crossbar Master 0
    input wire [ADDR_WIDTH-1:0]         a_address0_tb,
    input wire [DATA_WIDTH-1:0]         a_data0_tb,
    input wire [OPCODE_WIDTH-1:0]       a_opcode0_tb,
    input wire [PARAM_WIDTH-1:0]        a_param0_tb,
    input wire [7:0]                    a_size0_tb,
    input wire [7:0]                    a_mask0_tb,
    input wire [2:0]                    a_source0_tb,
    input wire                          a_valid0_tb,
    input wire                          a_ready0_tb,
    input wire [OPCODE_WIDTH-1:0]       d_opcode0_tb,
    input wire [PARAM_WIDTH-1:0]        d_param0_tb,
    input wire [7:0]                    d_size0_tb,
    input wire [SINK_WIDTH-1:0]         d_sink0_tb,
    input wire [2:0]                    d_source0_tb,
    input wire [DATA_WIDTH-1:0]         d_data0_tb,
    input wire                          d_valid0_tb,
    input wire                          d_ready0_tb,
    input wire                          d_error0_tb,
    
    // Debug - Memory
    input wire [31:0]                   mem_total_rd_count,
    input wire [31:0]                   mem_total_wr_count
);
endmodule

