`timescale 1ns/1ps

module dma_spi8_xbar_system_stimulus (
    output reg                          m_clk,
    output reg                          reset_m,

    output reg                          start,
    output reg                          stream_type,
    output reg  [63:0]                  dma_mem_base_addr,
    output reg  [63:0]                  dma_peri_base_addr,
    output reg  [15:0]                  num_bytes,

    output reg                          spi_rd_rsp_valid_tb,
    output reg  [63:0]                  spi_rd_rsp_data_tb
);

    // Test vectors: {stream_type, base_addr, num_bytes}
    reg [0:0]   tv_stream_type [0:2];
    reg [63:0]  tv_base_addr   [0:2];
    reg [15:0]  tv_num_bytes   [0:2];

    integer i;
 
    initial begin
        // init vectors - USE ADDRESSES IN SLAVE 0 RANGE (0x0000-0x01FF, DEPTH=512)
        tv_stream_type[0] = 1'b0;  tv_base_addr[0] = 64'h0000_0000_0000_0000;  tv_num_bytes[0] = 16'd64;   // Slave 0: addr 0x0000
        tv_stream_type[1] = 1'b1;  tv_base_addr[1] = 64'h0000_0000_0000_0080;  tv_num_bytes[1] = 16'd128;  // Slave 0: addr 0x0080
        tv_stream_type[2] = 1'b0;  tv_base_addr[2] = 64'h0000_0000_0000_0100;  tv_num_bytes[2] = 16'd16;   // Slave 0: addr 0x0100

        // defaults
        m_clk              = 1'b0;
        reset_m            = 1'b1;  // Active-high reset: 1=reset, 0=normal
        start              = 1'b0;
        stream_type        = 1'b0;
        dma_mem_base_addr  = 64'd0;
        dma_peri_base_addr = 64'd0;
        num_bytes          = 16'd0;
        spi_rd_rsp_valid_tb= 1'b0;
        spi_rd_rsp_data_tb = 64'd0;

        // reset pulse: assert reset, wait, then deassert
        #20;
        reset_m = 1'b0;  // Deassert reset (normal operation)
        #20;

        // run tests
        for (i = 0; i < 3; i = i + 1) begin
            // program descriptor
            stream_type   = tv_stream_type[i];
            dma_mem_base_addr  = tv_base_addr[i];
            dma_peri_base_addr = tv_base_addr[i];
            num_bytes     = tv_num_bytes[i];

            // pulse start
            #10;
            start = 1'b1;
            #10;
            start = 1'b0;

            // crude read-response driving (since stimulus has NO inputs by your rule)
            // This just provides occasional responses in case the DUT is waiting on them.
            spi_rd_rsp_valid_tb = 1'b0;
            spi_rd_rsp_data_tb  = 64'h1111_0000_0000_0000 ^ tv_base_addr[i];

            #40;  spi_rd_rsp_valid_tb = 1'b1;  #10; spi_rd_rsp_valid_tb = 1'b0;
            #60;  spi_rd_rsp_data_tb  = spi_rd_rsp_data_tb ^ 64'h0000_0000_0000_00FF;
                  spi_rd_rsp_valid_tb = 1'b1;  #10; spi_rd_rsp_valid_tb = 1'b0;

            // gap between tests
            #200;
        end

        // finish settle time
        #200;
    end

    // clock
    initial begin
        forever #5 m_clk = ~m_clk;
    end

endmodule
