`timescale 1ns/1ps

module dma_spi8_xbar_system_monitor (
    // Streamer descriptor/control (observe what stimulus drives)
    input  wire                          start,
    input  wire                          stream_type,
    input  wire [63:0]                   dma_mem_base_addr,
    input  wire [63:0]                   dma_peri_base_addr,
    input  wire [15:0]                   num_bytes,

    // Streamer status
    input  wire                          busy,
    input  wire                          done,

    // SPI TB ports
    input  wire                          spi_wr_evt_valid_tb,
    input  wire [47:0]                   spi_wr_evt_addr_tb,
    input  wire [63:0]                   spi_wr_evt_data_tb,
    input  wire                          spi_rd_req_valid_tb,
    input  wire [47:0]                   spi_rd_req_addr_tb,
    input  wire                          spi_rd_rsp_valid_tb,
    input  wire [63:0]                   spi_rd_rsp_data_tb,

    // Adapter burst completion visibility
    input  wire                          memc_write_burst_done,
    input  wire                          memc_read_burst_done,

    // Xbar master 0 TB outputs
    input  wire [63:0]                   a_address0_tb,
    input  wire [63:0]                   a_data0_tb,
    input  wire [2:0]                    a_opcode0_tb,
    input  wire [2:0]                    a_param0_tb,
    input  wire [7:0]                    a_size0_tb,
    input  wire [7:0]                    a_mask0_tb,
    input  wire [2:0]                    a_source0_tb,
    input  wire                          a_valid0_tb,
    input  wire                          a_ready0_tb,
    input  wire [2:0]                    d_opcode0_tb,
    input  wire [2:0]                    d_param0_tb,
    input  wire [7:0]                    d_size0_tb,
    input  wire [2:0]                    d_sink0_tb,
    input  wire [2:0]                    d_source0_tb,
    input  wire [63:0]                   d_data0_tb,
    input  wire                          d_valid0_tb,
    input  wire                          d_ready0_tb,
    input  wire                          d_error0_tb,

    // Xbar master 1 TB outputs
    input  wire [63:0]                   a_address1_tb,
    input  wire [63:0]                   a_data1_tb,
    input  wire [2:0]                    a_opcode1_tb,
    input  wire [2:0]                    a_param1_tb,
    input  wire [7:0]                    a_size1_tb,
    input  wire [7:0]                    a_mask1_tb,
    input  wire [2:0]                    a_source1_tb,
    input  wire                          a_valid1_tb,
    input  wire                          a_ready1_tb,
    input  wire [2:0]                    d_opcode1_tb,
    input  wire [2:0]                    d_param1_tb,
    input  wire [7:0]                    d_size1_tb,
    input  wire [2:0]                    d_sink1_tb,
    input  wire [2:0]                    d_source1_tb,
    input  wire [63:0]                   d_data1_tb,
    input  wire                          d_valid1_tb,
    input  wire                          d_ready1_tb,
    input  wire                          d_error1_tb,

    // Xbar master 2 TB outputs
    input  wire [63:0]                   a_address2_tb,
    input  wire [63:0]                   a_data2_tb,
    input  wire [2:0]                    a_opcode2_tb,
    input  wire [2:0]                    a_param2_tb,
    input  wire [7:0]                    a_size2_tb,
    input  wire [7:0]                    a_mask2_tb,
    input  wire [2:0]                    a_source2_tb,
    input  wire                          a_valid2_tb,
    input  wire                          a_ready2_tb,
    input  wire [2:0]                    d_opcode2_tb,
    input  wire [2:0]                    d_param2_tb,
    input  wire [7:0]                    d_size2_tb,
    input  wire [2:0]                    d_sink2_tb,
    input  wire [2:0]                    d_source2_tb,
    input  wire [63:0]                   d_data2_tb,
    input  wire                          d_valid2_tb,
    input  wire                          d_ready2_tb,
    input  wire                          d_error2_tb,

    // Xbar CDC debug outputs
    input  wire                          cdc_arb_a_valid,
    input  wire                          cdc_arb_a_ready,
    input  wire                          cdc_int_a_valid,
    input  wire                          cdc_int_a_ready,
    input  wire                          cdc_fifo_a_empty,
    input  wire                          cdc_fifo_a_full,
    input  wire                          cdc_slave0_a_ready,
    input  wire                          cdc_slave1_a_ready,
    input  wire                          cdc_slave_select
);

    integer in_flight;
    integer saw_write_burst_done;
    integer saw_read_burst_done;
    integer saw_spi_wr_evt;
    integer saw_spi_rd_req;
    integer any_error;

    initial begin
        in_flight = 0;
        saw_write_burst_done = 0;
        saw_read_burst_done  = 0;
        saw_spi_wr_evt = 0;
        saw_spi_rd_req = 0;
        any_error = 0;
    end

    // Track activity and print PASS/FAIL when done asserts
    always @(start or done or memc_write_burst_done or memc_read_burst_done or
             spi_wr_evt_valid_tb or spi_rd_req_valid_tb or
             d_error0_tb or d_error1_tb or d_error2_tb) begin

        if (d_error0_tb || d_error1_tb || d_error2_tb)
            any_error = 1;

        if (memc_write_burst_done)
            saw_write_burst_done = 1;

        if (memc_read_burst_done)
            saw_read_burst_done = 1;

        if (spi_wr_evt_valid_tb)
            saw_spi_wr_evt = 1;

        if (spi_rd_req_valid_tb)
            saw_spi_rd_req = 1;

        if (start) begin
            in_flight = 1;
            saw_write_burst_done = 0;
            saw_read_burst_done  = 0;
            saw_spi_wr_evt = 0;
            saw_spi_rd_req = 0;
            any_error = 0;

            $display("TEST START: t=%0t stream_type=%0b mem_base=0x%016h peri_base=0x%016h bytes=%0d",
                     $time, stream_type, dma_mem_base_addr, dma_peri_base_addr, num_bytes);
        end

        if (in_flight && done) begin
            if (stream_type == 1'b0) begin
                $display("RESULT: stream_type=%0b done=%0b busy=%0b wr_evt=%0d wr_burst_done=%0d tl_error=%0d => %s",
                         stream_type, done, busy, saw_spi_wr_evt, saw_write_burst_done, any_error,
                         (!any_error && saw_write_burst_done) ? "PASS" : "FAIL");
            end else begin
                $display("RESULT: stream_type=%0b done=%0b busy=%0b rd_req=%0d rd_burst_done=%0d tl_error=%0d => %s",
                         stream_type, done, busy, saw_spi_rd_req, saw_read_burst_done, any_error,
                         (!any_error && saw_read_burst_done) ? "PASS" : "FAIL");
            end

            $display("OBS: A0 v/r=%0b/%0b addr=0x%016h op=%0d size=%0d src=%0d | D0 v/r=%0b/%0b op=%0d err=%0b",
                     a_valid0_tb, a_ready0_tb, a_address0_tb, a_opcode0_tb, a_size0_tb, a_source0_tb,
                     d_valid0_tb, d_ready0_tb, d_opcode0_tb, d_error0_tb);

            $display("CDC: arb(v/r)=%0b/%0b int(v/r)=%0b/%0b fifo(e/f)=%0b/%0b sel=%0b s0_rdy=%0b s1_rdy=%0b\n",
                     cdc_arb_a_valid, cdc_arb_a_ready, cdc_int_a_valid, cdc_int_a_ready,
                     cdc_fifo_a_empty, cdc_fifo_a_full, cdc_slave_select, cdc_slave0_a_ready, cdc_slave1_a_ready);

            in_flight = 0;
        end
    end

    // Live event prints (still only $display)
    always @(spi_wr_evt_valid_tb or spi_rd_req_valid_tb or spi_rd_rsp_valid_tb) begin
        if (spi_wr_evt_valid_tb)
            $display("SPI_WR_EVT: t=%0t addr=0x%012h data=0x%016h",
                     $time, spi_wr_evt_addr_tb, spi_wr_evt_data_tb);

        if (spi_rd_req_valid_tb)
            $display("SPI_RD_REQ: t=%0t addr=0x%012h",
                     $time, spi_rd_req_addr_tb);

        if (spi_rd_rsp_valid_tb)
            $display("SPI_RD_RSP: t=%0t data=0x%016h",
                     $time, spi_rd_rsp_data_tb);
    end

endmodule
