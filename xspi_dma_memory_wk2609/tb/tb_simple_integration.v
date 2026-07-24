`timescale 1ns/1ps
`default_nettype none

/*==============================================================================
 * SIMPLE TESTBENCH FOR SPI-DMA-CROSSBAR-MEMORY INTEGRATION
 * 
 * PURPOSE: Direct test of the complete integration without monitor modules
 * - Instantiates top module (spi_dma_xbar_memory_system)
 * - Drives DMA control outputs (start, stream_type, address, num_bytes)
 * - Provides SPI read response signals
 * - Single test case to verify basic functionality
 *==============================================================================*/

module tb_simple_integration #(

    // =========================================================================
    // Parameters
    // =========================================================================
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
    parameter WR_LATENCY   = 1
) (

   output  reg                          m_clk,
    output  reg                          reset_m,
    
    // DMA Control Interface
    output  reg                          start,
    output  reg                          stream_type,
    output  reg [ADDR_WIDTH-1:0]         dma_mem_base_addr,
    output  reg [ADDR_WIDTH-1:0]         dma_peri_base_addr,
    output  reg [15:0]                   num_bytes,
    output  reg                          spi_rd_rsp_valid_tb,
    output  reg [63:0]                   spi_rd_rsp_data_tb,


////////////////////////////////////////////////////////

    input wire                          busy,
    input wire                          done,
    
    // SPI Interface
    input wire                          spi_wr_evt_valid_tb,
    input wire [47:0]                   spi_wr_evt_addr_tb,
    input wire [63:0]                   spi_wr_evt_data_tb,
    input wire                          spi_rd_req_valid_tb,
    input wire [47:0]                   spi_rd_req_addr_tb,
   
    
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
      parameter [2:0] IDLE            = 3'd0;
    parameter [2:0] RESET           = 3'd1;
    parameter [2:0] TEST1_WRITE     = 3'd2;
    parameter [2:0] TEST1_WAIT_DONE = 3'd3;
    parameter [2:0] TEST2_READ      = 3'd4;
    parameter [2:0] TEST2_WAIT_DONE = 3'd5;
    parameter [2:0] VERIFY          = 3'd6;
    parameter [2:0] DONE            = 3'd7;
   

    // =========================================================================
    // Test Data Storage for Verification
    // =========================================================================
    reg [63:0]              written_data [0:7];  // Store data written to memory
    reg [63:0]              read_data [0:7];     // Store data read back from memory
    integer                 write_count;
    integer                 read_count;

   // =========================================================================
// Test Control Registers
// =========================================================================
reg [2:0]               test_state;
reg [15:0]              cycle_count;
reg                     spi_rd_req_prev;  // Edge detection for SPI requests
reg [3:0]               spi_response_delay;
reg                     busy_prev;
reg                     done_prev;
integer                 errors;
integer                 i;

// =========================================================================
// DUT Instantiation
// =========================================================================
/*spi_dma_xbar_memory_system #(
    .DATA_WIDTH       (DATA_WIDTH),
    .ADDR_WIDTH       (ADDR_WIDTH),
    .SOURCE_WIDTH     (SOURCE_WIDTH),
    .SIZE_WIDTH       (SIZE_WIDTH),
    .OPCODE_WIDTH     (OPCODE_WIDTH),
    .PARAM_WIDTH      (PARAM_WIDTH),
    .SINK_WIDTH       (SINK_WIDTH),
    .BURST_MAX        (BURST_MAX),
    .FIFO_DEPTH       (FIFO_DEPTH),
    .DEPTH            (DEPTH),
    .NUM_BANKS        (NUM_BANKS),
    .BANK_DEPTH       (BANK_DEPTH),
    .RD_LATENCY       (RD_LATENCY),
    .WR_LATENCY       (WR_LATENCY),
    .INIT_FILE        ("")
) dut (
    .m_clk                   (m_clk),
    .reset_m                 (reset_m),
    
    // DMA Control
    .start                   (start),
    .stream_type             (stream_type),
    .dma_mem_base_addr       (dma_mem_base_addr),
    .dma_peri_base_addr      (dma_peri_base_addr),
    .num_bytes               (num_bytes),
    . busy                    (busy),
    .done                    (done),
    
    // SPI Interface
    .spi_wr_evt_valid_tb     (spi_wr_evt_valid_tb),
    .spi_wr_evt_addr_tb      (spi_wr_evt_addr_tb),
    .spi_wr_evt_data_tb      (spi_wr_evt_data_tb),
    .spi_rd_req_valid_tb     (spi_rd_req_valid_tb),
    .spi_rd_req_addr_tb      (spi_rd_req_addr_tb),
    .spi_rd_rsp_valid_tb     (spi_rd_rsp_valid_tb),
    .spi_rd_rsp_data_tb      (spi_rd_rsp_data_tb),
    
    // Debug - DMA
    .memc_write_burst_done   (memc_write_burst_done),
    .memc_read_burst_done    (memc_read_burst_done),
    
    // Debug - Crossbar Master 0
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
    
    // Debug - Memory
    .mem_total_rd_count      (mem_total_rd_count),
    .mem_total_wr_count      (mem_total_wr_count)
);*/



    // =========================================================================
    // Clock Generation - 100MHz (10ns period)
    // =========================================================================
    initial begin
        m_clk = 1'b0;
        forever #5 m_clk = ~m_clk;
    end

    // =========================================================================
    // Reset Control
    // =========================================================================
    initial begin
        reset_m = 1'b1;
        #60;
        reset_m = 1'b0;
    end

    // =========================================================================
    // Main Test State Machine
    // =========================================================================
    always @(posedge m_clk) begin
       // $display("[%0t] ALIVE: spi_req=%b spi_rsp=%b reset=%b", 
         //    $time, spi_rd_req_valid_tb, spi_rd_rsp_valid_tb, reset_m);

        if (reset_m) begin
            test_state <= RESET;
            cycle_count <= 16'd0;
            start <= 1'b0;
            stream_type <= 1'b0;
            dma_mem_base_addr <= 64'd0;
            dma_peri_base_addr <= 64'd0;
            num_bytes <= 16'd0;
            write_count <= 4'd0;
            read_count <= 4'd0;
            busy_prev <= 1'b0;
            done_prev <= 1'b0;
            errors = 0;
        end else begin
            cycle_count <= cycle_count + 16'd1;
            busy_prev <= busy;
            done_prev <= done;
            
            case (test_state)
                RESET: begin
                    if (cycle_count >= 16'd10) begin
                        $display("\n========================================");
                        $display("Complete Integration Test Starting");
                        $display("========================================\n");
                        $display("[%0t] Reset released", $time);
                        test_state <= TEST1_WRITE;
                        cycle_count <= 16'd0;
                    end
                end
                
                TEST1_WRITE: begin
                    if (cycle_count == 16'd0) begin
                        $display("\n[%0t] TEST 1: WRITE - 64 bytes from SPI to Memory at addr 0x0000", $time);
                        stream_type <= 1'b1;  // Write operation
                        dma_mem_base_addr <= 64'h0000_0000_0000_0000;
                        dma_peri_base_addr <= 64'h0000_0000_0000_0000;
                        num_bytes <= 16'd64;  // 64 bytes = 8 beats
                        write_count <= 4'd0;
                    end else if (cycle_count == 16'd1) begin
                        start <= 1'b1;
                        $display("[%0t] Start asserted", $time);
                    end else if (cycle_count == 16'd2) begin
                        start <= 1'b0;
                    end else if (cycle_count >= 16'd3) begin
                        // Detect busy rising edge 
                        if (busy && !busy_prev) begin
                            $display("[%0t] DMA busy asserted", $time);
                        end
                        // Detect done rising edge
                        if (done && !done_prev) begin
                            $display("[%0t] DMA done asserted - WRITE complete", $time);
                            $display("[%0t] Memory writes: %0d, Memory reads: %0d", 
                                     $time, mem_total_wr_count, mem_total_rd_count);
                            test_state <= TEST2_READ;
                            cycle_count <= 16'd0;
                        end
                    end
                end
                
                TEST2_READ: begin
                    if (cycle_count < 16'd10) begin
                        // Wait a bit between tests
                    end else if (cycle_count == 16'd10) begin
                        $display("\n[%0t] TEST 2: READ - 64 bytes from Memory to SPI at addr 0x0000", $time);
                        stream_type <= 1'b0;  // Read operation
                        dma_mem_base_addr <= 64'h0000_0000_0000_0000;
                        dma_peri_base_addr <= 64'h0000_0000_0000_0000;
                        num_bytes <= 16'd64;  // 64 bytes = 8 beats
                        read_count <= 4'd0;
                    end else if (cycle_count == 16'd11) begin
                        start <= 1'b1;
                        $display("[%0t] Start asserted", $time);
                    end else if (cycle_count == 16'd12) begin
                        start <= 1'b0;
                    end else if (cycle_count >= 16'd13) begin
                        // Detect busy rising edge
                        if (busy && !busy_prev) begin
                            $display("[%0t] DMA busy asserted", $time);
                        end
                        // Detect done rising edge
                        if (done && !done_prev) begin
                            $display("[%0t] DMA done asserted - READ complete", $time);
                            $display("[%0t] Memory writes: %0d, Memory reads: %0d", 
                                     $time, mem_total_wr_count, mem_total_rd_count);
                            test_state <= VERIFY;
                            cycle_count <= 16'd0;
                        end
                    end
                end
                
                VERIFY: begin
                    if (cycle_count < 16'd10) begin
                        // Wait a bit before verification
                    end else if (cycle_count == 16'd10) begin
                        // Perform verification
                        $display("\n========================================");
                        $display("DATA VERIFICATION");
                        $display("========================================");
                        
                        errors = 0;
                        for (i = 0; i < 8; i = i + 1) begin
                            if (written_data[i] === read_data[i]) begin
                                $display("[%0d] PASS: Written=0x%016h, Read=0x%h", 
                                         i, written_data[i], read_data[i]);
                            end else begin
                                $display("[%0d] FAIL: Written=0x%016h, Read=0x%h", 
                                         i, written_data[i], read_data[i]);
                                errors = errors + 1;
                            end
                        end
                        
                        $display("\n========================================");
                        if (errors == 0) begin
                            $display("ALL DATA VERIFIED - TEST PASSED");
                        end else begin
                            $display("VERIFICATION FAILED - %0d errors", errors);
                        end
                        $display("========================================");
                        
                        test_state <= DONE;
                        cycle_count <= 16'd0;
                    end
                end
                
                DONE: begin
                    if (cycle_count == 16'd0) begin
                        $display("\n========================================");
                        $display("Test Complete");
                        $display("Final Memory Statistics:");
                        $display("  Total Writes: %0d", mem_total_wr_count);
                        $display("  Total Reads:  %0d", mem_total_rd_count);
                        $display("========================================\n");
                    end else if (cycle_count >= 16'd10) begin
                        $finish;
                    end
                end
                
                default: begin
                    test_state <= RESET;
                end
            endcase
        end
    end

    // =========================================================================
    // SPI Read Request Handler
    // =========================================================================
    always @(posedge m_clk) begin
        if (reset_m) begin
            spi_rd_rsp_valid_tb <= 1'b0;
            spi_rd_rsp_data_tb <= 64'd0;
            spi_rd_req_prev <= 1'b0;
            spi_response_delay <= 4'd0;
        end else begin
            spi_rd_req_prev <= spi_rd_req_valid_tb;

             if (spi_rd_req_valid_tb) begin
        $display("[%0t] SPI_RD_REQ: valid=%b prev=%b state=%0d test_state=%0d", 
                 $time, spi_rd_req_valid_tb, spi_rd_req_prev, test_state, TEST1_WRITE);
    end

           // Debug: Print when request edge detected
        if (spi_rd_req_valid_tb && !spi_rd_req_prev) begin
            $display("[%0t] SPI REQ EDGE: valid=%b prev=%b state=%0d delay=%0d", 
                     $time, spi_rd_req_valid_tb, spi_rd_req_prev, test_state, spi_response_delay);
        end
            
            // Detect rising edge of SPI read request
            if (spi_rd_req_valid_tb && !spi_rd_req_prev && (test_state == TEST1_WRITE)) begin
                $display("[%0t] SPI read request for addr 0x%012h", 
                         $time, spi_rd_req_addr_tb);
                spi_response_delay <= 4'd2;  // Delay 2 cycles before response
            end
            
            // Generate delayed response
            if (spi_response_delay > 4'd0) begin
               $display("[%0t] SPI DELAY: %0d", $time, spi_response_delay);
                spi_response_delay <= spi_response_delay - 4'd1;
                if (spi_response_delay == 4'd1) begin
                    spi_rd_rsp_valid_tb <= 1'b1;
                    spi_rd_rsp_data_tb <= {8{spi_rd_req_addr_tb[7:0]}};  // Pattern data
                    
                    if (write_count < 4'd8) begin
                        written_data[write_count] <= {8{spi_rd_req_addr_tb[7:0]}};
                        $display("[%0t] SPI read response [%0d]: 0x%016h (STORED)", 
                                 $time, write_count, {8{spi_rd_req_addr_tb[7:0]}});
                        write_count <= write_count + 4'd1;
                    end
                end
            end else begin
                spi_rd_rsp_valid_tb <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Capture Read Data from Memory (SPI Write Events)
    // =========================================================================
    always @(posedge m_clk) begin
        if (reset_m) begin
            // Reset handled in main state machine
        end else begin
            if (spi_wr_evt_valid_tb && (test_state == TEST2_READ)) begin
                // During read operation, capture data coming out to SPI
                if (read_count < 4'd8) begin
                    read_data[read_count] <= spi_wr_evt_data_tb;
                    $display("[%0t] SPI Write Event [%0d]: addr=0x%012h data=0x%016h (CAPTURED)", 
                             $time, read_count, spi_wr_evt_addr_tb, spi_wr_evt_data_tb);
                    read_count <= read_count + 4'd1;
                end
            end
        end
    end

    // =========================================================================
    // Signal Monitoring (Optional Debug)
    // =========================================================================
    always @(posedge m_clk) begin
        if (!reset_m) begin
            if (a_valid0_tb && a_ready0_tb) begin
                $display("[%0t] A-Channel: opcode=%0d addr=0x%016h data=0x%016h", 
                         $time, a_opcode0_tb, a_address0_tb, a_data0_tb);
            end
            
            if (d_valid0_tb && d_ready0_tb) begin
                $display("[%0t] D-Channel: opcode=%0d data=0x%016h error=%0b", 
                         $time, d_opcode0_tb, d_data0_tb, d_error0_tb);
            end
        end
    end

    // =========================================================================
    // Initialization
    // =========================================================================
    initial begin
        // Initialize all control signals
        start = 1'b0;
        stream_type = 1'b0;
        dma_mem_base_addr = 64'd0;
        dma_peri_base_addr = 64'd0;
        num_bytes = 16'd0;
        spi_rd_rsp_valid_tb = 1'b0;
        spi_rd_rsp_data_tb = 64'd0;
        write_count = 4'd0;
        read_count = 4'd0;
        test_state = RESET;
        cycle_count = 16'd0;
        spi_rd_req_prev = 1'b0;
        spi_response_delay = 4'd0;
        busy_prev = 1'b0;
        done_prev = 1'b0;
        errors = 0;
    end

  
    initial begin
        #5000000;  // 5ms timeout
        $display("\n[ERROR] Simulation timeout at %0t", $time);
        $finish;
    end

 

endmodule

