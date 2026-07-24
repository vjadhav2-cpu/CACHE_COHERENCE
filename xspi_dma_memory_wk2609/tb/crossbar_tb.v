`timescale 1ns / 1ps

module tb_dual_master;

// TileLink parameters
parameter TL_ADDR_WIDTH = 64;
parameter TL_DATA_WIDTH = 64;
parameter TL_SIZE_WIDTH = 8;
parameter TL_MASK_WIDTH = 8;
parameter TL_SOURCE_WIDTH = 8;
parameter TL_SINK_WIDTH = 8;
parameter TL_OPCODE_WIDTH = 3;
parameter TL_PARAM_WIDTH = 3;
parameter FIFO_DEPTH = 16;

// TileLink opcodes
parameter PUT_FULL_DATA_A = 3'd0;
parameter GET_A = 3'd4;
parameter ACCESS_ACK_D = 3'd0;
parameter ACCESS_ACK_DATA_D = 3'd1;

// Clock and reset
reg m_clk, s_clk;
reg reset_m, reset_s;

// Test control
integer test_num;
integer passed_tests;
reg [63:0] expected_data_m0, expected_data_m1;
reg [63:0] read_data_m0, read_data_m1;
reg [2:0] captured_opcode_m0, captured_opcode_m1;

// Master 0 A-channel inputs
reg [TL_OPCODE_WIDTH-1:0]  a_opcode_in0;
reg [TL_PARAM_WIDTH-1:0]   a_param_in0;
reg [TL_SIZE_WIDTH-1:0]    a_size_in0;
reg [TL_SOURCE_WIDTH-1:0]  a_source_in0;
reg [TL_ADDR_WIDTH-1:0]    a_address_in0;
reg [TL_MASK_WIDTH-1:0]    a_mask_in0;
reg [TL_DATA_WIDTH-1:0]    a_data_in0;
reg                        a_valid_in0;

// Master 1 A-channel inputs
reg [TL_OPCODE_WIDTH-1:0]  a_opcode_in1;
reg [TL_PARAM_WIDTH-1:0]   a_param_in1;
reg [TL_SIZE_WIDTH-1:0]    a_size_in1;
reg [TL_SOURCE_WIDTH-1:0]  a_source_in1;
reg [TL_ADDR_WIDTH-1:0]    a_address_in1;
reg [TL_MASK_WIDTH-1:0]    a_mask_in1;
reg [TL_DATA_WIDTH-1:0]    a_data_in1;
reg                        a_valid_in1;

// Master 0 signals from system
wire [TL_MASK_WIDTH-1:0]   a_mask0_tb;
wire [TL_DATA_WIDTH-1:0]   a_data0_tb;
wire [TL_SOURCE_WIDTH-1:0] a_source0_tb;
wire                       a_valid0_tb;
wire                       a_ready0_tb;
wire [TL_OPCODE_WIDTH-1:0] d_opcode0_tb;
wire [TL_PARAM_WIDTH-1:0]  d_param0_tb;
wire [TL_SIZE_WIDTH-1:0]   d_size0_tb;
wire [TL_SINK_WIDTH-1:0]   d_sink0_tb;
wire [TL_SOURCE_WIDTH-1:0] d_source0_tb;
wire [TL_DATA_WIDTH-1:0]   d_data0_tb;
wire                       d_valid0_tb;
wire                       d_ready0_tb;
wire                       d_error0_tb;

// Master 1 signals from system
wire [TL_MASK_WIDTH-1:0]   a_mask1_tb;
wire [TL_DATA_WIDTH-1:0]   a_data1_tb;
wire [TL_SOURCE_WIDTH-1:0] a_source1_tb;
wire                       a_valid1_tb;
wire                       a_ready1_tb;
wire [TL_OPCODE_WIDTH-1:0] d_opcode1_tb;
wire [TL_PARAM_WIDTH-1:0]  d_param1_tb;
wire [TL_SIZE_WIDTH-1:0]   d_size1_tb;
wire [TL_SINK_WIDTH-1:0]   d_sink1_tb;
wire [TL_SOURCE_WIDTH-1:0] d_source1_tb;
wire [TL_DATA_WIDTH-1:0]   d_data1_tb;
wire                       d_valid1_tb;
wire                       d_ready1_tb;
wire                       d_error1_tb;

// Unused master 2 - tie off
wire a_valid2_tb, a_ready2_tb;
wire d_valid2_tb, d_ready2_tb;

// Clock generation
initial begin
    m_clk = 0;
    forever #5 m_clk = ~m_clk;  // 100MHz
end

initial begin
    s_clk = 0;
    forever #7.5 s_clk = ~s_clk;  // 66MHz
end

// Instantiate DUT
tilelink_uh_3M_2S #(
    .TL_ADDR_WIDTH(TL_ADDR_WIDTH),
    .TL_DATA_WIDTH(TL_DATA_WIDTH),
    .TL_SIZE_WIDTH(TL_SIZE_WIDTH),
    .TL_MASK_WIDTH(TL_MASK_WIDTH),
    .TL_SOURCE_WIDTH(TL_SOURCE_WIDTH),
    .TL_SINK_WIDTH(TL_SINK_WIDTH),
    .TL_OPCODE_WIDTH(TL_OPCODE_WIDTH),
    .TL_PARAM_WIDTH(TL_PARAM_WIDTH),
    .FIFO_DEPTH(FIFO_DEPTH)
) dut (
    .m_clk(m_clk),
    .s_clk(s_clk),
    .reset_m(reset_m),
    .reset_s(reset_s),
    
    // Master 0 inputs
    .a_valid_in0(a_valid_in0),
    .a_opcode_in0(a_opcode_in0),
    .a_param_in0(a_param_in0),
    .a_address_in0(a_address_in0),
    .a_size_in0(a_size_in0),
    .a_mask_in0(a_mask_in0),
    .a_data_in0(a_data_in0),
    .a_source_in0(a_source_in0),
    
    // Master 1 inputs
    .a_valid_in1(a_valid_in1),
    .a_opcode_in1(a_opcode_in1),
    .a_param_in1(a_param_in1),
    .a_address_in1(a_address_in1),
    .a_size_in1(a_size_in1),
    .a_mask_in1(a_mask_in1),
    .a_data_in1(a_data_in1),
    .a_source_in1(a_source_in1),
    
    // Master 2 - UNUSED
    .a_valid_in2(1'b0),
    .a_opcode_in2(3'b0),
    .a_param_in2(3'b0),
    .a_address_in2(64'b0),
    .a_size_in2(8'b0),
    .a_mask_in2(8'b0),
    .a_data_in2(64'b0),
    .a_source_in2(8'b0),
    
    // Master 0 outputs
    .a_address0_tb(),
    .a_data0_tb(a_data0_tb),
    .a_opcode0_tb(),
    .a_param0_tb(),
    .a_size0_tb(),
    .a_mask0_tb(a_mask0_tb),
    .a_source0_tb(a_source0_tb),
    .a_valid0_tb(a_valid0_tb),
    .a_ready0_tb(a_ready0_tb),
    .d_opcode0_tb(d_opcode0_tb),
    .d_param0_tb(d_param0_tb),
    .d_size0_tb(d_size0_tb),
    .d_sink0_tb(d_sink0_tb),
    .d_source0_tb(d_source0_tb),
    .d_data0_tb(d_data0_tb),
    .d_valid0_tb(d_valid0_tb),
    .d_ready0_tb(d_ready0_tb),
    .d_error0_tb(d_error0_tb),
    
    // Master 1 outputs
    .a_address1_tb(),
    .a_data1_tb(a_data1_tb),
    .a_opcode1_tb(),
    .a_param1_tb(),
    .a_size1_tb(),
    .a_mask1_tb(a_mask1_tb),
    .a_source1_tb(a_source1_tb),
    .a_valid1_tb(a_valid1_tb),
    .a_ready1_tb(a_ready1_tb),
    .d_opcode1_tb(d_opcode1_tb),
    .d_param1_tb(d_param1_tb),
    .d_size1_tb(d_size1_tb),
    .d_sink1_tb(d_sink1_tb),
    .d_source1_tb(d_source1_tb),
    .d_data1_tb(d_data1_tb),
    .d_valid1_tb(d_valid1_tb),
    .d_ready1_tb(d_ready1_tb),
    .d_error1_tb(d_error1_tb),
    
    // Master 2 outputs - tie off
    .a_address2_tb(), .a_data2_tb(), .a_opcode2_tb(), .a_param2_tb(),
    .a_size2_tb(), .a_mask2_tb(), .a_source2_tb(),
    .a_valid2_tb(a_valid2_tb), .a_ready2_tb(a_ready2_tb),
    .d_opcode2_tb(), .d_param2_tb(), .d_size2_tb(), .d_sink2_tb(),
    .d_source2_tb(), .d_data2_tb(), .d_valid2_tb(d_valid2_tb), .d_ready2_tb(d_ready2_tb), .d_error2_tb()
);

// Initialize Master 0 signals
task init_master0;
begin
    a_opcode_in0 = 0;
    a_param_in0 = 0;
    a_size_in0 = 0;
    a_source_in0 = 0;
    a_address_in0 = 0;
    a_mask_in0 = 0;
    a_data_in0 = 0;
    a_valid_in0 = 0;
end
endtask

// Initialize Master 1 signals
task init_master1;
begin
    a_opcode_in1 = 0;
    a_param_in1 = 0;
    a_size_in1 = 0;
    a_source_in1 = 0;
    a_address_in1 = 0;
    a_mask_in1 = 0;
    a_data_in1 = 0;
    a_valid_in1 = 0;
end
endtask

// Main test sequence
initial begin
    $dumpfile("tb_dual_master.vcd");
    $dumpvars(0, tb_dual_master);
    
    test_num = 0;
    passed_tests = 0;
    reset_m = 1;
    reset_s = 1;
    init_master0();
    init_master1();
    
    $display("\n================================================================");
    $display("  Dual Master Test: Master 0 to Slave 0, Master 1 to Slave 1");
    $display("  Testing beat and burst operations with CDC");
    $display("================================================================\n");
    
    // Release reset
    repeat(10) @(posedge m_clk);
    reset_m = 0;
    reset_s = 0;
    repeat(10) @(posedge m_clk);
    
    // ========================================================================
    // TEST 1: Master 0 - Single Beat Write + Read to Slave 0 (0x100)
    // ========================================================================
    test_num = 1;
    expected_data_m0 = 64'hDEAD_BEEF_CAFE_BABE;
    
    $display("[Test %0d] Master 0: Single Beat Write to Slave 0 (0x100)", test_num);
    @(posedge m_clk);
    a_valid_in0 = 1;
    a_opcode_in0 = PUT_FULL_DATA_A;
    a_address_in0 = 64'h0100;  // Slave 0 address
    a_size_in0 = 3;
    a_mask_in0 = 8'hFF;
    a_data_in0 = expected_data_m0;
    a_source_in0 = 0;
    
    wait(a_ready0_tb === 1'b1);
    @(posedge m_clk);
    a_valid_in0 = 0;
    
    while (!(d_valid0_tb === 1'b1 && d_source0_tb === 0)) @(posedge m_clk);
    if (d_opcode0_tb == ACCESS_ACK_D) begin
        $display("          ✓ Master 0 Write completed");
    end
    @(posedge m_clk);
    
    repeat(50) @(posedge m_clk);
    
    $display("[Test %0d] Master 0: Single Beat Read from Slave 0 (0x100)", test_num);
    a_valid_in0 = 1;
    a_opcode_in0 = GET_A;
    a_address_in0 = 64'h0100;
    a_size_in0 = 3;
    a_mask_in0 = 8'hFF;
    a_data_in0 = 0;
    a_source_in0 = 1;
    
    wait(a_ready0_tb === 1'b1);
    @(posedge m_clk);
    a_valid_in0 = 0;
    
    while (!(d_valid0_tb === 1'b1 && d_source0_tb === 1)) @(posedge m_clk);
    read_data_m0 = d_data0_tb;
    captured_opcode_m0 = d_opcode0_tb;
    @(posedge m_clk);
    
    if (captured_opcode_m0 == ACCESS_ACK_DATA_D && read_data_m0 == expected_data_m0) begin
        passed_tests = passed_tests + 1;
        $display("          ✓ Master 0 Read data matches! (0x%h)", read_data_m0);
        $display("          ✓ TEST 1 PASS\n");
    end else begin
        $display("          ✗ Master 0 Read failed! Got 0x%h, expected 0x%h", read_data_m0, expected_data_m0);
        $display("          ✗ TEST 1 FAIL\n");
    end
    
    repeat(100) @(posedge m_clk);
    
    // Ensure Master 0 is idle before starting Test 2
    init_master0();
    repeat(10) @(posedge m_clk);
    
    // ========================================================================
    // TEST 2: Master 1 - Single Beat Write + Read to Slave 1 (0x250) 
    // ========================================================================
    test_num = 2;
    expected_data_m1 = 64'h1234_5678_9ABC_DEF0;
    
    $display("[Test %0d] Master 1: Single Beat Write to Slave 1 (0x250)", test_num);
    @(posedge m_clk);
    a_valid_in1 = 1;
    a_opcode_in1 = PUT_FULL_DATA_A;
    a_address_in1 = 64'h0250;  // Slave 1 address range: 0x200-0x3FF
    a_size_in1 = 3;
    a_mask_in1 = 8'hFF;
    a_data_in1 = expected_data_m1;
    a_source_in1 = 4;
    $display("          [TB] Time=%0t Asserting a_valid_in1, waiting for a_ready1_tb", $time);
    
    wait(a_ready1_tb === 1'b1);
    a_valid_in1 = 0;  // FIX: Deassert immediately to prevent multiple cycles with valid=1
    $display("          [TB] Time=%0t Got a_ready1_tb, deasserted a_valid_in1", $time);
    @(posedge m_clk);
    
    while (!(d_valid1_tb === 1'b1 && d_source1_tb === 4)) @(posedge m_clk);
    if (d_opcode1_tb == ACCESS_ACK_D) begin
        $display("          ✓ Master 1 Write completed");
    end
    @(posedge m_clk);
    
    repeat(50) @(posedge m_clk);
    
    $display("[Test %0d] Master 1: Single Beat Read from Slave 1 (0x250)", test_num);
    a_valid_in1 = 1;
    a_opcode_in1 = GET_A;
    a_address_in1 = 64'h0250;
    a_size_in1 = 3;
    a_mask_in1 = 8'hFF;
    a_data_in1 = 0;
    a_source_in1 = 5;
    $display("          [TB] Time=%0t Asserting a_valid_in1 for READ, waiting for a_ready1_tb", $time);
    
    wait(a_ready1_tb === 1'b1);
    a_valid_in1 = 0;  // FIX: Deassert immediately
    $display("          [TB] Time=%0t Got a_ready1_tb for READ, deasserted a_valid_in1", $time);
    @(posedge m_clk);
    
    while (!(d_valid1_tb === 1'b1 && d_source1_tb === 5)) @(posedge m_clk);
    read_data_m1 = d_data1_tb;
    captured_opcode_m1 = d_opcode1_tb;
    @(posedge m_clk);
    
    if (captured_opcode_m1 == ACCESS_ACK_DATA_D && read_data_m1 == expected_data_m1) begin
        passed_tests = passed_tests + 1;
        $display("          ✓ Master 1 Read data matches! (0x%h)", read_data_m1);
        $display("          ✓ TEST 2 PASS\n");
    end else begin
        $display("          ✗ Master 1 Read failed! Got 0x%h, expected 0x%h", read_data_m1, expected_data_m1);
        $display("          ✗ TEST 2 FAIL\n");
    end
    
    repeat(100) @(posedge m_clk);
    
    // ========================================================================
    // TEST 3: Master 0 - Burst Write + Read to Slave 0 (0x080)
    // ========================================================================
    test_num = 3;
    expected_data_m0 = 64'hFEDC_BA98_7654_3210;
    
    $display("[Test %0d] Master 0: Burst Write to Slave 0 (0x080, 4 beats)", test_num);
    @(posedge m_clk);
    a_valid_in0 = 1;
    a_opcode_in0 = PUT_FULL_DATA_A;
    a_address_in0 = 64'h0080;
    a_size_in0 = 5;  // 2^5 = 32 bytes = 4 beats of 8 bytes each
    a_mask_in0 = 8'hFF;
    a_data_in0 = expected_data_m0;
    a_source_in0 = 2;
    
    // Wait for ready and send initial burst request  
    wait(a_ready0_tb === 1'b1);
    @(posedge m_clk);
    $display("          ✓ Master 0 Burst request sent (first beat data=0x%h)", a_data_in0);
    
    // For burst writes, keep sending data beats until master is ready
    for (integer beat = 1; beat < 4; beat = beat + 1) begin
        a_data_in0 = expected_data_m0 + beat;
        // Don't wait for ready - just keep sending data
        @(posedge m_clk);
        $display("          ✓ Master 0 Beat %0d data sent (data=0x%h)", beat+1, a_data_in0);
    end
    
    // Clear valid after sending all data
    a_valid_in0 = 0;
    
    while (!(d_valid0_tb === 1'b1 && d_source0_tb === 2)) @(posedge m_clk);
    if (d_opcode0_tb == ACCESS_ACK_D) begin
        $display("          ✓ Master 0 Burst write completed");
    end
    @(posedge m_clk);
    
    repeat(50) @(posedge m_clk);
    
    $display("[Test %0d] Master 0: Burst Read from Slave 0 (0x080, 4 beats)", test_num);
    a_valid_in0 = 1;
    a_opcode_in0 = GET_A;
    a_address_in0 = 64'h0080;
    a_size_in0 = 5;
    a_mask_in0 = 8'hFF;
    a_data_in0 = 0;
    a_source_in0 = 3;
    
    wait(a_ready0_tb === 1'b1);
    @(posedge m_clk);
    a_valid_in0 = 0;
    
    // Wait for first beat
    while (!(d_valid0_tb === 1'b1 && d_source0_tb === 3)) @(posedge m_clk);
    read_data_m0 = d_data0_tb;
    $display("          ✓ Master 0 Beat 1 received (data=0x%h)", read_data_m0);
    if (read_data_m0 == expected_data_m0) begin
        passed_tests = passed_tests + 1;
    end
    @(posedge m_clk);
    
    // Receive remaining 3 beats
    for (integer beat = 1; beat < 4; beat = beat + 1) begin
        while (!(d_valid0_tb === 1'b1 && d_source0_tb === 3)) @(posedge m_clk);
        read_data_m0 = d_data0_tb;
        $display("          ✓ Master 0 Beat %0d received (data=0x%h)", beat+1, read_data_m0);
        @(posedge m_clk);
    end
    
    if (passed_tests == 3) begin
        $display("          ✓ TEST 3 PASS\n");
    end else begin
        $display("          ✗ TEST 3 FAIL\n");
    end
    
    repeat(100) @(posedge m_clk);
    
    // ========================================================================
    // TEST 4: Master 1 - Burst Write + Read to Slave 1 (0x300)
    // ========================================================================
    test_num = 4;
    expected_data_m1 = 64'hAAAA_BBBB_CCCC_DDDD;
    
    // ========================================================================
    // TEST 4: Master 1 - Burst Write + Read to Slave 1 (0x300)
    // ========================================================================
    test_num = 4;
    expected_data_m1 = 64'hAAAA_BBBB_CCCC_DDDD;
    
    $display("[Test %0d] Master 1: Burst Write to Slave 1 (0x300, 4 beats)", test_num);
    @(posedge m_clk);
    a_valid_in1 = 1;
    a_opcode_in1 = PUT_FULL_DATA_A;
    a_address_in1 = 64'h0300;  // Slave 1 address range: 0x200-0x3FF
    a_size_in1 = 5;  // 2^5 = 32 bytes = 4 beats of 8 bytes each
    a_mask_in1 = 8'hFF;
    a_data_in1 = expected_data_m1;
    a_source_in1 = 6;
    
    // Wait for ready and send initial burst request
    wait(a_ready1_tb === 1'b1);
    @(posedge m_clk);
    $display("          ✓ Master 1 Burst request sent (first beat data=0x%h)", a_data_in1);
    
    // For burst writes, keep sending data beats until master is ready
    for (integer beat = 1; beat < 4; beat = beat + 1) begin
        a_data_in1 = expected_data_m1 + beat;
        // Don't wait for ready - just keep sending data
        @(posedge m_clk);
        $display("          ✓ Master 1 Beat %0d data sent (data=0x%h)", beat+1, a_data_in1);
    end
    
    // Clear valid after sending all data
    a_valid_in1 = 0;
    
    while (!(d_valid1_tb === 1'b1 && d_source1_tb === 6)) @(posedge m_clk);
    if (d_opcode1_tb == ACCESS_ACK_D) begin
        $display("          ✓ Master 1 Burst write completed");
    end
    @(posedge m_clk);
    
    repeat(50) @(posedge m_clk);
    
    $display("[Test %0d] Master 1: Burst Read from Slave 1 (0x300, 4 beats)", test_num);
    a_valid_in1 = 1;
    a_opcode_in1 = GET_A;
    a_address_in1 = 64'h0300;
    a_size_in1 = 5;
    a_mask_in1 = 8'hFF;
    a_data_in1 = 0;
    a_source_in1 = 7;
    
    wait(a_ready1_tb === 1'b1);
    @(posedge m_clk);
    a_valid_in1 = 0;
    
    // Wait for first beat
    while (!(d_valid1_tb === 1'b1 && d_source1_tb === 7)) @(posedge m_clk);
    read_data_m1 = d_data1_tb;
    $display("          ✓ Master 1 Beat 1 received (data=0x%h)", read_data_m1);
    if (read_data_m1 == expected_data_m1) begin
        passed_tests = passed_tests + 1;
    end
    @(posedge m_clk);
    
    // Receive remaining 3 beats
    for (integer beat = 1; beat < 4; beat = beat + 1) begin
        while (!(d_valid1_tb === 1'b1 && d_source1_tb === 7)) @(posedge m_clk);
        read_data_m1 = d_data1_tb;
        $display("          ✓ Master 1 Beat %0d received (data=0x%h)", beat+1, read_data_m1);
        @(posedge m_clk);
    end
    
    if (passed_tests == 4) begin
        $display("          ✓ TEST 4 PASS\n");
    end else begin
        $display("          ✗ TEST 4 FAIL\n");
    end
    
    repeat(20) @(posedge m_clk);
    
    // Summary
    $display("\n================================================================");
    $display("  TEST SUMMARY");
    $display("================================================================");
    $display("  Test 1 - M0 Beat Write+Read:   %s", (passed_tests >= 1) ? "✓ PASS" : "✗ FAIL");
    $display("  Test 2 - M1 Beat Write+Read:   %s", (passed_tests >= 2) ? "✓ PASS" : "✗ FAIL");
    $display("  Test 3 - M0 Burst Write+Read:  %s", (passed_tests >= 3) ? "✓ PASS" : "✗ FAIL");
    $display("  Test 4 - M1 Burst Write+Read:  %s", (passed_tests >= 4) ? "✓ PASS" : "✗ FAIL");
    $display("\n  Result: %0d/4 tests passed", passed_tests);
    $display("================================================================\n");
    
    $finish;
end

// Timeout
initial begin
    #5000000;
    $display("\n⚠ ERROR: Simulation timeout!");
    $finish;
end

endmodule
