`timescale 1ns/10ps
`default_nettype none

// Full-stack test of tlc_l2_dma_shared_mem_top.v: drives a real
// AcquireBlock into rv64g_l2_cache.v on master slot 0 (via l2_tlc_bridge.v)
// and confirms it reaches shared memory over the same crossbar the DMA
// uses on master slot 2, then runs the DMA's own existing write/read
// scenario to confirm adding L2 traffic doesn't disturb it. See
// docs/l2_tlc_xbar_integration_plan.md.
module tb_tlc_l2_dma_shared_mem;

    localparam integer CLK_HALF  = 5;
    localparam integer NUM_BEATS = 8;

    reg clk;
    reg rst_n;

    // L2 upstream (L1-facing) stimulus
    reg  [2:0]  l2_tl_a_opcode;
    reg  [2:0]  l2_tl_a_param;
    reg  [5:0]  l2_tl_a_source;
    reg  [63:0] l2_tl_a_address;
    reg         l2_tl_a_valid;
    wire        l2_tl_a_ready;

    wire [2:0]  l2_tl_b_opcode;
    wire [2:0]  l2_tl_b_param;
    wire [63:0] l2_tl_b_address;
    wire        l2_tl_b_valid;
    reg         l2_tl_b_ready;
    wire [1:0]  l2_tl_b_dest;

    reg  [2:0]  l2_tl_c_opcode;
    reg  [2:0]  l2_tl_c_param;
    reg  [5:0]  l2_tl_c_source;
    reg  [63:0] l2_tl_c_address;
    reg  [63:0] l2_tl_c_data;
    reg         l2_tl_c_valid;
    wire        l2_tl_c_ready;

    wire [2:0]  l2_tl_d_opcode;
    wire [1:0]  l2_tl_d_param;
    wire [63:0] l2_tl_d_data;
    wire [5:0]  l2_tl_d_source;
    wire [1:0]  l2_tl_d_sink;
    wire        l2_tl_d_valid;
    reg         l2_tl_d_ready;

    reg         l2_tl_e_valid;
    reg  [1:0]  l2_tl_e_sink;
    wire        l2_tl_e_ready;

    // Master 1 -- unused, tied off
    // DMA control
    reg         dma_start;
    reg         dma_stream_type;
    reg [63:0]  dma_mem_base_addr;
    reg [63:0]  dma_peri_base_addr;
    reg [15:0]  dma_num_bytes;

    wire        peri_req;
    wire        peri_wr_en;
    wire [63:0] peri_addr;
    wire [63:0] peri_wdata;
    wire [63:0] peri_rdata;
    wire        peri_ack;

    wire        dma_busy;
    wire        dma_done;

    wire [31:0] slv0_total_rd_count;
    wire [31:0] slv0_total_wr_count;
    wire [63:0] slv0_total_rd_latency_sum;
    wire [31:0] slv1_total_rd_count;
    wire [31:0] slv1_total_wr_count;
    wire [63:0] slv1_total_rd_latency_sum;

    localparam integer TOTAL_WORDS = 8;
    reg [63:0] source_words [0:TOTAL_WORDS-1];
    reg [63:0] sink_words   [0:TOTAL_WORDS-1];
    reg        sink_seen    [0:TOTAL_WORDS-1];
    integer errors;
    integer i;

    wire [31:0] peri_word_index = peri_addr[31:3];
    wire        peri_idx_valid  = (peri_word_index < TOTAL_WORDS);

    assign peri_ack   = peri_req;
    assign peri_rdata = peri_idx_valid ? source_words[peri_word_index] : 64'hDEAD_DEAD_DEAD_DEAD;

    initial begin
        clk = 1'b0;
        forever #(CLK_HALF) clk = ~clk;
    end

    tlc_l2_dma_shared_mem_top dut (
        .clk  (clk),
        .rst_n(rst_n),

        .l2_tl_a_opcode (l2_tl_a_opcode),
        .l2_tl_a_param  (l2_tl_a_param),
        .l2_tl_a_source (l2_tl_a_source),
        .l2_tl_a_address(l2_tl_a_address),
        .l2_tl_a_valid  (l2_tl_a_valid),
        .l2_tl_a_ready  (l2_tl_a_ready),

        .l2_tl_b_opcode (l2_tl_b_opcode),
        .l2_tl_b_param  (l2_tl_b_param),
        .l2_tl_b_address(l2_tl_b_address),
        .l2_tl_b_valid  (l2_tl_b_valid),
        .l2_tl_b_ready  (l2_tl_b_ready),
        .l2_tl_b_dest   (l2_tl_b_dest),

        .l2_tl_c_opcode (l2_tl_c_opcode),
        .l2_tl_c_param  (l2_tl_c_param),
        .l2_tl_c_source (l2_tl_c_source),
        .l2_tl_c_address(l2_tl_c_address),
        .l2_tl_c_data   (l2_tl_c_data),
        .l2_tl_c_valid  (l2_tl_c_valid),
        .l2_tl_c_ready  (l2_tl_c_ready),

        .l2_tl_d_opcode (l2_tl_d_opcode),
        .l2_tl_d_param  (l2_tl_d_param),
        .l2_tl_d_data   (l2_tl_d_data),
        .l2_tl_d_source (l2_tl_d_source),
        .l2_tl_d_sink   (l2_tl_d_sink),
        .l2_tl_d_valid  (l2_tl_d_valid),
        .l2_tl_d_ready  (l2_tl_d_ready),

        .l2_tl_e_valid  (l2_tl_e_valid),
        .l2_tl_e_sink   (l2_tl_e_sink),
        .l2_tl_e_ready  (l2_tl_e_ready),

        .m1_req_valid              (1'b0),
        .m1_req_addr               (32'b0),
        .m1_req_type               (3'b0),
        .m1_req_data               (256'b0),
        .m1_req_permissions        (3'b0),
        .m1_req_ready              (),
        .m1_data_valid             (),
        .m1_data                   (),
        .m1_data_error             (),
        .m1_probe_req_valid        (),
        .m1_probe_req_addr         (),
        .m1_probe_req_permissions  (),
        .m1_probe_ack_valid        (1'b0),
        .m1_probe_ack_addr         (32'b0),
        .m1_probe_ack_permissions  (3'b0),
        .m1_probe_ack_dirty_data   (256'b0),

        .dma_start          (dma_start),
        .dma_stream_type    (dma_stream_type),
        .dma_mem_base_addr  (dma_mem_base_addr),
        .dma_peri_base_addr (dma_peri_base_addr),
        .dma_num_bytes      (dma_num_bytes),
        .peri_req           (peri_req),
        .peri_wr_en         (peri_wr_en),
        .peri_addr          (peri_addr),
        .peri_wdata         (peri_wdata),
        .peri_rdata         (peri_rdata),
        .peri_ack           (peri_ack),
        .dma_busy           (dma_busy),
        .dma_done           (dma_done),
        .slv0_total_rd_count      (slv0_total_rd_count),
        .slv0_total_wr_count      (slv0_total_wr_count),
        .slv0_total_rd_latency_sum(slv0_total_rd_latency_sum),
        .slv1_total_rd_count      (slv1_total_rd_count),
        .slv1_total_wr_count      (slv1_total_wr_count),
        .slv1_total_rd_latency_sum(slv1_total_rd_latency_sum)
    );

    task automatic settle;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task wait_cycles;
        input integer count;
        integer n;
        begin
            for (n = 0; n < count; n = n + 1) @(posedge clk);
        end
    endtask

    task pulse_start;
        begin
            @(negedge clk);
            dma_start = 1'b1;
            @(negedge clk);
            dma_start = 1'b0;
        end
    endtask

    task wait_done;
        input integer max_cycles;
        integer cycles;
        begin
            cycles = 0;
            while (!dma_done && (cycles < max_cycles)) begin
                @(posedge clk);
                cycles = cycles + 1;
            end
            if (!dma_done) begin
                $display("[%0t] ERROR: timeout waiting for dma_done", $time);
                errors = errors + 1;
            end
        end
    endtask

    always @(posedge clk) begin
        if (rst_n && peri_req && peri_ack && peri_wr_en) begin
            sink_words[peri_word_index] <= peri_wdata;
            sink_seen[peri_word_index]  <= 1'b1;
        end
    end

    reg [63:0] l2_rdata [0:NUM_BEATS-1];

    initial begin
        rst_n              = 1'b0;
        l2_tl_a_opcode      = 3'b0;
        l2_tl_a_param       = 3'b0;
        l2_tl_a_source      = 6'b0;
        l2_tl_a_address     = 64'b0;
        l2_tl_a_valid       = 1'b0;
        l2_tl_b_ready       = 1'b1;
        l2_tl_c_opcode      = 3'b0;
        l2_tl_c_param       = 3'b0;
        l2_tl_c_source      = 6'b0;
        l2_tl_c_address     = 64'b0;
        l2_tl_c_data        = 64'b0;
        l2_tl_c_valid       = 1'b0;
        l2_tl_d_ready       = 1'b0;
        l2_tl_e_valid       = 1'b0;
        l2_tl_e_sink        = 2'b0;

        dma_start          = 1'b0;
        dma_stream_type    = 1'b0;
        dma_mem_base_addr  = 64'h0;
        dma_peri_base_addr = 64'h0;
        dma_num_bytes      = 16'd0;
        errors             = 0;

        for (i = 0; i < TOTAL_WORDS; i = i + 1) begin
            source_words[i] = 64'h0;
            sink_words[i]   = 64'h0;
            sink_seen[i]    = 1'b0;
        end

        wait_cycles(10);
        rst_n = 1'b1;
        wait_cycles(10);

        // === TEST 1: L2 AcquireBlock read miss reaches shared memory via m0 ===
        $display("\n=== TEST 1: L2 AcquireBlock(NtoB) read miss through shared crossbar ===");
        l2_tl_a_opcode  = 3'd6; // A_ACQUIRE_BLOCK
        l2_tl_a_param   = 3'd0; // NtoB
        l2_tl_a_source  = 6'd0;
        l2_tl_a_address = 64'h0000_0000_0000_2000; // 64B aligned
        l2_tl_a_valid   = 1'b1;
        while (!l2_tl_a_ready) settle();
        settle();
        l2_tl_a_valid = 1'b0;

        l2_tl_d_ready = 1'b1;
        for (i = 0; i < NUM_BEATS; i = i + 1) begin
            while (!l2_tl_d_valid) settle();
            if (l2_tl_d_opcode !== 3'd5) begin // D_GRANT_DATA
                $display("[%0t] ERROR: beat %0d d_opcode=%0d, expected GrantData(5)", $time, i, l2_tl_d_opcode);
                errors = errors + 1;
            end
            l2_rdata[i] = l2_tl_d_data;
            settle();
        end
        l2_tl_d_ready = 1'b0;
        $display("[%0t] TEST 1: received %0d GrantData beats, first=0x%016h last=0x%016h",
                  $time, NUM_BEATS, l2_rdata[0], l2_rdata[NUM_BEATS-1]);

        // GrantAck
        l2_tl_e_valid = 1'b1;
        l2_tl_e_sink  = 2'd0;
        settle();
        l2_tl_e_valid = 1'b0;
        wait_cycles(10);

        if (slv0_total_rd_count < NUM_BEATS) begin
            $display("[%0t] ERROR: expected at least %0d slv0 reads from L2 refill, got %0d",
                      $time, NUM_BEATS, slv0_total_rd_count);
            errors = errors + 1;
        end else begin
            $display("[%0t] TEST 1 PASS: slv0_total_rd_count=%0d confirms L2 refill reached shared memory",
                      $time, slv0_total_rd_count);
        end

        // === TEST 2: DMA still works concurrently sharing the same crossbar/memory ===
        $display("\n=== TEST 2: DMA write+read through shared crossbar (L2 present on m0) ===");
        for (i = 0; i < NUM_BEATS; i = i + 1) begin
            source_words[i] = 64'h1122_3344_5500_0000 + i;
            sink_words[i]   = 64'h0;
            sink_seen[i]    = 1'b0;
        end

        dma_stream_type    = 1'b1; // peripheral -> memory
        dma_mem_base_addr  = 64'h0000_0000_0000_0000;
        dma_peri_base_addr = 64'h0000_0000_0000_0000;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);

        for (i = 0; i < NUM_BEATS; i = i + 1) begin
            sink_words[i] = 64'h0;
            sink_seen[i]  = 1'b0;
        end

        dma_stream_type    = 1'b0; // memory -> peripheral
        dma_mem_base_addr  = 64'h0000_0000_0000_0000;
        dma_peri_base_addr = 64'h0000_0000_0000_0000;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);

        for (i = 0; i < NUM_BEATS; i = i + 1) begin
            if (!sink_seen[i]) begin
                $display("[%0t] ERROR: TEST 2 missing sink beat %0d", $time, i);
                errors = errors + 1;
            end else if (sink_words[i] !== source_words[i]) begin
                $display("[%0t] ERROR: TEST 2 beat %0d mismatch exp=0x%016h got=0x%016h",
                          $time, i, source_words[i], sink_words[i]);
                errors = errors + 1;
            end else begin
                $display("[%0t] TEST 2 PASS: beat %0d data=0x%016h", $time, i, sink_words[i]);
            end
        end

        if (errors != 0) begin
            $display("\nTEST FAILED with %0d errors", errors);
        end else begin
            $display("\nTEST PASSED");
        end
        $finish;
    end

    initial begin
        #2_000_000;
        $display("[TB][FAIL] GLOBAL TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
