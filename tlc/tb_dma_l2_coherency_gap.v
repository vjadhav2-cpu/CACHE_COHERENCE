`timescale 1ns/10ps
`default_nettype none

// Characterizes the DMA-vs-L2-cache coherency gap on
// tlc_l2_dma_shared_mem_top.v: DMA (m2) and L2 (m0) both reach shared
// memory as plain uncached Get/PutFullData clients. Neither
// tlc_slave_mem_manager.v's directory nor rv64g_l2_directory.v ever
// learns that the other has a stake in a line, so nothing invalidates
// L2's cached copy when DMA writes underneath it.
//
// This is a CHARACTERIZATION test: it currently PASSES by confirming the
// hazard is present (L2 serves stale data after a DMA write to the same
// address, with no additional memory traffic -- i.e. a real cache hit on
// stale data). Once a coherency fix is implemented (see
// docs/l2_tlc_xbar_integration_plan.md), this test's expectations should
// flip to require FRESH data, at which point it becomes a real
// regression test for the fix.
module tb_dma_l2_coherency_gap;

    localparam integer CLK_HALF  = 5;
    localparam integer NUM_BEATS = 8;
    localparam [63:0]  TEST_ADDR = 64'h0000_0000_0000_3000; // fresh, unused by other tests

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
    reg [63:0] source_words [0:TOTAL_WORDS-1]; // peripheral-side data DMA writes into memory
    reg [63:0] sink_words   [0:TOTAL_WORDS-1]; // peripheral-side data DMA reads back out
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

    // Acquire address TEST_ADDR with param NtoB (shared read), capture the
    // NUM_BEATS GrantData beats into `capture`, then send GrantAck.
    task automatic l2_acquire_and_capture(output [63:0] capture [0:NUM_BEATS-1]);
        integer n;
        begin
            l2_tl_a_opcode  = 3'd6; // A_ACQUIRE_BLOCK
            l2_tl_a_param   = 3'd0; // NtoB
            l2_tl_a_source  = 6'd0;
            l2_tl_a_address = TEST_ADDR;
            l2_tl_a_valid   = 1'b1;
            while (!l2_tl_a_ready) settle();
            settle();
            l2_tl_a_valid = 1'b0;

            l2_tl_d_ready = 1'b1;
            for (n = 0; n < NUM_BEATS; n = n + 1) begin
                while (!l2_tl_d_valid) settle();
                if (l2_tl_d_opcode !== 3'd5) begin // D_GRANT_DATA
                    $display("[%0t] ERROR: beat %0d d_opcode=%0d, expected GrantData(5)", $time, n, l2_tl_d_opcode);
                    errors = errors + 1;
                end
                capture[n] = l2_tl_d_data;
                settle();
            end
            l2_tl_d_ready = 1'b0;

            l2_tl_e_valid = 1'b1;
            l2_tl_e_sink  = 2'd0;
            settle();
            l2_tl_e_valid = 1'b0;
        end
    endtask

    reg [63:0] l2_data_before_dma [0:NUM_BEATS-1];
    reg [63:0] l2_data_after_dma  [0:NUM_BEATS-1];
    integer rd_count_before_reacquire;
    integer hazard_mismatches;
    integer stale_confirmations;

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

        // === Step 1: L2 caches TEST_ADDR (fresh memory, expect all zero) ===
        $display("\n=== STEP 1: L2 AcquireBlock(NtoB) caches addr 0x%016h ===", TEST_ADDR);
        l2_acquire_and_capture(l2_data_before_dma);
        $display("[%0t] STEP 1: cached beat0=0x%016h beat7=0x%016h (slv0_total_rd_count=%0d)",
                  $time, l2_data_before_dma[0], l2_data_before_dma[7], slv0_total_rd_count);
        wait_cycles(10);

        // === Step 2: DMA writes NEW data to the same address, bypassing L2 ===
        $display("\n=== STEP 2: DMA writes new data to addr 0x%016h (peripheral -> memory) ===", TEST_ADDR);
        for (i = 0; i < NUM_BEATS; i = i + 1) begin
            source_words[i] = 64'hFEED_FACE_0000_0000 + i;
        end
        dma_stream_type    = 1'b1; // peripheral -> memory
        dma_mem_base_addr  = TEST_ADDR;
        dma_peri_base_addr = 64'h0;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);

        // === Step 3: confirm the write really landed, via an independent DMA read-back ===
        $display("\n=== STEP 3: DMA reads back addr 0x%016h to confirm the write landed ===", TEST_ADDR);
        for (i = 0; i < NUM_BEATS; i = i + 1) begin
            sink_words[i] = 64'h0;
            sink_seen[i]  = 1'b0;
        end
        dma_stream_type    = 1'b0; // memory -> peripheral
        dma_mem_base_addr  = TEST_ADDR;
        dma_peri_base_addr = 64'h0;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);

        for (i = 0; i < NUM_BEATS; i = i + 1) begin
            if (!sink_seen[i]) begin
                $display("[%0t] ERROR: STEP 3 missing DMA read-back beat %0d", $time, i);
                errors = errors + 1;
            end else if (sink_words[i] !== source_words[i]) begin
                $display("[%0t] ERROR: STEP 3 DMA read-back beat %0d mismatch exp=0x%016h got=0x%016h",
                          $time, i, source_words[i], sink_words[i]);
                errors = errors + 1;
            end
        end
        $display("[%0t] STEP 3: memory confirmed to hold DMA-written data (0xFEED_FACE...)", $time);
        // Capture the read counter here, right before STEP 4 -- DMA's own
        // write path issues real memory reads too (dma_coherent_agent.v
        // does read-for-ownership: Acquire, modify locally, Release), and
        // so does STEP 3's read-back, so a baseline captured any earlier
        // would wrongly attribute that unrelated traffic to STEP 4.
        rd_count_before_reacquire = slv0_total_rd_count;

        // === Step 4: re-Acquire the SAME address from L2's upstream ===
        $display("\n=== STEP 4: L2 re-Acquires addr 0x%016h -- does it see the DMA's write? ===", TEST_ADDR);
        l2_acquire_and_capture(l2_data_after_dma);
        $display("[%0t] STEP 4: re-acquired beat0=0x%016h beat7=0x%016h (slv0_total_rd_count=%0d)",
                  $time, l2_data_after_dma[0], l2_data_after_dma[7], slv0_total_rd_count);

        // === Verdict ===
        hazard_mismatches   = 0;
        stale_confirmations = 0;
        for (i = 0; i < NUM_BEATS; i = i + 1) begin
            if (l2_data_after_dma[i] !== l2_data_before_dma[i]) begin
                hazard_mismatches = hazard_mismatches + 1;
            end
            if (l2_data_after_dma[i] === l2_data_before_dma[i] &&
                l2_data_after_dma[i] !== source_words[i]) begin
                stale_confirmations = stale_confirmations + 1;
            end
        end

        if (slv0_total_rd_count !== rd_count_before_reacquire) begin
            $display("[%0t] UNEXPECTED: re-Acquire triggered %0d new memory reads -- this would mean L2 did NOT serve from its own cache",
                      $time, slv0_total_rd_count - rd_count_before_reacquire);
            errors = errors + 1;
        end

        if (stale_confirmations == NUM_BEATS && hazard_mismatches == 0) begin
            $display("\n[%0t] HAZARD CONFIRMED (expected on current architecture):", $time);
            $display("  L2 served all %0d beats from its own cache, identical to the pre-DMA values,", NUM_BEATS);
            $display("  even though memory itself now holds the DMA's new data (confirmed in STEP 3).");
            $display("  slv0_total_rd_count did not increase (%0d) -- L2 never re-touched memory.", slv0_total_rd_count);
            $display("  This is the coherency gap docs/l2_tlc_xbar_integration_plan.md flags: DMA writes");
            $display("  are invisible to rv64g_l2_directory.v, so no probe/invalidate ever fires.");
        end else begin
            $display("\n[%0t] UNEXPECTED: hazard NOT reproduced as predicted (mismatches=%0d, stale_confirmations=%0d/%0d) --",
                      $time, hazard_mismatches, stale_confirmations, NUM_BEATS);
            $display("  either something already provides coherency here, or this characterization test's");
            $display("  assumptions about the current architecture are wrong. Investigate before trusting it");
            $display("  as a baseline for testing a fix.");
            errors = errors + 1;
        end

        if (errors != 0) begin
            $display("\nTEST FAILED with %0d errors", errors);
        end else begin
            $display("\nTEST PASSED (hazard successfully characterized)");
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
