`timescale 1ns/10ps
`default_nettype none

module tb_tlc_dma_system_basic_v3;

    localparam integer CLK_HALF    = 5;
    localparam integer NUM_BEATS   = 8;
    localparam integer TOTAL_WORDS = 24;

    reg clk;
    reg rst_n;

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

    reg [63:0] source_words [0:TOTAL_WORDS-1];
    reg [63:0] sink_words   [0:TOTAL_WORDS-1];
    reg [TOTAL_WORDS-1:0] source_seen;
    reg [TOTAL_WORDS-1:0] sink_seen;
    integer errors;
    integer i;
    integer src_hs_count;
    integer sink_hs_count;

    wire [31:0] peri_word_index = peri_addr[31:3];
    wire        peri_idx_valid  = (peri_word_index < TOTAL_WORDS);

    assign peri_ack   = peri_req;
    assign peri_rdata = peri_idx_valid ? source_words[peri_word_index] : 64'hDEAD_DEAD_DEAD_DEAD;

    initial begin
        clk = 1'b0;
        forever #(CLK_HALF) clk = ~clk;
    end

    tlc_dma_system_with_wrapper_top #(
        .MEM_INIT_FILE("tlc/tb_tlc_dma_system_basic_v3_init.mem")
    ) dut (
        .clk                      (clk),
        .rst_n                    (rst_n),
        .m0_req_valid             (1'b0),
        .m0_req_addr              (32'b0),
        .m0_req_type              (3'b0),
        .m0_req_data              (256'b0),
        .m0_req_permissions       (3'b0),
        .m0_req_ready             (),
        .m0_data_valid            (),
        .m0_data                  (),
        .m0_data_error            (),
        .m0_probe_req_valid       (),
        .m0_probe_req_addr        (),
        .m0_probe_req_permissions (),
        .m0_probe_ack_valid       (1'b0),
        .m0_probe_ack_addr        (32'b0),
        .m0_probe_ack_permissions (3'b0),
        .m0_probe_ack_dirty_data  (256'b0),
        .m1_req_valid             (1'b0),
        .m1_req_addr              (32'b0),
        .m1_req_type              (3'b0),
        .m1_req_data              (256'b0),
        .m1_req_permissions       (3'b0),
        .m1_req_ready             (),
        .m1_data_valid            (),
        .m1_data                  (),
        .m1_data_error            (),
        .m1_probe_req_valid       (),
        .m1_probe_req_addr        (),
        .m1_probe_req_permissions (),
        .m1_probe_ack_valid       (1'b0),
        .m1_probe_ack_addr        (32'b0),
        .m1_probe_ack_permissions (3'b0),
        .m1_probe_ack_dirty_data  (256'b0),
        .dma_start                (dma_start),
        .dma_stream_type          (dma_stream_type),
        .dma_mem_base_addr        (dma_mem_base_addr),
        .dma_peri_base_addr       (dma_peri_base_addr),
        .dma_num_bytes            (dma_num_bytes),
        .peri_req                 (peri_req),
        .peri_wr_en               (peri_wr_en),
        .peri_addr                (peri_addr),
        .peri_wdata               (peri_wdata),
        .peri_rdata               (peri_rdata),
        .peri_ack                 (peri_ack),
        .dma_busy                 (dma_busy),
        .dma_done                 (dma_done),
        .slv0_total_rd_count      (slv0_total_rd_count),
        .slv0_total_wr_count      (slv0_total_wr_count),
        .slv0_total_rd_latency_sum(slv0_total_rd_latency_sum),
        .slv1_total_rd_count      (slv1_total_rd_count),
        .slv1_total_wr_count      (slv1_total_wr_count),
        .slv1_total_rd_latency_sum(slv1_total_rd_latency_sum)
    );

    task wait_cycles;
        input integer count;
        integer n;
        begin
            for (n = 0; n < count; n = n + 1)
                @(posedge clk);
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
                $fatal(1);
            end
        end
    endtask

    task program_source_range;
        input integer base_idx;
        input [63:0] base_data;
        integer n;
        begin
            for (n = 0; n < NUM_BEATS; n = n + 1) begin
                source_words[base_idx + n] = base_data + n;
                source_seen[base_idx + n]  = 1'b0;
            end
        end
    endtask

    task clear_sink_range;
        input integer base_idx;
        integer n;
        begin
            for (n = 0; n < NUM_BEATS; n = n + 1) begin
                sink_words[base_idx + n] = 64'h0;
                sink_seen[base_idx + n]  = 1'b0;
            end
        end
    endtask

    task verify_sink_range;
        input integer base_idx;
        input integer test_id;
        integer n;
        begin
            for (n = 0; n < NUM_BEATS; n = n + 1) begin
                if (!sink_seen[base_idx + n]) begin
                    $display("[%0t] ERROR: test %0d missing sink beat idx=%0d abs_idx=%0d",
                             $time, test_id, n, base_idx + n);
                    errors = errors + 1;
                end else if (sink_words[base_idx + n] !== source_words[base_idx + n]) begin
                    $display("[%0t] ERROR: test %0d beat %0d mismatch exp=0x%016h got=0x%016h",
                             $time, test_id, n, source_words[base_idx + n], sink_words[base_idx + n]);
                    errors = errors + 1;
                end else begin
                    $display("[%0t] PASS: test %0d beat %0d data=0x%016h",
                             $time, test_id, n, sink_words[base_idx + n]);
                end
            end
        end
    endtask


    always @(posedge clk) begin
        if (rst_n && peri_req && peri_ack) begin
            if (!peri_idx_valid) begin
                $display("[%0t] ERROR: peripheral address out of programmed range addr=0x%016h", $time, peri_addr);
                $fatal(1);
            end

            if (!peri_wr_en) begin
                src_hs_count <= src_hs_count + 1;
                source_seen[peri_word_index] <= 1'b1;
                $display("[%0t] PERI->DMA hs addr=0x%016h idx=%0d data=0x%016h",
                         $time, peri_addr, peri_word_index, peri_rdata);
            end else begin
                sink_hs_count <= sink_hs_count + 1;
                sink_words[peri_word_index] <= peri_wdata;
                sink_seen[peri_word_index] <= 1'b1;
                $display("[%0t] DMA->PERI hs addr=0x%016h idx=%0d data=0x%016h",
                         $time, peri_addr, peri_word_index, peri_wdata);
            end
        end
    end

    initial begin
        rst_n              = 1'b0;
        dma_start          = 1'b0;
        dma_stream_type    = 1'b0;
        dma_mem_base_addr  = 64'h0;
        dma_peri_base_addr = 64'h0;
        dma_num_bytes      = 16'd0;
        source_seen        = {TOTAL_WORDS{1'b0}};
        sink_seen          = {TOTAL_WORDS{1'b0}};
        src_hs_count       = 0;
        sink_hs_count      = 0;
        errors             = 0;

        for (i = 0; i < TOTAL_WORDS; i = i + 1) begin
            source_words[i] = 64'h0;
            sink_words[i]   = 64'h0;
        end

        wait_cycles(10);
        rst_n = 1'b1;
        wait_cycles(10);

        program_source_range(0, 64'h1122_3344_5500_0000);
        clear_sink_range(0);

        $display("\n=== TEST 1: Peripheral to Memory DMA write through TL-C ===");
        dma_stream_type    = 1'b1;
        dma_mem_base_addr  = 64'h0000_0000_0000_0000;
        dma_peri_base_addr = 64'h0000_0000_0000_0000;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);

        $display("[%0t] After test 1 write: src_hs=%0d slv0 writes=%0d reads=%0d",
                 $time, src_hs_count, slv0_total_wr_count, slv0_total_rd_count);

        $display("\n=== TEST 2: Memory to Peripheral DMA read through TL-C ===");
        clear_sink_range(0);
        dma_stream_type    = 1'b0;
        dma_mem_base_addr  = 64'h0000_0000_0000_0000;
        dma_peri_base_addr = 64'h0000_0000_0000_0000;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);
        verify_sink_range(0, 2);

        $display("[%0t] After test 2 read: sink_hs=%0d slv0 writes=%0d reads=%0d",
                 $time, sink_hs_count, slv0_total_wr_count, slv0_total_rd_count);

        program_source_range(8, 64'hA1B2_C3D4_E500_0100);
        clear_sink_range(8);

        $display("\n=== TEST 3: Next 8-beat Peripheral to Memory DMA write ===");
        dma_stream_type    = 1'b1;
        dma_mem_base_addr  = 64'h0000_0000_0000_0040;
        dma_peri_base_addr = 64'h0000_0000_0000_0040;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);

        $display("[%0t] After test 3 write: src_hs=%0d slv0 writes=%0d reads=%0d",
                 $time, src_hs_count, slv0_total_wr_count, slv0_total_rd_count);

        $display("\n=== TEST 4: Next 8-beat Memory to Peripheral DMA read ===");
        clear_sink_range(8);
        dma_stream_type    = 1'b0;
        dma_mem_base_addr  = 64'h0000_0000_0000_0040;
        dma_peri_base_addr = 64'h0000_0000_0000_0040;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);
        verify_sink_range(8, 4);

        $display("[%0t] After test 4 read: sink_hs=%0d slv0 writes=%0d reads=%0d",
                 $time, sink_hs_count, slv0_total_wr_count, slv0_total_rd_count);

        program_source_range(16, 64'hCCDD_EE00_7700_0200);
        clear_sink_range(16);

        $display("\n=== TEST 5: DMA reads pre-existing memory data without prior DMA write ===");
        dma_stream_type    = 1'b0;
        dma_mem_base_addr  = 64'h0000_0000_0000_0080;
        dma_peri_base_addr = 64'h0000_0000_0000_0080;
        dma_num_bytes      = 16'd64;
        pulse_start();
        wait_done(5000);
        wait_cycles(10);
        verify_sink_range(16, 5);

        $display("[%0t] After test 5 read: sink_hs=%0d slv0 writes=%0d reads=%0d",
                 $time, sink_hs_count, slv0_total_wr_count, slv0_total_rd_count);

        if (src_hs_count != (2 * NUM_BEATS)) begin
            $display("[%0t] ERROR: expected %0d source handshakes, saw %0d",
                     $time, 2 * NUM_BEATS, src_hs_count);
            errors = errors + 1;
        end

        if (sink_hs_count != (3 * NUM_BEATS)) begin
            $display("[%0t] ERROR: expected %0d sink handshakes, saw %0d",
                     $time, 3 * NUM_BEATS, sink_hs_count);
            errors = errors + 1;
        end

        if (slv0_total_wr_count < (5 * NUM_BEATS)) begin
            $display("[%0t] ERROR: expected at least %0d memory writes, got %0d",
                     $time, 5 * NUM_BEATS, slv0_total_wr_count);
            errors = errors + 1;
        end

        if (slv0_total_rd_count < (4 * NUM_BEATS)) begin
            $display("[%0t] ERROR: expected at least %0d memory reads, got %0d",
                     $time, 4 * NUM_BEATS, slv0_total_rd_count);
            errors = errors + 1;
        end

        if (errors != 0) begin
            $display("\nTEST FAILED with %0d errors", errors);
            $fatal(1);
        end

        $display("\nTEST PASSED");
        $finish;
    end

endmodule

`default_nettype wire
