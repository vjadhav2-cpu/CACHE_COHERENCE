`timescale 1ns/1ps
`default_nettype none

module tb_dma_flat_full_duplex;

    parameter DATA_WIDTH = 64;
    parameter ADDR_WIDTH = 64;
    parameter BURST_MAX  = 16;
    parameter FIFO_DEPTH = 16;

    parameter MEM_WORDS  = 4096;
    parameter PERI_WORDS = 4096;

    reg                   clk;
    reg                   rst;
    reg                   start;
    reg                   stream_type;
    reg  [ADDR_WIDTH-1:0] mem_base_addr;
    reg  [ADDR_WIDTH-1:0] peri_base_addr;
    reg  [15:0]           num_bytes;

    wire                  mem_req;
    wire                  mem_wr_en;
    wire [ADDR_WIDTH-1:0] mem_addr;
    wire [DATA_WIDTH-1:0] mem_wdata;
    reg  [DATA_WIDTH-1:0] mem_rdata;
    reg                   mem_ack;

    wire                  peri_req;
    wire                  peri_wr_en;
    wire [ADDR_WIDTH-1:0] peri_addr;
    wire [DATA_WIDTH-1:0] peri_wdata;
    reg  [DATA_WIDTH-1:0] peri_rdata;
    reg                   peri_ack;

    wire                  busy;
    wire                  done;

    reg [DATA_WIDTH-1:0] mem_model [0:MEM_WORDS-1];
    reg [DATA_WIDTH-1:0] peri_model[0:PERI_WORDS-1];

    reg [31:0] mem_read_count;
    reg [31:0] mem_write_count;
    reg [31:0] peri_read_count;
    reg [31:0] peri_write_count;

    // Protocol checker context
    reg                    chk_active;
    reg                    chk_dir; // 0=M2P, 1=P2M
    reg                    chk_expect_transfer;
    reg [15:0]             chk_exp_beats;
    reg [15:0]             chk_src_acks;
    reg [15:0]             chk_snk_acks;
    reg [ADDR_WIDTH-1:0]   chk_src_next_addr;
    reg [ADDR_WIDTH-1:0]   chk_snk_next_addr;

    reg [1:0] mem_latency_mode;
    reg [1:0] peri_latency_mode;

    reg                  mem_pending;
    reg                  mem_p_wr;
    reg [ADDR_WIDTH-1:0] mem_p_addr;
    reg [DATA_WIDTH-1:0] mem_p_wdata;
    reg [3:0]            mem_wait;

    reg                  peri_pending;
    reg                  peri_p_wr;
    reg [ADDR_WIDTH-1:0] peri_p_addr;
    reg [DATA_WIDTH-1:0] peri_p_wdata;
    reg [3:0]            peri_wait;

    integer i;
    integer errors;
    integer checks;
    integer cycles;

    dma_flat_full_duplex_top #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .BURST_MAX  (BURST_MAX),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .start          (start),
        .stream_type    (stream_type),
        .mem_base_addr  (mem_base_addr),
        .peri_base_addr (peri_base_addr),
        .num_bytes      (num_bytes),
        .mem_req        (mem_req),
        .mem_wr_en      (mem_wr_en),
        .mem_addr       (mem_addr),
        .mem_wdata      (mem_wdata),
        .mem_rdata      (mem_rdata),
        .mem_ack        (mem_ack),
        .peri_req       (peri_req),
        .peri_wr_en     (peri_wr_en),
        .peri_addr      (peri_addr),
        .peri_wdata     (peri_wdata),
        .peri_rdata     (peri_rdata),
        .peri_ack       (peri_ack),
        .busy           (busy),
        .done           (done)
    );

    function integer mem_idx;
        input [ADDR_WIDTH-1:0] addr;
        begin
            mem_idx = (addr >> 3) % MEM_WORDS;
        end
    endfunction

    function integer peri_idx;
        input [ADDR_WIDTH-1:0] addr;
        begin
            peri_idx = (addr >> 3) % PERI_WORDS;
        end
    endfunction

    function [3:0] calc_latency;
        input [1:0] mode;
        input [ADDR_WIDTH-1:0] addr;
        reg [3:0] base;
        begin
            case (mode)
                2'd0: base = 4'd0;                          // immediate ack next cycle
                2'd1: base = 4'd1;                          // fixed 1-cycle wait
                2'd2: base = {2'b00, addr[4:3]};            // 0..3 from address
                default: base = {1'b0, addr[5:3]};          // 0..7 from address
            endcase
            calc_latency = base;
        end
    endfunction

    task clear_models;
        begin
            for (i = 0; i < MEM_WORDS; i = i + 1)
                mem_model[i] = 64'h0;
            for (i = 0; i < PERI_WORDS; i = i + 1)
                peri_model[i] = 64'h0;
        end
    endtask

    task clear_counters;
        begin
            mem_read_count  = 0;
            mem_write_count = 0;
            peri_read_count = 0;
            peri_write_count= 0;
        end
    endtask

    task pulse_start;
        begin
            @(posedge clk);
            start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;
        end
    endtask

    task wait_done_or_timeout;
        input integer max_cycles;
        begin
            cycles = 0;
            while ((done !== 1'b1) && (cycles < max_cycles)) begin
                @(posedge clk);
                cycles = cycles + 1;
            end
            if (done !== 1'b1) begin
                $display("[TB][FAIL] Timeout waiting for done (max_cycles=%0d)", max_cycles);
                errors = errors + 1;
            end
        end
    endtask

    task expect_equal64;
        input [DATA_WIDTH-1:0] exp;
        input [DATA_WIDTH-1:0] got;
        input [ADDR_WIDTH-1:0] addr;
        begin
            checks = checks + 1;
            if (exp !== got) begin
                $display("[TB][FAIL] Data mismatch @addr=0x%016h exp=0x%016h got=0x%016h", addr, exp, got);
                errors = errors + 1;
            end
        end
    endtask

    task tc_p2m_basic;
        integer beat;
        integer idx;
        reg [ADDR_WIDTH-1:0] a;
        begin
            $display("\n[TB] TC1: P2M basic (peripheral -> memory), 64B");
            clear_models();
            clear_counters();
            mem_latency_mode  = 2'd1;
            peri_latency_mode = 2'd1;

            mem_base_addr  = 64'h0000_0000_0000_0100;
            peri_base_addr = 64'h0000_0000_0000_0200;
            num_bytes      = 16'd64;
            stream_type    = 1'b1;

            for (beat = 0; beat < 8; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                idx = peri_idx(a);
                peri_model[idx] = 64'hA500_0000_0000_0000 + beat;
            end

            pulse_start();
            wait_done_or_timeout(2000);

            for (beat = 0; beat < 8; beat = beat + 1) begin
                a = mem_base_addr + (beat*8);
                expect_equal64(64'hA500_0000_0000_0000 + beat, mem_model[mem_idx(a)], a);
            end

            if (peri_read_count < 8 || mem_write_count < 8) begin
                $display("[TB][FAIL] TC1 insufficient traffic peri_reads=%0d mem_writes=%0d", peri_read_count, mem_write_count);
                errors = errors + 1;
            end
        end
    endtask

    task tc_m2p_basic;
        integer beat;
        integer idx;
        reg [ADDR_WIDTH-1:0] a;
        begin
            $display("\n[TB] TC2: M2P basic (memory -> peripheral), 64B");
            clear_models();
            clear_counters();
            mem_latency_mode  = 2'd1;
            peri_latency_mode = 2'd1;

            mem_base_addr  = 64'h0000_0000_0000_0300;
            peri_base_addr = 64'h0000_0000_0000_0400;
            num_bytes      = 16'd64;
            stream_type    = 1'b0;

            for (beat = 0; beat < 8; beat = beat + 1) begin
                a = mem_base_addr + (beat*8);
                idx = mem_idx(a);
                mem_model[idx] = 64'h5A00_0000_0000_0000 + beat;
            end

            pulse_start();
            wait_done_or_timeout(2000);

            for (beat = 0; beat < 8; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                expect_equal64(64'h5A00_0000_0000_0000 + beat, peri_model[peri_idx(a)], a);
            end

            if (mem_read_count < 8 || peri_write_count < 8) begin
                $display("[TB][FAIL] TC2 insufficient traffic mem_reads=%0d peri_writes=%0d", mem_read_count, peri_write_count);
                errors = errors + 1;
            end
        end
    endtask

    task tc_start_while_busy_ignored;
        integer beat;
        reg [ADDR_WIDTH-1:0] a;
        begin
            $display("\n[TB] TC3: start-while-busy should be ignored");
            clear_models();
            clear_counters();
            mem_latency_mode  = 2'd3;   // slow to keep DMA busy long enough
            peri_latency_mode = 2'd3;

            mem_base_addr  = 64'h0000_0000_0000_0500;
            peri_base_addr = 64'h0000_0000_0000_0600;
            num_bytes      = 16'd128;
            stream_type    = 1'b1;      // P2M first transfer

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                peri_model[peri_idx(a)] = 64'hC300_0000_0000_0000 + beat;
            end

            pulse_start();

            // Wait until in-flight, then try to push a conflicting new start.
            while (busy !== 1'b1)
                @(posedge clk);

            @(posedge clk);
            stream_type    <= 1'b0;
            mem_base_addr  <= 64'h0000_0000_0000_0700;
            peri_base_addr <= 64'h0000_0000_0000_0800;
            num_bytes      <= 16'd64;
            start          <= 1'b1;
            @(posedge clk);
            start          <= 1'b0;

            wait_done_or_timeout(5000);

            // First transfer destination must be updated.
            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = 64'h0000_0000_0000_0500 + (beat*8);
                expect_equal64(64'hC300_0000_0000_0000 + beat, mem_model[mem_idx(a)], a);
            end

            // Second (ignored) transfer destination should remain zero.
            for (beat = 0; beat < 8; beat = beat + 1) begin
                a = 64'h0000_0000_0000_0800 + (beat*8);
                checks = checks + 1;
                if (peri_model[peri_idx(a)] !== 64'h0) begin
                    $display("[TB][FAIL] TC3 ignored-start destination unexpectedly changed @addr=0x%016h data=0x%016h", a, peri_model[peri_idx(a)]);
                    errors = errors + 1;
                end
            end
        end
    endtask

    task tc_misaligned_rejected;
        begin
            $display("\n[TB] TC4: misaligned num_bytes should not perform writes");
            clear_models();
            clear_counters();
            mem_latency_mode  = 2'd1;
            peri_latency_mode = 2'd1;

            mem_base_addr  = 64'h0000_0000_0000_0900;
            peri_base_addr = 64'h0000_0000_0000_0A00;
            num_bytes      = 16'd14;   // not 8-byte aligned
            stream_type    = 1'b1;

            pulse_start();
            wait_done_or_timeout(200);

            checks = checks + 1;
            if (mem_write_count != 0 || peri_write_count != 0) begin
                $display("[TB][FAIL] TC4 writes observed despite misaligned request mem_w=%0d peri_w=%0d", mem_write_count, peri_write_count);
                errors = errors + 1;
            end
        end
    endtask

    task tc_zero_length;
        begin
            $display("\n[TB] TC5: zero-length transfer");
            clear_models();
            clear_counters();
            mem_latency_mode  = 2'd1;
            peri_latency_mode = 2'd1;

            mem_base_addr  = 64'h0000_0000_0000_0B00;
            peri_base_addr = 64'h0000_0000_0000_0C00;
            num_bytes      = 16'd0;
            stream_type    = 1'b0;

            pulse_start();
            wait_done_or_timeout(100);

            checks = checks + 1;
            if (mem_read_count != 0 || mem_write_count != 0 || peri_read_count != 0 || peri_write_count != 0) begin
                $display("[TB][FAIL] TC5 unexpected bus traffic on zero-length transfer mR=%0d mW=%0d pR=%0d pW=%0d",
                         mem_read_count, mem_write_count, peri_read_count, peri_write_count);
                errors = errors + 1;
            end
        end
    endtask

    task tc_variable_backpressure;
        integer beat;
        reg [ADDR_WIDTH-1:0] a;
        begin
            $display("\n[TB] TC6: variable-latency backpressure, 128B P2M + 128B M2P");
            clear_models();
            clear_counters();
            mem_latency_mode  = 2'd2;
            peri_latency_mode = 2'd3;

            // P2M first
            mem_base_addr  = 64'h0000_0000_0000_1000;
            peri_base_addr = 64'h0000_0000_0000_1100;
            num_bytes      = 16'd128;
            stream_type    = 1'b1;

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                peri_model[peri_idx(a)] = 64'hD100_0000_0000_0000 + beat;
            end

            pulse_start();
            wait_done_or_timeout(8000);

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = mem_base_addr + (beat*8);
                expect_equal64(64'hD100_0000_0000_0000 + beat, mem_model[mem_idx(a)], a);
            end

            // M2P second
            clear_counters();
            mem_base_addr  = 64'h0000_0000_0000_1200;
            peri_base_addr = 64'h0000_0000_0000_1300;
            num_bytes      = 16'd128;
            stream_type    = 1'b0;

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = mem_base_addr + (beat*8);
                mem_model[mem_idx(a)] = 64'hE200_0000_0000_0000 + beat;
            end

            pulse_start();
            wait_done_or_timeout(8000);

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                expect_equal64(64'hE200_0000_0000_0000 + beat, peri_model[peri_idx(a)], a);
            end
        end
    endtask

    task tc_burst_exact_16beats;
        integer beat;
        reg [ADDR_WIDTH-1:0] a;
        begin
            $display("\n[TB] TC7: explicit burst case, exactly 16 beats (128B) in both directions");
            clear_models();
            clear_counters();
            mem_latency_mode  = 2'd1;
            peri_latency_mode = 2'd1;

            // P2M 16 beats
            mem_base_addr  = 64'h0000_0000_0000_2000;
            peri_base_addr = 64'h0000_0000_0000_2100;
            num_bytes      = 16'd128; // 16 beats @ 8 bytes
            stream_type    = 1'b1;

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                peri_model[peri_idx(a)] = 64'hB100_0000_0000_0000 + beat;
            end

            pulse_start();
            wait_done_or_timeout(5000);

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = mem_base_addr + (beat*8);
                expect_equal64(64'hB100_0000_0000_0000 + beat, mem_model[mem_idx(a)], a);
            end

            // M2P 16 beats
            clear_counters();
            mem_base_addr  = 64'h0000_0000_0000_2200;
            peri_base_addr = 64'h0000_0000_0000_2300;
            num_bytes      = 16'd128;
            stream_type    = 1'b0;

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = mem_base_addr + (beat*8);
                mem_model[mem_idx(a)] = 64'hB200_0000_0000_0000 + beat;
            end

            pulse_start();
            wait_done_or_timeout(5000);

            for (beat = 0; beat < 16; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                expect_equal64(64'hB200_0000_0000_0000 + beat, peri_model[peri_idx(a)], a);
            end
        end
    endtask

    task tc_burst_multichunk_32beats;
        integer beat;
        reg [ADDR_WIDTH-1:0] a;
        begin
            $display("\n[TB] TC8: explicit burst case, 32 beats (256B) requiring multiple burst chunks");
            clear_models();
            clear_counters();
            mem_latency_mode  = 2'd2;
            peri_latency_mode = 2'd2;

            // P2M 32 beats
            mem_base_addr  = 64'h0000_0000_0000_2400;
            peri_base_addr = 64'h0000_0000_0000_2500;
            num_bytes      = 16'd256; // 32 beats @ 8 bytes
            stream_type    = 1'b1;

            for (beat = 0; beat < 32; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                peri_model[peri_idx(a)] = 64'hC100_0000_0000_0000 + beat;
            end

            pulse_start();
            wait_done_or_timeout(12000);

            for (beat = 0; beat < 32; beat = beat + 1) begin
                a = mem_base_addr + (beat*8);
                expect_equal64(64'hC100_0000_0000_0000 + beat, mem_model[mem_idx(a)], a);
            end

            // M2P 32 beats
            clear_counters();
            mem_base_addr  = 64'h0000_0000_0000_2600;
            peri_base_addr = 64'h0000_0000_0000_2700;
            num_bytes      = 16'd256;
            stream_type    = 1'b0;

            for (beat = 0; beat < 32; beat = beat + 1) begin
                a = mem_base_addr + (beat*8);
                mem_model[mem_idx(a)] = 64'hC200_0000_0000_0000 + beat;
            end

            pulse_start();
            wait_done_or_timeout(12000);

            for (beat = 0; beat < 32; beat = beat + 1) begin
                a = peri_base_addr + (beat*8);
                expect_equal64(64'hC200_0000_0000_0000 + beat, peri_model[peri_idx(a)], a);
            end
        end
    endtask

    task init_checker;
        begin
            chk_active          = 1'b0;
            chk_dir             = 1'b0;
            chk_expect_transfer = 1'b0;
            chk_exp_beats       = 16'd0;
            chk_src_acks        = 16'd0;
            chk_snk_acks        = 16'd0;
            chk_src_next_addr   = {ADDR_WIDTH{1'b0}};
            chk_snk_next_addr   = {ADDR_WIDTH{1'b0}};
        end
    endtask

    // Memory-side bus model
    // Drive ack/rdata on negedge so DUT sees stable values on next posedge.
    always @(negedge clk) begin
        mem_ack <= 1'b0;

        if (rst) begin
            mem_pending <= 1'b0;
            mem_wait    <= 4'd0;
            mem_rdata   <= {DATA_WIDTH{1'b0}};
        end else begin
            if (!mem_pending && mem_req) begin
                mem_pending <= 1'b1;
                mem_p_wr    <= mem_wr_en;
                mem_p_addr  <= mem_addr;
                mem_p_wdata <= mem_wdata;
                mem_wait    <= calc_latency(mem_latency_mode, mem_addr);
            end else if (mem_pending) begin
                if (mem_wait != 0)
                    mem_wait <= mem_wait - 1'b1;
                else begin
                    mem_ack <= 1'b1;
                    if (mem_p_wr) begin
                        mem_model[mem_idx(mem_p_addr)] <= mem_p_wdata;
                        mem_write_count <= mem_write_count + 1;
                    end else begin
                        mem_rdata <= mem_model[mem_idx(mem_p_addr)];
                        mem_read_count <= mem_read_count + 1;
                    end
                    mem_pending <= 1'b0;
                end
            end
        end
    end

    // Peripheral-side bus model
    // Drive ack/rdata on negedge so DUT sees stable values on next posedge.
    always @(negedge clk) begin
        peri_ack <= 1'b0;

        if (rst) begin
            peri_pending <= 1'b0;
            peri_wait    <= 4'd0;
            peri_rdata   <= {DATA_WIDTH{1'b0}};
        end else begin
            if (!peri_pending && peri_req) begin
                peri_pending <= 1'b1;
                peri_p_wr    <= peri_wr_en;
                peri_p_addr  <= peri_addr;
                peri_p_wdata <= peri_wdata;
                peri_wait    <= calc_latency(peri_latency_mode, peri_addr);
            end else if (peri_pending) begin
                if (peri_wait != 0)
                    peri_wait <= peri_wait - 1'b1;
                else begin
                    peri_ack <= 1'b1;
                    if (peri_p_wr) begin
                        peri_model[peri_idx(peri_p_addr)] <= peri_p_wdata;
                        peri_write_count <= peri_write_count + 1;
                    end else begin
                        peri_rdata <= peri_model[peri_idx(peri_p_addr)];
                        peri_read_count <= peri_read_count + 1;
                    end
                    peri_pending <= 1'b0;
                end
            end
        end
    end

    // Signal/protocol checker (no RTL changes; verification-only checks)
    always @(posedge clk) begin
        if (rst) begin
            init_checker();
        end else begin
            // Basic legality checks while transfer is active.
            if (busy) begin
                // Address alignment on all requests.
                if (mem_req && (mem_addr[2:0] != 3'b000)) begin
                    $display("[TB][FAIL] Unaligned mem_addr on request: 0x%016h", mem_addr);
                    errors = errors + 1;
                end
                if (peri_req && (peri_addr[2:0] != 3'b000)) begin
                    $display("[TB][FAIL] Unaligned peri_addr on request: 0x%016h", peri_addr);
                    errors = errors + 1;
                end
            end

            // Accepted start defines expected protocol context.
            if (start && !busy) begin
                chk_active          <= 1'b1;
                chk_dir             <= stream_type;
                chk_expect_transfer <= ((num_bytes[2:0] == 3'b000) && (num_bytes != 16'd0));
                chk_exp_beats       <= (num_bytes >> 3);
                chk_src_acks        <= 16'd0;
                chk_snk_acks        <= 16'd0;

                if (stream_type) begin
                    // P2M: source=peri(read), sink=mem(write)
                    chk_src_next_addr <= peri_base_addr;
                    chk_snk_next_addr <= mem_base_addr;
                end else begin
                    // M2P: source=mem(read), sink=peri(write)
                    chk_src_next_addr <= mem_base_addr;
                    chk_snk_next_addr <= peri_base_addr;
                end
            end

            if (chk_active) begin
                if (chk_dir) begin
                    // P2M direction checks
                    if (mem_req && !mem_wr_en) begin
                        $display("[TB][FAIL] P2M violation: mem_req with mem_wr_en=0");
                        errors = errors + 1;
                    end
                    if (peri_req && peri_wr_en) begin
                        $display("[TB][FAIL] P2M violation: peri_req with peri_wr_en=1");
                        errors = errors + 1;
                    end

                    // Source beats (peri read)
                    if (peri_req && peri_ack && !peri_wr_en) begin
                        checks = checks + 1;
                        if (peri_addr !== chk_src_next_addr) begin
                            $display("[TB][FAIL] P2M source addr mismatch exp=0x%016h got=0x%016h",
                                     chk_src_next_addr, peri_addr);
                            errors = errors + 1;
                        end
                        chk_src_next_addr <= chk_src_next_addr + 64'd8;
                        chk_src_acks <= chk_src_acks + 16'd1;
                    end

                    // Sink beats (mem write)
                    if (mem_req && mem_ack && mem_wr_en) begin
                        checks = checks + 1;
                        if (mem_addr !== chk_snk_next_addr) begin
                            $display("[TB][FAIL] P2M sink addr mismatch exp=0x%016h got=0x%016h",
                                     chk_snk_next_addr, mem_addr);
                            errors = errors + 1;
                        end
                        chk_snk_next_addr <= chk_snk_next_addr + 64'd8;
                        chk_snk_acks <= chk_snk_acks + 16'd1;
                    end
                end else begin
                    // M2P direction checks
                    if (mem_req && mem_wr_en) begin
                        $display("[TB][FAIL] M2P violation: mem_req with mem_wr_en=1");
                        errors = errors + 1;
                    end
                    if (peri_req && !peri_wr_en) begin
                        $display("[TB][FAIL] M2P violation: peri_req with peri_wr_en=0");
                        errors = errors + 1;
                    end

                    // Source beats (mem read)
                    if (mem_req && mem_ack && !mem_wr_en) begin
                        checks = checks + 1;
                        if (mem_addr !== chk_src_next_addr) begin
                            $display("[TB][FAIL] M2P source addr mismatch exp=0x%016h got=0x%016h",
                                     chk_src_next_addr, mem_addr);
                            errors = errors + 1;
                        end
                        chk_src_next_addr <= chk_src_next_addr + 64'd8;
                        chk_src_acks <= chk_src_acks + 16'd1;
                    end

                    // Sink beats (peri write)
                    if (peri_req && peri_ack && peri_wr_en) begin
                        checks = checks + 1;
                        if (peri_addr !== chk_snk_next_addr) begin
                            $display("[TB][FAIL] M2P sink addr mismatch exp=0x%016h got=0x%016h",
                                     chk_snk_next_addr, peri_addr);
                            errors = errors + 1;
                        end
                        chk_snk_next_addr <= chk_snk_next_addr + 64'd8;
                        chk_snk_acks <= chk_snk_acks + 16'd1;
                    end
                end
            end

            // Completion consistency check at done.
            if (done && chk_active) begin
                if (chk_expect_transfer) begin
                    checks = checks + 2;
                    if (chk_src_acks !== chk_exp_beats) begin
                        $display("[TB][FAIL] Beat-count mismatch on source side exp=%0d got=%0d",
                                 chk_exp_beats, chk_src_acks);
                        errors = errors + 1;
                    end
                    if (chk_snk_acks !== chk_exp_beats) begin
                        $display("[TB][FAIL] Beat-count mismatch on sink side exp=%0d got=%0d",
                                 chk_exp_beats, chk_snk_acks);
                        errors = errors + 1;
                    end
                end
                chk_active <= 1'b0;
            end
        end
    end

    // Clock generation
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Wave dump for GTKWave
    initial begin
        $dumpfile("/tmp/dma_tb.vcd");
        $dumpvars(0, tb_dma_flat_full_duplex);
    end

    // Main sequence
    initial begin
        errors = 0;
        checks = 0;

        start = 1'b0;
        stream_type = 1'b0;
        mem_base_addr = {ADDR_WIDTH{1'b0}};
        peri_base_addr = {ADDR_WIDTH{1'b0}};
        num_bytes = 16'd0;
        mem_rdata = {DATA_WIDTH{1'b0}};
        peri_rdata = {DATA_WIDTH{1'b0}};
        mem_ack = 1'b0;
        peri_ack = 1'b0;

        mem_latency_mode = 2'd1;
        peri_latency_mode = 2'd1;

        mem_pending = 1'b0;
        peri_pending = 1'b0;
        mem_wait = 4'd0;
        peri_wait = 4'd0;
        init_checker();

        clear_models();
        clear_counters();

        rst = 1'b1;
        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        tc_p2m_basic();
        tc_m2p_basic();
        tc_start_while_busy_ignored();
        tc_misaligned_rejected();
        tc_zero_length();
        tc_variable_backpressure();
        tc_burst_exact_16beats();
        tc_burst_multichunk_32beats();

        $display("\n[TB] ===========================================================");
        $display("[TB] DMA TEST SUMMARY: checks=%0d errors=%0d", checks, errors);
        if (errors == 0)
            $display("[TB] RESULT: PASS");
        else
            $display("[TB] RESULT: FAIL");
        $display("[TB] ===========================================================");

        if (errors == 0)
            $finish;
        else
            $fatal(1, "DMA testbench failed with %0d errors", errors);
    end

endmodule

`default_nettype wire
