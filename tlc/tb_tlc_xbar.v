// =============================================================================
// tb_tlc_xbar.v  —  Phase 1 testbench for tlc_xbar_top (3M × 2S)
//
// Verifies the TileLink-C crossbar's basic routing and protocol paths:
//   T1 : M0 read miss to Slave 0  → A,D,E + address decode
//   T2 : M1 read miss to Slave 1  → A,D,E + address decode
//   T3 : M0+M1+M2 concurrent reads to different addresses
//   T4 : M0 voluntary writeback (Release/ReleaseAck) → C,D
//
// Additional requested coverage:
//   Test 2 : two-master concurrent reads to different slave ports
//   Test 3 : write miss with no sharers
//   Test 4 : write miss after a shared read, exercising probe/invalidate
//   Test 5 : simultaneous requests to the same address (arb arbitration)
//   Test 6 : writeback / voluntary release
//
// Build instructions: see run_sim.sh in the same directory.
// =============================================================================
`timescale 1ns/10ps

module tb_tlc_xbar;

    // ── Local constants (mirror tlc64b2M_params.v) ──────────────────────────
    localparam [2:0] L1_READ_MISS      = 3'd0;
    localparam [2:0] L1_WRITE_MISS     = 3'd1;
    localparam [2:0] L1_WRITE_BACK     = 3'd2;
    localparam [2:0] L1_UNCACHED_READ  = 3'd3;
    localparam [2:0] L1_UNCACHED_WRITE = 3'd4;

    localparam [2:0] PERM_NtoB = 3'd0;
    localparam [2:0] PERM_NtoT = 3'd1;
    localparam [2:0] PERM_BtoT = 3'd2;
    localparam [2:0] PERM_TtoN = 3'd1;
    localparam [2:0] PERM_BtoN = 3'd2;

    localparam [1:0] COH_I = 2'b00;
    localparam [1:0] COH_B = 2'b01;
    localparam [1:0] COH_T = 2'b10;

    localparam integer CLK_HALF = 5;     // 100 MHz

    // ── Clock / reset ──────────────────────────────────────────────────────
    reg clk;
    reg rst_n;

    initial begin
        clk = 1'b0;
        forever #(CLK_HALF) clk = ~clk;
    end

    // ── Master 0 ports ─────────────────────────────────────────────────────
    reg          m0_req_valid;
    reg  [31:0]  m0_req_addr;
    reg  [2:0]   m0_req_type;
    reg  [255:0] m0_req_data;
    reg  [2:0]   m0_req_permissions;
    wire         m0_req_ready;
    wire         m0_data_valid;
    wire [255:0] m0_data;
    wire         m0_data_error;
    wire         m0_probe_req_valid;
    wire [31:0]  m0_probe_req_addr;
    wire [2:0]   m0_probe_req_permissions;
    reg          m0_probe_ack_valid;
    reg  [31:0]  m0_probe_ack_addr;
    reg  [2:0]   m0_probe_ack_permissions;
    reg  [255:0] m0_probe_ack_dirty_data;

    // ── Master 1 ports ─────────────────────────────────────────────────────
    reg          m1_req_valid;
    reg  [31:0]  m1_req_addr;
    reg  [2:0]   m1_req_type;
    reg  [255:0] m1_req_data;
    reg  [2:0]   m1_req_permissions;
    wire         m1_req_ready;
    wire         m1_data_valid;
    wire [255:0] m1_data;
    wire         m1_data_error;
    wire         m1_probe_req_valid;
    wire [31:0]  m1_probe_req_addr;
    wire [2:0]   m1_probe_req_permissions;
    reg          m1_probe_ack_valid;
    reg  [31:0]  m1_probe_ack_addr;
    reg  [2:0]   m1_probe_ack_permissions;
    reg  [255:0] m1_probe_ack_dirty_data;

    // ── Master 2 ports ─────────────────────────────────────────────────────
    reg          m2_req_valid;
    reg  [31:0]  m2_req_addr;
    reg  [2:0]   m2_req_type;
    reg  [255:0] m2_req_data;
    reg  [2:0]   m2_req_permissions;
    wire         m2_req_ready;
    wire         m2_data_valid;
    wire [255:0] m2_data;
    wire         m2_data_error;
    wire         m2_probe_req_valid;
    wire [31:0]  m2_probe_req_addr;
    wire [2:0]   m2_probe_req_permissions;
    reg          m2_probe_ack_valid;
    reg  [31:0]  m2_probe_ack_addr;
    reg  [2:0]   m2_probe_ack_permissions;
    reg  [255:0] m2_probe_ack_dirty_data;

    // ── DUT ────────────────────────────────────────────────────────────────
    tlc_xbar_top dut (
        .clk                        (clk),
        .rst_n                      (rst_n),

        .m0_req_valid               (m0_req_valid),
        .m0_req_addr                (m0_req_addr),
        .m0_req_type                (m0_req_type),
        .m0_req_data                (m0_req_data),
        .m0_req_permissions         (m0_req_permissions),
        .m0_req_ready               (m0_req_ready),
        .m0_data_valid              (m0_data_valid),
        .m0_data                    (m0_data),
        .m0_data_error              (m0_data_error),
        .m0_probe_req_valid         (m0_probe_req_valid),
        .m0_probe_req_addr          (m0_probe_req_addr),
        .m0_probe_req_permissions   (m0_probe_req_permissions),
        .m0_probe_ack_valid         (m0_probe_ack_valid),
        .m0_probe_ack_addr          (m0_probe_ack_addr),
        .m0_probe_ack_permissions   (m0_probe_ack_permissions),
        .m0_probe_ack_dirty_data    (m0_probe_ack_dirty_data),

        .m1_req_valid               (m1_req_valid),
        .m1_req_addr                (m1_req_addr),
        .m1_req_type                (m1_req_type),
        .m1_req_data                (m1_req_data),
        .m1_req_permissions         (m1_req_permissions),
        .m1_req_ready               (m1_req_ready),
        .m1_data_valid              (m1_data_valid),
        .m1_data                    (m1_data),
        .m1_data_error              (m1_data_error),
        .m1_probe_req_valid         (m1_probe_req_valid),
        .m1_probe_req_addr          (m1_probe_req_addr),
        .m1_probe_req_permissions   (m1_probe_req_permissions),
        .m1_probe_ack_valid         (m1_probe_ack_valid),
        .m1_probe_ack_addr          (m1_probe_ack_addr),
        .m1_probe_ack_permissions   (m1_probe_ack_permissions),
        .m1_probe_ack_dirty_data    (m1_probe_ack_dirty_data),

        .m2_req_valid               (m2_req_valid),
        .m2_req_addr                (m2_req_addr),
        .m2_req_type                (m2_req_type),
        .m2_req_data                (m2_req_data),
        .m2_req_permissions         (m2_req_permissions),
        .m2_req_ready               (m2_req_ready),
        .m2_data_valid              (m2_data_valid),
        .m2_data                    (m2_data),
        .m2_data_error              (m2_data_error),
        .m2_probe_req_valid         (m2_probe_req_valid),
        .m2_probe_req_addr          (m2_probe_req_addr),
        .m2_probe_req_permissions   (m2_probe_req_permissions),
        .m2_probe_ack_valid         (m2_probe_ack_valid),
        .m2_probe_ack_addr          (m2_probe_ack_addr),
        .m2_probe_ack_permissions   (m2_probe_ack_permissions),
        .m2_probe_ack_dirty_data    (m2_probe_ack_dirty_data)
    );

    // ── Pass/fail counters ─────────────────────────────────────────────────
    integer pass_count;
    integer fail_count;
    reg     trace_on;     // enable cycle-by-cycle adapter trace

    // ── Cycle-by-cycle adapter trace (driven by tests when needed) ─────────
    always @(posedge clk) begin
        if (trace_on) begin
            $display("[%0t] M0 st=%0d a_v=%b a_r=%b c_v=%b c_r=%b d_v=%b d_r=%b | M1 st=%0d p_rq=%b p_ack=%b | S0 st=%0d a_r=%b c_r=%b d_v=%b d_r=%b",
                $time,
                dut.u_l1_m0.state, dut.u_l1_m0.a_valid, dut.u_l1_m0.a_ready,
                dut.u_l1_m0.c_valid, dut.u_l1_m0.c_ready,
                dut.u_l1_m0.d_valid, dut.u_l1_m0.d_ready,
                dut.u_l1_m1.state, m1_probe_req_valid, m1_probe_ack_valid,
                dut.u_slv0.state, dut.u_slv0.a_ready, dut.u_slv0.c_ready,
                dut.u_slv0.d_valid, dut.u_slv0.d_ready);
        end
    end

    // Track each master's response separately (one outstanding txn per master)
    reg        m0_resp_seen;
    reg [255:0] m0_resp_data;
    reg        m1_resp_seen;
    reg [255:0] m1_resp_data;
    reg        m2_resp_seen;
    reg [255:0] m2_resp_data;

    always @(posedge clk) begin
        if (m0_data_valid) begin m0_resp_seen <= 1'b1; m0_resp_data <= m0_data; end
        if (m1_data_valid) begin m1_resp_seen <= 1'b1; m1_resp_data <= m1_data; end
        if (m2_data_valid) begin m2_resp_seen <= 1'b1; m2_resp_data <= m2_data; end
    end

    // ── Issue an L1 cache controller request (blocking until handshake) ────
    task drive_m0_req;
        input [31:0] addr;
        input [2:0]  rtype;
        input [2:0]  perm;
        integer i;
        begin
            @(negedge clk);
            m0_req_valid       = 1'b1;
            m0_req_addr        = addr;
            m0_req_type        = rtype;
            m0_req_permissions = perm;
            m0_resp_seen       = 1'b0;
            // Wait for request handshake
            i = 0;
            while (!m0_req_ready && i < 100) begin
                @(posedge clk); i = i + 1;
            end
            @(negedge clk);
            m0_req_valid = 1'b0;
        end
    endtask

    task drive_m1_req;
        input [31:0] addr;
        input [2:0]  rtype;
        input [2:0]  perm;
        integer i;
        begin
            @(negedge clk);
            m1_req_valid       = 1'b1;
            m1_req_addr        = addr;
            m1_req_type        = rtype;
            m1_req_permissions = perm;
            m1_resp_seen       = 1'b0;
            i = 0;
            while (!m1_req_ready && i < 100) begin
                @(posedge clk); i = i + 1;
            end
            @(negedge clk);
            m1_req_valid = 1'b0;
        end
    endtask

    task drive_m2_req;
        input [31:0] addr;
        input [2:0]  rtype;
        input [2:0]  perm;
        integer i;
        begin
            @(negedge clk);
            m2_req_valid       = 1'b1;
            m2_req_addr        = addr;
            m2_req_type        = rtype;
            m2_req_permissions = perm;
            m2_resp_seen       = 1'b0;
            i = 0;
            while (!m2_req_ready && i < 100) begin
                @(posedge clk); i = i + 1;
            end
            @(negedge clk);
            m2_req_valid = 1'b0;
        end
    endtask

    // Wait until a given master's response flag goes high, or timeout
    task wait_resp_m0;
        input integer max_cycles;
        input [255:0] tag;
        integer i;
        begin
            i = 0;
            while (!m0_resp_seen && i < max_cycles) begin
                @(posedge clk); i = i + 1;
            end
            if (m0_resp_seen) begin
                $display("[%0t] PASS M0  %0s  cycles=%0d  data[63:0]=0x%016x",
                         $time, tag, i, m0_resp_data[63:0]);
                pass_count = pass_count + 1;
            end else begin
                $display("[%0t] FAIL M0  %0s  TIMEOUT after %0d cycles",
                         $time, tag, max_cycles);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task wait_resp_m1;
        input integer max_cycles;
        input [255:0] tag;
        integer i;
        begin
            i = 0;
            while (!m1_resp_seen && i < max_cycles) begin
                @(posedge clk); i = i + 1;
            end
            if (m1_resp_seen) begin
                $display("[%0t] PASS M1  %0s  cycles=%0d  data[63:0]=0x%016x",
                         $time, tag, i, m1_resp_data[63:0]);
                pass_count = pass_count + 1;
            end else begin
                $display("[%0t] FAIL M1  %0s  TIMEOUT after %0d cycles",
                         $time, tag, max_cycles);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task wait_resp_m2;
        input integer max_cycles;
        input [255:0] tag;
        integer i;
        begin
            i = 0;
            while (!m2_resp_seen && i < max_cycles) begin
                @(posedge clk); i = i + 1;
            end
            if (m2_resp_seen) begin
                $display("[%0t] PASS M2  %0s  cycles=%0d  data[63:0]=0x%016x",
                         $time, tag, i, m2_resp_data[63:0]);
                pass_count = pass_count + 1;
            end else begin
                $display("[%0t] FAIL M2  %0s  TIMEOUT after %0d cycles",
                         $time, tag, max_cycles);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Wait for a master's adapter to return to idle (m*_req_ready high)
    task wait_idle_m0;
        input integer max_cycles;
        integer i;
        begin
            i = 0;
            while (!m0_req_ready && i < max_cycles) begin
                @(posedge clk); i = i + 1;
            end
            if (!m0_req_ready)
                $display("[%0t] WARN M0 not idle after %0d cycles", $time, max_cycles);
        end
    endtask

    task wait_and_ack_probe_any01;
        input integer max_cycles;
        input [2:0]  ack_perm;
        input [255:0] dirty_data;
        integer i;
        integer selected_master;
        begin
            m0_probe_ack_valid = 1'b0;
            m1_probe_ack_valid = 1'b0;
            i = 0;
            selected_master = -1;
            while (!(m0_probe_req_valid || m1_probe_req_valid) && i < max_cycles) begin
                @(posedge clk); i = i + 1;
            end
            if (m0_probe_req_valid || m1_probe_req_valid) begin
                selected_master = m0_probe_req_valid ? 0 : 1;
                @(negedge clk);
                if (selected_master == 0) begin
                    m0_probe_ack_addr        = m0_probe_req_addr;
                    m0_probe_ack_permissions = ack_perm;
                    m0_probe_ack_dirty_data  = dirty_data;
                    m0_probe_ack_valid       = 1'b1;
                end else begin
                    m1_probe_ack_addr        = m1_probe_req_addr;
                    m1_probe_ack_permissions = ack_perm;
                    m1_probe_ack_dirty_data  = dirty_data;
                    m1_probe_ack_valid       = 1'b1;
                end
                repeat (4) @(posedge clk);
                @(negedge clk);
                m0_probe_ack_valid = 1'b0;
                m1_probe_ack_valid = 1'b0;
                $display("[%0t] INFO probe acked on M%0d perm=%0d",
                         $time, selected_master, ack_perm);
            end else begin
                $display("[%0t] FAIL probe request TIMEOUT after %0d cycles",
                         $time, max_cycles);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task check_dir_state;
        input integer slave_id;
        input [31:0] addr;
        input [23:0] expected_tag;
        input expected_valid;
        input [1:0] expected_coh;
        input [2:0] expected_pres;
        input [255:0] tag;
        integer idx;
        reg actual_valid;
        reg [23:0] actual_tag;
        reg [1:0] actual_coh;
        reg [2:0] actual_pres;
        begin
            idx = addr[7:3];
            actual_valid = 1'b0;
            actual_tag   = 24'b0;
            actual_coh   = 2'b0;
            actual_pres  = 3'b0;

            if (slave_id == 0) begin
                actual_valid = dut.u_slv0.dir_v[idx];
                actual_tag   = dut.u_slv0.dir_tag[idx];
                actual_coh   = dut.u_slv0.dir_coh[idx];
                actual_pres  = dut.u_slv0.dir_pres[idx];
            end else if (slave_id == 1) begin
                actual_valid = dut.u_slv1.dir_v[idx];
                actual_tag   = dut.u_slv1.dir_tag[idx];
                actual_coh   = dut.u_slv1.dir_coh[idx];
                actual_pres  = dut.u_slv1.dir_pres[idx];
            end

            if (actual_valid === expected_valid &&
                actual_tag   === expected_tag &&
                actual_coh   === expected_coh &&
                actual_pres  === expected_pres) begin
                $display("[%0t] PASS DIR%0d  %0s  valid=%b tag=0x%06x coh=%0d pres=0b%03b",
                         $time, slave_id, tag, actual_valid, actual_tag, actual_coh, actual_pres);
                pass_count = pass_count + 1;
            end else begin
                $display("[%0t] FAIL DIR%0d  %0s  expected v=%b tag=0x%06x coh=%0d pres=0b%03b  got v=%b tag=0x%06x coh=%0d pres=0b%03b",
                         $time, slave_id, tag,
                         expected_valid, expected_tag, expected_coh, expected_pres,
                         actual_valid, actual_tag, actual_coh, actual_pres);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ── Initial values + main test sequence ────────────────────────────────
    initial begin
        // Defaults
        rst_n              = 1'b0;
        pass_count         = 0;
        fail_count         = 0;
        trace_on           = 1'b0;

        m0_req_valid       = 1'b0;
        m0_req_addr        = 32'b0;
        m0_req_type        = 3'b0;
        m0_req_data        = 256'b0;
        m0_req_permissions = 3'b0;
        m0_probe_ack_valid = 1'b0;
        m0_probe_ack_addr  = 32'b0;
        m0_probe_ack_permissions = 3'd5;  // PARAM_NtoN
        m0_probe_ack_dirty_data  = 256'b0;
        m0_resp_seen       = 1'b0;
        m0_resp_data       = 256'b0;

        m1_req_valid       = 1'b0;
        m1_req_addr        = 32'b0;
        m1_req_type        = 3'b0;
        m1_req_data        = 256'b0;
        m1_req_permissions = 3'b0;
        m1_probe_ack_valid = 1'b0;
        m1_probe_ack_addr  = 32'b0;
        m1_probe_ack_permissions = 3'd5;
        m1_probe_ack_dirty_data  = 256'b0;
        m1_resp_seen       = 1'b0;
        m1_resp_data       = 256'b0;

        m2_req_valid       = 1'b0;
        m2_req_addr        = 32'b0;
        m2_req_type        = 3'b0;
        m2_req_data        = 256'b0;
        m2_req_permissions = 3'b0;
        m2_probe_ack_valid = 1'b0;
        m2_probe_ack_addr  = 32'b0;
        m2_probe_ack_permissions = 3'd5;
        m2_probe_ack_dirty_data  = 256'b0;
        m2_resp_seen       = 1'b0;
        m2_resp_data       = 256'b0;

        // Reset for 5 cycles
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        $display("\n=================================================");
        $display("  Phase 1 — TL-C Crossbar Routing Testbench");
        $display("=================================================\n");

        // ───────────────────────────────────────────────────────────
        // T1 : Master 0 read miss to Slave 0 (addr[31] = 0)
        //      Exercises: A (M0→S0), D (S0→M0), E (M0→S0)
        // ───────────────────────────────────────────────────────────
        $display("\n--- T1: M0 read miss → Slave 0 (addr=0x0000_1000) ---");
        drive_m0_req(32'h0000_1000, L1_READ_MISS, PERM_NtoT);
        wait_resp_m0(200, "T1 read S0");

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // T2 : Master 1 read miss to Slave 1 (addr[31] = 1)
        //      Tests address decoder routing to the other slave
        // ───────────────────────────────────────────────────────────
        $display("\n--- T2: M1 read miss → Slave 1 (addr=0x8000_2000) ---");
        drive_m1_req(32'h8000_2000, L1_READ_MISS, PERM_NtoT);
        wait_resp_m1(200, "T2 read S1");

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // T3a: Repeat M0 read alone (verify back-to-back single-master)
        // ───────────────────────────────────────────────────────────
        $display("\n--- T3a: M0 repeat read → Slave 0 (addr=0x0000_3000) ---");
        drive_m0_req(32'h0000_3000, L1_READ_MISS, PERM_NtoT);
        wait_resp_m0(200, "T3a M0→S0 repeat");

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // T3b: M2 read alone (verify all masters can transact)
        // ───────────────────────────────────────────────────────────
        $display("\n--- T3b: M2 read → Slave 0 (addr=0x0000_5000) ---");
        drive_m2_req(32'h0000_5000, L1_READ_MISS, PERM_NtoT);
        wait_resp_m2(200, "T3b M2→S0");

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // T3 : All three masters concurrent reads
        // ───────────────────────────────────────────────────────────
        $display("\n--- T3: Concurrent M0/M1/M2 reads (mixed slaves) ---");
        m0_resp_seen = 1'b0; m1_resp_seen = 1'b0; m2_resp_seen = 1'b0;
        trace_on = 1'b1;
        fork
            drive_m0_req(32'h0000_7000, L1_READ_MISS, PERM_NtoT);
            drive_m1_req(32'h8000_8000, L1_READ_MISS, PERM_NtoT);
            drive_m2_req(32'h0000_9000, L1_READ_MISS, PERM_NtoT);
        join

        wait_resp_m0(40, "T3 M0→S0");
        wait_resp_m1(40, "T3 M1→S1");
        wait_resp_m2(40, "T3 M2→S0");
        trace_on = 1'b0;

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // Test 2: two masters issue reads at the same time
        //         Uses one address in each slave half so routing is truly
        //         independent across both slave ports.
        // ───────────────────────────────────────────────────────────
        $display("\n--- Test 2: M0/M1 concurrent reads to different slave ports ---");
        fork
            drive_m0_req(32'h0000_1100, L1_READ_MISS, PERM_NtoB);
            drive_m1_req(32'h8000_2100, L1_READ_MISS, PERM_NtoB);
        join
        wait_resp_m0(200, "Test 2 M0 read");
        wait_resp_m1(200, "Test 2 M1 read");
        check_dir_state(0, 32'h0000_1100, 24'h000011, 1'b1, COH_B, 3'b001, "Test 2 S0 shared read");
        check_dir_state(1, 32'h8000_2100, 24'h800021, 1'b1, COH_B, 3'b010, "Test 2 S1 shared read");

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // Test 3: write miss, no conflict
        //         Master 0 acquires exclusive access to a clean line.
        // ───────────────────────────────────────────────────────────
        $display("\n--- Test 3: M0 write miss with no sharers ---");
        drive_m0_req(32'h0000_3100, L1_WRITE_MISS, PERM_NtoT);
        wait_resp_m0(200, "Test 3 write no-conflict");
        check_dir_state(0, 32'h0000_3100, 24'h000031, 1'b1, COH_T, 3'b001, "Test 3 S0 exclusive");

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // Test 4: write miss with one sharer (probe / invalidate)
        //         First M1 reads a line in shared mode, then M0 writes it.
        //         The slave probes M1, M1 invalidates, and M0 gets exclusive.
        // ───────────────────────────────────────────────────────────
        $display("\n--- Test 4: M1 shared read, then M0 write miss with probe ---");
        drive_m1_req(32'h0000_4000, L1_READ_MISS, PERM_NtoB);
        wait_resp_m1(200, "Test 4 M1 shared read");
        check_dir_state(0, 32'h0000_4000, 24'h000040, 1'b1, COH_B, 3'b010, "Test 4 S0 shared setup");
        repeat (4) @(posedge clk);
        trace_on = 1'b1;
        fork
            drive_m0_req(32'h0000_4000, L1_WRITE_MISS, PERM_NtoT);
            wait_and_ack_probe_any01(200, PERM_BtoN, 256'b0);
        join
        wait_resp_m0(200, "Test 4 M0 write after probe");
        check_dir_state(0, 32'h0000_4000, 24'h000040, 1'b1, COH_T, 3'b001, "Test 4 S0 exclusive after probe");
        trace_on = 1'b0;

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // Test 5: simultaneous requests to the same address
        //         One master wins first, the other stalls until completion.
        // ───────────────────────────────────────────────────────────
        $display("\n--- Test 5: M0/M1 concurrent acquires to the same address ---");
        fork
            drive_m0_req(32'h0000_5100, L1_READ_MISS, PERM_NtoB);
            drive_m1_req(32'h0000_5100, L1_READ_MISS, PERM_NtoB);
        join
        wait_resp_m0(200, "Test 5 M0 same-addr acquire");
        wait_resp_m1(200, "Test 5 M1 same-addr acquire");
        check_dir_state(0, 32'h0000_5100, 24'h000051, 1'b1, COH_B, 3'b011, "Test 5 shared same-addr arbitration");

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // Test 6: writeback / voluntary release
        //         Master 0 first acquires exclusive ownership, then sends
        //         a dirty Release and the directory returns to invalid.
        // ───────────────────────────────────────────────────────────
        $display("\n--- Test 6: M0 exclusive acquire followed by Release ---");
        drive_m0_req(32'h0000_6000, L1_WRITE_MISS, PERM_NtoT);
        wait_resp_m0(200, "Test 6 exclusive setup");
        check_dir_state(0, 32'h0000_6000, 24'h000060, 1'b1, COH_T, 3'b001, "Test 6 exclusive setup");

        repeat (4) @(posedge clk);

        // ───────────────────────────────────────────────────────────
        // T4 : M0 writeback (Release) → Slave 0
        //      Exercises C channel (M0→S0) and D ReleaseAck (S0→M0)
        //      Adapter sees ReleaseAck and returns to IDLE
        // ───────────────────────────────────────────────────────────
        $display("\n--- T4: M0 writeback Release → Slave 0 ---");
        m0_resp_seen = 1'b0;  // Release doesn't fire data_to_l1
        m0_req_data  = {{4{64'hDEAD_BEEF_CAFE_BABE}}};
        drive_m0_req(32'h0000_6000, L1_WRITE_BACK, PERM_TtoN);
        // After drive_m0_req returns, adapter is mid-Release.  Wait until
        // it returns to STATE_IDLE = 0 by sampling the FSM register directly.
        // (req_ready only re-asserts on a NEW request; we don't have one.)
        begin : t4_wait
            integer i;
            i = 0;
            // Skip one cycle so the adapter has time to leave IDLE
            @(posedge clk);
            while (i < 200 && dut.u_l1_m0.state != 4'd0) begin
                @(posedge clk); i = i + 1;
            end
            if (dut.u_l1_m0.state == 4'd0) begin
                $display("[%0t] PASS M0  T4 Release  completed in %0d cycles", $time, i);
                pass_count = pass_count + 1;
            end else begin
                $display("[%0t] FAIL M0  T4 Release  TIMEOUT after %0d cycles", $time, i);
                fail_count = fail_count + 1;
            end
        end

        check_dir_state(0, 32'h0000_6000, 24'h000000, 1'b0, COH_I, 3'b000, "Test 6 invalid after release");

        repeat (10) @(posedge clk);

        // ── Final report ────────────────────────────────────────────────
        $display("\n=================================================");
        $display("  TEST SUMMARY:  pass=%0d  fail=%0d", pass_count, fail_count);
        $display("=================================================");
        if (fail_count == 0)
            $display("  *** ALL TESTS PASSED ***\n");
        else
            $display("  *** %0d TEST(S) FAILED ***\n", fail_count);

        $finish;
    end

    // ── Watchdog ──────────────────────────────────────────────────────────
    initial begin
        #50000;  // 50 us = 5000 cycles
        $display("\n[WATCHDOG] Simulation hung — terminating.");
        $display("  pass=%0d  fail=%0d", pass_count, fail_count);
        $finish;
    end

endmodule
