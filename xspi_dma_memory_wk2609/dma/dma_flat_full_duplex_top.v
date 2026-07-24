// ============================================================================
// dma_flat_full_duplex_top.v
// Top-level: two symmetric dma_streamers + one shared FIFO
//
// stream_type selects the direction:
//   - 0 (M2P): Memory → FIFO → Peripheral
//        READ  streamer sources the Memory bus  (mem_* rdata/ack)
//        WRITE streamer sinks   to Peripheral   (peri_* wdata/ack)
//   - 1 (P2M): Peripheral → FIFO → Memory
//        READ  streamer sources the Peripheral (peri_* rdata/ack)
//        WRITE streamer sinks   to Memory       (mem_*  wdata/ack)
//
// Handshake model: single-beat request/acknowledge on each side.
// Status: 'busy' ORs the two streamers; 'done' asserts after both complete.
// ============================================================================
`timescale 1ns/1ps
`default_nettype none           // forbid implicit nets

module dma_flat_full_duplex_top #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 64,
    parameter BURST_MAX  = 16,
    parameter FIFO_DEPTH = 16
)(
    // -----------------------------------------------------------------------
    // Control & descriptors
    // -----------------------------------------------------------------------
    input  wire                  clk,
    input  wire                  rst,               // synchronous, active-high
    input  wire                  start,             // one-cycle start pulse
    input  wire                  stream_type,       // 0=M2P, 1=P2M
    input  wire [ADDR_WIDTH-1:0] mem_base_addr,
    input  wire [ADDR_WIDTH-1:0] peri_base_addr,
    input  wire [15:0]           num_bytes,

    // -----------------------------------------------------------------------
    // Memory-side bus (single-beat handshake)
    // -----------------------------------------------------------------------
    output wire                  mem_req,
    output wire                  mem_wr_en,
    output wire [ADDR_WIDTH-1:0] mem_addr,
    output wire [DATA_WIDTH-1:0] mem_wdata,
    input  wire [DATA_WIDTH-1:0] mem_rdata,
    input  wire                  mem_ack,

    // -----------------------------------------------------------------------
    // Peripheral-side bus (single-beat handshake)
    // -----------------------------------------------------------------------
    output wire                  peri_req,
    output wire                  peri_wr_en,
    output wire [ADDR_WIDTH-1:0] peri_addr,
    output wire [DATA_WIDTH-1:0] peri_wdata,
    input  wire [DATA_WIDTH-1:0] peri_rdata,
    input  wire                  peri_ack,

    // -----------------------------------------------------------------------
    // Status
    // -----------------------------------------------------------------------
    output wire                  busy,
    output wire                  done
);

    // ---------------------------------------------------------------------
    // Latched descriptor context (accepted only when idle).
    // Mid-transfer start pulses are ignored to keep the active transaction
    // atomic and prevent live pin changes from perturbing routing.
    // ---------------------------------------------------------------------
    reg                     start_accepted;
    reg                     stream_type_ctx;
    reg [ADDR_WIDTH-1:0]    mem_base_addr_ctx;
    reg [ADDR_WIDTH-1:0]    peri_base_addr_ctx;
    reg [15:0]              num_bytes_ctx;
    reg                     start_while_busy_seen ;

    // =====================================================================
    // Shared FIFO between READ and WRITE streamers
    //   - READ pushes into FIFO (fifo_wr/fifo_din)
    //   - WRITE pops   from FIFO (fifo_rd/fifo_dout)
    // =====================================================================
    wire [DATA_WIDTH-1:0] fifo_din,  fifo_dout;
    wire                  fifo_wr,   fifo_rd;
    wire                  fifo_full  ;
    wire                  fifo_almost_full;
    wire                  fifo_empty;
    wire [15:0]           fifo_count;

    dma_fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .DEPTH      (FIFO_DEPTH)
    ) u_fifo (
        .clk        (clk),
        .rst_n      (~rst),
        .write_en   (fifo_wr),
        .write_data (fifo_din),
        .full       (fifo_full),
        .almost_full(fifo_almost_full),
        .read_en    (fifo_rd),
        .read_data  (fifo_dout),
        .empty      (fifo_empty),
        .level      (fifo_count)
    );

    // =====================================================================
    // READ-streamer instance (stream_type decides which bus it reads from)
    //   Expected per-beat (while in REQ):
    //     - External side: asserts rd_req; when rd_ack=1, captures rd_rdata
    //     - FIFO side: pulses fifo_wr with captured word
    //     - Address/byte count advance by DATA_WIDTH/8 per beat
    // =====================================================================
    wire                     rd_req, rd_wr_en, rd_ack;
    wire [ADDR_WIDTH-1:0]    rd_addr;
    wire [DATA_WIDTH-1:0]    rd_wdata, rd_rdata;
    wire                     rd_busy, rd_done;
    wire                     rd_cfg_error;

    wire                     fifo_rd_sink          ;
    wire [DATA_WIDTH-1:0]    rs_wdata_sink         ;
    wire                     rs_wvalid_sink        ;

    // Select READ source bus per active stream context.
    assign rd_rdata = (stream_type_ctx == 1'b0) ? mem_rdata : peri_rdata;
    assign rd_ack   = (stream_type_ctx == 1'b0) ? mem_ack   : peri_ack;

    assign {rs_wdata_sink, rs_wvalid_sink} = '0;   // unused in READ

    dma_streamer #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .BURST_MAX  (BURST_MAX)
    ) u_read (
        .clk              (clk),
        .rst              (rst),
        .start            (start_accepted),
        .stream_type      (1'b0),                // READ instance
        .base_addr        ((stream_type_ctx==1'b0) ? mem_base_addr_ctx : peri_base_addr_ctx),
        .num_bytes        (num_bytes_ctx),

        // FIFO (READ pushes)
        .write_data       (rs_wdata_sink),       // unused in READ
        // Keep one slot of headroom to absorb response latency and avoid
        // overflow on the full transition edge.
        .write_data_valid (!fifo_almost_full),   // READ-side sink ready
        .read_data        (fifo_din),
        .read_data_valid  (fifo_wr),

        .busy             (rd_busy),
        .done             (rd_done),

        // External bus
        .bus_req          (rd_req),
        .bus_wr_en        (rd_wr_en),            // driven low internally for READ
        .bus_addr         (rd_addr),
        .bus_wdata        (rd_wdata),
        .bus_rdata        (rd_rdata),
        .bus_ack          (rd_ack),

        // FIFO handshake (unused in READ)
        .fifo_empty       (1'b0),
        .fifo_count       (16'd0),
        .fifo_rd          (fifo_rd_sink),
        .mem_side         (stream_type_ctx == 1'b0),
        .cfg_error        (rd_cfg_error)
    );

    // =====================================================================
    // WRITE-streamer instance (stream_type decides which bus it writes to)
    //   Expected per-beat (while in REQ):
    //     - Prefetch: when FIFO has data and no buffered word, pulse fifo_rd once
    //     - Two cycles later: latch fifo_dout into internal buffer
    //     - External side: asserts wr_req only when buffered; when wr_ack=1,
    //       drives wr_wdata and advances address/byte count
    // =====================================================================
    wire                     wr_req, wr_wr_en, wr_ack;
    wire [ADDR_WIDTH-1:0]    wr_addr;
    wire [DATA_WIDTH-1:0]    wr_wdata, wr_rdata;
    wire                     wr_busy, wr_done;
    wire                     wr_cfg_error;

    wire [DATA_WIDTH-1:0]    ws_rdata_sink         ;
    wire                     ws_rvalid_sink        ;

    assign wr_rdata = {DATA_WIDTH{1'b0}};               // not used in WRITE
    assign wr_ack   = (stream_type_ctx == 1'b0) ? peri_ack : mem_ack;

    dma_streamer #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .BURST_MAX  (BURST_MAX)
    ) u_write (
        .clk              (clk),
        .rst              (rst),
        .start            (start_accepted),
        .stream_type      (1'b1),               // WRITE instance
        .base_addr        ((stream_type_ctx==1'b0) ? peri_base_addr_ctx : mem_base_addr_ctx),
        .num_bytes        (num_bytes_ctx),

        // FIFO (WRITE pops)
        .write_data       (fifo_dout),
        .write_data_valid (!fifo_empty),        // hint only
        .read_data        (ws_rdata_sink),      // unused in WRITE
        .read_data_valid  (ws_rvalid_sink),     // unused in WRITE

        .busy             (wr_busy),
        .done             (wr_done),

        // External bus
        .bus_req          (wr_req),
        .bus_wr_en        (wr_wr_en),
        .bus_addr         (wr_addr),
        .bus_wdata        (wr_wdata),
        .bus_rdata        (wr_rdata),           // unused in WRITE
        .bus_ack          (wr_ack),

        // FIFO handshake
        .fifo_empty       (fifo_empty),
        .fifo_count       (fifo_count),
        .fifo_rd          (fifo_rd),
        .mem_side         (stream_type_ctx == 1'b1),
        .cfg_error        (wr_cfg_error)
    );

    // Sticky config-error visibility inside DMA hierarchy (no interface change).
    wire dma_cfg_error ;
    assign dma_cfg_error = rd_cfg_error | wr_cfg_error;

    wire busy_int = rd_busy | wr_busy;
    wire done_int = rd_done & wr_done;

    // Descriptor capture / start filtering
    always @(posedge clk) begin
        if (rst) begin
            start_accepted       <= 1'b0;
            stream_type_ctx      <= 1'b0;
            mem_base_addr_ctx    <= {ADDR_WIDTH{1'b0}};
            peri_base_addr_ctx   <= {ADDR_WIDTH{1'b0}};
            num_bytes_ctx        <= 16'd0;
            start_while_busy_seen <= 1'b0;
        end else begin
            start_accepted <= 1'b0;
            if (start) begin
                if (!busy_int) begin
                    stream_type_ctx    <= stream_type;
                    mem_base_addr_ctx  <= mem_base_addr;
                    peri_base_addr_ctx <= peri_base_addr;
                    num_bytes_ctx      <= num_bytes;
                    start_accepted     <= 1'b1;
                    if (num_bytes[2:0] != 3'b000) begin
                        $display("[DMA_CFG_ERR][%0t][TOP] dir=%s num_bytes=%0d is not 8B aligned; transfer rejected",
                                 $time, (stream_type ? "P2M" : "M2P"), num_bytes);
                    end
                end else begin
                    start_while_busy_seen <= 1'b1;
                    $display("[DMA_IGNORED_START][%0t][TOP] busy=1 active_dir=%s new_dir=%s new_base=0x%016h new_bytes=%0d",
                             $time,
                             (stream_type_ctx ? "P2M" : "M2P"),
                             (stream_type ? "P2M" : "M2P"),
                             (stream_type ? mem_base_addr : peri_base_addr),
                             num_bytes);
                end
            end
        end
    end

    // =====================================================================
    // External bus multiplexing per stream_type
    //   M2P (0): READ<->mem_* , WRITE<->peri_*
    //   P2M (1): READ<->peri_*, WRITE<->mem_*
    // =====================================================================
    assign mem_req   = (stream_type_ctx==1'b0) ? rd_req   : wr_req;
    assign mem_wr_en = (stream_type_ctx==1'b0) ? rd_wr_en : wr_wr_en;
    assign mem_addr  = (stream_type_ctx==1'b0) ? rd_addr  : wr_addr;
    assign mem_wdata = (stream_type_ctx==1'b0) ? rd_wdata : wr_wdata;

    assign peri_req   = (stream_type_ctx==1'b0) ? wr_req   : rd_req;
    assign peri_wr_en = (stream_type_ctx==1'b0) ? wr_wr_en : rd_wr_en;
    assign peri_addr  = (stream_type_ctx==1'b0) ? wr_addr  : rd_addr;
    assign peri_wdata = (stream_type_ctx==1'b0) ? wr_wdata : rd_wdata;

    // ---------------------------------------------------------------------
    // Status aggregation
    // ---------------------------------------------------------------------
    assign busy = busy_int;
    // Suppress stale done during accepted-start handoff (start pulse is
    // delivered to streamers on the following edge via start_accepted).
    assign done = done_int & ~start_accepted;

endmodule


// ============================================================================
// dma_fifo.v
// Small, fully-synchronous FIFO used between the two streamers.
//
// Interface:
//   - Write:  write_en + write_data (when !full) pushes one word
//   - Read:   read_en  (when !empty) pops one word into read_data
//   - empty/full derived from an internal occupancy counter
//
// Timing:
//   - read_data updates on the same rising edge as a valid pop (read_en && !empty)
//   - simultaneous read & write leaves the occupancy count unchanged
// ============================================================================
`timescale 1ns/1ps
`default_nettype none

module dma_fifo #(
    parameter DATA_WIDTH = 64,
    parameter DEPTH      = 16
)(
    input  wire                   clk,
    input  wire                   rst_n,       // active-low
    input  wire                   write_en,
    input  wire [DATA_WIDTH-1:0]  write_data,
    output wire                   full,
    output wire                   almost_full,

    input  wire                   read_en,
    output reg  [DATA_WIDTH-1:0]  read_data,
    output wire                   empty,
    output wire [15:0]            level
);

    // Storage and state
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    localparam PTR_W = $clog2(DEPTH);
    localparam CNT_W = $clog2(DEPTH+1);

    reg [PTR_W-1:0]  wr_ptr, rd_ptr;
    reg [CNT_W-1:0]  count;

    // Write port
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (write_en && !full) begin
            mem[wr_ptr] <= write_data;
            wr_ptr      <= wr_ptr + 1'b1;
        end
    end

    // Read port (registered output)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr    <= '0;
            read_data <= '0;
        end else if (read_en && !empty) begin
            read_data <= mem[rd_ptr];
            rd_ptr    <= rd_ptr + 1'b1;
        end
    end

    // Occupancy counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= '0;
        end else begin
            case ({write_en && !full, read_en && !empty})
                2'b10 : count <= count + 1'b1; // write only
                2'b01 : count <= count - 1'b1; // read only
                default: /* no change (00 or 11) */;
            endcase
        end
    end

    assign empty = (count == 0);
    assign full  = (count == DEPTH);
    assign almost_full = (count >= (DEPTH-1));
    assign level = {{(16-CNT_W){1'b0}}, count};

endmodule



// ============================================================================
// dma_streamer.v
// Single-beat DMA streamer used in two roles:
//   - READ  (stream_type=0): External bus → FIFO
//   - WRITE (stream_type=1): FIFO → External bus
//
// Expected behavior per mode (while in STATE_REQ):
//   READ  (stream_type=0)
//     - Drives bus_req=1
//     - When bus_ack=1: captures bus_rdata into read_data
//     - Pulses read_data_valid=1 for one cycle to push into FIFO
//     - Increments address by DATA_WIDTH/8 and decrements bytes_left
//
//   WRITE (stream_type=1)
//     - If no buffered word and FIFO has data: pulses fifo_rd once to prefetch
//     - Two cycles later: latches write_data into internal wdata_hold, sets wdata_valid
//     - While wdata_valid=1: drives bus_req=1 and bus_wdata=wdata_hold
//     - On (bus_req & bus_ack): clears wdata_valid, advances address/bytes_left
//
// Status:
//   - busy goes high on start (from IDLE) and low when FSM reaches DONE
//   - done asserts when FSM enters DONE and is cleared on the next start/reset
// ============================================================================
`timescale 1ns/1ps
`default_nettype none

module dma_streamer #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 64,
    
    parameter BURST_MAX  = 16
    
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   start,
    input  wire                   stream_type,      // 0 = READ, 1 = WRITE
    input  wire [ADDR_WIDTH-1:0]  base_addr,
    input  wire [15:0]            num_bytes,

    // FIFO side
    input  wire [DATA_WIDTH-1:0]  write_data,        // FIFO dout (to bus on WRITE)
    input  wire                   write_data_valid,  // unused (kept for symmetry)
    output wire [DATA_WIDTH-1:0]  read_data,         // bus data to FIFO on READ handshake
    output wire                   read_data_valid,   // FIFO write enable on READ handshake

    // Status
    output reg                    busy,
    output reg                    done,

    // External bus
    output wire                   bus_req,
    output wire                   bus_wr_en,
    output wire [ADDR_WIDTH-1:0]  bus_addr,
    output wire [DATA_WIDTH-1:0]  bus_wdata,
    input  wire [DATA_WIDTH-1:0]  bus_rdata,
    input  wire                   bus_ack,

    // FIFO handshake (WRITE only)
    input  wire                   fifo_empty,
    input  wire [15:0]            fifo_count,
    output reg                    fifo_rd,
    // 1 when this streamer's external bus is the memory side for current transfer.
    input  wire                   mem_side,
    // Sticky configuration error (e.g., non-8B-aligned num_bytes).
    output reg                    cfg_error
);
    localparam BEAT_BYTES = DATA_WIDTH/8;
    localparam BEAT_SHIFT = $clog2(BEAT_BYTES);
    localparam integer BURST_MAX_I = (BURST_MAX < 1) ? 1 : BURST_MAX;
    localparam integer WAIT_SOFT_CYCLES = 64;
    localparam integer WAIT_HARD_CYCLES = 1024;
    localparam integer MIN_TIMEOUT_BURST_I = 4;
    localparam [31:0] PAGE_4KB = 32'd4096;
    localparam STATE_IDLE = 2'd0,
               STATE_REQ  = 2'd1,
               STATE_DONE = 2'd2;

    reg [1:0]               state,  next_state;
    reg [ADDR_WIDTH-1:0]    addr,   next_addr;
    reg [15:0]              bytes_left, next_bytes_left;

    // WRITE prefetch/buffer pipeline
    // Four-word queue to sustain short contiguous burst drains.
    reg [DATA_WIDTH-1:0]    wdata_hold;
    reg                     wdata_valid;
    reg [DATA_WIDTH-1:0]    wdata_next_hold;
    reg                     wdata_next_valid;
    reg [DATA_WIDTH-1:0]    wdata_2_hold;
    reg                     wdata_2_valid;
    reg [DATA_WIDTH-1:0]    wdata_3_hold;
    reg                     wdata_3_valid;
    reg                     pop_d1, pop_d2; // two-cycle delay of fifo_rd
    reg [15:0]              burst_limit_beats;
    reg [15:0]              drain_beats_remaining;
    reg [15:0]              drain_target_beats;
    reg [15:0]              drain_start_beats_n;
    reg                     drain_start_n;
    reg                     drain_active;
    reg [1:0]               drain_reason_n;
    reg [10:0]              wait_ctr;
    integer                 beats_left_i;
    integer                 beats_4kb_i;
    integer                 burst_limit_i;
    reg [DATA_WIDTH-1:0]    q0_data_n;
    reg                     q0_valid_n;
    reg [DATA_WIDTH-1:0]    q1_data_n;
    reg                     q1_valid_n;
    reg [DATA_WIDTH-1:0]    q2_data_n;
    reg                     q2_valid_n;
    reg [DATA_WIDTH-1:0]    q3_data_n;
    reg                     q3_valid_n;
    reg                     rd_outstanding;

    // Qualify WRITE handshake (must be in REQ, have data buffered, and see ack)
    wire ack_qual   = bus_ack & bus_req;
    wire do_write_hs = stream_type && (state==STATE_REQ) && wdata_valid && ack_qual;
    wire do_read_hs  = (!stream_type) && (state==STATE_REQ) && bus_ack && bus_req;
    assign read_data       = bus_rdata;
    assign read_data_valid = do_read_hs;
    wire beat_hs     = do_write_hs || do_read_hs;
    wire bad_len     = (num_bytes[BEAT_SHIFT-1:0] != {BEAT_SHIFT{1'b0}});
    wire [2:0] queue_used_beats = {1'd0, wdata_valid} + {1'd0, wdata_next_valid} +
                                  {1'd0, wdata_2_valid} + {1'd0, wdata_3_valid};
    wire [2:0] pending_pop_beats = {2'd0, fifo_rd} + {2'd0, pop_d1} + {2'd0, pop_d2};
    wire [2:0] issue_slots_used = queue_used_beats + pending_pop_beats;
    // Buffered beats visible for chunk qualification:
    // queue_used_beats are immediately issueable; fifo_count are already
    // captured upstream and can be prefetched into the queue.
    wire [15:0] buffered_ready_beats = {13'd0, queue_used_beats} + fifo_count;
    wire timeout_soft_reached = (wait_ctr >= WAIT_SOFT_CYCLES[10:0]);
    wire timeout_hard_reached = (wait_ctr >= WAIT_HARD_CYCLES[10:0]);
    wire full_chunk_ready = (burst_limit_beats != 16'd0) && (buffered_ready_beats >= burst_limit_beats);
    wire [15:0] timeout_chunk_beats = (buffered_ready_beats < burst_limit_beats) ? buffered_ready_beats : burst_limit_beats;
    wire one_beat_only = (timeout_chunk_beats == 16'd1);
    wire in_p2m_mem_write = stream_type && mem_side && (state == STATE_REQ) && (bytes_left != 16'd0);
    // Prefetch policy:
    //   - memory-side WRITE (P2M): aggressive prefetch for burst draining
    //   - non-memory-side WRITE: keep legacy conservative behavior
    wire need_refill_mem = stream_type && mem_side && (state==STATE_REQ) && (bytes_left != 16'd0) &&
                           !fifo_rd && !pop_d1 && !pop_d2 &&
                           (fifo_count > 16'd0) && (issue_slots_used < 3'd4);
    wire need_refill_nonmem = stream_type && !mem_side && (state==STATE_REQ) && (bytes_left != 16'd0) &&
                              !fifo_empty && !fifo_rd && !pop_d1 && !pop_d2 && !wdata_valid;
    wire need_refill = need_refill_mem || need_refill_nonmem;
    wire can_drain_now = !mem_side || drain_active;

    // WRITE data queue next-state (consume first, then enqueue refill beat).
    always @* begin
        q0_data_n   = wdata_hold;
        q0_valid_n  = wdata_valid;
        q1_data_n   = wdata_next_hold;
        q1_valid_n  = wdata_next_valid;
        q2_data_n   = wdata_2_hold;
        q2_valid_n  = wdata_2_valid;
        q3_data_n   = wdata_3_hold;
        q3_valid_n  = wdata_3_valid;

        if (stream_type) begin
            if (do_write_hs) begin
                q0_data_n  = q1_data_n;
                q0_valid_n = q1_valid_n;
                q1_data_n  = q2_data_n;
                q1_valid_n = q2_valid_n;
                q2_data_n  = q3_data_n;
                q2_valid_n = q3_valid_n;
                q3_valid_n = 1'b0;
            end

            if (pop_d2) begin
                if (!q0_valid_n) begin
                    q0_data_n  = write_data;
                    q0_valid_n = 1'b1;
                end else if (!q1_valid_n) begin
                    q1_data_n  = write_data;
                    q1_valid_n = 1'b1;
                end else if (!q2_valid_n) begin
                    q2_data_n  = write_data;
                    q2_valid_n = 1'b1;
                end else if (!q3_valid_n) begin
                    q3_data_n  = write_data;
                    q3_valid_n = 1'b1;
                end
            end
        end
    end

    // Decide when to start a memory-side drain chunk:
    //   - normal: wait until full chunk is buffered
    //   - fallback: on timeout, start short chunk from available buffered beats
    always @* begin
        drain_start_n = 1'b0;
        drain_start_beats_n = 16'd0;
        drain_reason_n = 2'd0; // 0=full, 1=tail, 2=timeout
        if (in_p2m_mem_write && !drain_active) begin
            if (full_chunk_ready) begin
                drain_start_n = 1'b1;
                drain_start_beats_n = burst_limit_beats;
                if (burst_limit_beats == ({16'd0, bytes_left} >> BEAT_SHIFT))
                    drain_reason_n = 2'd1; // tail/final chunk
                else
                    drain_reason_n = 2'd0; // full chunk
            end else if (timeout_soft_reached &&
                         (timeout_chunk_beats >= MIN_TIMEOUT_BURST_I[15:0]) &&
                         (burst_limit_beats <= MIN_TIMEOUT_BURST_I[15:0])) begin
                // Soft-timeout fallback for small/tail chunks only.
                // For long transfers (burst_limit > MIN_TIMEOUT_BURST_I),
                // wait for full-chunk qualification; hard-timeout still
                // guarantees forward progress if source is too slow.
                drain_start_n = 1'b1;
                drain_start_beats_n = timeout_chunk_beats;
                drain_reason_n = 2'd2;
            end else if (timeout_hard_reached && (timeout_chunk_beats != 16'd0)) begin
                // Hard-timeout fallback: guarantee forward progress.
                // 1-beat is allowed here only if no larger timeout chunk is present.
                if ((timeout_chunk_beats > 16'd1) || one_beat_only) begin
                    drain_start_n = 1'b1;
                    drain_start_beats_n = timeout_chunk_beats;
                    drain_reason_n = 2'd3;
                end
            end
        end
    end

    // Sequential
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= STATE_IDLE;
            addr         <= '0;
            bytes_left   <= '0;
            busy         <= 1'b0;
            done         <= 1'b0;
            fifo_rd      <= 1'b0;
            pop_d1       <= 1'b0;
            pop_d2       <= 1'b0;
            wdata_hold   <= '0;
            wdata_valid  <= 1'b0;
            wdata_next_hold  <= '0;
            wdata_next_valid <= 1'b0;
            wdata_2_hold <= '0;
            wdata_2_valid <= 1'b0;
            wdata_3_hold <= '0;
            wdata_3_valid <= 1'b0;
            rd_outstanding <= 1'b0;
            drain_beats_remaining <= 16'd0;
            drain_target_beats <= 16'd0;
            drain_active <= 1'b0;
            wait_ctr <= 11'd0;
            cfg_error    <= 1'b0;
        end else begin
            // FSM / counters
            state       <= next_state;
            addr        <= next_addr;
            bytes_left  <= next_bytes_left;
            // READ request hold: once issued, keep request asserted until ack.
            // This prevents req drops when FIFO backpressure toggles after launch.
            if (stream_type || (state != STATE_REQ) || (bytes_left == 16'd0)) begin
                rd_outstanding <= 1'b0;
            end else if (do_read_hs) begin
                rd_outstanding <= 1'b0;
            end else if (write_data_valid && !bus_ack) begin
                rd_outstanding <= 1'b1;
            end

            // WRITE role: prefetch → buffer → drive
            fifo_rd <= 1'b0; // default

            if (need_refill && !fifo_empty)
                fifo_rd <= 1'b1;                    // prefetch one word

            pop_d1 <= fifo_rd;                      // 1st delay
            pop_d2 <= pop_d1;                       // 2nd delay

            if (state != STATE_REQ || !mem_side || !stream_type) begin
                drain_beats_remaining <= 16'd0;
                drain_target_beats <= 16'd0;
                drain_active <= 1'b0;
                wait_ctr <= 11'd0;
            end else begin
                if ((bytes_left != 16'd0) && (drain_active || drain_start_n || do_write_hs || pop_d2 || fifo_rd || pop_d1)) begin
                    $display("[DMA_DBG][%0t] addr=0x%016h bytes_left=%0d fifo_cnt=%0d qv=%0b qn=%0b fifo_rd=%0b p1=%0b p2=%0b wr_hs=%0b req=%0b ack=%0b wv=%0b drain_act=%0b drain_rem=%0d start=%0b start_beats=%0d wait=%0d",
                             $time, addr, bytes_left, fifo_count, wdata_valid, wdata_next_valid, fifo_rd, pop_d1, pop_d2,
                             do_write_hs, bus_req, bus_ack, wdata_valid, drain_active, drain_beats_remaining, drain_start_n, drain_start_beats_n, wait_ctr);
                end
                if (drain_start_n) begin
                    drain_active <= 1'b1;
                    drain_beats_remaining <= drain_start_beats_n;
                    drain_target_beats <= drain_start_beats_n;
                    wait_ctr <= 11'd0;
                    $display("[DRAIN_DECIDE][%0t] burst_limit=%0d buffered=%0d issue_slots=%0d fifo_cnt=%0d reason=%0d",
                             $time, burst_limit_beats, buffered_ready_beats, issue_slots_used, fifo_count, drain_reason_n);
                    $display("[DMA_BURST_START][%0t] addr=0x%016h bytes_left=%0d chunk_beats=%0d buffered=%0d reason=%0d",
                             $time, addr, bytes_left, drain_start_beats_n, buffered_ready_beats, drain_reason_n);
                end else if (drain_active && do_write_hs) begin
                    if (drain_beats_remaining > 16'd1) begin
                        drain_beats_remaining <= drain_beats_remaining - 16'd1;
                        $display("[DMA_BURST_BEAT][%0t] idx=%0d addr=0x%016h wdata=0x%016h rem_after=%0d",
                                 $time,
                                 (drain_target_beats - drain_beats_remaining + 16'd1),
                                 addr, wdata_hold, (drain_beats_remaining - 16'd1));
                    end else begin
                        drain_beats_remaining <= 16'd0;
                        drain_active <= 1'b0;
                        $display("[DMA_BURST_BEAT][%0t] idx=%0d addr=0x%016h wdata=0x%016h rem_after=0",
                                 $time, drain_target_beats, addr, wdata_hold);
                        $display("[DMA_BURST_END][%0t] addr=0x%016h total_beats=%0d",
                                 $time, addr, drain_target_beats);
                    end
                end

                if (!drain_active && !drain_start_n && (bytes_left != 16'd0)) begin
                    if (wait_ctr < WAIT_HARD_CYCLES[10:0])
                        wait_ctr <= wait_ctr + 11'd1;
                end else if (!drain_active) begin
                    wait_ctr <= 11'd0;
                end
            end

            // Keep buffered WRITE data valid until accepted, and prefetch
            // ahead so memory-side writes can run as contiguous beat streams.
            if (stream_type) begin
                wdata_hold       <= q0_data_n;
                wdata_valid      <= q0_valid_n;
                wdata_next_hold  <= q1_data_n;
                wdata_next_valid <= q1_valid_n;
                wdata_2_hold     <= q2_data_n;
                wdata_2_valid    <= q2_valid_n;
                wdata_3_hold     <= q3_data_n;
                wdata_3_valid    <= q3_valid_n;
            end

            // Status
                if (start && (state == STATE_IDLE)) begin
                // Zero-length request completes immediately.
                if (bad_len) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    cfg_error <= 1'b1;
                    // Top-level prints one consolidated cfg-error line.
                end else begin
                    busy <= (num_bytes != 16'd0);
                    done <= (num_bytes == 16'd0);
                    cfg_error <= 1'b0;
                end
                wdata_valid <= 1'b0;
                wdata_next_valid <= 1'b0;
                wdata_2_valid <= 1'b0;
                wdata_3_valid <= 1'b0;
                drain_active <= 1'b0;
                drain_beats_remaining <= 16'd0;
                drain_target_beats <= 16'd0;
                wait_ctr <= 11'd0;
            end else begin
                // Keep legacy early-done behavior and also cover the
                // STATE_DONE cycle itself (important for zero-length case).
                if ((next_state == STATE_DONE) || (state == STATE_DONE)) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end

            // Cosmetic scrub on return to IDLE
            if (next_state == STATE_IDLE && state != STATE_IDLE) begin
                addr      <= {ADDR_WIDTH{1'b0}};            end
        end
    end

    // Next-state / counters
    always @* begin
        // Burst chunk sizing for memory-side traffic:
        //   - cap by remaining beats
        //   - cap by BURST_MAX
        //   - cap to 4KB boundary
        beats_left_i   = {16'd0, bytes_left} >> BEAT_SHIFT;
        beats_4kb_i    = (PAGE_4KB - {20'd0, addr[11:0]}) >> BEAT_SHIFT;
        burst_limit_i  = beats_left_i;
        if (burst_limit_i > BURST_MAX_I)
            burst_limit_i = BURST_MAX_I;
        if (burst_limit_i > beats_4kb_i)
            burst_limit_i = beats_4kb_i;
        if ((burst_limit_i == 0) && (beats_left_i != 0))
            burst_limit_i = 1;
        burst_limit_beats = burst_limit_i[15:0];

        next_state       = state;
        next_addr        = addr;
        next_bytes_left  = bytes_left;

        case (state)
            STATE_IDLE: begin
                if (start) begin
                    if (bad_len)
                        next_state = STATE_DONE;
                    else
                        next_state = (num_bytes == 0) ? STATE_DONE : STATE_REQ;
                    next_addr        = base_addr;
                    next_bytes_left  = bad_len ? 16'd0 : num_bytes;
                end
            end

            STATE_REQ: begin
                if (stream_type) begin
                    // WRITE: advance only on real WRITE handshake
                    if (do_write_hs) begin
                        next_addr        = addr + BEAT_BYTES;
                        next_bytes_left  = (bytes_left >= BEAT_BYTES) ? (bytes_left - BEAT_BYTES) : 16'd0;
                        if (next_bytes_left == 0)
                            next_state = STATE_DONE;
                    end
                end else begin
                    // READ: advance only when both source and sink handshake
                    if (bus_ack && bus_req) begin
                        next_addr        = addr + BEAT_BYTES;
                        next_bytes_left  = (bytes_left >= BEAT_BYTES) ? (bytes_left - BEAT_BYTES) : 16'd0;
                        if (next_bytes_left == 0)
                            next_state = STATE_DONE;
                    end
                end
            end

            default: begin // STATE_DONE
                next_state = STATE_IDLE;           // auto-return after one cycle
            end
        endcase
    end

    // External bus drives are combinational from internal state/data so the
    // first REQ cycle presents the correct address/data (no stale registered 0s).
    assign bus_wr_en = stream_type;
    assign bus_addr  = addr;
    assign bus_wdata = (stream_type && bus_req) ? wdata_hold : {DATA_WIDTH{1'b0}};

    // bus_req generation (combinational) to avoid one-cycle stale requests
    // when sink-side backpressure changes.
    assign bus_req = (!stream_type)
                   ? ((state == STATE_REQ) && (bytes_left != 16'd0) && (rd_outstanding || write_data_valid))
                   : ((state == STATE_REQ) && wdata_valid && can_drain_now);
endmodule
`default_nettype wire
