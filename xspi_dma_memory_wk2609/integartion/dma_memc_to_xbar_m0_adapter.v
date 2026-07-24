`timescale 1ns/1ps
`default_nettype none


/*This is a burst-packing adapter between the streamer’s memc_* interface and 
crossbar master-0 TileLink request/response ports. 
It queues read/write requests, groups consecutive accesses into bursts, 
drives a_*_in0, watches d_*0_tb, and raises memc_write_done, memc_acc_done, 
memc_write_burst_done, and memc_read_burst_done.
Instantiates: none.
Best label: DMA/memory-side request stream -> TLUH/xbar master-0 adapter.
*/


///this is the integrtaion of dma bridge to tilelink uh master 0
module dma_memc_to_xbar_m0_adapter #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 64,
    parameter TL_OPCODE_WIDTH = 3,
    parameter TL_PARAM_WIDTH = 3,
    parameter TL_SIZE_WIDTH = 8,
    parameter TL_SOURCE_WIDTH = 3,
    parameter TL_SINK_WIDTH = 3,
    parameter MAX_BURST_BEATS = 16,
    parameter REQ_Q_DEPTH = 64,
    parameter DISPATCH_IDLE_CYCLES = 2
)(
    input  wire                         m_clk,
    input  wire                         reset_m,

    // Streamer-facing memc boundary
    input  wire [ADDR_WIDTH-1:0]        memc_waddr,
    input  wire                         memc_wen,
    input  wire [DATA_WIDTH-1:0]        memc_wdata,
    input  wire [ADDR_WIDTH-1:0]        memc_raddr,
    input  wire                         memc_ren,
    output reg  [DATA_WIDTH-1:0]        memc_rdata,
    output reg                          memc_write_done,
    output reg                          memc_acc_done,
    output reg                          memc_write_burst_done,
    output reg                          memc_read_burst_done,

    // Xbar M0 stimulus ports
    output reg                          a_valid_in0,
    output reg  [TL_OPCODE_WIDTH-1:0]   a_opcode_in0,
    output reg  [TL_PARAM_WIDTH-1:0]    a_param_in0,
    output reg  [ADDR_WIDTH-1:0]        a_address_in0,
    output reg  [TL_SIZE_WIDTH-1:0]     a_size_in0,
    output reg  [(DATA_WIDTH/8)-1:0]    a_mask_in0,
    output reg  [DATA_WIDTH-1:0]        a_data_in0,
    output reg  [TL_SOURCE_WIDTH-1:0]   a_source_in0,
    input  wire                         a_ready0_tb,

    // Xbar M0 response monitor ports
    input  wire [TL_OPCODE_WIDTH-1:0]   d_opcode0_tb,
    input  wire [TL_PARAM_WIDTH-1:0]    d_param0_tb,
    input  wire [TL_SIZE_WIDTH-1:0]     d_size0_tb,
    input  wire [TL_SINK_WIDTH-1:0]     d_sink0_tb,
    input  wire [TL_SOURCE_WIDTH-1:0]   d_source0_tb,
    input  wire [DATA_WIDTH-1:0]        d_data0_tb,
    input  wire                         d_valid0_tb,
    input  wire                         d_ready0_tb,
    input  wire                         d_error0_tb
);
    localparam [TL_OPCODE_WIDTH-1:0] TL_PUT_FULL = 3'd0;
    localparam [TL_OPCODE_WIDTH-1:0] TL_GET      = 3'd4;
    localparam integer BEAT_BYTES = DATA_WIDTH/8;
    localparam integer BEAT_LOG2  = $clog2(BEAT_BYTES);
    localparam integer QPTR_W     = $clog2(REQ_Q_DEPTH);
    localparam integer COUNT_W    = $clog2(REQ_Q_DEPTH+1);

    localparam ST_IDLE   = 2'd0;
    localparam ST_SEND_A = 2'd1;
    localparam ST_WAIT_D = 2'd2;

    reg [1:0] state;

    reg req_is_read [0:REQ_Q_DEPTH-1];
    reg [ADDR_WIDTH-1:0] req_addr [0:REQ_Q_DEPTH-1];
    reg [DATA_WIDTH-1:0] req_wdata [0:REQ_Q_DEPTH-1];

    reg [QPTR_W-1:0] q_wr_ptr;
    reg [QPTR_W-1:0] q_rd_ptr;
    reg [COUNT_W-1:0] q_count;

    reg [7:0] idle_ctr;
    wire enqueue_now = ((memc_wen || memc_ren) && !queue_full);

    reg tx_is_read;
    reg [ADDR_WIDTH-1:0] tx_base_addr;
    reg [4:0] tx_beats;
    reg [TL_SIZE_WIDTH-1:0] tx_size;
    reg [4:0] tx_a_sent;
    reg [4:0] tx_d_seen;

    reg launch_txn;
    reg [4:0] launch_beats;
    reg [TL_SIZE_WIDTH-1:0] launch_size;
    reg launch_is_read;
    reg [ADDR_WIDTH-1:0] launch_base_addr;

    integer i;
    integer run_len_i;
    integer max_scan_i;
    integer idx_i;
    integer next_idx_i;
    integer chosen_i;
    integer align_bytes_i;
    reg queue_nonempty;
    reg queue_full;
    reg flush_now;
    reg [ADDR_WIDTH-1:0] head_addr;
    reg head_is_read;

    wire d_hs = d_valid0_tb && d_ready0_tb;

    function integer pow2_floor;
        input integer val;
        integer p;
        begin
            p = 1;
            while ((p << 1) <= val)
                p = (p << 1);
            pow2_floor = p;
        end
    endfunction

    function [TL_SIZE_WIDTH-1:0] beats_to_size;
        input integer beats;
        integer lg2b;
        integer tmp;
        begin
            lg2b = 0;
            tmp = beats;
            while (tmp > 1) begin
                tmp = tmp >> 1;
                lg2b = lg2b + 1;
            end
            beats_to_size = TL_SIZE_WIDTH'(BEAT_LOG2 + lg2b);
        end
    endfunction

    function [QPTR_W-1:0] q_inc;
        input [QPTR_W-1:0] p;
        begin
            if (p == QPTR_W'(REQ_Q_DEPTH-1))
                q_inc = {QPTR_W{1'b0}};
            else
                q_inc = p + QPTR_W'(1);
        end
    endfunction

/* verilator lint_off BLKSEQ */
    function [QPTR_W-1:0] q_add;
        input [QPTR_W-1:0] p;
        input integer n;
        integer t;
        begin
            t = p + n;
            while (t >= REQ_Q_DEPTH)
                t = t - REQ_Q_DEPTH;
            q_add = t[QPTR_W-1:0];
        end
    endfunction

/* verilator lint_on BLKSEQ */

    always @* begin
        queue_nonempty = (q_count != 0);
        queue_full = (q_count == REQ_Q_DEPTH);
        flush_now = 1'b0;
        head_addr = {ADDR_WIDTH{1'b0}};
        head_is_read = 1'b0;
        run_len_i = 0;
        max_scan_i = 0;
        idx_i = 0;
        next_idx_i = 0;
        chosen_i = 0;
        align_bytes_i = 0;
        launch_txn = 1'b0;
        launch_beats = 5'd0;
        launch_size = {TL_SIZE_WIDTH{1'b0}};
        launch_is_read = 1'b0;
        launch_base_addr = {ADDR_WIDTH{1'b0}};

        if (queue_nonempty && (state == ST_IDLE)) begin
            head_addr = req_addr[q_rd_ptr];
            head_is_read = req_is_read[q_rd_ptr];

            flush_now = (q_count >= MAX_BURST_BEATS) ||
                        (idle_ctr >= DISPATCH_IDLE_CYCLES) ||
                        (!head_is_read && d_error0_tb);

            if (flush_now) begin
                run_len_i = 1;
                max_scan_i = (q_count > MAX_BURST_BEATS) ? MAX_BURST_BEATS : q_count;
                for (i = 1; i < MAX_BURST_BEATS; i = i + 1) begin
                    if (i < max_scan_i) begin
                        idx_i = q_rd_ptr + i - REQ_Q_DEPTH * ((q_rd_ptr + i) / REQ_Q_DEPTH);
                        next_idx_i = q_rd_ptr + (i-1) - REQ_Q_DEPTH * ((q_rd_ptr + (i-1)) / REQ_Q_DEPTH);
                        if (req_is_read[idx_i] == head_is_read &&
                            req_addr[idx_i] == (req_addr[next_idx_i] + BEAT_BYTES) &&
                            (req_addr[idx_i][ADDR_WIDTH-1:12] == head_addr[ADDR_WIDTH-1:12]))
                            run_len_i = i + 1;
                    end
                end

                chosen_i = pow2_floor(run_len_i);
                if (chosen_i > MAX_BURST_BEATS)
                    chosen_i = MAX_BURST_BEATS;

                while ((chosen_i > 1) && ((head_addr % (chosen_i * BEAT_BYTES)) != 0))
                    chosen_i = (chosen_i >> 1);

                launch_txn = 1'b1;
                launch_beats = chosen_i[4:0];
                launch_size = beats_to_size(chosen_i);
                launch_is_read = head_is_read;
                launch_base_addr = head_addr;
            end
        end
    end

    always @(posedge m_clk or posedge reset_m) begin
        if (reset_m) begin
            state <= ST_IDLE;

            q_wr_ptr <= {QPTR_W{1'b0}};
            q_rd_ptr <= {QPTR_W{1'b0}};
            q_count  <= {COUNT_W{1'b0}};

            idle_ctr <= 8'd0;

            tx_is_read <= 1'b0;
            tx_base_addr <= {ADDR_WIDTH{1'b0}};
            tx_beats <= 5'd0;
            tx_size <= {TL_SIZE_WIDTH{1'b0}};
            tx_a_sent <= 5'd0;
            tx_d_seen <= 5'd0;

            a_valid_in0 <= 1'b0;
            a_opcode_in0 <= {TL_OPCODE_WIDTH{1'b0}};
            a_param_in0 <= {TL_PARAM_WIDTH{1'b0}};
            a_address_in0 <= {ADDR_WIDTH{1'b0}};
            a_size_in0 <= {TL_SIZE_WIDTH{1'b0}};
            a_mask_in0 <= {(DATA_WIDTH/8){1'b0}};
            a_data_in0 <= {DATA_WIDTH{1'b0}};
            a_source_in0 <= {TL_SOURCE_WIDTH{1'b0}};

            memc_rdata <= {DATA_WIDTH{1'b0}};
            memc_write_done <= 1'b0;
            memc_acc_done <= 1'b0;
            memc_write_burst_done <= 1'b0;
            memc_read_burst_done <= 1'b0;
        end else begin
            memc_write_done <= 1'b0;
            memc_acc_done <= 1'b0;
            memc_write_burst_done <= 1'b0;
            memc_read_burst_done <= 1'b0;

            if (memc_wen && memc_ren) begin
                $error("[ADAPT] memc_wen and memc_ren high simultaneously");
            end

            if (memc_wen && !queue_full) begin
                req_is_read[q_wr_ptr] <= 1'b0;
                req_addr[q_wr_ptr]    <= memc_waddr;
                req_wdata[q_wr_ptr]   <= memc_wdata;
                q_wr_ptr <= q_inc(q_wr_ptr);
                q_count <= q_count + COUNT_W'(1);
            end else if (memc_ren && !queue_full) begin
                req_is_read[q_wr_ptr] <= 1'b1;
                req_addr[q_wr_ptr]    <= memc_raddr;
                req_wdata[q_wr_ptr]   <= {DATA_WIDTH{1'b0}};
                q_wr_ptr <= q_inc(q_wr_ptr);
                q_count <= q_count + COUNT_W'(1);
            end else if ((memc_wen || memc_ren) && queue_full) begin
                $error("[ADAPT] request queue overflow");
            end

            if (enqueue_now)
                idle_ctr <= 8'd0;
            else if (queue_nonempty && state == ST_IDLE)
                idle_ctr <= idle_ctr + 8'd1;
            else
                idle_ctr <= 8'd0;

            case (state)
                ST_IDLE: begin
                    a_valid_in0 <= 1'b0;
                    tx_a_sent <= 5'd0;
                    tx_d_seen <= 5'd0;
                    if (launch_txn) begin
                        tx_is_read <= launch_is_read;
                        tx_base_addr <= launch_base_addr;
                        tx_beats <= launch_beats;
                        tx_size <= launch_size;
                        state <= ST_SEND_A;
                    end
                end

                ST_SEND_A: begin
                    if (!a_valid_in0 && (tx_a_sent < tx_beats)) begin
                        a_valid_in0   <= 1'b1;
                        a_opcode_in0  <= tx_is_read ? TL_GET : TL_PUT_FULL;
                        a_param_in0   <= {TL_PARAM_WIDTH{1'b0}};
                        a_address_in0 <= tx_base_addr + (tx_a_sent * BEAT_BYTES);
                        a_size_in0    <= tx_size;
                        a_mask_in0    <= {(DATA_WIDTH/8){1'b1}};
                        a_source_in0  <= {TL_SOURCE_WIDTH{1'b0}};
                        if (tx_is_read)
                            a_data_in0 <= {DATA_WIDTH{1'b0}};
                        else begin
                            idx_i = q_rd_ptr + tx_a_sent - REQ_Q_DEPTH * ((q_rd_ptr + tx_a_sent) / REQ_Q_DEPTH);
                            a_data_in0 <= req_wdata[idx_i];
                        end
                    end

                    if (a_valid_in0 && a_ready0_tb) begin
                        a_valid_in0 <= 1'b0;
                        tx_a_sent <= tx_a_sent + 5'd1;
                        if ((tx_a_sent + 5'd1) == tx_beats)
                            state <= ST_WAIT_D;
                    end
                end

                ST_WAIT_D: begin
                    if (d_hs) begin
                        tx_d_seen <= tx_d_seen + 5'd1;
                        if (tx_is_read) begin
                            memc_rdata <= d_data0_tb;
                            memc_acc_done <= 1'b1;
                        end else begin
                            memc_write_done <= 1'b1;
                        end

                        if ((tx_d_seen + 5'd1) == tx_beats) begin
                            if (tx_is_read)
                                memc_read_burst_done <= 1'b1;
                            else
                                memc_write_burst_done <= 1'b1;

                            q_rd_ptr <= q_add(q_rd_ptr, tx_beats);
                            q_count <= q_count - tx_beats;
                            state <= ST_IDLE;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase

            if (d_error0_tb && d_hs) begin
                $error("[ADAPT] D-channel error observed on M0 response");
            end

            // Keep these monitors intentionally referenced to avoid accidental pruning.
            if (d_hs && (d_opcode0_tb[0] | d_param0_tb[0] | d_size0_tb[0] | d_sink0_tb[0] | d_source0_tb[0]))
                ;
        end
    end

endmodule

`default_nettype wire
