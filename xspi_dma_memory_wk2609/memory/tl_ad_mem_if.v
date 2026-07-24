`include "tl_sram_ctrl_pkg.vh"
`include "./atomic_unit.v"

module tl_ad_mem_if #(
    parameter AW        = 64,
    parameter DW        = 64,
    parameter SOURCE_W  = 4,
    parameter SINK_W    = 1,
    parameter SZW       = 8,
    parameter MAX_BURST = 16
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // A channel request interface
    input  wire                  a_valid,
    output wire                  a_ready,
    input  wire [2:0]            a_opcode,
    input  wire [2:0]            a_param,
    input  wire [SZW-1:0]        a_size,
    input  wire [SOURCE_W-1:0]   a_source,
    input  wire [AW-1:0]         a_address,
    input  wire [DW/8-1:0]       a_mask,
    input  wire [DW-1:0]         a_data,
    input  wire                  a_corrupt,

    // D channel response interface
    output reg                   d_valid,
    input  wire                  d_ready,
    output reg  [2:0]            d_opcode,
    output reg  [1:0]            d_param,
    output reg  [SZW-1:0]        d_size,
    output reg  [SOURCE_W-1:0]   d_source,
    output wire [SINK_W-1:0]     d_sink,
    output reg                   d_denied,
    output reg  [DW-1:0]         d_data,
    output reg                   d_corrupt,

    // Raw memory backend
    output reg                   mem_req,
    output reg                   mem_we,
    output reg  [AW-1:0]         mem_addr,
    output reg  [DW-1:0]         mem_wdata,
    output reg  [DW/8-1:0]       mem_wmask,
    input  wire [DW-1:0]         mem_rdata,
    input  wire                  mem_rvalid,
    input  wire                  mem_ready
);

    localparam [3:0] ST_IDLE        = 4'd0;
    localparam [3:0] ST_READ        = 4'd1;
    localparam [3:0] ST_WRITE       = 4'd2;
    localparam [3:0] ST_ATOMIC_RD   = 4'd3;
    localparam [3:0] ST_ATOMIC_WR   = 4'd4;
    localparam [3:0] ST_RESPOND     = 4'd5;
    localparam [3:0] ST_BURST_RD    = 4'd6;
    localparam [3:0] ST_BURST_RD_RSP= 4'd7;
    localparam [3:0] ST_BURST_WR    = 4'd8;
    localparam [3:0] ST_BURST_WR_REQ= 4'd9;

    reg [3:0]              state;
    reg [2:0]              saved_opcode;
    reg [2:0]              saved_param;
    reg [SZW-1:0]          saved_size;
    reg [SOURCE_W-1:0]     saved_source;
    reg [AW-1:0]           saved_address;
    reg [DW-1:0]           saved_wdata;
    reg [DW/8-1:0]         saved_mask;
    reg [DW-1:0]           atomic_orig;
    reg [4:0]              burst_count;
    reg [4:0]              burst_remaining;
    reg [4:0]              burst_total;
    reg [3:0]              burst_wr_idx;
    reg [3:0]              burst_rd_idx;

    reg [DW-1:0]           burst_buffer [0:MAX_BURST-1];

    wire [DW-1:0] atomic_result;
    wire [DW-1:0] atomic_original;

    atomic_unit #(
        .DW(DW)
    ) u_atomic (
        .opcode(saved_opcode),
        .param(saved_param),
        .operand_a(mem_rdata),
        .operand_b(saved_wdata),
        .result(atomic_result),
        .original(atomic_original)
    );

    wire is_read   = (a_opcode == TL_A_GET);
    wire is_write  = (a_opcode == TL_A_PUT_FULL) || (a_opcode == TL_A_PUT_PARTIAL);
    wire is_atomic = (a_opcode == TL_A_ARITHMETIC) || (a_opcode == TL_A_LOGICAL);
    wire is_hint   = (a_opcode == TL_A_INTENT);

    localparam [SZW-1:0] DATA_SIZE_LOG = 3;
    wire needs_burst = (a_size > DATA_SIZE_LOG);
    wire [4:0] num_beats = needs_burst ? (5'd1 << (a_size - DATA_SIZE_LOG)) : 5'd1;

    wire can_accept       = (state == ST_IDLE) && mem_ready;
    wire can_accept_burst = (state == ST_BURST_WR) && ({1'b0, burst_wr_idx} < burst_total);

    assign a_ready = can_accept || can_accept_burst;
    assign d_sink  = {SINK_W{1'b0}};

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= ST_IDLE;
            d_valid          <= 1'b0;
            d_opcode         <= 3'd0;
            d_param          <= 2'd0;
            d_size           <= {SZW{1'b0}};
            d_source         <= {SOURCE_W{1'b0}};
            d_denied         <= 1'b0;
            d_data           <= {DW{1'b0}};
            d_corrupt        <= 1'b0;
            mem_req          <= 1'b0;
            mem_we           <= 1'b0;
            mem_addr         <= {AW{1'b0}};
            mem_wdata        <= {DW{1'b0}};
            mem_wmask        <= {(DW/8){1'b0}};
            saved_opcode     <= 3'd0;
            saved_param      <= 3'd0;
            saved_size       <= {SZW{1'b0}};
            saved_source     <= {SOURCE_W{1'b0}};
            saved_address    <= {AW{1'b0}};
            saved_wdata      <= {DW{1'b0}};
            saved_mask       <= {(DW/8){1'b0}};
            atomic_orig      <= {DW{1'b0}};
            burst_count      <= 5'd0;
            burst_remaining  <= 5'd0;
            burst_total      <= 5'd0;
            burst_wr_idx     <= 4'd0;
            burst_rd_idx     <= 4'd0;
            for (i = 0; i < MAX_BURST; i = i + 1)
                burst_buffer[i] <= {DW{1'b0}};
        end else begin
            mem_req <= 1'b0;

            if (d_valid && d_ready)
                d_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (a_valid && can_accept) begin
                        saved_opcode   <= a_opcode;
                        saved_param    <= a_param;
                        saved_size     <= a_size;
                        saved_source   <= a_source;
                        saved_address  <= a_address;
                        saved_wdata    <= a_data;
                        saved_mask     <= a_mask;
                        burst_total    <= num_beats;
                        burst_count    <= 5'd0;
                        burst_wr_idx   <= 4'd0;
                        burst_rd_idx   <= 4'd0;

                        if (is_hint) begin
                            d_valid   <= 1'b1;
                            d_opcode  <= TL_D_HINT_ACK;
                            d_param   <= 2'd0;
                            d_size    <= a_size;
                            d_source  <= a_source;
                            d_data    <= {DW{1'b0}};
                            d_denied  <= 1'b0;
                            d_corrupt <= a_corrupt;
                        end else if (is_read) begin
                            mem_req  <= 1'b1;
                            mem_we   <= 1'b0;
                            mem_addr <= a_address;
                            if (needs_burst) begin
                                burst_remaining <= num_beats;
                                state <= ST_BURST_RD;
                            end else begin
                                state <= ST_READ;
                            end
                        end else if (is_write) begin
                            if (needs_burst) begin
                                burst_buffer[0] <= a_data;
                                burst_wr_idx <= 4'd1;
                                burst_remaining <= num_beats - 5'd1;
                                state <= ST_BURST_WR;
                            end else begin
                                mem_req   <= 1'b1;
                                mem_we    <= 1'b1;
                                mem_addr  <= a_address;
                                mem_wdata <= a_data;
                                mem_wmask <= a_mask;
                                state <= ST_WRITE;
                            end
                        end else if (is_atomic) begin
                            mem_req  <= 1'b1;
                            mem_we   <= 1'b0;
                            mem_addr <= a_address;
                            state <= ST_ATOMIC_RD;
                        end else begin
                            d_valid   <= 1'b1;
                            d_opcode  <= TL_D_ACCESS_ACK;
                            d_param   <= 2'd0;
                            d_size    <= a_size;
                            d_source  <= a_source;
                            d_data    <= {DW{1'b0}};
                            d_denied  <= 1'b1;
                            d_corrupt <= 1'b1;
                            state <= ST_RESPOND;
                        end
                    end
                end

                ST_BURST_RD: begin
                    if (mem_rvalid) begin
                        burst_buffer[burst_count[3:0]] <= mem_rdata;
                        burst_count <= burst_count + 5'd1;
                        burst_remaining <= burst_remaining - 5'd1;
                        if (burst_remaining == 5'd1) begin
                            burst_rd_idx <= 4'd0;
                            state <= ST_BURST_RD_RSP;
                        end else begin
                            mem_req  <= 1'b1;
                            mem_we   <= 1'b0;
                            mem_addr <= saved_address + ({{(AW-8){1'b0}}, burst_count + 5'd1, 3'b0});
                        end
                    end
                end

                ST_BURST_RD_RSP: begin
                    if (!d_valid || d_ready) begin
                        d_valid   <= 1'b1;
                        d_opcode  <= TL_D_ACCESS_ACK_DATA;
                        d_param   <= 2'd0;
                        d_size    <= saved_size;
                        d_source  <= saved_source;
                        d_data    <= burst_buffer[burst_rd_idx];
                        d_denied  <= 1'b0;
                        d_corrupt <= 1'b0;
                        burst_rd_idx <= burst_rd_idx + 4'd1;
                        if (burst_rd_idx == burst_total[3:0] - 4'd1)
                            state <= ST_RESPOND;
                    end
                end

                ST_BURST_WR: begin
                    if (a_valid && can_accept_burst) begin
                        burst_buffer[burst_wr_idx] <= a_data;
                        burst_wr_idx <= burst_wr_idx + 4'd1;
                        burst_remaining <= burst_remaining - 5'd1;
                        if (burst_remaining == 5'd1) begin
                            burst_count <= 5'd0;
                            state <= ST_BURST_WR_REQ;
                        end
                    end
                end

                ST_BURST_WR_REQ: begin
                    if (mem_ready) begin
                        mem_req   <= 1'b1;
                        mem_we    <= 1'b1;
                        mem_addr  <= saved_address + ({{(AW-8){1'b0}}, burst_count, 3'b0});
                        mem_wdata <= burst_buffer[burst_count[3:0]];
                        mem_wmask <= saved_mask;
                        burst_count <= burst_count + 5'd1;
                        if (burst_count == burst_total - 5'd1) begin
                            d_valid   <= 1'b1;
                            d_opcode  <= TL_D_ACCESS_ACK;
                            d_param   <= 2'd0;
                            d_size    <= saved_size;
                            d_source  <= saved_source;
                            d_data    <= {DW{1'b0}};
                            d_denied  <= 1'b0;
                            d_corrupt <= 1'b0;
                            state <= ST_RESPOND;
                        end
                    end
                end

                ST_READ: begin
                    if (mem_rvalid) begin
                        d_valid   <= 1'b1;
                        d_opcode  <= TL_D_ACCESS_ACK_DATA;
                        d_param   <= 2'd0;
                        d_size    <= saved_size;
                        d_source  <= saved_source;
                        d_data    <= mem_rdata;
                        d_denied  <= 1'b0;
                        d_corrupt <= 1'b0;
                        state <= ST_RESPOND;
                    end
                end

                ST_WRITE: begin
                    d_valid   <= 1'b1;
                    d_opcode  <= TL_D_ACCESS_ACK;
                    d_param   <= 2'd0;
                    d_size    <= saved_size;
                    d_source  <= saved_source;
                    d_data    <= {DW{1'b0}};
                    d_denied  <= 1'b0;
                    d_corrupt <= 1'b0;
                    state <= ST_RESPOND;
                end

                ST_ATOMIC_RD: begin
                    if (mem_rvalid) begin
                        atomic_orig <= mem_rdata;
                        mem_req   <= 1'b1;
                        mem_we    <= 1'b1;
                        mem_addr  <= saved_address;
                        mem_wdata <= atomic_result;
                        mem_wmask <= saved_mask;
                        state <= ST_ATOMIC_WR;
                    end
                end

                ST_ATOMIC_WR: begin
                    d_valid   <= 1'b1;
                    d_opcode  <= TL_D_ACCESS_ACK_DATA;
                    d_param   <= 2'd0;
                    d_size    <= saved_size;
                    d_source  <= saved_source;
                    d_data    <= atomic_orig;
                    d_denied  <= 1'b0;
                    d_corrupt <= 1'b0;
                    state <= ST_RESPOND;
                end

                ST_RESPOND: begin
                    if (d_ready) begin
                        d_valid <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
