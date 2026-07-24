`timescale 1ns/1ps
`default_nettype none
`include "./tluh_bridge.v"
`include "./tilelink_uh_wrapper.v"

module dma_tluh_bridge_into_tluh_wrapper_top #(
    parameter integer ADDR_WIDTH       = 64,
    parameter integer DATA_WIDTH       = 64,

    // TL widths (match your files)
    parameter integer TL_ADDR_WIDTH    = 64,
    parameter integer TL_DATA_WIDTH    = 64,
    parameter integer TL_STRB_WIDTH    = (TL_DATA_WIDTH/8),
    parameter integer TL_SOURCE_WIDTH  = 3,
    parameter integer TL_SINK_WIDTH    = 3,
    parameter integer TL_OPCODE_WIDTH  = 3,
    parameter integer TL_PARAM_WIDTH   = 3,
    parameter integer TL_SIZE_WIDTH    = 8
)(
    input  wire                       clk,
    input  wire                       rst,       // active-high (wrapper)

    // -------- upstream DMA interface into the bridge ----------
    input  wire                       mem_req,
    input  wire                       mem_wr_en,
    input  wire [ADDR_WIDTH-1:0]      mem_addr,
    input  wire [DATA_WIDTH-1:0]      mem_wdata,
    output wire [DATA_WIDTH-1:0]      mem_rdata,
    output wire                       mem_ack,

    // -------- external memory interface out of wrapper slave ------
    output wire [TL_ADDR_WIDTH-1:0]   memc_waddr,
    output wire                       memc_wen,
    output wire [TL_DATA_WIDTH-1:0]   memc_wdata,
    output wire [TL_ADDR_WIDTH-1:0]   memc_raddr,
    output wire                       memc_ren,
    input  wire [TL_DATA_WIDTH-1:0]   memc_rdata,
    input  wire                       memc_write_done,
    input  wire                       memc_acc_done
    ,
    input  wire                       tb_override_a_valid,
    input  wire [TL_OPCODE_WIDTH-1:0] tb_override_a_opcode,
    input  wire [TL_PARAM_WIDTH-1:0]  tb_override_a_param,
    input  wire [TL_ADDR_WIDTH-1:0]   tb_override_a_address,
    input  wire [TL_SIZE_WIDTH-1:0]   tb_override_a_size,
    input  wire [TL_STRB_WIDTH-1:0]   tb_override_a_mask,
    input  wire [TL_DATA_WIDTH-1:0]   tb_override_a_data,
    input  wire [TL_SOURCE_WIDTH-1:0] tb_override_a_source,
    input  wire                       tb_override_a_corrupt,
    input  wire                       tb_drive_d_ready,
    input  wire                       tb_d_ready_override
);

    // ------------------------------------------------------------
    // Bridge <-> Adapter TL signals
    // ------------------------------------------------------------
    wire                       br_tl_a_valid;
    wire [TL_OPCODE_WIDTH-1:0] br_tl_a_opcode;
    wire [TL_PARAM_WIDTH-1:0]  br_tl_a_param;
    wire [TL_ADDR_WIDTH-1:0]   br_tl_a_address;
    wire [TL_SIZE_WIDTH-1:0]   br_tl_a_size;
    wire [TL_STRB_WIDTH-1:0]   br_tl_a_mask;
    wire [TL_DATA_WIDTH-1:0]   br_tl_a_data;
    wire [TL_SOURCE_WIDTH-1:0] br_tl_a_source;
    wire                       br_tl_a_corrupt;
    wire                       br_tl_a_ready;

    wire                       br_tl_d_valid;
    wire                       br_tl_d_ready;
    wire [TL_OPCODE_WIDTH-1:0] br_tl_d_opcode;
    wire [TL_PARAM_WIDTH-1:0]  br_tl_d_param;
    wire [TL_SIZE_WIDTH-1:0]   br_tl_d_size;
    wire [TL_SINK_WIDTH-1:0]   br_tl_d_sink;
    wire [TL_SOURCE_WIDTH-1:0] br_tl_d_source;
    wire [TL_DATA_WIDTH-1:0]   br_tl_d_data;
    wire                       br_tl_d_error;
    wire                       wrap_d_ready_tb;

    // ------------------------------------------------------------
    // Wrapper injection signals driven by adapter
    // ------------------------------------------------------------
    wire                       inj_a_valid;
    wire [TL_OPCODE_WIDTH-1:0] inj_a_opcode;
    wire [TL_PARAM_WIDTH-1:0]  inj_a_param;
    wire [TL_ADDR_WIDTH-1:0]   inj_a_address;
    wire [TL_SIZE_WIDTH-1:0]   inj_a_size;
    wire [TL_STRB_WIDTH-1:0]   inj_a_mask;
    wire [TL_DATA_WIDTH-1:0]   inj_a_data;
    wire [TL_SOURCE_WIDTH-1:0] inj_a_source;
    wire                       inj_a_corrupt;

    // ------------------------------------------------------------
    // Wrapper TL tap signals (used to know when the internal A beat
    // was actually accepted: a_valid_tb && a_ready_tb)
    // ------------------------------------------------------------
    wire                       a_ready_tb;
    wire                       a_valid_tb;
    wire [TL_OPCODE_WIDTH-1:0] a_opcode_tb;
    wire [TL_PARAM_WIDTH-1:0]  a_param_tb;
    wire [TL_ADDR_WIDTH-1:0]   a_address_tb;
    wire [TL_SIZE_WIDTH-1:0]   a_size_tb;
    wire [TL_STRB_WIDTH-1:0]   a_mask_tb;
    wire [TL_DATA_WIDTH-1:0]   a_data_tb;
    wire [TL_SOURCE_WIDTH-1:0] a_source_tb;
    wire                       a_corrupt_tb;

    wire                       d_valid_tb;
    wire [TL_OPCODE_WIDTH-1:0] d_opcode_tb;
    wire [TL_PARAM_WIDTH-1:0]  d_param_tb;
    wire [TL_SIZE_WIDTH-1:0]   d_size_tb;
    wire [TL_SINK_WIDTH-1:0]   d_sink_tb;
    wire [TL_SOURCE_WIDTH-1:0] d_source_tb;
    wire [TL_DATA_WIDTH-1:0]   d_data_tb;
    wire                       d_error_tb;

    wire                       use_tb_override = tb_override_a_valid;
    wire                       mux_a_valid        = use_tb_override ? tb_override_a_valid : inj_a_valid;
    wire [TL_OPCODE_WIDTH-1:0] mux_a_opcode       = use_tb_override ? tb_override_a_opcode : inj_a_opcode;
    wire [TL_PARAM_WIDTH-1:0]  mux_a_param        = use_tb_override ? tb_override_a_param : inj_a_param;
    wire [TL_ADDR_WIDTH-1:0]   mux_a_address      = use_tb_override ? tb_override_a_address : inj_a_address;
    wire [TL_SIZE_WIDTH-1:0]   mux_a_size         = use_tb_override ? tb_override_a_size : inj_a_size;
    wire [TL_STRB_WIDTH-1:0]   mux_a_mask         = use_tb_override ? tb_override_a_mask : inj_a_mask;
    wire [TL_DATA_WIDTH-1:0]   mux_a_data         = use_tb_override ? tb_override_a_data : inj_a_data;
    wire [TL_SOURCE_WIDTH-1:0] mux_a_source       = use_tb_override ? tb_override_a_source : inj_a_source;
    wire                       mux_a_corrupt      = use_tb_override ? tb_override_a_corrupt : inj_a_corrupt;
    // ------------------------------------------------------------
    // Instantiate tluh_bridge
    // (bridge reset is active-low)
    // ------------------------------------------------------------
    dma_tluh_bridge u_bridge (
        .clk        (clk),
        .rst_n      (~rst),

        .mem_req    (mem_req),
        .mem_wr_en  (mem_wr_en),
        .mem_addr   (mem_addr),
        .mem_wdata  (mem_wdata),
        .mem_rdata  (mem_rdata),
        .mem_ack    (mem_ack),

        .tl_a_valid   (br_tl_a_valid),
        .tl_a_opcode  (br_tl_a_opcode),
        .tl_a_param   (br_tl_a_param),
        .tl_a_address (br_tl_a_address),
        .tl_a_size    (br_tl_a_size),
        .tl_a_mask    (br_tl_a_mask),
        .tl_a_data    (br_tl_a_data),
        .tl_a_source  (br_tl_a_source),
        .tl_a_corrupt (br_tl_a_corrupt),
        .tl_a_ready   (br_tl_a_ready),

        .tl_d_valid   (br_tl_d_valid),
        .tl_d_ready   (br_tl_d_ready),
        .tl_d_opcode  (br_tl_d_opcode),
        .tl_d_param   (br_tl_d_param),
        .tl_d_size    (br_tl_d_size),
        .tl_d_sink    (br_tl_d_sink),
        .tl_d_source  (br_tl_d_source),
        .tl_d_data    (br_tl_d_data),
        .tl_d_error   (br_tl_d_error)
    );

    // ------------------------------------------------------------
    // Adapter: TL(A) from bridge -> wrapper injection a_*_in
    // - single-entry buffer
    // - br_tl_a_ready is 1 when buffer empty
    // - when buffer full, drive wrapper a_*_in stable
    // - pop buffer when wrapper internal A actually handshakes:
    //     (a_valid_tb && a_ready_tb)
    // ------------------------------------------------------------
    reg                        buf_full;
    reg [TL_OPCODE_WIDTH-1:0]   buf_opcode;
    reg [TL_PARAM_WIDTH-1:0]    buf_param;
    reg [TL_ADDR_WIDTH-1:0]     buf_address;
    reg [TL_SIZE_WIDTH-1:0]     buf_size;
    reg [TL_STRB_WIDTH-1:0]     buf_mask;
    reg [TL_DATA_WIDTH-1:0]     buf_data;
    reg [TL_SOURCE_WIDTH-1:0]   buf_source;
    reg                         buf_corrupt;

    wire adapter_ready = ~buf_full && ~use_tb_override;
    assign br_tl_a_ready = adapter_ready;

    // drive wrapper injection
    assign inj_a_valid   = buf_full;
    assign inj_a_opcode  = buf_opcode;
    assign inj_a_param   = buf_param;
    assign inj_a_address = buf_address;
    assign inj_a_size    = buf_size;
    assign inj_a_mask    = buf_mask;
    assign inj_a_data    = buf_data;
    assign inj_a_source  = buf_source;
    assign inj_a_corrupt = buf_corrupt;

    wire wrapper_a_accepted = (a_valid_tb && a_ready_tb);

    always @(posedge clk) begin
        if (rst) begin
            buf_full    <= 1'b0;
            buf_opcode  <= {TL_OPCODE_WIDTH{1'b0}};
            buf_param   <= {TL_PARAM_WIDTH{1'b0}};
            buf_address <= {TL_ADDR_WIDTH{1'b0}};
            buf_size    <= {TL_SIZE_WIDTH{1'b0}};
            buf_mask    <= {TL_STRB_WIDTH{1'b0}};
            buf_data    <= {TL_DATA_WIDTH{1'b0}};
            buf_source  <= {TL_SOURCE_WIDTH{1'b0}};
            buf_corrupt <= 1'b0;
        end else begin
            // latch from bridge when it handshakes into adapter
            if (~buf_full && br_tl_a_valid) begin
                // handshake occurs because br_tl_a_ready=1 when ~buf_full
                buf_full    <= 1'b1;
                buf_opcode  <= br_tl_a_opcode;
                buf_param   <= br_tl_a_param;
                buf_address <= br_tl_a_address;
                buf_size    <= br_tl_a_size;
                buf_mask    <= br_tl_a_mask;
                buf_data    <= br_tl_a_data;
                buf_source  <= br_tl_a_source;
                buf_corrupt <= br_tl_a_corrupt;
            end

            // pop when wrapper internal A beat actually accepted
            if (buf_full && wrapper_a_accepted) begin
                buf_full <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------
    // Instantiate wrapper (master + slave stays present)
    // D-channel ready at the wrapper is independently controllable from TB.
    // ------------------------------------------------------------
    wire d_ready_mux = tb_drive_d_ready ? tb_d_ready_override : br_tl_d_ready;
    assign wrap_d_ready_tb = d_ready_mux;

    tilelink_uh_wrapper u_wrap (
        .clk            (clk),
        .rst            (rst),

        // injection into master
        .a_valid_in     (mux_a_valid),
        .a_opcode_in    (mux_a_opcode),
        .a_param_in     (mux_a_param),
        .a_address_in   (mux_a_address),
        .a_size_in      (mux_a_size),
        .a_mask_in      (mux_a_mask),
        .a_data_in      (mux_a_data),
        .a_source_in    (mux_a_source),
        .a_corrupt_in   (mux_a_corrupt),

        // slave external memory interface
        .memc_waddr     (memc_waddr),
        .memc_wen       (memc_wen),
        .memc_wdata     (memc_wdata),
        .memc_raddr     (memc_raddr),
        .memc_ren       (memc_ren),
        .memc_rdata     (memc_rdata),
        .memc_write_done(memc_write_done),
        .memc_acc_done  (memc_acc_done),

        // TL taps
        .a_ready_tb     (a_ready_tb),
        .a_valid_tb     (a_valid_tb),
        .a_opcode_tb    (a_opcode_tb),
        .a_param_tb     (a_param_tb),
        .a_address_tb   (a_address_tb),
        .a_size_tb      (a_size_tb),
        .a_mask_tb      (a_mask_tb),
        .a_data_tb      (a_data_tb),
        .a_source_tb    (a_source_tb),
        .a_corrupt_tb   (a_corrupt_tb),

        .d_ready_tb     (wrap_d_ready_tb),
        .d_valid_tb     (d_valid_tb),
        .d_opcode_tb    (d_opcode_tb),
        .d_param_tb     (d_param_tb),
        .d_size_tb      (d_size_tb),
        .d_sink_tb      (d_sink_tb),
        .d_source_tb    (d_source_tb),
        .d_data_tb      (d_data_tb),
        .d_error_tb     (d_error_tb)
    );

    // ------------------------------------------------------------
    // Wire wrapper D taps -> bridge D inputs
    // ------------------------------------------------------------
    assign br_tl_d_valid  = d_valid_tb;
    assign br_tl_d_opcode = d_opcode_tb;
    assign br_tl_d_param  = d_param_tb;
    assign br_tl_d_size   = d_size_tb;
    assign br_tl_d_sink   = d_sink_tb;
    assign br_tl_d_source = d_source_tb;
    assign br_tl_d_data   = d_data_tb;
    assign br_tl_d_error  = d_error_tb;

endmodule

`default_nettype wire
