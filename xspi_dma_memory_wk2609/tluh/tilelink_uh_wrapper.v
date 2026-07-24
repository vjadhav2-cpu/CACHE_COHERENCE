/// @brief Wrapper for TileLink UH Master and Slave Top Modules.
/// This module instantiates both the Master and Slave modules and connects them appropriately.
`timescale 1ns/1ps
`default_nettype none

//`include "./mem_block.v"
`include "./tilelink_uh_master_top.v"
`include "./tilelink_uh_slave_top.v"

module tilelink_uh_wrapper #(
    parameter TL_ADDR_WIDTH     = 64,
    parameter TL_DATA_WIDTH     = 64,
    parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8,
    parameter TL_BEAT_WIDTH     = 8,
    parameter BEAT_LOG2         = $clog2(TL_BEAT_WIDTH),
    parameter TL_SOURCE_WIDTH   = 3,
    parameter TL_SINK_WIDTH     = 3,
    parameter TL_OPCODE_WIDTH   = 3,
    parameter TL_PARAM_WIDTH    = 3,
    parameter TL_SIZE_WIDTH     = 8,
    parameter MAX_BURST_LENGTH  = 16,
    parameter MEM_BASE_ADDR     = 64'h0000_0000_0000_0000,
    parameter DEPTH             = 512,
    parameter FIFO_DEPTH        = 16
)(
    input  wire clk,
    input  wire rst,

    // Inputs to master from external TB/DMA --> should be driven by DMA
    input  wire                            a_valid_in,
    input  wire [TL_OPCODE_WIDTH-1:0]      a_opcode_in,
    input  wire [TL_PARAM_WIDTH-1:0]       a_param_in,
    input  wire [TL_ADDR_WIDTH-1:0]        a_address_in,
    input  wire [TL_SIZE_WIDTH-1:0]        a_size_in,
    input  wire [TL_STRB_WIDTH-1:0]        a_mask_in,
    input  wire [TL_DATA_WIDTH-1:0]        a_data_in,
    input  wire [TL_SOURCE_WIDTH-1:0]      a_source_in,
    input  wire                            a_corrupt_in,



    // Expose memory interface to TB / external mem ctrl --> should be connected to Memory Controller (Slave using D-Channel to respond)
    output wire [TL_ADDR_WIDTH-1:0]  memc_waddr,
    output wire                      memc_wen,
    output wire [TL_DATA_WIDTH-1:0]  memc_wdata,
    output wire [TL_ADDR_WIDTH-1:0]  memc_raddr,
    output wire                      memc_ren,
    input  wire [TL_DATA_WIDTH-1:0]  memc_rdata,
    input  wire                      memc_write_done,
    input  wire                      memc_acc_done,


	// Testbench outputs
	output wire        a_ready_tb,
	output wire        a_valid_tb,
	output wire [TL_OPCODE_WIDTH-1:0] a_opcode_tb,
	output wire [TL_PARAM_WIDTH-1:0]  a_param_tb,
	output wire [TL_ADDR_WIDTH-1:0]   a_address_tb,
	output wire [TL_SIZE_WIDTH-1:0]   a_size_tb,
	output wire [TL_STRB_WIDTH-1:0]   a_mask_tb,
	output wire [TL_DATA_WIDTH-1:0]   a_data_tb,
	output wire [TL_SOURCE_WIDTH-1:0] a_source_tb,
	output wire        a_corrupt_tb,


	// output wire        d_ready_tb,
	input wire         d_ready_tb,
	output wire        d_valid_tb,
	output wire [TL_OPCODE_WIDTH-1:0] d_opcode_tb,
	output wire [TL_PARAM_WIDTH-1:0]  d_param_tb,
	output wire [TL_SIZE_WIDTH-1:0]   d_size_tb,
	output wire [TL_SINK_WIDTH-1:0]   d_sink_tb,
	output wire [TL_SOURCE_WIDTH-1:0] d_source_tb,
	output wire [TL_DATA_WIDTH-1:0]   d_data_tb,
	output wire        d_error_tb
);

    /////////////////////////
    // Internal TL Signals //
    /////////////////////////

    // A Channel
    wire a_ready;
    wire a_valid;
    wire [TL_OPCODE_WIDTH-1:0] a_opcode;
    wire [TL_PARAM_WIDTH-1:0]  a_param;
    wire [TL_ADDR_WIDTH-1:0]   a_address;
    wire [TL_SIZE_WIDTH-1:0]   a_size;
    wire [TL_STRB_WIDTH-1:0]   a_mask;
    wire [TL_DATA_WIDTH-1:0]   a_data;
    wire [TL_SOURCE_WIDTH-1:0] a_source;
    wire a_corrupt;

    // D Channel
    wire d_ready;
    wire d_valid;
    wire [TL_OPCODE_WIDTH-1:0] d_opcode;
    wire [TL_PARAM_WIDTH-1:0]  d_param;
    wire [TL_SIZE_WIDTH-1:0]   d_size;
    wire [TL_SINK_WIDTH-1:0]   d_sink;
    wire [TL_SOURCE_WIDTH-1:0] d_source;
    wire [TL_DATA_WIDTH-1:0]   d_data;
    wire d_error;

    ////////////////////////////////////////////////////
    // Instantiate Master
    ////////////////////////////////////////////////////

    tilelink_uh_master_top #(
        .TL_ADDR_WIDTH(TL_ADDR_WIDTH),
        .TL_DATA_WIDTH(TL_DATA_WIDTH),
        .TL_STRB_WIDTH(TL_STRB_WIDTH),
        .TL_BEAT_WIDTH(TL_BEAT_WIDTH),
        .BEAT_LOG2(BEAT_LOG2),
        .TL_SOURCE_WIDTH(TL_SOURCE_WIDTH),
        .TL_SINK_WIDTH(TL_SINK_WIDTH),
        .TL_OPCODE_WIDTH(TL_OPCODE_WIDTH),
        .TL_PARAM_WIDTH(TL_PARAM_WIDTH),
        .TL_SIZE_WIDTH(TL_SIZE_WIDTH),
        .MAX_BURST_LENGTH(MAX_BURST_LENGTH),
        .MEM_BASE_ADDR(MEM_BASE_ADDR),
        .DEPTH(DEPTH),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) dma_controller (
        .clk(clk),
        .rst(rst),
        .a_valid_in(a_valid_in),
        .a_opcode_in(a_opcode_in),
        .a_param_in(a_param_in),
        .a_address_in(a_address_in),
        .a_size_in(a_size_in),
        .a_mask_in(a_mask_in),
        .a_data_in(a_data_in),
        .a_source_in(a_source_in),
        .a_corrupt_in(a_corrupt_in),

        // A Channel
        .a_ready(a_ready),
        .a_valid(a_valid),
        .a_opcode(a_opcode),
        .a_param(a_param),
        .a_address(a_address),
        .a_size(a_size),
        .a_mask(a_mask),
        .a_data(a_data),
        .a_source(a_source),
        .a_corrupt(a_corrupt),

        // D Channel
        .d_valid(d_valid),
        .d_ready(d_ready),
        .d_opcode(d_opcode),
        .d_param(d_param),
        .d_size(d_size),
        .d_sink(d_sink),
        .d_source(d_source),
        .d_data(d_data),
        .d_error(d_error)
    );

    ////////////////////////////////////////////////////
    // Instantiate Slave
    ////////////////////////////////////////////////////

    tilelink_uh_slave_top #(
        .TL_ADDR_WIDTH(TL_ADDR_WIDTH),
        .TL_DATA_WIDTH(TL_DATA_WIDTH),
        .TL_STRB_WIDTH(TL_STRB_WIDTH),
        .TL_BEAT_WIDTH(TL_BEAT_WIDTH),
        .BEAT_LOG2(BEAT_LOG2),
        .TL_SOURCE_WIDTH(TL_SOURCE_WIDTH),
        .TL_SINK_WIDTH(TL_SINK_WIDTH),
        .TL_OPCODE_WIDTH(TL_OPCODE_WIDTH),
        .TL_PARAM_WIDTH(TL_PARAM_WIDTH),
        .TL_SIZE_WIDTH(TL_SIZE_WIDTH),
        .MEM_BASE_ADDR(MEM_BASE_ADDR),
        .DEPTH(DEPTH),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) memory_controller (
        .clk(clk),
        .rst(rst),

        // A Channel
        .a_ready(a_ready),
        .a_valid(a_valid),
        .a_opcode(a_opcode),
        .a_param(a_param),
        .a_address(a_address),
        .a_size(a_size),
        .a_mask(a_mask),
        .a_data(a_data),
        .a_source(a_source),
        .a_corrupt(a_corrupt),

        // D Channel
        .d_valid(d_valid),
        .d_ready(d_ready),
        .d_opcode(d_opcode),
        .d_param(d_param),
        .d_size(d_size),
        .d_sink(d_sink),
        .d_source(d_source),
        .d_data(d_data),
        .d_error(d_error),

// External memory interface
    .waddr         (memc_waddr),
    .wen           (memc_wen),
    .wdata         (memc_wdata),
    .raddr         (memc_raddr),
    .ren           (memc_ren),
    .rdata         (memc_rdata),
    .mem_write_done(memc_write_done),
    .mem_acc_done  (memc_acc_done)

    );



	// Testbench Outputs
	assign a_ready_tb   = a_ready;
	assign a_valid_tb   = a_valid;
	assign a_opcode_tb  = a_opcode;
	assign a_param_tb   = a_param;
	assign a_address_tb = a_address;
	assign a_size_tb    = a_size;
	assign a_mask_tb    = a_mask;
	assign a_data_tb    = a_data;
	assign a_source_tb  = a_source;
	assign a_corrupt_tb = a_corrupt;

	//assign d_ready_tb   = d_ready;
	assign d_ready      = d_ready_tb;
	assign d_valid_tb   = d_valid;
	assign d_opcode_tb  = d_opcode;
	assign d_param_tb   = d_param;
	assign d_size_tb    = d_size;
	assign d_sink_tb    = d_sink;
	assign d_source_tb  = d_source;
	assign d_data_tb    = d_data;
	assign d_error_tb   = d_error;


endmodule

