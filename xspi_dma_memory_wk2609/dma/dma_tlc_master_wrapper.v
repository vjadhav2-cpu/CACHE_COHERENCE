`timescale 1ns/1ps
`default_nettype none

module dma_tlc_master_wrapper #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 64,
    parameter BURST_MAX  = 16,
    parameter FIFO_DEPTH = 16
) (
    input  wire                   clk,
    input  wire                   rst_n,

    // DMA control / descriptor interface
    input  wire                   dma_start,
    input  wire                   dma_stream_type,
    input  wire [ADDR_WIDTH-1:0]  dma_mem_base_addr,
    input  wire [ADDR_WIDTH-1:0]  dma_peri_base_addr,
    input  wire [15:0]            dma_num_bytes,

    // Peripheral-side DMA bus
    output wire                   peri_req,
    output wire                   peri_wr_en,
    output wire [ADDR_WIDTH-1:0]  peri_addr,
    output wire [DATA_WIDTH-1:0]  peri_wdata,
    input  wire [DATA_WIDTH-1:0]  peri_rdata,
    input  wire                   peri_ack,

    // DMA status
    output wire                   dma_busy,
    output wire                   dma_done,

    // l1_tilelink_adapter-facing interface for tlc_xbar_fabric_top master 2
    output wire                   l1_request_valid,
    output wire [31:0]            l1_request_addr,
    output wire [2:0]             l1_request_type,
    output wire [255:0]           l1_request_data,
    output wire [2:0]             l1_request_permissions,
    input  wire                   l1_request_ready,

    input  wire                   data_to_l1_valid,
    input  wire [255:0]           data_to_l1_data,
    input  wire                   data_to_l1_error,

    input  wire                   probe_req_to_l1_valid,
    input  wire [31:0]            probe_req_to_l1_addr,
    input  wire [2:0]             probe_req_to_l1_permissions,

    output wire                   probe_ack_from_l1_valid,
    output wire [31:0]            probe_ack_from_l1_addr,
    output wire [2:0]             probe_ack_from_l1_permissions,
    output wire [255:0]           probe_ack_from_l1_dirty_data
);

    wire                  mem_req;
    wire                  mem_wr_en;
    wire [ADDR_WIDTH-1:0] mem_addr;
    wire [DATA_WIDTH-1:0] mem_wdata;
    wire [DATA_WIDTH-1:0] mem_rdata;
    wire                  mem_ack;

    dma_flat_full_duplex_top #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .BURST_MAX  (BURST_MAX),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) u_dma (
        .clk            (clk),
        .rst            (~rst_n),
        .start          (dma_start),
        .stream_type    (dma_stream_type),
        .mem_base_addr  (dma_mem_base_addr),
        .peri_base_addr (dma_peri_base_addr),
        .num_bytes      (dma_num_bytes),
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
        .busy           (dma_busy),
        .done           (dma_done)
    );

    dma_coherent_agent #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH)
    ) u_dma_tlc_bridge (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .mem_req                    (mem_req),
        .mem_wr_en                  (mem_wr_en),
        .mem_addr                   (mem_addr),
        .mem_wdata                  (mem_wdata),
        .mem_rdata                  (mem_rdata),
        .mem_ack                    (mem_ack),
        .l1_request_valid           (l1_request_valid),
        .l1_request_addr            (l1_request_addr),
        .l1_request_type            (l1_request_type),
        .l1_request_data            (l1_request_data),
        .l1_request_permissions     (l1_request_permissions),
        .l1_request_ready           (l1_request_ready),
        .data_to_l1_valid           (data_to_l1_valid),
        .data_to_l1_data            (data_to_l1_data),
        .data_to_l1_error           (data_to_l1_error),
        .probe_req_to_l1_valid      (probe_req_to_l1_valid),
        .probe_req_to_l1_addr       (probe_req_to_l1_addr),
        .probe_req_to_l1_permissions(probe_req_to_l1_permissions),
        .probe_ack_from_l1_valid    (probe_ack_from_l1_valid),
        .probe_ack_from_l1_addr     (probe_ack_from_l1_addr),
        .probe_ack_from_l1_permissions(probe_ack_from_l1_permissions),
        .probe_ack_from_l1_dirty_data(probe_ack_from_l1_dirty_data)
    );

endmodule

`default_nettype wire
