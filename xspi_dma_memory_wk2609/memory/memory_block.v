`timescale 1ns/1ps

module memory_block #(
    parameter TL_DATA_WIDTH = 64,
    parameter MEM_BASE_ADDR = 64'h0000_0000_0000_0000,
    parameter DEPTH = 512,
    parameter TL_ADDR_WIDTH = $clog2(DEPTH),
    parameter LATENCY = 3
)(
    input  wire                        clk,
    input  wire                        rst,
    input  wire [TL_ADDR_WIDTH-1:0]    waddr,
    input  wire [TL_ADDR_WIDTH-1:0]    raddr,
    input  wire [TL_DATA_WIDTH-1:0]    wdata,
    output wire [TL_DATA_WIDTH-1:0]    rdata,
    input  wire                        ren,
    input  wire                        wen,
    output wire                        mem_write_done,
    output wire                        mem_acc_done
);

    reg [TL_DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [TL_DATA_WIDTH-1:0] rdata_reg;
    assign rdata = rdata_reg;

    reg mem_write_done_reg, mem_acc_done_reg;
    assign mem_write_done = mem_write_done_reg;
    assign mem_acc_done   = mem_acc_done_reg;

    reg [2:0] wr_counter, rd_counter;
    reg       wr_active, rd_active;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rdata_reg <= 0;
            mem_write_done_reg <= 0;
            mem_acc_done_reg   <= 0;
            wr_counter <= 0; rd_counter <= 0;
            wr_active <= 0; rd_active <= 0;
        end else begin
            mem_write_done_reg <= 0;
            mem_acc_done_reg   <= 0;

            // WRITE latency
            if (!wr_active && wen) begin
                wr_active <= 1;
                mem[waddr[8:0]] <= wdata;   // ✅ fixed indexing
                $display("[MEM-WRITE] Time=%0t waddr[8:0]=0x%h wdata=0x%h", $time, waddr[8:0], wdata);
                wr_counter <= 1;
            end else if (wr_active) begin
                if (wr_counter < LATENCY) wr_counter <= wr_counter + 1;
                else begin
                    mem_write_done_reg <= 1;
                    wr_active <= 0; wr_counter <= 0;
                end
            end

            // READ latency
            if (!rd_active && ren) begin
                $display("[MEM-START-RD] Time=%0t Starting read raddr=0x%h ren=%b", $time, raddr[8:0], ren);
                rd_active <= 1;
                rd_counter <= 1;
            end else if (rd_active) begin
                if (rd_counter < LATENCY) rd_counter <= rd_counter + 1;
                else begin
                    rdata_reg <= mem[raddr[8:0]];  // ✅ fixed indexing
                    $display("[MEM-READ-DONE] Time=%0t raddr[8:0]=0x%h mem[raddr]=0x%h mem_acc_done=1", $time, raddr[8:0], mem[raddr[8:0]]);
                    mem_acc_done_reg <= 1;
                    rd_active <= 0; 
                    rd_counter <= 0;
                end
            end
        end
    end
endmodule

