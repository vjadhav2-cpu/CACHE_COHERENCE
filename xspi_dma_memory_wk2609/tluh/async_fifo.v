// async_fifo.v - A simple dual-clock asynchronous FIFO
`timescale 1ns/1ps

module async_fifo #(
  parameter DATA_WIDTH = 8,
  parameter DEPTH      = 8,
  parameter PTR_WIDTH = $clog2(DEPTH),
  parameter ADDR_WIDTH = $clog2(DEPTH)
)(
  input   wire            wr_clk,
  input   wire           rd_clk,
  input   wire            reset,
  input   wire            wr_en,
  input  wire [DATA_WIDTH-1:0] wr_data,
  output   reg               full,
  input    wire              rd_en,
  output wire [DATA_WIDTH-1:0] rd_data,
  output   reg            empty
);

  // FIFO memory
  reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

  // Write/read pointers (binary)
  reg [ADDR_WIDTH:0] wr_ptr, rd_ptr;

  wire [ADDR_WIDTH:0] wr_ptr_nxt, rd_ptr_nxt;
 
  // Gray-coded pointers (REGS for proper reset!)
  reg [ADDR_WIDTH:0] wr_ptr_gray, rd_ptr_gray;
 
  // Initialize memory to zeros
  integer i;
  initial begin
    for (i = 0; i < DEPTH; i = i + 1) begin
      mem[i] = {DATA_WIDTH{1'b0}};
    end
  end

  // Synchronized pointers
  reg [ADDR_WIDTH:0] wr_ptr_gray_rd_sync1, wr_ptr_gray_rd_sync2;
  reg [ADDR_WIDTH:0] rd_ptr_gray_wr_sync1, rd_ptr_gray_wr_sync2;
 
  // Write pointer (wr_clk domain)
  always @(posedge wr_clk or posedge reset) begin
    if (reset) begin
      wr_ptr <= 0;
      wr_ptr_gray <= 0;  // RESET Gray pointer too!
    end else if(wr_en) begin
      wr_ptr <= wr_ptr_nxt;// + 1;
      wr_ptr_gray <= wr_ptr_nxt ^ (wr_ptr_nxt >> 1);  // Compute Gray from next pointer
      mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
      if (!full) begin
        $display("[ASYNC_FIFO] WRITE: Time=%0t wr_en=%b wr_ptr=%0d->%0d full=%b data=0x%h",
                 $time, wr_en, wr_ptr, wr_ptr_nxt, full, wr_data);
      end else begin
        $display("[ASYNC_FIFO] WRITE-FULL: Time=%0t wr_en=%b but FIFO is full! (overwriting)", $time, wr_en);
      end
    end
  end
 
  ///Write Gray Pointer and next pointer Conversion///
  //assign wr_ptr_nxt = wr_ptr +  (wr_en & !full);
  assign wr_ptr_nxt = wr_ptr + {{(PTR_WIDTH-1){1'b0}}, wr_en};
  // wr_ptr_gray now computed in always block
 
 
  // Read pointer (rd_clk domain)
  always @(posedge rd_clk or posedge reset) begin
    if (reset) begin
      rd_ptr  <= 0;
      rd_ptr_gray <= 0;  // RESET Gray pointer too!
    end else if(rd_en && !empty) begin  // FIXED: Don't read when empty
      rd_ptr  <= rd_ptr_nxt;// + 1;
      rd_ptr_gray <= rd_ptr_nxt ^ (rd_ptr_nxt >> 1);  // Compute Gray from next pointer
      $display("[ASYNC_FIFO] READ: Time=%0t rd_en=%b rd_ptr=%0d->%0d empty=%b data=0x%h", 
               $time, rd_en, rd_ptr, rd_ptr_nxt, empty, mem[rd_ptr[ADDR_WIDTH-1:0]]);
    end
  end
 
  assign rd_data = mem[rd_ptr[ADDR_WIDTH-1:0]]; // Combinatorial read
 
 ///Read Gray Pointer and next pointer Conversion///
  //assign rd_ptr_nxt = rd_ptr +  (rd_en & !empty);
  
  assign rd_ptr_nxt = rd_ptr + {{(PTR_WIDTH-1){1'b0}}, rd_en};
  // ------------------------
  // Synchronizers
  // ------------------------
  always @(posedge rd_clk or posedge reset) begin
    if (reset) begin
      wr_ptr_gray_rd_sync1 <= 0;
      wr_ptr_gray_rd_sync2 <= 0;
    end else begin
      wr_ptr_gray_rd_sync1 <= wr_ptr_gray;
      wr_ptr_gray_rd_sync2 <= wr_ptr_gray_rd_sync1;
    end
  end

  always @(posedge wr_clk or posedge reset) begin
    if (reset) begin
      rd_ptr_gray_wr_sync1 <= 0;
      rd_ptr_gray_wr_sync2 <= 0;
    end else begin
      rd_ptr_gray_wr_sync1 <= rd_ptr_gray;
      rd_ptr_gray_wr_sync2 <= rd_ptr_gray_wr_sync1;
    end
  end

  // ------------------------
  // Full / Empty conditions
  // ------------------------
   always @(posedge wr_clk or posedge reset) begin
    if (reset)
      full <= 0;
    else
      full  <= (wr_ptr_gray == {~rd_ptr_gray_wr_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                  rd_ptr_gray_wr_sync2[ADDR_WIDTH-2:0]});
   end
 
   always @(posedge rd_clk or posedge reset) begin
    if (reset)
       empty <= 1;
     else begin   
       empty <= (wr_ptr_gray_rd_sync2 == rd_ptr_gray);
       if ((wr_ptr_gray_rd_sync2 == rd_ptr_gray) != empty) begin
         $display("[ASYNC_FIFO] EMPTY_CHG: Time=%0t empty=%b->%b rd_ptr_gray=%0d wr_ptr_gray_rd_sync2=%0d", 
                  $time, empty, (wr_ptr_gray_rd_sync2 == rd_ptr_gray), rd_ptr_gray, wr_ptr_gray_rd_sync2);
       end
     end
    end

endmodule
