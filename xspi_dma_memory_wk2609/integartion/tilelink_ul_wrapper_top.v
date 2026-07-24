/*******************************************************************************************************************************************

This top-level module acts as a wrapper for the TileLink UltraLite (TL-UL) interface.
It connects the TileLink UltraLite interface to a TileLink UltraLite master and slave.

*******************************************************************************************************************************************/
`timescale 1ns/1ps
`default_nettype none
/////this module has tlul wrapper and slave of tlul is connected to xspi
module tilelink_ul_wrapper_top #( 
    parameter TL_ADDR_WIDTH     = 64,
    parameter TL_DATA_WIDTH     = 64,
    parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8,
    parameter TL_SOURCE_WIDTH   = 3,
    parameter TL_SINK_WIDTH     = 3,
    parameter TL_OPCODE_WIDTH   = 3,
    parameter TL_PARAM_WIDTH    = 3,
    parameter TL_SIZE_WIDTH     = 8,
    
    parameter MEM_BASE_ADDR = 64'h0000_0000_0000_0000, // Base address for memory
    parameter DEPTH           = 512                      // Memory depth (number of entries)    
)(
    input  wire                              clk,
    input  wire                              rst,

    // Inputs to drive the master from testbench
    input  wire                              a_valid_in,
    input  wire [TL_OPCODE_WIDTH-1:0]        a_opcode_in,
    input  wire [TL_PARAM_WIDTH-1:0]         a_param_in,
    input  wire [TL_ADDR_WIDTH-1:0]          a_address_in,
    input  wire [TL_SIZE_WIDTH-1:0]          a_size_in,
    input  wire [TL_STRB_WIDTH-1:0]          a_mask_in,
    input  wire [TL_DATA_WIDTH-1:0]          a_data_in,
    input  wire [TL_SOURCE_WIDTH-1:0]        a_source_in,

    // Outputs for testbench
    // Make all the following output wires 
    output wire a_valid_tb,
    output wire [TL_OPCODE_WIDTH-1:0] a_opcode_tb,
    output wire [TL_PARAM_WIDTH-1:0]  a_param_tb,
    output wire [TL_ADDR_WIDTH-1:0]   a_address_tb,
    output wire [TL_SIZE_WIDTH-1:0]   a_size_tb,
    output wire [TL_STRB_WIDTH-1:0]   a_mask_tb,
    output wire [TL_DATA_WIDTH-1:0]   a_data_tb,
    output wire [TL_SOURCE_WIDTH-1:0] a_source_tb,
    output wire a_ready_tb,

    output wire d_valid_tb,
    input  wire d_ready_tb,
    output wire [TL_OPCODE_WIDTH-1:0] d_opcode_tb,
    output wire [TL_PARAM_WIDTH-1:0]  d_param_tb,
    output wire [TL_SIZE_WIDTH-1:0]   d_size_tb,
    output wire [TL_SINK_WIDTH-1:0]   d_sink_tb,
    output wire [TL_SOURCE_WIDTH-1:0] d_source_tb,
    output wire [TL_DATA_WIDTH-1:0]   d_data_tb,
    output wire d_error_tb,
    output wire         spi_wr_evt_valid_tb,
    output wire [47:0]  spi_wr_evt_addr_tb,
    output wire [63:0]  spi_wr_evt_data_tb,
    output wire         spi_rd_req_valid_tb,
    output wire [47:0]  spi_rd_req_addr_tb,
    input  wire         spi_rd_rsp_valid_tb,
    input  wire [63:0]  spi_rd_rsp_data_tb
    
);

    /////////////////////////////////////////
    //////// Localparams for opcodes ////////
    /////////////////////////////////////////

    localparam PUT_FULL_DATA_A     = 3'd0;
    localparam PUT_PARTIAL_DATA_A  = 3'd1;
    localparam ARITHMETIC_DATA_A   = 3'd2;
    localparam LOGICAL_DATA_A      = 3'd3;
    localparam GET_A               = 3'd4;
    localparam INTENT_A            = 3'd5;
    localparam ACQUIRE_BLOCK_A     = 3'd6;
    localparam ACQUIRE_PERM_A      = 3'd7;

    localparam ACCESS_ACK_D        = 3'd0;
    localparam ACCESS_ACK_DATA_D   = 3'd1;
    localparam HINT_ACK_D          = 3'd2;
    localparam GRANT_D             = 3'd4;
    localparam GRANT_DATA_D        = 3'd5;
    localparam RELEASE_ACK_D       = 3'd6;

    localparam IDLE     = 2'd0;
    localparam REQUEST  = 2'd1;
    localparam RESPONSE = 2'd2;
    localparam CLEANUP  = 2'd3;


    // Internal wires for A and D channels between master and slave
    wire a_ready;
    wire a_valid;
    wire [TL_OPCODE_WIDTH-1:0] a_opcode;
    wire [TL_PARAM_WIDTH-1:0]  a_param;
    wire [TL_ADDR_WIDTH-1:0]   a_address;
    wire [TL_SIZE_WIDTH-1:0]   a_size;
    wire [TL_STRB_WIDTH-1:0]   a_mask;
    wire [TL_DATA_WIDTH-1:0]   a_data;
    wire [TL_SOURCE_WIDTH-1:0] a_source;

    wire d_valid;
    wire d_ready;
    wire [TL_OPCODE_WIDTH-1:0] d_opcode;
    wire [TL_PARAM_WIDTH-1:0]  d_param;
    wire [TL_SIZE_WIDTH-1:0]   d_size;
    wire [TL_SINK_WIDTH-1:0]   d_sink;
    wire [TL_SOURCE_WIDTH-1:0] d_source;
    wire [TL_DATA_WIDTH-1:0]   d_data;
    wire                       d_error;



    // Assign outputs to internal wires for testbench visibility
    assign a_valid_tb    = a_valid;
    assign a_opcode_tb   = a_opcode;
    assign a_param_tb    = a_param;
    assign a_address_tb  = a_address;
    assign a_size_tb     = a_size;
    assign a_mask_tb     = a_mask;
    assign a_data_tb     = a_data;
    assign a_source_tb   = a_source;
    assign a_ready_tb    = a_ready;

    assign d_valid_tb    = d_valid;
    assign d_opcode_tb   = d_opcode;
    assign d_param_tb    = d_param;
    assign d_size_tb     = d_size;
    assign d_sink_tb     = d_sink;
    assign d_source_tb   = d_source;
    assign d_data_tb     = d_data;
    assign d_error_tb    = d_error;

	// Instantiate the TileLink Uncached Lightweight master

    tilelink_ul_master_top #(
        .TL_ADDR_WIDTH     (TL_ADDR_WIDTH),
        .TL_DATA_WIDTH     (TL_DATA_WIDTH),
        .TL_STRB_WIDTH     (TL_STRB_WIDTH),
        .TL_SOURCE_WIDTH   (TL_SOURCE_WIDTH),
        .TL_SINK_WIDTH     (TL_SINK_WIDTH),
        .TL_OPCODE_WIDTH   (TL_OPCODE_WIDTH),
        .TL_PARAM_WIDTH    (TL_PARAM_WIDTH),
        .TL_SIZE_WIDTH     (TL_SIZE_WIDTH),
        .PUT_FULL_DATA_A   (PUT_FULL_DATA_A),
        .PUT_PARTIAL_DATA_A(PUT_PARTIAL_DATA_A),
        .ARITHMETIC_DATA_A (ARITHMETIC_DATA_A),
        .LOGICAL_DATA_A    (LOGICAL_DATA_A),
        .GET_A             (GET_A),
        .INTENT_A          (INTENT_A),
        .ACQUIRE_BLOCK_A   (ACQUIRE_BLOCK_A),
        .ACQUIRE_PERM_A    (ACQUIRE_PERM_A),
        .ACCESS_ACK_D      (ACCESS_ACK_D),
        .ACCESS_ACK_DATA_D (ACCESS_ACK_DATA_D),
        .HINT_ACK_D        (HINT_ACK_D),
        .GRANT_D           (GRANT_D),
        .GRANT_DATA_D      (GRANT_DATA_D),
        .RELEASE_ACK_D     (RELEASE_ACK_D),
        .REQUEST           (REQUEST),
        .RESPONSE          (RESPONSE),
        .CLEANUP           (CLEANUP),
        .IDLE              (IDLE)
    ) master_inst (
        .clk           (clk),
        .rst           (rst),

        .a_valid_in    (a_valid_in),
        .a_opcode_in   (a_opcode_in),
        .a_param_in    (a_param_in),
        .a_address_in  (a_address_in),
        .a_size_in     (a_size_in),
        .a_mask_in     (a_mask_in),
        .a_data_in     (a_data_in),
        .a_source_in   (a_source_in),

        .a_ready       (a_ready),
        .a_valid       (a_valid),
        .a_opcode      (a_opcode),
        .a_param       (a_param),
        .a_address     (a_address),
        .a_size        (a_size),
        .a_mask        (a_mask),
        .a_data        (a_data),
        .a_source      (a_source),

        .d_valid       (d_valid),
        .d_ready       (d_ready),
        .d_opcode      (d_opcode),
        .d_param       (d_param),
        .d_size        (d_size),
        .d_sink        (d_sink),
        .d_source      (d_source),
        .d_data        (d_data),
        .d_error       (d_error)
    );

	// Instantiate the TileLink Uncached Lightweight slave

    tilelink_ul_slave_top #(
        .TL_ADDR_WIDTH      (TL_ADDR_WIDTH),
        .TL_DATA_WIDTH      (TL_DATA_WIDTH),
        .TL_STRB_WIDTH      (TL_STRB_WIDTH),
        .TL_SOURCE_WIDTH    (TL_SOURCE_WIDTH),
        .TL_SINK_WIDTH      (TL_SINK_WIDTH),
        .TL_OPCODE_WIDTH    (TL_OPCODE_WIDTH),
        .TL_PARAM_WIDTH     (TL_PARAM_WIDTH),
        .TL_SIZE_WIDTH      (TL_SIZE_WIDTH),
        .PUT_FULL_DATA_A    (PUT_FULL_DATA_A),
        .PUT_PARTIAL_DATA_A (PUT_PARTIAL_DATA_A),
        .ARITHMETIC_DATA_A  (ARITHMETIC_DATA_A),
        .LOGICAL_DATA_A     (LOGICAL_DATA_A),
        .GET_A              (GET_A),
        .INTENT_A           (INTENT_A),
        .ACQUIRE_BLOCK_A    (ACQUIRE_BLOCK_A),
        .ACQUIRE_PERM_A     (ACQUIRE_PERM_A),
        .ACCESS_ACK_D       (ACCESS_ACK_D),
        .ACCESS_ACK_DATA_D  (ACCESS_ACK_DATA_D),
        .HINT_ACK_D         (HINT_ACK_D),
        .GRANT_D            (GRANT_D),
        .GRANT_DATA_D       (GRANT_DATA_D),
        .RELEASE_ACK_D      (RELEASE_ACK_D),
        .REQUEST            (REQUEST),
        .RESPONSE           (RESPONSE),

        
        .MEM_BASE_ADDR(MEM_BASE_ADDR),
        .DEPTH(DEPTH),
        .USE_EXTERNAL_SPI_MEM(1)
      )slave_inst (
        .clk        (clk),
        .rst        (rst),

        .a_ready    (a_ready),
        .a_valid    (a_valid),
        .a_opcode   (a_opcode),
        .a_param    (a_param),
        .a_address  (a_address),
        .a_size     (a_size),
        .a_mask     (a_mask),
        .a_data     (a_data),
        .a_source   (a_source),

        .d_valid    (d_valid),
        .d_ready    (d_ready & d_ready_tb),
        .d_opcode   (d_opcode),
        .d_param    (d_param),
        .d_size     (d_size),
        .d_sink     (d_sink),
        .d_source   (d_source),
        .d_data     (d_data),
        .d_error    (d_error),
        .spi_wr_evt_valid_tb(spi_wr_evt_valid_tb),
        .spi_wr_evt_addr_tb(spi_wr_evt_addr_tb),
        .spi_wr_evt_data_tb(spi_wr_evt_data_tb),
        .spi_rd_req_valid_tb(spi_rd_req_valid_tb),
        .spi_rd_req_addr_tb(spi_rd_req_addr_tb),
        .spi_rd_rsp_valid_tb(spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data_tb(spi_rd_rsp_data_tb)
    );
	
endmodule


/*******************************************************************************************************************************************

This top-level module sets up signals for the slave instance of the TileLink UL protocol. It will work in the low speed domain with 
peripherals like GPIO and Flash.

*******************************************************************************************************************************************/ 
`timescale 1ns/1ps
`default_nettype none


module tilelink_ul_slave_top #( 

	//////////////////////////////////////////////////////////////////
	////////////////////// Core interface widths /////////////////////
	//////////////////////////////////////////////////////////////////
	
	parameter TL_ADDR_WIDTH     = 64,            		// Address width
	parameter TL_DATA_WIDTH     = 64,            		// Data width
	parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8, 	// Byte mask width/Byte strobe. Each bit represents one byte of the data.

	
	//////////////////////////////////////////////////////////////////
	//////////////////// TileLink metadata widths ////////////////////
	//////////////////////////////////////////////////////////////////
	
	// Check again! Bigger the source width, more the number of active transactions.
	parameter TL_SOURCE_WIDTH   = 3,			 // Tags each request with a unique ID. The same ID must appear in the corresponding response.
	parameter TL_SINK_WIDTH     = 3,			 // Tags each response with an ID that matches that of the request.
	parameter TL_OPCODE_WIDTH   = 3,			 // Opcode width for instructions
	parameter TL_PARAM_WIDTH    = 3,             // Currently reserved for future performance hints and must be 0 
	parameter TL_SIZE_WIDTH     = 8,             // Width of size field, value of which determines data beat size in bytes as 2^size.
	

	// Define opcodes for channels
	// A Channel Opcodes
	parameter PUT_FULL_DATA_A     = 3'd0,
	parameter PUT_PARTIAL_DATA_A  = 3'd1,
	parameter ARITHMETIC_DATA_A   = 3'd2,
	parameter LOGICAL_DATA_A      = 3'd3,
	parameter GET_A               = 3'd4,
	parameter INTENT_A            = 3'd5,
	parameter ACQUIRE_BLOCK_A     = 3'd6,
	parameter ACQUIRE_PERM_A      = 3'd7,

	// D Channel Opcodes
	parameter ACCESS_ACK_D      = 3'd0,
	parameter ACCESS_ACK_DATA_D = 3'd1,
	parameter HINT_ACK_D        = 3'd2,
	parameter GRANT_D           = 3'd4,
	parameter GRANT_DATA_D      = 3'd5,
	parameter RELEASE_ACK_D     = 3'd6,
	
	// Slave FSM States
	parameter REQUEST 			 = 2'd1,
	parameter RESPONSE   		 = 2'd2,

	// Memory parameters
	parameter MEM_BASE_ADDR 	  = 64'h0000_0000_0000_0000, // Base address for memory
	parameter DEPTH           = 512,                     // Memory depth (number of entries)
    parameter USE_EXTERNAL_SPI_MEM = 1
)(
	input  wire                              clk,
	input  wire                              rst,

	// A Channel: Received from MASTER
	output wire                              a_ready,		// Slave sends a_ready to Master to indicate that it is ready to accept data.
	input  wire                              a_valid, 		// Asserted to indicate valid instruction
	input  wire [TL_OPCODE_WIDTH-1:0]        a_opcode,		// Opcode for instruction
	input  wire [TL_PARAM_WIDTH-1:0]         a_param,		// Reserved, always 0.
	input  wire [TL_ADDR_WIDTH-1:0]          a_address,	    // Address 
	input  wire [TL_SIZE_WIDTH-1:0]          a_size,		// Width of full data sent in one go = 2^size. For TLUL, size = Data Width of Channel.
	input  wire [TL_STRB_WIDTH-1:0]          a_mask,		// Bit masking
	input  wire [TL_DATA_WIDTH-1:0]          a_data,		// Incoming data
	input  wire [TL_SOURCE_WIDTH-1:0]        a_source,		// Transaction ID

	// D Channel Sent to MASTER
	output reg 				d_valid,
	input  wire                             d_ready, 		// Master ends d_ready to Slave to indicate that it is ready to accept data.
	output reg [TL_OPCODE_WIDTH-1:0]        d_opcode,
	output reg [TL_PARAM_WIDTH-1:0]         d_param,
	output reg [TL_SIZE_WIDTH-1:0]          d_size,
	output reg [TL_SINK_WIDTH-1:0]          d_sink,
	output reg [TL_SOURCE_WIDTH-1:0]        d_source,	
	output reg [TL_DATA_WIDTH-1:0]          d_data,
	output reg                              d_error,
    output wire                             spi_wr_evt_valid_tb,
    output wire [47:0]                      spi_wr_evt_addr_tb,
    output wire [63:0]                      spi_wr_evt_data_tb,
    output wire                             spi_rd_req_valid_tb,
    output wire [47:0]                      spi_rd_req_addr_tb,
    input  wire                             spi_rd_rsp_valid_tb,
    input  wire [63:0]                      spi_rd_rsp_data_tb
);

	// State variable for slave FSM
	reg [1:0] slave_state;

	// State flags
	wire in_idle;
	wire in_request;
	wire in_response;
	wire in_cleanup;
	
	// Memory Flags
	
	reg  [TL_ADDR_WIDTH-1:0] waddr;
	reg              	   wen,ren;
	reg  [TL_DATA_WIDTH-1:0] wdata;
	reg  [TL_ADDR_WIDTH-1:0] raddr;
	wire [TL_DATA_WIDTH-1:0] rdata;
	
	
	// For loop variables
	integer i,j;

	
	// Registers for A Channel (Slave side input)
	reg                             a_ready_reg;
	reg                             a_valid_reg;
	reg [TL_OPCODE_WIDTH-1:0]       a_opcode_reg;
	reg [TL_PARAM_WIDTH-1:0]        a_param_reg;
	reg [TL_ADDR_WIDTH-1:0]         a_address_reg;
	reg [TL_SIZE_WIDTH-1:0]         a_size_reg;
	reg [TL_STRB_WIDTH-1:0]         a_mask_reg;
	reg [TL_DATA_WIDTH-1:0]         a_data_reg;
	reg [TL_SOURCE_WIDTH-1:0]       a_source_reg;
	
	wire check;
	


	
	reg response_pending;


	wire mem_acc_done;
	wire mem_write_done;

	reg response_done;
	
	
	// Wait signal
	
	reg wait_flag;
	reg mem_enable;
	assign check = a_valid | wait_flag;

	// Enabled during write or read operations
	wire tlul_spi_bridge_enable;
	assign tlul_spi_bridge_enable = ren | wen;

	// For Interconnect

	wire bridge_output_valid;

	assign mem_acc_done = bridge_output_valid; // Memory access done signal from the memory block
	assign mem_write_done = bridge_output_valid; // Memory write done signal from the memory block

	/////////////////////////////////////////////////////////////
	//////////// 				  FSM BLOCK    	 	 ////////////
	/////////////////////////////////////////////////////////////	

	// Assign flags based on state
	assign in_request  = (slave_state == REQUEST);
	assign in_response = (slave_state == RESPONSE);
	
	// assign response_done = d_ready & response_pending;
	
    // assign raddr = in_request      ? a_address :
    //                wait_flag   ? a_address_reg :
    //                             64'd0;
       
	
	// State machine

	always @(posedge clk or posedge rst) begin
		if (rst) begin
			slave_state <= REQUEST;
		end
		else begin
			case (slave_state)

				REQUEST: begin
				// wait_flag <= 0;
				if (a_valid)
					slave_state <= RESPONSE;
				else
					slave_state <= REQUEST;
				end

				RESPONSE: begin
				    if (d_ready & response_pending) begin
				        slave_state <= REQUEST; 
						// wait_flag <= 0;
				    end
                                    else begin
						slave_state <= RESPONSE;
						// wait_flag <= 1;
					end
				end

				default: slave_state <= REQUEST;

			endcase
		end
	end 
					// else if (!d_ready & d_ready_reg) begin // Means that response is done



	
	/////////////////////////////////////////////////////////////
	//////////// 			   DATAPATH        	 	 ////////////
	/////////////////////////////////////////////////////////////		
	
	assign a_ready =  in_request;

	
	
	// Slave response
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset logic for any control registers or outputs
            wen   <= 1'b0;
            waddr <= {TL_ADDR_WIDTH{1'b0}};
            wdata <= {TL_DATA_WIDTH{1'b0}};
			response_pending <= 0;
			mem_enable <= 0;
			response_done <= 0;
			ren   <= 1'b0;
            // Add others as needed
        end else if (in_response) begin
            case (a_opcode_reg)
                PUT_FULL_DATA_A: begin
                    // Full memory write

					// Raise write enable and set address/data only for 1 cycle.
					if (!mem_enable & !wen) begin
						wen   <= 1'b1;
						waddr <= a_address_reg;
						wdata <= a_data_reg;
						mem_enable <= 1'b1; // Enable memory write
						response_pending <= 1'b0; // Reset response pending flag
					end else begin
						wen   <= 1'b0;
						waddr <= {TL_ADDR_WIDTH{1'b0}};
						wdata <= {TL_DATA_WIDTH{1'b0}};
						mem_enable <= mem_enable;
					end
            
                    // Latch response_pending when mem_write_done is high
                                        if (mem_write_done && !response_pending) begin
                                               response_pending <= 1'b1;

                                               d_valid   <= 1'b1;
                        		       d_opcode  <= ACCESS_ACK_D; 
                        		       d_param   <= {TL_PARAM_WIDTH{1'b0}};
                                               d_size    <= a_size_reg;
                                               d_sink    <= {TL_SINK_WIDTH{1'b0}};
                                               d_source  <= a_source_reg;
                                               d_data    <= 0; 
                                               d_error   <= 1'b0;
                                        end else if (response_pending) begin
                                            if(!d_ready) begin
							d_valid   <= 1'b1;
							d_opcode  <= ACCESS_ACK_D; 
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= a_size_reg;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= a_source_reg;
							d_data    <= 0; 
							d_error   <= 1'b0;
						end
						else begin
                                                        response_pending <= 1'b0;
							mem_enable <= 0;
							d_valid   <= 1'b0;
							d_opcode  <= 0;
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= 0;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= 0;
							d_data    <= {TL_DATA_WIDTH{1'b0}};
							d_error   <= 0;							
                        end						
					end
					else begin
						d_valid   <= 1'b0;
						d_opcode  <= 0;
						d_param   <= {TL_PARAM_WIDTH{1'b0}};
						d_size    <= 0;
						d_sink    <= {TL_SINK_WIDTH{1'b0}};
						d_source  <= 0;
						d_data    <= {TL_DATA_WIDTH{1'b0}};
						d_error   <= 0;
					end
					
					ren <= 1'b0;
                end

                PUT_PARTIAL_DATA_A: begin
                    // Partial write example (you may use mask  to gate bytes)

					// Raise write enable and set address/data only for 1 cycle.
					if (!mem_enable & !wen) begin
						wen   <= 1'b1;
						waddr <= a_address_reg;
						
                        for (i = 0; i < TL_STRB_WIDTH; i=i+1)
                            for (j = 0; j < TL_STRB_WIDTH; j=j+1) 
                                wdata[j + i*8] <= a_data_reg [j + i*8] & a_mask_reg [i];


						mem_enable <= 1'b1; // Enable memory write
						response_pending <= 1'b0; // Reset response pending flag
					end else begin
						wen   <= 1'b0;
						waddr <= {TL_ADDR_WIDTH{1'b0}};
						wdata <= {TL_DATA_WIDTH{1'b0}};
						mem_enable <= mem_enable;
					end
            
                    // Latch response_pending when mem_write_done is high
                    if (mem_write_done && !response_pending) begin
                        response_pending <= 1'b1;

                        d_valid   <= 1'b1;
                        d_opcode  <= ACCESS_ACK_D; 
                        d_param   <= {TL_PARAM_WIDTH{1'b0}};
                        d_size    <= a_size_reg;
                        d_sink    <= {TL_SINK_WIDTH{1'b0}};
                        d_source  <= a_source_reg;
                        d_data    <= 0; 
                        d_error   <= 1'b0;
                    end else if (response_pending) begin
                        if(!d_ready) begin
							d_valid   <= 1'b1;
							d_opcode  <= ACCESS_ACK_D; 
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= a_size_reg;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= a_source_reg;
							d_data    <= 0; 
							d_error   <= 1'b0;
						end
						else begin
                            response_pending <= 1'b0;
							mem_enable <= 0;
							d_valid   <= 1'b0;
							d_opcode  <= 0;
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= 0;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= 0;
							d_data    <= {TL_DATA_WIDTH{1'b0}};
							d_error   <= 0;							
                        end						
					end
					else begin
						d_valid   <= 1'b0;
						d_opcode  <= 0;
						d_param   <= {TL_PARAM_WIDTH{1'b0}};
						d_size    <= 0;
						d_sink    <= {TL_SINK_WIDTH{1'b0}};
						d_source  <= 0;
						d_data    <= {TL_DATA_WIDTH{1'b0}};
						d_error   <= 0;
					end

					ren <= 1'b0;

                end

                GET_A: begin
                    // Read only - no write enable
					// Raise write enable and set address/data only for 1 cycle.

					if (!mem_enable & !ren) begin
						ren   <= 1'b1;
						raddr <= a_address_reg;
						wdata <= {TL_DATA_WIDTH{1'b0}}; // No write data for read
						mem_enable <= 1'b1; 			// Enable memory write
						response_pending <= 1'b0; 		// Reset response pending flag
					end else begin
						ren   <= 1'b0;
						raddr <= {TL_ADDR_WIDTH{1'b0}};
						wdata <= {TL_DATA_WIDTH{1'b0}};
						mem_enable <= mem_enable;
					end
            
                    // Latch response_pending when mem_write_done is high
                    if (mem_acc_done && !response_pending) begin
                        response_pending <= 1'b1;

                        d_valid   <= 1'b1;
                        d_opcode  <= ACCESS_ACK_DATA_D; 
                        d_param   <= {TL_PARAM_WIDTH{1'b0}};
                        d_size    <= a_size_reg;
                        d_sink    <= {TL_SINK_WIDTH{1'b0}};
                        d_source  <= a_source_reg;
                        d_data    <= rdata; 
                        d_error   <= 1'b0;
                    end else if (response_pending) begin
                        if(!d_ready) begin
							d_valid   <= 1'b1;
							d_opcode  <= ACCESS_ACK_DATA_D; 
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= a_size_reg;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= a_source_reg;
							d_data    <= d_data; 
							d_error   <= 1'b0;
						end
						else begin
                            response_pending <= 1'b0;
							mem_enable <= 0;
							d_valid   <= 1'b0;
							d_opcode  <= 0;
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= 0;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= 0;
							d_data    <= {TL_DATA_WIDTH{1'b0}};
							d_error   <= 0;							
                        end						
					end
					else begin
						d_valid   <= 1'b0;
						d_opcode  <= 0;
						d_param   <= {TL_PARAM_WIDTH{1'b0}};
						d_size    <= 0;
						d_sink    <= {TL_SINK_WIDTH{1'b0}};
						d_source  <= 0;
						d_data    <= {TL_DATA_WIDTH{1'b0}};
						d_error   <= 0;
					end

					wen <= 1'b0; // Deassert write when not responding
                end

                default: begin
                    wen <= 1'b0;
					ren <= 1'b0; // Deassert read when not responding
					response_done <= 0;
					mem_enable <= 0; // Disable memory access

					// Default response for unsupported opcodes
                    d_valid   <= 1'b0;                                                                                                      
                    d_opcode  <= 0;                                                                                              
                    d_param   <= {TL_PARAM_WIDTH{1'b0}};     // Reserved at 0                                                               
                    d_size    <= 0 ;                 // Same as from MASTER                                                            
                    d_sink    <= {TL_SINK_WIDTH{1'b0}};      // Ignored                                                                     
                    d_source  <= 0 ;               // Same as sent by MASTER                                                         
                    d_data    <= {TL_DATA_WIDTH{1'b0}};      // Ignored. Dataless response                                                  
                    d_error   <= 1'b0;                       // No error. Change later! Add error logic from memory (failed mem access etc).                    
                end
            endcase

        end else begin
            wen <= 1'b0; // Deassert write when not responding
			ren <= 1'b0; // Deassert read when not responding
			response_done <= 0;
			mem_enable <= 0; // Disable memory access
            d_valid   <= 1'b0;                                                                                                      
            d_opcode  <= 0;                                                                                              
            d_param   <= {TL_PARAM_WIDTH{1'b0}};     // Reserved at 0                                                               
            d_size    <= 0 ;                 // Same as from MASTER                                                            
            d_sink    <= {TL_SINK_WIDTH{1'b0}};      // Ignored                                                                     
            d_source  <= 0 ;               // Same as sent by MASTER                                                         
            d_data    <= {TL_DATA_WIDTH{1'b0}};      // Ignored. Dataless response                                                  
            d_error   <= 1'b0;                       // No error. Change later! Add error logic from memory (failed mem access etc).             
			mem_enable <= 0;
		end
    end


	always @(posedge clk or posedge rst) begin
		if (rst) begin
			// A Channel Registers
			a_ready_reg   <= 1'b0;
			a_opcode_reg  <= {TL_OPCODE_WIDTH{1'b0}};
			a_param_reg   <= {TL_PARAM_WIDTH{1'b0}};
			a_address_reg <= {TL_ADDR_WIDTH{1'b0}};
			a_size_reg    <= {TL_SIZE_WIDTH{1'b0}};
			a_mask_reg    <= {TL_STRB_WIDTH{1'b0}};
			a_data_reg    <= {TL_DATA_WIDTH{1'b0}};
			a_source_reg  <= {TL_SOURCE_WIDTH{1'b0}};
			a_valid_reg <= 0;

			
		end
		else begin
		    // d_ready_reg <= d_ready;
			if (a_valid) begin
				a_opcode_reg  <= a_opcode;
				a_param_reg   <= a_param;
				a_address_reg <= a_address;
				a_size_reg    <= a_size;
				a_mask_reg    <= a_mask;
				a_data_reg    <= a_data;
				a_source_reg  <= a_source;
				a_valid_reg <= a_valid;
			end
		end
	end




    top_tlul_spi #(
        .USE_EXTERNAL_SPI_MEM(USE_EXTERNAL_SPI_MEM)
    ) u_top_tlul_spi (
        .clk       (clk),
        .rst       (rst),
        .enable    (tlul_spi_bridge_enable),
        .a_opcode  (a_opcode_reg),
        .a_address (a_address_reg),
        .a_data    (wdata),
        .a_ready   (),
        .valid     (bridge_output_valid),
        .d_data    (rdata),
        .spi_wr_evt_valid(spi_wr_evt_valid_tb),
        .spi_wr_evt_addr(spi_wr_evt_addr_tb),
        .spi_wr_evt_data(spi_wr_evt_data_tb),
        .spi_rd_req_valid(spi_rd_req_valid_tb),
        .spi_rd_req_addr(spi_rd_req_addr_tb),
        .spi_rd_rsp_valid(spi_rd_rsp_valid_tb),
        .spi_rd_rsp_data(spi_rd_rsp_data_tb)
    );	


	
	
	
// memory_block #(
//     .TL_DATA_WIDTH(TL_DATA_WIDTH),      // Data width for memory
// 	.MEM_BASE_ADDR(MEM_BASE_ADDR),         // Base address for memory
//     .DEPTH(DEPTH),                		// Memory depth (number of entries)
//     .TL_ADDR_WIDTH(TL_ADDR_WIDTH),       // Address width
// 	.LATENCY(3)                          // Latency for memory access
// ) memory_inst (
//     .clk(clk),                  // Clock input
//     .rst(rst),                // Reset input
// 	.ren(ren),
//     .waddr(waddr),              // Write address from slave logic
//     .wen(wen),                  // Write enable from slave logic
//     .wdata(wdata),              // Write data from slave logic
//     .raddr(raddr),              // Read address from slave logic
//     .rdata(rdata),              // Read data to be sent back to master
// 	.mem_write_done(mem_write_done), // Memory write done signal
// 	.mem_acc_done(mem_acc_done)      // Memory access done signal
// );      
      
      
endmodule
      
      
      

	/////////////////////////////////////////////////////////////
	//////////// 				CPU MEMORY     	 	 	 ////////////
	/////////////////////////////////////////////////////////////	
/*
module memory_block #(
    parameter TL_DATA_WIDTH = 64,
    parameter MEM_BASE_ADDR = 64'h0000_0000_0000_0000, // Slave-specific base address
    parameter DEPTH = 512,
    parameter TL_ADDR_WIDTH = $clog2(DEPTH),
	parameter LATENCY = 3
)(
    input  wire                        clk,
    input  wire                        rst,

	input  wire					      ren,
    input  wire [TL_ADDR_WIDTH-1:0]   waddr,
    input  wire                        wen,
    input  wire [TL_DATA_WIDTH-1:0]   wdata,
    input  wire [TL_ADDR_WIDTH-1:0]   raddr,
    output wire [TL_DATA_WIDTH-1:0]   rdata,
	output 							  mem_write_done, 
	output wire 					  mem_acc_done
);

    // 1. Local memory (addressed from 0 to DEPTH-1)
    reg [TL_DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // 2. Local address offset computation
    wire [TL_ADDR_WIDTH-1:0] local_waddr = waddr - MEM_BASE_ADDR[TL_ADDR_WIDTH-1:0];
    wire [TL_ADDR_WIDTH-1:0] local_raddr = raddr - MEM_BASE_ADDR[TL_ADDR_WIDTH-1:0];

    // 3. Output register
    reg [TL_DATA_WIDTH-1:0] r_rdata;
    assign rdata = r_rdata;

	// Pipeline registers to induce latency for memory access
	reg [TL_DATA_WIDTH-1:0] rdata_pipe [0:LATENCY-1];
	reg [TL_ADDR_WIDTH-1:0] waddr_pipe [0:LATENCY-1];
	reg [TL_DATA_WIDTH-1:0] wdata_pipe [0:LATENCY-1];
	reg [TL_ADDR_WIDTH-1:0] raddr_pipe [0:LATENCY-1];
	reg ren_pipe [0:LATENCY-1];
	reg wen_pipe [0:LATENCY-1];
	reg r_mem_acc_done, r_mem_write_done;
	
	
	assign mem_write_done = r_mem_write_done;
	assign mem_acc_done = r_mem_acc_done;
	

    // 4. Memory logic
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < DEPTH; i = i + 1)
                mem[i] <= 0;
            r_rdata <= 0;
        end else begin

			// Update pipeline registers
			rdata_pipe[0] <= r_rdata; // First stage gets the current read data
			raddr_pipe[0] <= local_raddr; // First stage gets the current read address
			ren_pipe[0] <= ren; // First stage gets the current read enable
			wen_pipe[0] <= wen; // First stage gets the current write enable
			waddr_pipe[0] <= local_waddr; // First stage gets the current write address
			wdata_pipe[0] <= wdata; // First stage gets the current write data

			// Shift pipeline registers
			for (i = LATENCY-1; i > 0; i = i - 1) begin
				rdata_pipe[i] <= rdata_pipe[i-1];
				raddr_pipe[i] <= raddr_pipe[i-1];
				ren_pipe[i] <= ren_pipe[i-1];
				wen_pipe[i] <= wen_pipe[i-1];
				waddr_pipe[i] <= waddr_pipe[i-1];
				wdata_pipe[i] <= wdata_pipe[i-1];
			end

			// Last stage of the pipeline
			if (ren_pipe[LATENCY-1]) begin
				// Read operation
				r_rdata <= mem[raddr_pipe[LATENCY-1]];
				r_mem_acc_done <= 1'b1; // Indicate memory access is done
			end 
			else begin
				r_rdata <= 0; // Hold previous read data if not reading
				r_mem_acc_done <= 1'b0; // Indicate memory access is not done
			end
			
			if (wen_pipe[LATENCY-1]) begin
				// Write operation
				mem[waddr_pipe[LATENCY-1]] <= wdata_pipe[LATENCY-1]; // Write data to the local address
				r_mem_write_done <= 1'b1; // Indicate memory write is done
			end
			else begin
				r_mem_write_done <= 1'b0; // Indicate memory write is not done
			end

        end
    end

endmodule*/
/*******************************************************************************************************************************************

This top-level module sets up signals for the master instance of the TileLink UL protocol. It will work in the low speed domain with 
peripherals like GPIO and Flash.

*******************************************************************************************************************************************/
`timescale 1ns/1ps
`default_nettype none



module tilelink_ul_master_top #( 

	//////////////////////////////////////////////////////////////////
	////////////////////// Core interface widths /////////////////////
	//////////////////////////////////////////////////////////////////
	
	parameter TL_ADDR_WIDTH     = 64,            		// Address width
	parameter TL_DATA_WIDTH     = 64,            		// Data width
	parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8, 	// Byte mask width/Byte strobe

	//////////////////////////////////////////////////////////////////
	//////////////////// TileLink metadata widths ////////////////////
	//////////////////////////////////////////////////////////////////
	
	parameter TL_SOURCE_WIDTH   = 3,			        // Request ID
	parameter TL_SINK_WIDTH     = 3,			        // Response ID
	parameter TL_OPCODE_WIDTH   = 3,			        // Opcode width
	parameter TL_PARAM_WIDTH    = 3,                  // Reserved (0)
	parameter TL_SIZE_WIDTH     = 8,                  // log2(size in bytes)
	
	// Opcodes for A Channel
	parameter PUT_FULL_DATA_A     = 3'd0,
	parameter PUT_PARTIAL_DATA_A  = 3'd1,
	parameter ARITHMETIC_DATA_A   = 3'd2,
	parameter LOGICAL_DATA_A      = 3'd3,
	parameter GET_A               = 3'd4,
	parameter INTENT_A            = 3'd5,
	parameter ACQUIRE_BLOCK_A     = 3'd6,
	parameter ACQUIRE_PERM_A      = 3'd7,

	// Opcodes for D Channel
	parameter ACCESS_ACK_D        = 3'd0,
	parameter ACCESS_ACK_DATA_D   = 3'd1,
	parameter HINT_ACK_D          = 3'd2,
	parameter GRANT_D             = 3'd4,
	parameter GRANT_DATA_D        = 3'd5,
	parameter RELEASE_ACK_D       = 3'd6,

	// Master FSM States
	parameter REQUEST 			 = 2'd1,
	parameter RESPONSE   		 = 2'd2,
	parameter CLEANUP 			 = 2'd3,
	parameter IDLE   			 = 2'd0	

)(
	input  wire                              clk,
	input  wire                              rst,

	// Inputs for commands from testbench, defined as a_valid_in, a_opcode_in, etc.
	input  wire                              a_valid_in,		// Slave ready to accept data
	input  wire [TL_OPCODE_WIDTH-1:0]        a_opcode_in,
	input  wire [TL_PARAM_WIDTH-1:0]         a_param_in,		// Reserved, 0
	input  wire [TL_ADDR_WIDTH-1:0]          a_address_in,
	input  wire [TL_SIZE_WIDTH-1:0]          a_size_in,
	input  wire [TL_STRB_WIDTH-1:0]          a_mask_in,
	input  wire [TL_DATA_WIDTH-1:0]          a_data_in,
	input  wire [TL_SOURCE_WIDTH-1:0]        a_source_in,

	// A Channel: Sent TO SLAVE (Master drives it)
	input  wire                              a_ready,			// Slave ready to accept data
	output reg                               a_valid, 			// Master asserts to send valid request
	output reg  [TL_OPCODE_WIDTH-1:0]        a_opcode,
	output reg  [TL_PARAM_WIDTH-1:0]         a_param,			// Reserved, 0
	output reg  [TL_ADDR_WIDTH-1:0]          a_address,
	output reg  [TL_SIZE_WIDTH-1:0]          a_size,
	output reg  [TL_STRB_WIDTH-1:0]          a_mask,
	output reg  [TL_DATA_WIDTH-1:0]          a_data,
	output reg  [TL_SOURCE_WIDTH-1:0]        a_source,

	// D Channel: Received FROM SLAVE
	input  wire                              d_valid, 			// Slave responds
	output wire                              d_ready, 			// Master ready to accept
	input  wire [TL_OPCODE_WIDTH-1:0]        d_opcode,
	input  wire [TL_PARAM_WIDTH-1:0]         d_param,
	input  wire [TL_SIZE_WIDTH-1:0]          d_size,
	input  wire [TL_SINK_WIDTH-1:0]          d_sink,
	input  wire [TL_SOURCE_WIDTH-1:0]        d_source,
	input  wire [TL_DATA_WIDTH-1:0]          d_data,
	input  wire                              d_error
);

	// State variable for slave FSM
	reg [1:0] master_state, next_state;

	// State flags
	wire is_request;
	wire is_response;
	
	reg r_d_valid;

	// Registers for flopping A Channel signals
	// These registers hold the values for the A Channel signals that are sent to the slave.
	// They are useful in case the slave is not ready to accept the request in the current cycle.
	// They will be used to send the request in the next cycle when the slave is ready.
	reg                               r_a_valid; 			// Masterr_asserts to send valid request
	reg   [TL_OPCODE_WIDTH-1:0]       r_a_opcode;
	reg   [TL_PARAM_WIDTH-1:0]        r_a_param;			// Reserved; 0
	reg   [TL_ADDR_WIDTH-1:0]         r_a_address;
	reg   [TL_SIZE_WIDTH-1:0]         r_a_size;
	reg   [TL_STRB_WIDTH-1:0]         r_a_mask;
	reg   [TL_DATA_WIDTH-1:0]         r_a_data;
	reg   [TL_SOURCE_WIDTH-1:0]       r_a_source;


	///////////////////////////////////////////////////////////////
	//////////// 			 STATE MACHINE     	 	   ////////////
	///////////////////////////////////////////////////////////////
	
	assign is_request  = (master_state == REQUEST);   // High when in REQUEST state
	assign is_response = (master_state == RESPONSE);  // High when in RESPONSE state
	

	// State machine for the master
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			master_state <= REQUEST; // Reset to IDLE state
		end else begin
			master_state <= next_state; // Transition to the next state
		end
	end

	// Next state logic

	always @(*) begin
		case (master_state)
				
			REQUEST: begin
				if (a_valid_in & a_ready) begin
					next_state = RESPONSE; // Transition to RESPONSE if slave response is valid
				end else begin
					next_state = REQUEST; // Stay in REQUEST state otherwise
				end
			end
			
			RESPONSE: begin
			    if (d_valid) begin
			         next_state = REQUEST;
				end else begin
					next_state = RESPONSE; // Stay in RESPONSE state otherwise
				end
			end
			
		
			default: next_state = REQUEST; // Default case to handle unexpected states
			
		endcase
	end




	/////////////////////////////////////////////////////////////
	//////////// 			   DATAPATH        	 	 ////////////
	/////////////////////////////////////////////////////////////
	
    assign d_ready = is_response;
	
	// Block for flopping A Channel signals
	// This block is used to flop the A Channel signals that are sent to the slave.
	
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			// Reset logic for A Channel registers
			r_a_valid <= 1'b0; // Master is not ready to send data
			r_a_opcode <= 0;
			r_a_param  <= 0; // Reserved, 0
			r_a_address <= 0;
			r_a_size   <= 0;
			r_a_mask   <= 0;
			r_a_data   <= 0;
			r_a_source <= 0; 
		end 
		else if (a_valid_in) begin
			// Flop the A Channel signals
			r_a_valid <= a_valid_in;
			r_a_opcode <= a_opcode_in;
			r_a_param  <= a_param_in;
			r_a_address <= a_address_in;
			r_a_size   <= a_size_in;
			r_a_mask   <= a_mask_in;
			r_a_data   <= a_data_in;
			r_a_source <= a_source_in; 
		end
		else if (is_response) begin
			// If in RESPONSE state, clear the valid signal and reset the values
			r_a_valid <= 1'b0; // Master is not ready to send data
			r_a_opcode <= 0;
			r_a_param  <= 0; // Reserved, 0
			r_a_address <= 0;
			r_a_size   <= 0;
			r_a_mask   <= 0;
			r_a_data   <= 0;
			r_a_source <= 0;			
		end
		// If not in RESPONSE state, keep the previous values
		else begin
			// Keep the previous values
			r_a_valid <= r_a_valid;
			r_a_opcode <= r_a_opcode;
			r_a_param  <= r_a_param;
			r_a_address <= r_a_address;
			r_a_size   <= r_a_size;
			r_a_mask   <= r_a_mask;
			r_a_data   <= r_a_data;
			r_a_source <= r_a_source;
		end
	end


	always @(*) begin
		if (rst) begin
			// Reset logic for A Channel registers
			a_valid   = 1'b0;
			a_opcode  = 0;
			a_param   = 0;
			a_address = 0;
			a_size    = 0;
			a_mask    = 0;
			a_data    = 0;
			a_source  = 0;
		end else if (a_valid_in) begin
			// New transaction in same cycle
			a_valid   = a_valid_in;
			a_opcode  = a_opcode_in;
			a_param   = a_param_in;
			a_address = a_address_in;
			a_size    = a_size_in;
			a_mask    = a_mask_in;
			a_data    = a_data_in;
			a_source  = a_source_in;
		end else if (is_request) begin
			// Hold previous values (registered versions)
			a_valid   = r_a_valid;
			a_opcode  = r_a_opcode;
			a_param   = r_a_param;
			a_address = r_a_address;
			a_size    = r_a_size;
			a_mask    = r_a_mask;
			a_data    = r_a_data;
			a_source  = r_a_source;
		end else begin
			// Default case: no transaction
			a_valid   = 1'b0;
			a_opcode  = 0;
			a_param   = 0;
			a_address = 0;
			a_size    = 0;
			a_mask    = 0;
			a_data    = 0;
			a_source  = 0;
		end
	end

	// Response logic
	// This logic is for the D channel, which is the response from the slave to the master.
	// Opcode is used to determine the type of response.
	 always @(posedge clk) begin
	 	if(rst) begin
	 		r_d_valid <=  0;
	 	end
	 	else begin
	 		r_d_valid <= d_valid;
	 	end
	 end

endmodule
