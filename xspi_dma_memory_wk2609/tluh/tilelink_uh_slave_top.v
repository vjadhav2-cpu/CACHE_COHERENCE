/*******************************************************************************************************************************************

This top-level module sets up signals for the slave instance of the TileLink UH protocol. 

*******************************************************************************************************************************************/
`timescale 1ns/1ps
`default_nettype none
 
module tilelink_uh_slave_top #(  

	//////////////////////////////////////////////////////////////////
	////////////////////// Core interface widths /////////////////////
	//////////////////////////////////////////////////////////////////

	// Add comments for the following in Doxygen format
	
	
	parameter TL_ADDR_WIDTH     = 64,            		   // Address width
	parameter TL_DATA_WIDTH     = 64,            		   // Data width
	parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8, 	   // Byte mask width/Byte strobe. Each bit represents one byte of the data.
	parameter TL_BEAT_WIDTH     = 8,            		   // Width of each beat in bytes. USed to calculate number of beats.
	parameter BEAT_LOG2         = $clog2(TL_BEAT_WIDTH),   // Log2 of BEAT_WIDTH. Used to calculate number of beats in a transaction.
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
	// // A Channel Opcodes
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
//	parameter REQUEST 			 = 2'd0,
//	parameter BURST_WRITE 		 = 2'd1,
//	parameter BURST_READ 		 = 2'd2,
//	parameter RESPONSE   		 = 2'd3,

	// Memory parameters
	parameter MEM_BASE_ADDR 	  = 64'h0000_0000_0000_0000, // Base address for memory
	parameter DEPTH           	  = 512,                     // Memory depth (number of entries)
	parameter FIFO_DEPTH		  = 16                       // FIFO depth for burst read data buffer
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
	input  wire 			         a_corrupt,		// Corruption flag. If set, the transaction is corrupted and must be ignored.

	// D Channel Sent to MASTER
	output reg				 d_valid,
	input  wire                              d_ready, 		// Master ends d_ready to Slave to indicate that it is ready to accept data.
	output reg [TL_OPCODE_WIDTH-1:0]         d_opcode,
	output reg [TL_PARAM_WIDTH-1:0]          d_param,
	output reg [TL_SIZE_WIDTH-1:0]           d_size,
	output reg [TL_SINK_WIDTH-1:0]           d_sink,
	output reg [TL_SOURCE_WIDTH-1:0]         d_source,	
	output reg [TL_DATA_WIDTH-1:0]           d_data,
	output reg                               d_error,


	// ---------------- External memory interface ---------------
   	output reg  [TL_ADDR_WIDTH-1:0]          waddr,
    	output reg                               wen,
    	output reg  [TL_DATA_WIDTH-1:0]          wdata,
    	output reg  [TL_ADDR_WIDTH-1:0]          raddr,
    	output reg                               ren,
    	input  wire [TL_DATA_WIDTH-1:0]          rdata,
    	input  wire                              mem_write_done,
    	input  wire                              mem_acc_done
);

	// Localparams for ARITHMETIC_DATA_A

	localparam ATOM_MIN   = 3'd0;
	localparam ATOM_MAX   = 3'd1;
	localparam ATOM_MINU  = 3'd2;
	localparam ATOM_MAXU  = 3'd3;
	localparam ATOM_ADD   = 3'd4;

	// Localparams for LOGICAL_DATA_A

	localparam LOGIC_XOR  = 3'd0;
	localparam LOGIC_OR   = 3'd1;
	localparam LOGIC_AND  = 3'd2;
	localparam LOGIC_SWAP  = 3'd3;


	// State variable for Global slave FSM

// State encoding using localparam (no explicit width)
localparam IDLE        = 3'b000;
localparam REQUEST     = 3'b001;
localparam BURST_READ  = 3'b010;
localparam BURST_WRITE = 3'b011;
localparam ATOMIC_INST = 3'b100;
localparam RESPONSE    = 3'b101;

// State register
reg [2:0] slave_state;


	// Burst write FSM states - IDLE, BURST WRITE, DONE
// State encoding for burst write (implied 1-bit states)
localparam BURST_WRITE_PENDING = 1'b0;
localparam BURST_WRITE_DONE    = 1'b1;

// Register declaration
reg burst_write_state;


	// Response FSM state
localparam MEM_ACCESS_PENDING  = 2'b00;
localparam RESPONSE_PENDING    = 2'b01;
localparam RESPONSE_DONE       = 2'b10;

reg [1:0] response_state;


	// Burst read response FSM states
localparam MEM_READ_PENDING        = 2'b00;
localparam BURST_RESPONSE_PENDING  = 2'b01;
localparam BURST_RESPONSE_DONE     = 2'b10;

reg [1:0] burst_read_response_state;


	// Atomic Operation FSM states
localparam ATOMIC_READ       = 2'b00;
localparam ATOMIC_OPERATION  = 2'b01;
localparam ATOMIC_WRITE      = 2'b10;
localparam ATOMIC_RESPONSE   = 2'b11;

reg [1:0] atomic_state;


	// State flags for slave_state
	wire in_idle;
	wire in_request;
	wire in_burst_write;
	wire in_burst_read;
	wire in_atomic_inst;
	wire in_response;

	// State flags for response_state
	//wire in_mem_access_pending;
	//wire in_response_pending;
	//wire in_response_done;

	// FIFO flags
	//wire full;
	//wire empty;

	// Assigning state flags based on response_state
	//assign in_mem_access_pending = (response_state == MEM_ACCESS_PENDING);
	//assign in_response_pending = (response_state == RESPONSE_PENDING);
	//assign in_response_done = (response_state == RESPONSE_DONE);

	// Assigning state flags based on slave_state
	assign in_idle = (slave_state == IDLE);
	assign in_request = (slave_state == REQUEST);
	assign in_burst_write = (slave_state == BURST_WRITE);
	assign in_burst_read = (slave_state == BURST_READ);
	assign in_atomic_inst = (slave_state == ATOMIC_INST);
	assign in_response = (slave_state == RESPONSE);

	// a_ready signal
	assign a_ready = (slave_state == REQUEST) || (slave_state == BURST_WRITE);

	
	// Memory Flags
	
/*	reg  [TL_ADDR_WIDTH-1:0] waddr;
	reg              	   wen,ren;
	reg  [TL_DATA_WIDTH-1:0] wdata;
	reg  [TL_ADDR_WIDTH-1:0] raddr;
	wire [TL_DATA_WIDTH-1:0] rdata;
*/

	//reg  [TL_DATA_WIDTH-1:0] fifo_wdata;
	//reg  [TL_DATA_WIDTH-1:0] fifo_rdata;
	//reg  [TL_SIZE_WIDTH-1:0] mem_write_counter;
	reg mem_enable;
	reg response_pending;
//	wire mem_acc_done;
//	wire mem_write_done;
	reg response_done;

	// FIFO controls
	//reg fifo_wen, fifo_ren;

	// Burst and single beat write/read flags
	reg burst_write_active;
	reg burst_read_active;
	reg single_beat_write_active;
	reg single_beat_read_active;
	reg burst_write_done;
	reg burst_read_done;
	reg single_beat_write_done;
	reg single_beat_read_done;
	reg atomic_inst_active;
	reg atomic_inst_done;


	reg [TL_SIZE_WIDTH-1:0] incr_address,burst_read_counter;
	reg [TL_SIZE_WIDTH-1:0] num_beats;
	reg [TL_DATA_WIDTH-1:0] burst_read_data_buf [0:FIFO_DEPTH-1]; // Buffer for burst read data
	reg [$clog2(FIFO_DEPTH)-1:0] burst_read_buf_ptr, burst_read_buf_counter;
	
	reg [TL_DATA_WIDTH-1:0] unsigned_atomic_operand [1:0];
	reg signed [TL_DATA_WIDTH-1:0] signed_atomic_operand [1:0];


	// For loop variables
	integer i;
	
	// Registers for A Channel (Slave side input)
	//reg                             r_a_ready;
	reg                             r_a_valid;
	reg [TL_OPCODE_WIDTH-1:0]       r_a_opcode;
	reg [TL_PARAM_WIDTH-1:0]        r_a_param;
	reg [TL_ADDR_WIDTH-1:0]         r_a_address;
	reg [TL_SIZE_WIDTH-1:0]         r_a_size;
	reg [TL_STRB_WIDTH-1:0]         r_a_mask;
	reg [TL_DATA_WIDTH-1:0]         r_a_data;
	reg [TL_SOURCE_WIDTH-1:0]       r_a_source;

	// Global FSM -> slave_state
	// The slave_state is used to determine the current state of the slave FSM.

	// RESET D CHANNEL TASK

	task automatic reset_d_channel;
	begin
		d_valid  <= 1'b0;
		d_opcode <= {TL_OPCODE_WIDTH{1'b0}};
		d_param  <= {TL_PARAM_WIDTH{1'b0}};
		d_size   <= {TL_SIZE_WIDTH{1'b0}};
		d_sink   <= {TL_SINK_WIDTH{1'b0}};
		d_source <= {TL_SOURCE_WIDTH{1'b0}};
		d_data   <= {TL_DATA_WIDTH{1'b0}};
		d_error  <= 1'b0;
	end
	endtask

	task automatic hold_d_channel;
	begin
		d_valid  <= d_valid;
		d_opcode <= d_opcode;
		d_param  <= d_param;
		d_size   <= d_size;
		d_sink   <= d_sink;
		d_source <= d_source;
		d_data   <= d_data;
		d_error  <= d_error;
	end
	endtask


	task d_channel_response (
	input [TL_OPCODE_WIDTH-1:0] opcode_in,
	input [TL_DATA_WIDTH-1:0]   data_in
	);
	begin
		d_valid  <= 1'b1;
		d_opcode <= opcode_in;
		d_param  <= r_a_param;
		d_size   <= r_a_size;
		d_sink   <= 0;
		d_source <= r_a_source;
		d_data   <= data_in;
		d_error  <= 0;
	end
	endtask


	/////////////////////////////////////////////////////////////
	//////////// 		      FSM BLOCK    	    	 ////////////
	/////////////////////////////////////////////////////////////


	always @(posedge clk or posedge rst) begin
		if (rst) begin
			slave_state <= REQUEST; // Reset to REQUEST state on reset
		end else begin
			// State transitions are handled in the FSM block below
			case (slave_state)
				REQUEST: begin
					if (a_valid) begin
						if ({24'b0, a_size} > BEAT_LOG2) begin
							// If size is greater than or equal to BEAT_LOG2, it is a burst transaction
							case (a_opcode)
								GET_A: begin
									slave_state <= BURST_READ; // Transition to BURST_READ state for GET_A opcode
								end
								PUT_FULL_DATA_A: begin
									slave_state <= BURST_WRITE; // Transition to BURST_WRITE state for PUT_FULL_DATA_A opcode
								end							
								INTENT_A: begin
									slave_state <= RESPONSE; // Transition to RESPONSE state for INTENT_A opcode
								end
								default: begin
									
								end
							endcase
						end else if(a_opcode == ARITHMETIC_DATA_A  || a_opcode == LOGICAL_DATA_A) begin
							slave_state <= ATOMIC_INST; // Transition to ATOMIC_INST state for atomic operations
						end
						else begin
							// If size is less than BEAT_LOG2, it is a single beat transaction
							slave_state <= RESPONSE; // Transition to RESPONSE state for single beat transactions
						end
					end else begin
						slave_state <= REQUEST; // Stay in REQUEST state if no valid request
					end
				end 
				BURST_WRITE: begin
					// Handle burst write operations
					if (burst_write_done) begin
						// Transition to RESPONSE state. 
						slave_state <= RESPONSE; // If response is pending, go to RESPONSE state
					end
					else begin
						// Stay in BURST_WRITE state until all beats are written
						slave_state <= BURST_WRITE;
					end
				end
				BURST_READ: begin
					// Handle burst read operations
					if (burst_read_done) begin
						slave_state <= REQUEST; // If all beats are read, go to RESPONSE state
					end else begin
						slave_state <= BURST_READ; // Stay in BURST_READ state until all beats are read
					end
				end
				ATOMIC_INST: begin
					// Handle atomic burst operations
					if(atomic_inst_done) begin
						slave_state <= REQUEST; // If atomic instruction is done, go to RESPONSE state
					end
					else begin
						slave_state <= ATOMIC_INST; // Stay in ATOMIC_INST state until atomic instruction is done
					end
				end
				// RESPONSE: This state is used to send responses back to the master
				RESPONSE: begin
					if (d_ready && response_pending) begin
						slave_state <= REQUEST; // If d_ready is high and response is pending, go back to REQUEST state
					end else begin
						slave_state <= RESPONSE; // Stay in RESPONSE state until d_ready is high
					end
				end

				default: begin 
				
				end
			endcase
		end
	end


	// Intercept requests

	always @(posedge clk or posedge rst) begin
		if (rst) begin
			// Reset FSM states
			burst_write_state <= BURST_WRITE_PENDING; // Reset to BURST_WRITE state
			response_state <= MEM_ACCESS_PENDING; // Reset to RESPONSE_PENDING state
			burst_read_response_state <= MEM_READ_PENDING; // Reset to MEM_READ_PENDING state
			atomic_state <= ATOMIC_READ; // Reset to ATOMIC_READ state
			
			for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
				burst_read_data_buf[i] <= {TL_DATA_WIDTH{1'b0}}; // Reset each entry of burst read data buffer
			end
			burst_read_buf_ptr <= {($clog2(FIFO_DEPTH)){1'b0}}; // Reset burst read buffer pointer
			burst_read_buf_counter <= {($clog2(FIFO_DEPTH)){1'b0}}; // Reset burst read buffer counter


			incr_address <= {TL_SIZE_WIDTH{1'b0}}; // Reset increment address
			num_beats <= {TL_SIZE_WIDTH{1'b0}}; // Reset number of beats
			wen <= 1'b0; // Disable write enable
			waddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset write address
			wdata <= {TL_DATA_WIDTH{1'b0}}; // Reset write data	
			ren <= 1'b0; // Disable read enable
			raddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset read address
			mem_enable <= 1'b0; // Disable memory enable
			single_beat_write_done <= 1'b0; // Reset single beat write done
			single_beat_read_done <= 1'b0; // Reset single beat read done
			burst_write_done <= 1'b0; // Reset burst write done
			burst_read_done <= 1'b0; // Reset burst read done
			single_beat_write_active <= 1'b0; // Reset single beat write active
			single_beat_read_active <= 1'b0; // Reset single beat read active
			burst_write_active <= 1'b0; // Reset burst write active
			burst_read_active <= 1'b0; // Reset burst read active
			response_pending <= 1'b0; // Reset response pending
			response_done <= 1'b0; // Reset response done

			burst_read_counter <= {TL_SIZE_WIDTH{1'b0}};

		end else begin

			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// SINGLE BEAT REQUESTS //////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////
			if (in_request) begin
				// Reset other flags
				burst_read_counter <= {TL_SIZE_WIDTH{1'b0}}; // Reset burst read counter

				response_done <= 1'b0; // Reset response done flag
				response_pending <= 1'b0; // Reset response pending flag
				burst_write_done <= 1'b0; // Reset burst write done flag
				burst_read_done <= 1'b0; // Reset burst read done flag
				burst_write_active <= 1'b0; // Reset burst write active flag
				burst_read_active <= 1'b0; // Reset burst read active flag
				single_beat_write_done <= 1'b0; // Reset single beat write done flag
				single_beat_read_done <= 1'b0; // Reset single beat read done flag
				single_beat_write_active <= 1'b0; // Reset single beat write active flag
				single_beat_read_active <= 1'b0; // Reset single beat read active flag
				atomic_inst_active <= 1'b0; // Reset atomic instruction active flag
				atomic_inst_done <= 1'b0; // Reset atomic instruction done flag

				// This block handles all single beat R/W requests that are not atomic.
				// Includes: GET_A, PUT_FULL_DATA_A and PUT_PARTIAL_DATA_A.
				if (a_valid) begin
					if ({24'b0, a_size} <= BEAT_LOG2) begin
						case (a_opcode)
							GET_A: begin
								// Single beat read operation

									ren <= 1; 							// Enable read
									raddr <= a_address; 				// Set read address
									wdata <= {TL_DATA_WIDTH{1'b0}}; 	// No write data for read
									single_beat_read_active <= 1'b1;
									single_beat_read_done <= 1'b0; 

							end
							PUT_FULL_DATA_A: begin
								// Single beat write operation
								
								wen <= 1; 							// Enable write
								waddr <= a_address; 				// Set write address
								wdata <= a_data; 					// Set write data
								single_beat_write_active <= 1'b1; // Set single beat write active flag
								single_beat_write_done <= 1'b0;   // Reset single beat write done flag

							end
							ARITHMETIC_DATA_A: begin
								// Handle atomic operation
								ren <= 1; 							// Enable write
								raddr <= a_address; 				// Set write address
								atomic_inst_active <= 1'b1; 		// Reset atomic instruction active flag
								atomic_inst_done <=	0;
								if(a_opcode == ARITHMETIC_DATA_A && a_param < 2) begin
									// Signed Operation
									signed_atomic_operand[0] <= a_data; // Store signed operand
								end
								else begin
									unsigned_atomic_operand[0] <= a_data; // Store unsigned operand
								end
							end
							LOGICAL_DATA_A: begin
								// Handle atomic operation
								ren <= 1; 							// Enable write
								raddr <= a_address; 				// Set write address
								atomic_inst_active <= 1'b1; 		// Reset atomic instruction active flag
								atomic_inst_done <=	0;
								if(a_opcode == ARITHMETIC_DATA_A && a_param < 2) begin
									// Signed Operation
									signed_atomic_operand[0] <= a_data; // Store signed operand
								end
								else begin
									unsigned_atomic_operand[0] <= a_data; // Store unsigned operand
								end
							end							
							default: begin
								waddr <= {TL_ADDR_WIDTH{1'b0}};
								wdata <= {TL_DATA_WIDTH{1'b0}};
								ren <= 1'b0;
								wen <= 1'b0;
								raddr <= {TL_ADDR_WIDTH{1'b0}};
								single_beat_write_active <= 1'b0;
								single_beat_read_active <= 1'b0;
								single_beat_write_done <= 1'b0;
								single_beat_read_done <= 1'b0;
								burst_write_done <= 1'b0;
								burst_read_done <= 1'b0;
								burst_write_active <= 1'b0;
								burst_read_active <= 1'b0;
							end
						endcase
					end
				end
			end

			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// BURST WRITE REQUEST ///////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////			
			else if (in_burst_write) begin
				// Handle burst write interception
				single_beat_write_done <= 1'b0; // Reset single beat write done flag
				single_beat_read_done <= 0; // Reset single beat read done flag		
				single_beat_write_active <= 0; // Reset single beat write active flag
				single_beat_read_active <= 0; // Reset single beat read active flag		

				response_done <= 1'b0; // Reset response done flag
				response_pending <= 1'b0; // Reset response pending flag
				
				case (burst_write_state)
					BURST_WRITE_PENDING: begin
						if (incr_address < num_beats) begin
							// Handle BURST_WRITE state
							if (r_a_valid) begin
								incr_address <= incr_address + 1; // Increment address for next beat
								burst_write_state <= BURST_WRITE_PENDING; // Stay in BURST_WRITE state until all beats are written
								burst_write_active <= 1'b1; // Set burst write active flag
								burst_write_done <= 1'b0; // Reset burst write done flag
								waddr <= r_a_address + {{56{1'b0}}, incr_address}; // Increment address for burst write
								wdata <= r_a_data; // Assuming same data for burst write							
								wen <= 1'b1; // Enable write
							end else begin
								// Handle additional beats for burst write
								incr_address <= incr_address; // Increment address for next beat
								burst_write_state <= BURST_WRITE_PENDING; // Stay in BURST_WRITE state until all beats are written
								burst_write_active <= 1'b1; // Set burst write active flag
								burst_write_done <= 1'b0; // Reset burst write done flag
								waddr <= r_a_address; // Increment address for burst write
								wdata <= r_a_data; // Assuming same data for burst write
								wen <= 1'b0; // Enable write
							end
						end
						else begin
							// Reset all. Move to BURST_WRITE_DONE state
							burst_write_state <= BURST_WRITE_DONE;
							incr_address <= {TL_SIZE_WIDTH{1'b0}}; // Reset increment address
							burst_write_active <= 1'b0; // Reset burst write active flag
							burst_write_done <= 1'b1; // Set burst write done flag
							wen <= 1'b0; // Disable write
							wdata <= {TL_DATA_WIDTH{1'b0}}; // Reset write data
							waddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset write address
						end
					end
					BURST_WRITE_DONE: begin
						// Burst write is done. One cycle latency to show the end of burst write
						wen <= 1'b0; // Disable write
						burst_write_done <= 1'b0; // Reset burst write done flag
						incr_address <= {TL_SIZE_WIDTH{1'b0}}; // Reset increment address
						burst_write_state <= BURST_WRITE_PENDING; // Reset to BURST_WRITE state for next burst write
						burst_read_done <= 0; // Reset burst read done flag
						burst_read_active <= 0; // Reset burst read active flag

						// The write is complete. This state adds a one cycle latency to indicate the end of a burst write
						// response_pending <= 1'b1; // Reset response pending flag
						// response_done <= 1'b0; // Reset response done flag

					end
					default: begin
						// Reset all. Move to BURST_WRITE_DONE state
						burst_write_state <= BURST_WRITE_PENDING;
						incr_address <= {TL_SIZE_WIDTH{1'b0}}; // Reset increment address
						burst_write_active <= 1'b0; // Reset burst write active flag
						burst_write_done <= 1'b1; // Set burst write done flag
						wen <= 1'b0; // Disable write
						wdata <= {TL_DATA_WIDTH{1'b0}}; // Reset write data
						waddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset write address						
					end
				endcase
			end

			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// BURST READ REQUEST ////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////	


			else if (in_burst_read) begin
				// Handle burst read interception

				// FSM for burst read
				case (burst_read_response_state)
					MEM_READ_PENDING: begin
						// Assumption: a_size has exceeded BEAT_LOG2.
						if (incr_address < num_beats) begin
							// Handle BURST_READ state
							incr_address <= incr_address + 1; 
							burst_read_done <= 1'b0; 
							burst_read_active <= 1'b1;
							raddr <= r_a_address + {{56{1'b0}}, incr_address}; 
							ren <= 1'b1; 
							burst_read_active <= 1'b1;
							burst_read_done <= 1'b0; // Reset burst read done flag
						end
						else begin
							// incr_address <= {TL_SIZE_WIDTH{1'b0}}; // Reset increment address
							burst_read_active <= 1'b1; // Reset burst read active flag
							burst_read_done <= 1'b0; // Set burst read done flag
							ren <= 1'b0; // Disable read
							raddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset read address	
							burst_read_active <= 1'b1;
							burst_read_done <= 1'b0; // Reset burst read done flag
						end

						if(burst_read_counter < num_beats) begin
							if(mem_acc_done) begin
								// Memory access is done. Update read data.
								burst_read_data_buf[burst_read_counter[3:0]] <= rdata; // Store read data in burst read data buffer
								burst_read_counter <= burst_read_counter + 1; // Increment burst read counter
							end
							// burst_read_active <= 1'b1;
							// burst_read_done <= 1'b0;
						end
						else begin
							// Memory access is done for all beats. Move to BURST_READ_DONE state
							burst_read_response_state <= BURST_RESPONSE_PENDING; // Transition to BURST_RESPONSE_PENDING state
							// Dump the first value into the D channel
							d_channel_response (
								ACCESS_ACK_DATA_D, // Set d_opcode for GET_A
								burst_read_data_buf[burst_read_buf_ptr] // Set d_data from burst read data buffer
							);
							burst_read_buf_ptr <= burst_read_buf_ptr + 1; // Increment burst read buffer pointer
							response_pending <= 1'b1; // Set response pending flag
							response_done <= 1'b0; // Reset response done flag
							// burst_read_done <= 1'b1; // Reset burst read done flag
							// burst_read_active <= 1'b0; // Reset burst read active flag
						end

					end
					BURST_RESPONSE_PENDING: begin
						// Response is pending.
						if ({4'b0, burst_read_buf_ptr} < num_beats) begin
							// If there are more data in the buffer, send the next data to channel D.
							if(d_ready) begin // The current data is accepted by the master
								d_channel_response (
									ACCESS_ACK_DATA_D, // Set d_opcode for GET_A
									burst_read_data_buf[burst_read_buf_ptr] // Set d_data from burst read data buffer
								);
								burst_read_buf_ptr <= burst_read_buf_ptr + 1; // Increment burst read buffer pointer
							end
							else begin
								// Hold D channel signals if d_ready is not high
								hold_d_channel(); // Hold D channel signals
							end
							// Maintain the burst read response state
							burst_read_response_state <= BURST_RESPONSE_PENDING;

							// Response flags
							response_pending <= 1'b1; // Set response pending flag
							response_done <= 1'b0; // Reset response done flag
							burst_read_done <= 1'b0; // Reset burst read done flag
							burst_read_active <= 1'b1; // Set burst read active flag
						end
						else begin
							// No more data in the buffer. Move to BURST_RESPONSE_DONE state.
							burst_read_response_state <= BURST_RESPONSE_DONE; 

							// Response flags
							response_pending <= 1'b0; // Reset response pending flag
							response_done <= 1'b1; // Set response done flag
							burst_read_done <= 1'b1; // Set burst read done flag
							burst_read_active <= 1'b0; // Reset burst read active flag

							// Reset D channel signals
							reset_d_channel(); 
						end
					end
					BURST_RESPONSE_DONE: begin
						// Response is done.
						burst_read_response_state <= MEM_READ_PENDING; // Transition to BURST_READ_PENDING state
						burst_read_buf_ptr <= {($clog2(FIFO_DEPTH)){1'b0}}; // Reset burst read buffer pointer
						burst_read_buf_counter <= {($clog2(FIFO_DEPTH)){1'b0}}; // Reset burst read buffer counter
						burst_read_active <= 1'b0; // Reset burst read active flag
						burst_read_done <= 1'b0; // Reset burst read done flag
						ren <= 1'b0; // Disable read
						raddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset read address
						reset_d_channel(); // Reset D channel signals
					end
					default: begin
						// Reset all. Move to BURST_READ_PENDING state

					end
				endcase
			end

			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// ATOMIC INST REQUEST ///////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////	


			else if (in_atomic_inst) begin
				// Handle atomic burst interception
				case (atomic_state)
					ATOMIC_READ: begin
						// Handle atomic read operation
						ren <= 0;
						if(!mem_acc_done) begin
							// Remain in this state until memory access is done
							atomic_state <= ATOMIC_READ; 
						end
						else begin
							atomic_state <= ATOMIC_OPERATION;

							if(r_a_opcode == ARITHMETIC_DATA_A && r_a_param < 2) begin
								// Signed Operation
								signed_atomic_operand[1] <= rdata; // Store signed operand
							end
							else begin
								unsigned_atomic_operand[1] <= rdata; // Store unsigned operand
							end
						end
					end
					ATOMIC_OPERATION: begin
						// Change state to ATOMIC_RESPONSE
						atomic_state <= ATOMIC_WRITE; // Transition to ATOMIC_WRITE state
						// Write the minimum into memory. 
						wen <= 1'b1; // Enable write
						waddr <= r_a_address; // Set write address								
						// Handle atomic operation
						if (r_a_opcode == ARITHMETIC_DATA_A) begin
					

							case (r_a_param)
								ATOM_MIN: begin
									// Write the minimum value into memory. 
									wdata <= (signed_atomic_operand[0] < signed_atomic_operand[1])
											? signed_atomic_operand[0]
											: signed_atomic_operand[1];
								end
								ATOM_MAX: begin
									wdata <= (signed_atomic_operand[0] > signed_atomic_operand[1])
											? signed_atomic_operand[0]
											: signed_atomic_operand[1];
								end
								ATOM_MINU: begin
									wdata <= (unsigned_atomic_operand[0] < unsigned_atomic_operand[1])
											? unsigned_atomic_operand[0]
											: unsigned_atomic_operand[1];
								end
								ATOM_MAXU: begin
									wdata <= (unsigned_atomic_operand[0] > unsigned_atomic_operand[1])
											? unsigned_atomic_operand[0]
											: unsigned_atomic_operand[1];
								end
								ATOM_ADD: begin
									wdata <= unsigned_atomic_operand[0] + unsigned_atomic_operand[1];
								end
								default: begin
									
								end
							endcase
						end

						else if(r_a_opcode == LOGICAL_DATA_A) begin
							// wen <= 1'b1; 
							// waddr <= r_a_address;
							case (r_a_param)
								LOGIC_XOR: begin
									wdata <= unsigned_atomic_operand[1] ^ r_a_data; // Perform XOR operation
								end
								LOGIC_OR: begin
									wdata <= unsigned_atomic_operand[1] | r_a_data; // Perform OR operation
								end
								LOGIC_AND: begin
									wdata <= unsigned_atomic_operand[1] & r_a_data; // Perform AND operation
								end
								LOGIC_SWAP: begin
									wdata <= unsigned_atomic_operand[0]; // Swap operation
								end
								default: begin

								end
							endcase
						end

					end

					ATOMIC_WRITE: begin
						wen <= 0;
						ren <= 0;
						if(!mem_write_done) begin
							// Wait for the memory write to complete
							atomic_state <= ATOMIC_WRITE; // Stay in ATOMIC_WRITE state until memory write
							reset_d_channel(); // Reset D channel signals
						end
						else begin
							atomic_state <= ATOMIC_RESPONSE; // Transition to ATOMIC_RESPONSE state
							if(r_a_opcode == ARITHMETIC_DATA_A && r_a_param < 2) begin
								d_channel_response(
									ACCESS_ACK_DATA_D, // Set d_opcode for ARITHMETIC_DATA_A
									signed_atomic_operand[1] // Set d_data to the second signed operand
								);
							end
							else begin
								d_channel_response(
									ACCESS_ACK_DATA_D, // Set d_opcode for LOGICAL_DATA_A
									unsigned_atomic_operand[1] // Set d_data to the second unsigned operand
								);
							end
							response_done <= 0;
							response_pending <= 1;
							atomic_inst_active <= 0;
							atomic_inst_done <= 0; // Set atomic instruction done flag
						end
					end

					ATOMIC_RESPONSE: begin
						if(d_ready) begin
							response_done <= 1;
							reset_d_channel();
							atomic_inst_active <= 0;
							atomic_inst_done <= 1;
							response_pending <= 0;
							atomic_state <= ATOMIC_READ; // Reset to ATOMIC_READ state
						end
						else begin
							// Hold D channel and other signals
							hold_d_channel();
							response_done <= 0;
							response_pending <= 1;
							atomic_inst_active <= 0;
							atomic_inst_done <= 0;
							atomic_state <= ATOMIC_RESPONSE;
						end

						
					end
					default: begin
						// Handle default case
						atomic_state <= ATOMIC_READ; // Reset to ATOMIC_READ state
					end
				endcase
			end

			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// SINGLE BEAT RESPONSE //////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////	

			else if (in_response) begin
				// This else-if block handles single cycle responses.
				// Includes responses to single beat writes, burst writes, and single beat reads.
				// 

				// Reset enable signals
				ren <= 1'b0; // Disable read enable
				wen <= 1'b0; // Disable write enable
				raddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset read address
				waddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset write address
				wdata <= {TL_DATA_WIDTH{1'b0}}; // Reset write data

				// POTENTIAL OPTIMIZATION: ISOLATE THE D CHANNEL FROM THE FSM.
				// USE A NEW BLOCK TO ASSIGN THE D SIGNALS BASED ON THE SAME CONTROL SIGNALS USED BELOW.
				case (response_state)
					MEM_ACCESS_PENDING: begin
						case (r_a_opcode)
							GET_A: begin
								// Single beat read is done. Memory access is raised high for 1 clock.
								if (mem_acc_done) begin
									response_state <= RESPONSE_PENDING; // Transition to RESPONSE_PENDING state
									response_pending <= 1'b1; // Set response pending flag

									// Set D Channel signals for response
									d_valid <= 1'b1; // Set d_valid to indicate response is ready
									d_opcode <= ACCESS_ACK_DATA_D; // Set d_opcode for GET_A
									d_param <= r_a_param; // Set d_param from A Channel
									d_size <= r_a_size; // Set d_size from A Channel
									d_sink <= 0; // Set d_sink from A Channel
									d_source <= r_a_source; // Set d_source from A Channel
									d_data <= rdata; // Set d_data from A Channel
									d_error <= 1'b0; // Set d_error to indicate no error
								end
								else begin
									response_state <= MEM_ACCESS_PENDING; // Stay in MEM_ACCESS_PENDING state until memory access is done

									// Set D channel to indicate that response is not ready
									reset_d_channel();

								end
							end
							PUT_FULL_DATA_A: begin
								if ({24'b0, r_a_size} <= BEAT_LOG2) begin
									// For single beat write, memory access flag is raised high for 1 clock.
									if (mem_write_done) begin
										response_state <= RESPONSE_PENDING; // Transition to RESPONSE_PENDING state
										response_pending <= 1'b1; // Set response pending flag

										// Set D Channel signals for response
										d_valid <= 1'b1; // Set d_valid to indicate response is ready
										d_opcode <= ACCESS_ACK_D; // Set d_opcode for PUT_FULL_DATA_A
										d_param <= r_a_param; // Set d_param from A Channel
										d_size <= r_a_size; // Set d_size from A Channel
										d_sink <= 0; // Set d_sink from A Channel
										d_source <= r_a_source; // Set d_source from A Channel
										d_data <= 0; // Set d_data from A Channel
										d_error <= 1'b0; // Set d_error to indicate no error
									end
									else begin
										response_state <= MEM_ACCESS_PENDING; // Stay in MEM_ACCESS_PENDING state until memory access is done

										// Reset D channel signals
										reset_d_channel();
									end
								end
								else begin
									// For burst write, memory access flag is raised high for as many clocks as the number of beats.
									// So, we wait for the flag to be lowered. That indicates the end of the burst write.
									if(mem_write_done) begin
										response_state <= MEM_ACCESS_PENDING; // Transition to RESPONSE_PENDING state

										// Reset D channel signals
										reset_d_channel();
									end
									else begin
										response_state <= RESPONSE_PENDING; // Stay in MEM_ACCESS_PENDING state until memory access is done
										response_pending <= 1'b1; // Set response pending flag
										
										// Set D Channel signals for response
										d_valid <= 1'b1; // Set d_valid to indicate response is ready
										d_opcode <= ACCESS_ACK_D; // Set d_opcode for PUT_FULL_DATA_A
										d_param <= r_a_param; // Set d_param from A Channel
										d_size <= r_a_size; // Set d_size from A Channel
										d_sink <= 0; // Set d_sink from A Channel
										d_source <= r_a_source; // Set d_source from A Channel
										d_data <= 0; // Set d_data from A Channel
										d_error <= 1'b0; // Set d_error to indicate no error
									end
								end
							end
							default: begin
								
							end
						endcase
					end
					RESPONSE_PENDING: begin
						if (response_pending) begin

							if (d_ready) begin
								response_pending <= 1'b0; // Reset response pending flag
								response_done <= 1'b1; // Set response done flag
								reset_d_channel();
								response_state <= MEM_ACCESS_PENDING; // Transition to RESPONSE_DONE state
							end
							else begin
								response_pending <= 1'b1; // Keep response pending flag high
								response_done <= 1'b0; // Reset response done flag
								hold_d_channel(); // Hold D channel signals
								response_state <= RESPONSE_PENDING; // Stay in RESPONSE_PENDING state
							end

						end
					end 
					default: begin
						reset_d_channel();
					end
				endcase
			end
			else begin
				// Default case, do nothing or reset signals
				reset_d_channel();
				response_done <= 1'b0;
				response_pending <= 1'b0; // Reset response pending flag
				incr_address <= {TL_SIZE_WIDTH{1'b0}}; // Reset increment address
				num_beats <= {TL_SIZE_WIDTH{1'b0}}; // Reset number of beats
				wen <= 1'b0; // Disable write enable
				waddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset write address
				wdata <= {TL_DATA_WIDTH{1'b0}}; // Reset write data
				ren <= 1'b0; // Disable read enable
				raddr <= {TL_ADDR_WIDTH{1'b0}}; // Reset read address
				mem_enable <= 1'b0; // Disable memory enable
				single_beat_write_done <= 1'b0; // Reset single beat write done
				single_beat_read_done <= 1'b0; // Reset single beat read done
				burst_write_done <= 1'b0; // Reset burst write done
				burst_read_done <= 1'b0; // Reset burst read done
				single_beat_write_active <= 1'b0; // Reset single beat write active
				single_beat_read_active <= 1'b0; // Reset single beat read active
				burst_write_active <= 1'b0; // Reset burst write active
				burst_read_active <= 1'b0; // Reset burst read active
				
				burst_read_counter <= {TL_SIZE_WIDTH{1'b0}}; // Reset burst read counter
			end
		end
	end









	// Flopped version of num_beats
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			num_beats <= {TL_SIZE_WIDTH{1'b0}};
		end else begin
			if (a_valid) begin
				num_beats <= ({24'b0, a_size} > BEAT_LOG2) ? (1 << ({24'b0, a_size} - BEAT_LOG2)) : 1;
			end
			else if (!response_done) begin // If response is not done, keep the previous value
				num_beats <= num_beats; // Keep the previous value if a_valid is low
			end
			else begin
				num_beats <= {TL_SIZE_WIDTH{1'b0}}; // Reset num_beats when response is done
			end
		end
	end


	
	
	// Wait signal
	





    // Flopping the A Channel signals to registers
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			// A Channel Registers
			// a_ready_reg   <= 1'b0;
			r_a_opcode  <= {TL_OPCODE_WIDTH{1'b0}};
			r_a_param   <= {TL_PARAM_WIDTH{1'b0}};
			r_a_address <= {TL_ADDR_WIDTH{1'b0}};
			r_a_size    <= {TL_SIZE_WIDTH{1'b0}};
			r_a_mask    <= {TL_STRB_WIDTH{1'b0}};
			r_a_data    <= {TL_DATA_WIDTH{1'b0}};
			r_a_source  <= {TL_SOURCE_WIDTH{1'b0}};
			r_a_valid   <= 1'b0;

			
		end
		else begin
		    // d_ready_reg <= d_ready;
			if (a_valid) begin
				r_a_opcode  <= a_opcode;
				r_a_param   <= a_param;
				r_a_address <= a_address;
				r_a_size    <= a_size;
				r_a_mask    <= a_mask;
				r_a_data    <= a_data;
				r_a_source  <= a_source;
				r_a_valid   <= a_valid;
			end
			else if (response_done) begin
				// Reset A Channel registers when response is done
				r_a_opcode  <= {TL_OPCODE_WIDTH{1'b0}};
				r_a_param   <= {TL_PARAM_WIDTH{1'b0}};
				r_a_address <= {TL_ADDR_WIDTH{1'b0}};
				r_a_size    <= {TL_SIZE_WIDTH{1'b0}};
				r_a_mask    <= {TL_STRB_WIDTH{1'b0}};
				r_a_data    <= {TL_DATA_WIDTH{1'b0}};
				r_a_source  <= {TL_SOURCE_WIDTH{1'b0}};
				r_a_valid   <= 1'b0;
			end
			else begin
				// Keep the previous values if a_valid is low
				r_a_opcode  <= r_a_opcode;
				r_a_param   <= r_a_param;
				r_a_address <= r_a_address;
				r_a_size    <= r_a_size;
				r_a_mask    <= r_a_mask;
				r_a_data    <= r_a_data;
				r_a_source  <= r_a_source;
				r_a_valid   <= 0;
			end
		end
	end
	







	

/*
	
	
mem_block #(
    .TL_DATA_WIDTH(TL_DATA_WIDTH),      // Data width for memory
	.MEM_BASE_ADDR(MEM_BASE_ADDR),         // Base address for memory
    .DEPTH(DEPTH),                		// Memory depth (number of entries)
    .TL_ADDR_WIDTH(TL_ADDR_WIDTH),       // Address width
	.LATENCY(3)                          // Latency for memory access
) DRAM_HBM_memory (
    .clk(clk),                  // Clock input
    .rst(rst),                // Reset input
	.ren(ren),
    .waddr(waddr),              // Write address from slave logic
    .wen(wen),                  // Write enable from slave logic
    .wdata(wdata),              // Write data from slave logic
    .raddr(raddr),              // Read address from slave logic
    .rdata(rdata),              // Read data to be sent back to master
	.mem_write_done(mem_write_done), // Memory write done signal
	.mem_acc_done(mem_acc_done)      // Memory access done signal
);      
      
*/


// FIFO instantiation
// sync_fifo_fwft #(
// 	.DATA_WIDTH(TL_DATA_WIDTH),  // Data width for FIFO
// 	.FIFO_DEPTH(16)               // FIFO depth
// ) fifo_inst (
// 	.clk(clk),
// 	.rst(rst),
// 	.wr_en(fifo_wen),
// 	.rd_en(fifo_ren),
// 	.data_in(fifo_wdata),
// 	.data_out(fifo_rdata),
// 	.full(full),
// 	.empty(empty)
// );

endmodule







