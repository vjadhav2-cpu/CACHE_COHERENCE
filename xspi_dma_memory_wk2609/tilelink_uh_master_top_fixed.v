`timescale 1ns / 1ps

/*******************************************************************************************************************************************

This top-level module sets up signals for the master instance of the TileLink UH protocol with FIXED atomic operations. 

*******************************************************************************************************************************************/

 
module tilelink_uh_master_top_fixed #(  

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
	parameter MAX_BURST_LENGTH  = 16,             // Maximum burst length in bytes. This is the maximum number of beats in a burst transaction.
	

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

	//////////////////////////////////////////////////////////////////
	/////////////////// Inputs from testbench/DMA ////////////////////
	//////////////////////////////////////////////////////////////////

	input wire 							     a_valid_in, 
	input wire [TL_OPCODE_WIDTH-1:0]         a_opcode_in,
	input wire [TL_PARAM_WIDTH-1:0]          a_param_in,
	input wire [TL_ADDR_WIDTH-1:0]          a_address_in,
	input wire [TL_SIZE_WIDTH-1:0]          a_size_in,
	input wire [TL_STRB_WIDTH-1:0]          a_mask_in,
	input wire [TL_DATA_WIDTH-1:0]          a_data_in,
	input wire [TL_SOURCE_WIDTH-1:0]        a_source_in,
	input wire                              a_corrupt_in, 

	// A Channel: Received from MASTER
	input  wire                              a_ready,		// Slave sends a_ready to Master to indicate that it is ready to accept data.
	output reg                              a_valid, 		// Asserted to indicate valid instruction
	output reg [TL_OPCODE_WIDTH-1:0]        a_opcode,		// Opcode for instruction
	output reg [TL_PARAM_WIDTH-1:0]         a_param,		// Reserved, always 0.
	output reg [TL_ADDR_WIDTH-1:0]          a_address,	    // Address 
	output reg [TL_SIZE_WIDTH-1:0]          a_size,		// Width of full data sent in one go = 2^size. For TLUL, size = Data Width of Channel.
	output reg [TL_STRB_WIDTH-1:0]          a_mask,		// Bit masking
	output reg [TL_DATA_WIDTH-1:0]          a_data,		// Incoming data
	output reg [TL_SOURCE_WIDTH-1:0]        a_source,		// Transaction ID
	output reg                              a_corrupt,		// Corruption flag. If set, the transaction is corrupted and must be ignored.

	// D Channel Sent to MASTER
	input  wire                               d_valid,
	output wire                               d_ready, 		// Master ends d_ready to Slave to indicate that it is ready to accept data.
	input  wire [TL_OPCODE_WIDTH-1:0]         d_opcode,
	input  wire [TL_PARAM_WIDTH-1:0]          d_param,
	input  wire [TL_SIZE_WIDTH-1:0]           d_size,
	input  wire [TL_SINK_WIDTH-1:0]           d_sink,
	input  wire [TL_SOURCE_WIDTH-1:0]         d_source,	
	input  wire [TL_DATA_WIDTH-1:0]           d_data,
	input  wire                               d_error
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

// Master state (6 states, need 3 bits)
localparam IDLE          = 3'b000;
localparam REQUEST       = 3'b001;
localparam BURST_READ    = 3'b010;
localparam BURST_WRITE   = 3'b011;
localparam ATOMIC_INST   = 3'b100;
localparam RESPONSE      = 3'b101;

reg [2:0] master_state;

// Burst write FSM states (2 states, 1 bit)
localparam BURST_WRITE_PENDING = 1'b0;
localparam BURST_WRITE_DONE    = 1'b1;

reg burst_write_state;

// Response FSM state (2 states, 1 bit)
localparam RESPONSE_PENDING = 1'b0;
localparam RESPONSE_DONE    = 1'b1;

reg response_state;

// Burst read response FSM states (3 states, need 2 bits)
localparam BURST_READ_REQUEST   = 2'b00;
localparam BURST_READ_RESPONSE  = 2'b01;
localparam BURST_READ_DONE      = 2'b10;

reg [1:0] burst_read_response_state;

// Atomic Operation FSM states (3 states, need 2 bits)
localparam ATOMIC_REQUEST  = 2'b00;
localparam ATOMIC_RESPONSE = 2'b01;
localparam ATOMIC_DONE     = 2'b10;

reg [1:0] atomic_state;


	// State flags for master_state
	wire in_idle;
	wire in_request;
	wire in_burst_write;
	wire in_burst_read;
	wire in_atomic_inst;
	wire in_response;

	// State flags for response_state
	wire in_mem_access_pending;
	wire in_response_pending;
	wire in_response_done;

	// FIFO flags
	//wire full;
	//wire empty;

	// Assigning state flags based on response_state
//	assign in_mem_access_pending = (response_state == MEM_ACCESS_PENDING);
	assign in_response_pending = (response_state == RESPONSE_PENDING);
	assign in_response_done = (response_state == RESPONSE_DONE);

	// Assigning state flags based on master_state
	assign in_idle = (master_state == IDLE);
	assign in_request = (master_state == REQUEST);
	assign in_burst_write = (master_state == BURST_WRITE);
	assign in_burst_read = (master_state == BURST_READ);
	assign in_atomic_inst = (master_state == ATOMIC_INST);
	assign in_response = (master_state == RESPONSE);

	// d_ready signal - using OR logic to prevent X propagation
	// Accept responses in: RESPONSE state, BURST_READ_RESPONSE state, ATOMIC_RESPONSE state  
	// CRITICAL FIX: Only accept data when source ID matches current transaction
	// This prevents stale data from different transactions from appearing on d_data
	assign d_ready = ((response_state == RESPONSE_PENDING && master_state == RESPONSE) && 
	                  (!d_valid || d_source == r_a_source)) || 
	                 ((burst_read_response_state == BURST_READ_RESPONSE) && 
	                  (!d_valid || d_source == r_a_source)) || 
	                 ((atomic_state == ATOMIC_RESPONSE) && 
	                  (!d_valid || d_source == r_a_source)) ||
	                 ((master_state == IDLE && single_beat_read_active) && 
	                  (!d_valid || d_source == last_a_source)) ||
	                 ((master_state == IDLE && single_beat_write_active) && 
	                  (!d_valid || d_source == last_a_source)) ||
	                 ((master_state == IDLE && burst_read_active) && 
	                  (!d_valid || d_source == last_a_source));

	// Registers

	reg [$clog2(MAX_BURST_LENGTH)-1:0] burst_write_counter, burst_write_buf_ptr, burst_write_buf_counter;
	reg [TL_DATA_WIDTH-1:0] burst_write_data_buf [MAX_BURST_LENGTH-1:0];

	// Memory Flags
	
	reg  [TL_ADDR_WIDTH-1:0] waddr;
	reg              	   wen,ren;
	reg  [TL_DATA_WIDTH-1:0] wdata;
	reg  [TL_ADDR_WIDTH-1:0] raddr;
	//wire [TL_DATA_WIDTH-1:0] rdata;
	//reg  [TL_DATA_WIDTH-1:0] fifo_wdata;
	//reg  [TL_DATA_WIDTH-1:0] fifo_rdata;
	//reg  [TL_SIZE_WIDTH-1:0] mem_write_counter;
	reg mem_enable;
	reg response_pending;
	//wire mem_acc_done;
	//wire mem_write_done;
	reg response_done;

	// FIFO controls
	//reg fifo_wen, fifo_ren;

	// Burst and single beat write/read flags
	reg burst_write_active;
	reg burst_read_active;
	reg single_beat_write_active;
	reg single_beat_read_active;
	reg [TL_SOURCE_WIDTH-1:0] single_beat_read_source; // Track source ID for single-beat reads
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
	
	// FIXED: Re-enabled atomic operand storage in master
	reg [TL_DATA_WIDTH-1:0] unsigned_atomic_operand [1:0];
	reg signed [TL_DATA_WIDTH-1:0] signed_atomic_operand [1:0];


	// For loop variables
	integer i;
	
	// Registers for A Channel (Slave side input)
	// reg                             r_a_ready;
	reg                             r_a_valid;
	reg                             r_a_corrupt;
	reg [TL_OPCODE_WIDTH-1:0]       r_a_opcode;
	reg [TL_PARAM_WIDTH-1:0]        r_a_param;
	reg [TL_ADDR_WIDTH-1:0]         r_a_address;
	reg [TL_SIZE_WIDTH-1:0]         r_a_size;
	reg [TL_STRB_WIDTH-1:0]         r_a_mask;
	reg [TL_DATA_WIDTH-1:0]         r_a_data;
	reg [TL_SOURCE_WIDTH-1:0]       r_a_source;
	reg [TL_SOURCE_WIDTH-1:0]       last_a_source; // Track last transaction source for stale data filtering

	// Global FSM -> master_state
	// The master_state is used to determine the current state of the slave FSM.

	// RESET D CHANNEL TASK


	task a_channel_request_passthrough (input [TL_DATA_WIDTH-1:0] data);
	begin
		a_valid  <= a_valid_in;
		a_opcode <= a_opcode_in;
		a_param  <= a_param_in;
		a_address <= a_address_in;
		a_size   <= a_size_in;
		a_mask   <= a_mask_in;
		a_data   <= data;
		a_source <= a_source_in;
		a_corrupt <= a_corrupt_in; // Corruption flag is set to 0
	end
	endtask

	task a_channel_request (input [TL_DATA_WIDTH-1:0] data);
	begin
		a_valid  <= r_a_valid;
		a_opcode <= r_a_opcode;
		a_param  <= r_a_param;
		a_address <= r_a_address;
		a_size   <= r_a_size;
		a_mask   <= r_a_mask;
		a_data   <= data;
		a_source <= r_a_source;
		a_corrupt <= r_a_corrupt; // Corruption flag is set to 0
	end
	endtask	

	task hold_a_channel();
	begin
		a_valid  <= a_valid;
		a_opcode <= a_opcode;
		a_param  <= a_param;
		a_address <= a_address;
		a_size   <= a_size;
		a_mask   <= a_mask;
		a_data   <= a_data;
		a_source <= a_source;
		a_corrupt <= a_corrupt; // Corruption flag is set to 0
	end
	endtask

	task reset_a_channel();
	begin
		a_valid  <= 1'b0;
		a_opcode <= {TL_OPCODE_WIDTH{1'b0}};
		a_param  <= {TL_PARAM_WIDTH{1'b0}};
		a_address <= {TL_ADDR_WIDTH{1'b0}};
		a_size   <= {TL_SIZE_WIDTH{1'b0}};
		a_mask   <= {TL_STRB_WIDTH{1'b0}};
		a_data   <= {TL_DATA_WIDTH{1'b0}};
		a_source <= {TL_SOURCE_WIDTH{1'b0}};
		a_corrupt <= 1'b0;
	end
	endtask

	always @(posedge clk or posedge rst) begin
		if (rst) begin
			num_beats <= {TL_SIZE_WIDTH{1'b0}};
		end else begin
			if (a_valid_in) begin
				num_beats <= ({24'b0, a_size_in} > BEAT_LOG2) ? (1 << ({24'b0, a_size_in} - BEAT_LOG2)) : 1;
			end
			else if (!response_done | !burst_read_done) begin // If response is not done, keep the previous value
				num_beats <= num_beats; // Keep the previous value if a_valid is low
			end
			else begin
				num_beats <= {TL_SIZE_WIDTH{1'b0}}; // Reset num_beats when response is done
			end
		end
	end
	

	
	/////////////////////////////////////////////////////////////
	//////////// 		      FSM BLOCK    	    	 ////////////
	/////////////////////////////////////////////////////////////


	always @(posedge clk or posedge rst) begin
		if (rst) begin
			master_state <= IDLE; // Reset to REQUEST state on reset
		end else begin
			// State transitions are handled in the FSM block below
			case (master_state)
				IDLE: begin
					if(a_valid_in) begin
						$display("[IDLE->STATE] Time=%0t a_valid_in=1 a_opcode_in=%d a_size_in=%d BEAT_LOG2=%d", 
						          $time, a_opcode_in, a_size_in, BEAT_LOG2);
						case (a_opcode_in)
							PUT_FULL_DATA_A: begin
								if({24'b0, a_size_in} <= BEAT_LOG2) begin
									// Single beat write operation
									master_state <= REQUEST; 
									$display("[IDLE->STATE] Time=%0t Transitioning to REQUEST (single write)", $time);
								end else begin
									// Burst write operation
									master_state <= BURST_WRITE; 
									$display("[IDLE->STATE] Time=%0t Transitioning to BURST_WRITE", $time);
								end
							end 
							GET_A: begin
								if({24'b0, a_size_in} <= BEAT_LOG2) begin
									// Single beat read operation
									master_state <= REQUEST; 
									$display("[IDLE->STATE] Time=%0t Transitioning to REQUEST (single read)", $time);
								end else begin
									// Burst read operation
									master_state <= BURST_READ; 
									$display("[IDLE->STATE] Time=%0t Transitioning to BURST_READ", $time);
								end								
							end
							ARITHMETIC_DATA_A: begin
								master_state <= ATOMIC_INST;
							end
							LOGICAL_DATA_A: begin
								master_state <= ATOMIC_INST;
							end
							default: master_state <= IDLE; // Stay in IDLE state for unsupported opcodes
						endcase
					end
					else begin
						master_state <= IDLE;
					end
				end

				////////////////////////////////////////////////////////////////////////////
				////////////////////////// SINGLE BEAT REQUESTS ////////////////////////////
				////////////////////////////////////////////////////////////////////////////

				REQUEST: begin
					if(a_ready) begin
						master_state <= RESPONSE;
					end
					else begin
						master_state <= REQUEST;
					end
				end 

				////////////////////////////////////////////////////////////////////////////
				////////////////////////// BURST WRITE REQUESTS ////////////////////////////
				////////////////////////////////////////////////////////////////////////////

				BURST_WRITE: begin
					// Handle burst write operations
					if (burst_write_done) begin 
						master_state <= RESPONSE; 
					end
					else begin
						master_state <= BURST_WRITE;
					end
				end

				////////////////////////////////////////////////////////////////////////////
				////////////////////////// BURST READ REQUESTS /////////////////////////////
				////////////////////////////////////////////////////////////////////////////

				BURST_READ: begin
					// Handle burst read operations
					if (burst_read_done) begin
						master_state <= IDLE;
					end else begin
						master_state <= BURST_READ;
					end
				end

				////////////////////////////////////////////////////////////////////////////
				////////////////////////// ATOMIC INST REQUESTS ////////////////////////////
				////////////////////////////////////////////////////////////////////////////

				ATOMIC_INST: begin
					// Handle atomic burst operations
					if(atomic_inst_done) begin
						master_state <= IDLE; // If atomic instruction is done, go to RESPONSE state
					end
					else begin
						master_state <= ATOMIC_INST; // Stay in ATOMIC_INST state until atomic instruction is done
					end
				end

				////////////////////////////////////////////////////////////////////////////
				////////////////////////// SINGLE BEAT RESPONSE ////////////////////////////
				////////////////////////////////////////////////////////////////////////////

				// RESPONSE: This state is used to send responses back to the master
				RESPONSE: begin
					if (response_done) begin
						master_state <= IDLE; 
					end else begin
						master_state <= RESPONSE; 
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
			atomic_state <= ATOMIC_REQUEST; // Reset to ATOMIC_READ state
			burst_write_counter <= 0;
			response_state <= RESPONSE_PENDING; // Reset to RESPONSE state
			burst_read_response_state <= BURST_READ_REQUEST; // Reset to BURST_READ state

			burst_write_buf_ptr <= 0; 
			burst_write_buf_counter <= 0; 
			// Reset burst write data buffer
			for (i = 0; i < $clog2(TL_SIZE_WIDTH); i = i + 1) begin
				burst_write_data_buf[i] <= {TL_DATA_WIDTH{1'b0}}; // Reset each entry of burst write data buffer
			end
			
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
		single_beat_read_source <= {TL_SOURCE_WIDTH{1'b0}}; // Reset source ID tracker  
		burst_write_active <= 1'b0; // Reset burst write active
		burst_read_active <= 1'b0; // Reset burst read active
			burst_read_counter <= {TL_SIZE_WIDTH{1'b0}};

		end else begin
			
			if(in_idle) begin
				// Reset other signals
				if(a_valid_in) begin
					$display("[MASTER-IDLE] Time=%0t a_valid_in=1 a_ready=%b opcode=%0d src=%0d", 
						         $time, a_ready, a_opcode_in, a_source_in);
					// FIXED: Properly capture atomic operands in master
					if(a_opcode_in == ARITHMETIC_DATA_A || a_opcode_in == LOGICAL_DATA_A) begin
						// Store the atomic operand
						if(a_opcode_in == ARITHMETIC_DATA_A && a_param_in < 2) begin
							signed_atomic_operand[0] <= a_data_in;
						end
						else begin
							unsigned_atomic_operand[0] <= a_data_in;
						end
					end
					
					a_channel_request_passthrough(a_data_in); // Request A channel with incoming data
					if(a_opcode_in == PUT_FULL_DATA_A && {24'b0, a_size_in} > BEAT_LOG2) begin
						// Indicates a burst write sequence. The below counter tracks the number of beats.
						burst_write_counter <= burst_write_counter + 1;
					end				
				// Set single_beat_write_active for single-beat writes to handle late acknowledgements
				if((a_opcode_in == PUT_FULL_DATA_A || a_opcode_in == PUT_PARTIAL_DATA_A) && {24'b0, a_size_in} <= BEAT_LOG2) begin
					single_beat_write_active <= 1'b1;
					$display("[DEBUG] Time=%0t Setting single_beat_write_active=1 for SINGLE-BEAT write", $time);
				end				// Note: Don't pre-increment burst_read_counter for GET_A
				// Counter will increment when beats are actually received
				end
				else begin
					reset_a_channel();
				end
				
				// Clear single_beat_read_active when handshake completes in IDLE (late-arriving response)
				// Only clear if this is the read response for our transaction
				if(d_valid && d_ready && single_beat_read_active && d_opcode == ACCESS_ACK_DATA_D && d_source == single_beat_read_source) begin
					single_beat_read_active <= 1'b0;
					single_beat_read_source <= {TL_SOURCE_WIDTH{1'b0}}; // Clear source tracker
					$display("[DEBUG] Time=%0t Clearing single_beat_read_active after IDLE handshake d_source=%d expected=%d", $time, d_source, single_beat_read_source);
				end			
			// Clear single_beat_write_active when write acknowledgement arrives in IDLE
			if(d_valid && d_ready && single_beat_write_active && d_opcode == ACCESS_ACK_D) begin
				single_beat_write_active <= 1'b0;
				$display("[DEBUG] Time=%0t Clearing single_beat_write_active after IDLE write ack", $time);
			end			end
			
			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// SINGLE BEAT REQUEST ///////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////
			
			else if (in_request) begin
				// Set single_beat_read_active ONLY for single-beat read operations (not bursts)
				// Burst reads have r_a_size > BEAT_LOG2 (3), single beats have r_a_size == BEAT_LOG2
				$display("[DEBUG REQUEST] Time=%0t r_a_opcode=%d GET_A=%d r_a_size=%d BEAT_LOG2=%d master_state=%d sbrd_act=%b r_a_source=%d", 
						 $time, r_a_opcode, GET_A, r_a_size, BEAT_LOG2, master_state, single_beat_read_active, r_a_source);
				if(r_a_opcode == GET_A && r_a_size == BEAT_LOG2) begin
					single_beat_read_active <= 1'b1;
					single_beat_read_source <= r_a_source; // Store source ID for later comparison
					$display("[DEBUG] Time=%0t Setting single_beat_read_active=1 for SINGLE-BEAT read, source=%d", $time, r_a_source);
				end
				
				burst_write_counter <= 0;
				burst_write_buf_ptr <= 0;
				burst_write_buf_counter <= 0;
			end
	///////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////// BURST WRITE REQUEST ///////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////	

	else if (in_burst_write) begin
				// Handle burst write interception
				case (burst_write_state)
					BURST_WRITE_PENDING: begin
						
						// Logic needs to be changed. You may have everyting stored in the
						// buffer, and a_valid_in may not be high anymore
						
						$display("[MASTER BURST DEBUG] Time=%0t counter=%0d<%0d buf_ptr=%0d<%0d a_valid_in=%b", 
						         $time, burst_write_counter, num_beats, burst_write_buf_ptr, burst_write_buf_counter, a_valid_in);

						if({4'b0, burst_write_counter} < num_beats) begin
							// Always try to send buffered data first if available
							if (burst_write_buf_ptr < burst_write_buf_counter) begin
								// Send from buffer
								a_valid  <= 1'b1;  
								a_opcode <= r_a_opcode;
								a_param  <= r_a_param;
								a_address <= r_a_address;
								a_size   <= r_a_size;
								a_mask   <= r_a_mask;
								a_data   <= burst_write_data_buf[burst_write_buf_ptr];
								a_source <= r_a_source;
								a_corrupt <= r_a_corrupt;
								
								if(a_ready) begin
									burst_write_buf_ptr <= burst_write_buf_ptr + 1;
									burst_write_counter <= burst_write_counter + 1;
									$display("[MASTER BURST] Time=%0t Sent buffered beat %0d data=0x%h", 
									         $time, burst_write_counter+1, burst_write_data_buf[burst_write_buf_ptr]);
									
									// Buffer new incoming data AFTER successful send
									if(a_valid_in) begin
										burst_write_data_buf[burst_write_buf_counter] <= a_data_in;
										burst_write_buf_counter <= burst_write_buf_counter + 1;
										$display("[MASTER BURST] Time=%0t Buffered data[%0d] = 0x%h AFTER send", 
										         $time, burst_write_buf_counter, a_data_in);
									end
								end else begin
									$display("[MASTER BURST] Time=%0t Waiting for a_ready for buffered beat %0d", 
									         $time, burst_write_counter+1);
								end
							end
							else if(a_valid_in) begin
								// No buffered data, send incoming data directly
								a_valid  <= 1'b1;
								a_opcode <= r_a_opcode;
								a_param  <= r_a_param;
								a_address <= r_a_address;
								a_size   <= r_a_size;
								a_mask   <= r_a_mask;
								a_data   <= a_data_in;
								a_source <= r_a_source;
								a_corrupt <= r_a_corrupt;
								
								if(a_ready) begin
									burst_write_counter <= burst_write_counter + 1;
									$display("[MASTER BURST] Time=%0t Sent direct beat %0d data=0x%h", 
									         $time, burst_write_counter+1, a_data_in);
								end else begin
									// Buffer the data for next cycle since not ready
									burst_write_data_buf[burst_write_buf_counter] <= a_data_in;
									burst_write_buf_counter <= burst_write_buf_counter + 1;
									$display("[MASTER BURST] Time=%0t Buffered data[%0d] = 0x%h (not ready)", 
									         $time, burst_write_buf_counter, a_data_in);
								end
							end
							else begin
								// No data available - wait
								reset_a_channel();
								$display("[MASTER BURST] Time=%0t No data available, waiting...", $time);
							end
						end
						else begin
							// If burst write is done, reset the burst write state
							burst_write_state <= BURST_WRITE_DONE; 
							burst_write_active <= 0; // Reset burst write active flag
							burst_write_done <= 1; // Set burst write done flag
							reset_a_channel(); // Reset A channel signals
							$display("[MASTER BURST] Time=%0t Burst write completed! %0d beats sent", 
							         $time, burst_write_counter);
						end
					end 
					BURST_WRITE_DONE: begin
						burst_write_state <= BURST_WRITE_PENDING; 
						reset_a_channel();
						burst_write_active <= 0; 
						burst_write_done <= 0; 
					end
					default: begin
					
					end
				endcase

			end

			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// BURST READ REQUEST ////////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////	


			else if (in_burst_read) begin
				// Handle burst read interception
				$display("[MASTER BURST_READ] Time=%0t burst_rd_state=%d d_valid=%b d_ready=%b counter=%d num_beats=%d burst_rd_active=%b",
				         $time, burst_read_response_state, d_valid, d_ready, burst_read_counter, num_beats, burst_read_active);

				// FSM for burst read
				case (burst_read_response_state)
					BURST_READ_REQUEST: begin
						burst_read_active <= 1; 
						burst_read_done <= 0;
						burst_read_counter <= 0;  // Reset counter when starting new burst read
						$display("[MASTER BURST_READ_REQ] Time=%0t Waiting for a_ready=%b", $time, a_ready);
						if(a_ready) begin
							burst_read_response_state <= BURST_READ_RESPONSE; 
							reset_a_channel(); // Reset A channel signals
							$display("[MASTER BURST_READ_REQ] Time=%0t Transitioning to BURST_READ_RESPONSE", $time);
						end
						else begin
							hold_a_channel(); // Hold A channel if not ready
						end
end 
					BURST_READ_RESPONSE: begin
					// Check if all beats received first (unconditional)
					if ({4'b0, burst_read_counter} >= num_beats) begin
					// All beats received, transition to DONE
					$display("[MASTER BURST_READ_RESP] Time=%0t All beats received! counter=%d >= num_beats=%d", 
					         $time, burst_read_counter, num_beats);
					burst_read_response_state <= BURST_READ_DONE;
					burst_read_active <= 0;
					burst_read_done <= 1;
					end
					else if (d_valid && d_ready && d_source == r_a_source) begin
					// Handshake successful with correct source ID, increment counter
					$display("[MASTER BURST_READ_RESP] Time=%0t **SUCCESS** d_data=0x%h d_source=%d counter %d->%d", 
					         $time, d_data, d_source, burst_read_counter, burst_read_counter+1);
					burst_read_counter <= burst_read_counter + 1;
					end
					else begin
					// Maintain the state - debug why handshake failed
					if (d_valid && !d_ready) begin
						$display("[MASTER BURST_READ_RESP] Time=%0t **WAIT** d_ready=0", $time);
					end else if (!d_valid) begin
						$display("[MASTER BURST_READ_RESP] Time=%0t **NO_DATA** d_valid=0", $time);
					end else if (d_source != r_a_source) begin
						$display("[MASTER BURST_READ_RESP] Time=%0t **WRONG_SRC** d_source=%d != expected=%d", 
						         $time, d_source, r_a_source);
					end
					burst_read_response_state <= BURST_READ_RESPONSE; 
					burst_read_done <= 0;
					burst_read_active <= 1; 
					burst_read_counter <= burst_read_counter; 
					end
					
end
					BURST_READ_DONE: begin
						burst_read_response_state <= BURST_READ_REQUEST;
						burst_read_active <= 0;
						burst_read_done <= 0;
						burst_read_counter <= 0;
					end
					default: begin
					
					end
				endcase
			end

			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// ATOMIC INST REQUEST ///////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////	


			else if (in_atomic_inst) begin
				// FIXED: Enhanced atomic instruction handling in master
				case (atomic_state)
					ATOMIC_REQUEST: begin
						atomic_inst_active <= 1; 
						atomic_inst_done <= 0;
						if(a_ready) begin
							atomic_state <= ATOMIC_RESPONSE;
							reset_a_channel();
						end
						else begin
							hold_a_channel(); 
						end
					end 
					ATOMIC_RESPONSE: begin
						if(d_valid) begin
							atomic_state <= ATOMIC_DONE; 
							atomic_inst_done <= 1;
							atomic_inst_active <= 0;
							
							// FIXED: Store the returned original value for debugging/verification
							if(r_a_opcode == ARITHMETIC_DATA_A && r_a_param < 2) begin
								signed_atomic_operand[1] <= d_data; // Store returned original value
							end
							else begin
								unsigned_atomic_operand[1] <= d_data; // Store returned original value
							end
						end
						else begin
							atomic_state <= ATOMIC_RESPONSE;
							atomic_inst_done <= 0;
							atomic_inst_active <= 1;							
						end
					end
					ATOMIC_DONE: begin
						// Reset atomic state
						atomic_state <= ATOMIC_REQUEST; 
						atomic_inst_active <= 0; 
						atomic_inst_done <= 0; 

						// Reset A channel signals
						reset_a_channel();
					end
					default: begin end
				endcase
			end

			///////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////// SINGLE BEAT RESPONSE //////////////////////////////////
			///////////////////////////////////////////////////////////////////////////////////////	

			else if (in_response) begin
				// This else-if block handles single cycle responses.
				case (response_state)
					RESPONSE_PENDING: begin
						if(d_valid) begin
							// If d_valid is high, send response
							response_done <= 1'b1;
							response_pending <= 1'b0;
							response_state <= RESPONSE_DONE;
							// Clear single_beat_read_active ONLY when THIS transaction's read response completes
							// Check: d_opcode is ACCESS_ACK_DATA (read response), d_source matches, and handshake completes
							if(single_beat_read_active && d_ready && d_opcode == ACCESS_ACK_DATA_D && d_source == single_beat_read_source) begin
								single_beat_read_active <= 1'b0;
								single_beat_read_source <= {TL_SOURCE_WIDTH{1'b0}}; // Clear source tracker
								$display("[DEBUG] Time=%0t Clearing single_beat_read_active after RESPONSE handshake d_source=%d expected=%d", $time, d_source, single_beat_read_source);
							end
						end
						else begin
							response_pending <= 1'b1;
							response_done <= 1'b0; 
							response_state <= RESPONSE_PENDING; 
						end
					end
					RESPONSE_DONE: begin
						response_done <= 1'b0; 
						response_pending <= 1'b0; 
						response_state <= RESPONSE_PENDING;
					end
					default: begin end
				endcase
			end
			else begin
				// Default case, do nothing or reset signals
				// reset_d_channel();
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
				// Note: Don't reset single_beat_read_active here - it persists until response received
				burst_write_active <= 1'b0; // Reset burst write active
				burst_read_active <= 1'b0; // Reset burst read active
				
				burst_read_counter <= {TL_SIZE_WIDTH{1'b0}}; // Reset burst read counter
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
			last_a_source <= {TL_SOURCE_WIDTH{1'b0}};
			r_a_valid   <= 1'b0;
			r_a_corrupt <= 1'b0; // Corruption flag is set to 0

			
		end
		else begin
		    // d_ready_reg <= d_ready;
			if (a_valid_in) begin
				r_a_opcode  <= a_opcode_in;
				r_a_param   <= a_param_in;
				r_a_address <= a_address_in;
				r_a_size    <= a_size_in;
				r_a_mask    <= a_mask_in;
				r_a_data    <= a_data_in;
				r_a_source  <= a_source_in;
				last_a_source <= a_source_in; // Track this transaction's source for filtering
				r_a_valid   <= a_valid_in;
				r_a_corrupt <= a_corrupt_in; // Corruption flag is set to 0
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
				r_a_corrupt <= 1'b0; // Corruption flag is set to 0
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
				r_a_valid   <= r_a_valid;
				r_a_corrupt <= r_a_corrupt; // Corruption flag is set to 0
			end
		end
	end
	








// DEBUG: Monitor D-channel received by master
always @(posedge clk) begin
    if (d_valid) begin
        $display("[MASTER RX] Time=%0t d_valid=%b d_ready=%b d_opcode=%d d_data=0x%h d_source=%d master_state=%d sbrd_act=%b",
                 $time, d_valid, d_ready, d_opcode, d_data, d_source, master_state, single_beat_read_active);
    end
end

// Track single_beat_read_active at specific times
always @(posedge clk) begin
    if (single_beat_read_active == 1'b1) begin
        $display("[SBRD_ACTIVE] Time=%0t master_state=%d", $time, master_state);
    end
end

endmodule
