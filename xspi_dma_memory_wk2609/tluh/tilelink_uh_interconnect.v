////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*Interconnect*/
//`include "/home/akshathad/arails/ts40tilelink25a/ts40_p240_1P10M_7X0Y2Z0R0U_0p9_1p5/TL_CrossBar_3M_5S_wk2543/tilelink_ul_decoder.v"
module tilelink_uh_interconnect #(
    parameter NUM_SLAVES    = 3,
    parameter ADDR_WIDTH    = 64,
    parameter DATA_WIDTH    = 64,
    parameter MASK_WIDTH    = DATA_WIDTH / 8,
    parameter SOURCE_WIDTH  = 4,
    parameter PARAM_WIDTH   = 3,
    parameter SIZE_WIDTH    = 3,
    parameter SINK_WIDTH    = 2,
    parameter OPCODE_WIDTH  = 3,
    parameter DEPTH         = 512,
  	parameter TL_ADDR_WIDTH = 64,
    parameter TL_DATA_WIDTH = 64,
    parameter TL_BEAT_WIDTH = 8,
    parameter BEAT_LOG2     = $clog2(TL_BEAT_WIDTH)
)(
    input  wire                         clk,
    input  wire                         rst,
    // Master A
    input  wire                         m_a_valid,
    input  wire [ADDR_WIDTH-1:0]        m_a_address,
    input  wire [DATA_WIDTH-1:0]        m_a_data,
    input  wire [OPCODE_WIDTH-1:0]      m_a_opcode,
    input  wire [PARAM_WIDTH-1:0]       m_a_param,
    input  wire [SIZE_WIDTH-1:0]        m_a_size,
    input  wire [MASK_WIDTH-1:0]        m_a_mask,
    input  wire [SOURCE_WIDTH-1:0]      m_a_source,
    output wire                         m_a_ready,
    // Master D
    output reg  [OPCODE_WIDTH-1:0]      m_d_opcode,
    output reg  [PARAM_WIDTH-1:0]       m_d_param,
    output reg  [SIZE_WIDTH-1:0]        m_d_size,
    output reg  [SINK_WIDTH-1:0]        m_d_sink,
    output reg  [SOURCE_WIDTH-1:0]      m_d_source,
    output reg  [DATA_WIDTH-1:0]        m_d_data,
    output reg                          m_d_error,
    output reg                          m_d_valid,
    input  wire                         m_d_ready,
    // Slave A
    output reg                                s_a_valid,
    output reg  [ADDR_WIDTH-1:0]              s_a_address,
    output reg  [DATA_WIDTH-1:0]              s_a_data,
    output reg  [OPCODE_WIDTH-1:0]            s_a_opcode,
    output reg  [PARAM_WIDTH-1:0]             s_a_param,
    output reg  [SIZE_WIDTH-1:0]              s_a_size,
    output reg  [MASK_WIDTH-1:0]              s_a_mask,
    output reg  [SOURCE_WIDTH-1:0]            s_a_source,
    input  wire                               s_a_ready,
    // Slave D
    input  wire [OPCODE_WIDTH-1:0]            s_d_opcode,
    input  wire [PARAM_WIDTH-1:0]             s_d_param,
    input  wire [SIZE_WIDTH-1:0]              s_d_size,
    input  wire [SINK_WIDTH-1:0]              s_d_sink,
    input  wire [SOURCE_WIDTH-1:0]            s_d_source,
    input  wire [DATA_WIDTH-1:0]              s_d_data,
    input  wire                               s_d_error,
    input  wire                               s_d_valid,
    output wire                               s_d_ready,
    output reg  [$clog2(NUM_SLAVES)-1:0]      selected_slave
);
    wire  [$clog2(NUM_SLAVES)-1:0]     w_selected_slave;
    wire decoder_valid;
    
    // =====================================================================
    // A-CHANNEL LATCH (SKID BUFFER) - Decouples CDC FIFO from FSM
    // =====================================================================
    reg                            a_latch_valid;
    reg [ADDR_WIDTH-1:0]           a_latch_address;
    reg [DATA_WIDTH-1:0]           a_latch_data;
    reg [OPCODE_WIDTH-1:0]         a_latch_opcode;
    reg [PARAM_WIDTH-1:0]          a_latch_param;
    reg [SIZE_WIDTH-1:0]           a_latch_size;
    reg [MASK_WIDTH-1:0]           a_latch_mask;
    reg [SOURCE_WIDTH-1:0]         a_latch_source;
    
    // FSM working registers (loaded from latch)
    reg                            r_m_a_valid;
    reg [ADDR_WIDTH-1:0]           r_m_a_address;
    reg [DATA_WIDTH-1:0]           r_m_a_data;
    reg [OPCODE_WIDTH-1:0]         r_m_a_opcode;
    reg [PARAM_WIDTH-1:0]          r_m_a_param;
    reg [SIZE_WIDTH-1:0]           r_m_a_size;
    reg [MASK_WIDTH-1:0]           r_m_a_mask;
    reg [SOURCE_WIDTH-1:0]         r_m_a_source;
    reg [2:0] interconnect_state;

  	reg [7:0] beat_cnt;
    reg [7:0] total_beats;
    reg       burst_active;
  
    localparam ARBITER_IDLE     = 0;
    localparam ARBITER_SLAVE_SELECT     = 1;    
    localparam ARBITER_REQUEST  = 2;
    localparam ARBITER_RESPONSE = 3;
    localparam ARBITER_DONE     = 4;
    wire state_idle;
    wire state_slave_select;
    wire state_request;
    wire state_response;
    wire state_done;
    assign state_idle           = (interconnect_state == ARBITER_IDLE);
    assign state_slave_select   = (interconnect_state == ARBITER_SLAVE_SELECT);
    assign state_request        = (interconnect_state == ARBITER_REQUEST);
    assign state_response       = (interconnect_state == ARBITER_RESPONSE);
    assign state_done           = (interconnect_state == ARBITER_DONE);
    
    // ✅ CRITICAL FIX: Accept requests when latch is free (not when FSM is idle)
    assign m_a_ready            = !a_latch_valid;
    assign s_d_ready            = state_response;
    
    // FSM consume signal - pulses when FSM starts processing
    wire fsm_accepts_request = (state_idle && a_latch_valid);

    // =====================================================================
    // A-CHANNEL LATCH LOGIC - Load from upstream CDC, clear when consumed
    // =====================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            a_latch_valid   <= 1'b0;
            a_latch_address <= {ADDR_WIDTH{1'b0}};
            a_latch_data    <= {DATA_WIDTH{1'b0}};
            a_latch_opcode  <= {OPCODE_WIDTH{1'b0}};
            a_latch_param   <= {PARAM_WIDTH{1'b0}};
            a_latch_size    <= {SIZE_WIDTH{1'b0}};
            a_latch_mask    <= {MASK_WIDTH{1'b0}};
            a_latch_source  <= {SOURCE_WIDTH{1'b0}};
        end else begin
            // Load from CDC FIFO when latch is empty
            if (!a_latch_valid && m_a_valid) begin
                a_latch_valid   <= 1'b1;
                a_latch_opcode  <= m_a_opcode;
                a_latch_param   <= m_a_param;
                a_latch_size    <= m_a_size;
                a_latch_source  <= m_a_source;
                a_latch_address <= m_a_address;
                a_latch_mask    <= m_a_mask;
                a_latch_data    <= m_a_data;
                $display("[INT-LATCH-LOAD] Time=%0t LOADING: addr=0x%h src=%d opc=%d size=%d", 
                         $time, m_a_address, m_a_source, m_a_opcode, m_a_size);
            end
            // Clear when FSM consumes request (at transaction completion)
            else if (state_done && m_d_ready) begin
                a_latch_valid <= 1'b0;
                $display("[INT-LATCH-CLEAR] Time=%0t CLEARING: was addr=0x%h src=%d", 
                         $time, a_latch_address, a_latch_source);
            end
        end
    end

    // =====================================================================
    // FSM STATE MACHINE
    // =====================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            interconnect_state <= ARBITER_IDLE;
        end else begin
            case (interconnect_state)
                ARBITER_IDLE: begin
                    // Wait for latch to have valid data (not direct m_a_valid)
                    if(a_latch_valid) begin
                        interconnect_state <= ARBITER_SLAVE_SELECT;
                        $display("[INT-FSM] Time=%0t IDLE->SLAVE_SELECT: addr=0x%h src=%d", 
                                 $time, a_latch_address, a_latch_source);
                    end
                    else begin
                        interconnect_state <= ARBITER_IDLE;
                    end
                end

                ARBITER_SLAVE_SELECT: begin
                    if(decoder_valid) begin
                        interconnect_state <= ARBITER_REQUEST;
                        $display("[INT-FSM] Time=%0t SLAVE_SELECT->REQUEST: addr=0x%h src=%d slave=%d", 
                                 $time, r_m_a_address, r_m_a_source, w_selected_slave);
                    end
                    else begin
                        interconnect_state <= ARBITER_SLAVE_SELECT;
                    end
                end

                ARBITER_REQUEST: begin
                    if (s_a_ready) begin
                        if (burst_active) begin
                            if (beat_cnt + 1 == total_beats) begin
                                interconnect_state <= ARBITER_RESPONSE;
                                $display("[INT-FSM] Time=%0t REQUEST->RESPONSE: burst complete, beat=%d/%d", 
                                         $time, beat_cnt+1, total_beats);
                            end else begin
                                interconnect_state <= ARBITER_REQUEST;  // stay here for next beat
                                $display("[INT-FSM] Time=%0t REQUEST (burst continue): beat=%d/%d", 
                                         $time, beat_cnt+1, total_beats);
                            end
                        end else begin
                            interconnect_state <= ARBITER_RESPONSE;
                            $display("[INT-FSM] Time=%0t REQUEST->RESPONSE: single beat complete", $time);
                        end
                    end
                end

                ARBITER_RESPONSE: begin
                        if (s_d_valid) begin
                        interconnect_state <= ARBITER_DONE;
                        $display("[INT-FSM] Time=%0t RESPONSE->DONE: got D-channel, src=%d data=0x%h", 
                                 $time, s_d_source, s_d_data);
                      	end 
                  		else begin
                          interconnect_state <= ARBITER_RESPONSE;
                    end
                end

                ARBITER_DONE: begin
                    if(m_d_ready) begin
                        interconnect_state <= ARBITER_IDLE;
                        $display("[INT-FSM] Time=%0t DONE->IDLE: m_d_ready=1, transaction complete", $time);
                    end else begin
                        $display("[INT-FSM] Time=%0t DONE (waiting): m_d_ready=0", $time);
                    end
                end

                default: begin
                    interconnect_state <= ARBITER_IDLE;
                end
            endcase
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s_a_valid   <= 0;
            s_a_address <= 0;
            s_a_data    <= 0;
            s_a_opcode  <= 0;
            s_a_param   <= 0;
            s_a_size    <= 0;
            s_a_mask    <= 0;
            s_a_source  <= 0;
            selected_slave <= 0;
          	burst_active <= 0;
            beat_cnt <= 0;
            total_beats <= 0;
        end else begin
            if (state_idle) begin
                s_a_valid <= 0; 
                s_a_address <= 0;
                s_a_data    <= 0;
                s_a_opcode  <= 0;
                s_a_param   <= 0;
                s_a_size    <= 0;
                s_a_mask    <= 0;
                s_a_source  <= 0;
                selected_slave <= 0;
              	burst_active <= 0;
                beat_cnt <= 0;           
            end 
            else if (state_slave_select) begin
                if(decoder_valid) begin
                    s_a_valid   <= r_m_a_valid;
                    s_a_address <= r_m_a_address;
                    s_a_data    <= r_m_a_data;
                    s_a_opcode  <= r_m_a_opcode;
                    s_a_param   <= r_m_a_param;
                    s_a_size    <= r_m_a_size;
                    s_a_mask    <= r_m_a_mask;
                    s_a_source  <= r_m_a_source;
                    selected_slave <= w_selected_slave;
                  		if (r_m_a_size > BEAT_LOG2) begin
                            total_beats <= (1 << (r_m_a_size - BEAT_LOG2));
                            beat_cnt <= 0;
                            burst_active <= 1;
                        end else begin
                            total_beats <= 1;
                            beat_cnt <= 0;
                            burst_active <= 0;
                        end
                end
            end
            else if (state_request) begin
                selected_slave <= selected_slave;

                if (burst_active) begin
                    s_a_valid <= 1;

                    // -------------------------
                    // FIXED: proper burst read/write addr increment
                    // -------------------------
                    s_a_address <= r_m_a_address + 
                                   (beat_cnt * (DATA_WIDTH/8));   // FIX

                    // For writes keep data
                    if (r_m_a_opcode == 3'd0)
                        s_a_data <= r_m_a_data + beat_cnt;
                    else
                        s_a_data <= 0;

                    s_a_opcode <= r_m_a_opcode;
                    s_a_param  <= r_m_a_param;
                    s_a_size   <= r_m_a_size;
                    s_a_mask   <= r_m_a_mask;
                    s_a_source <= r_m_a_source;

                    if (s_a_ready) begin
                        beat_cnt <= beat_cnt + 1;

                        if (beat_cnt + 1 == total_beats)
                            burst_active <= 0;
                    end
                end
               else if (!burst_active) begin 
                if(s_a_ready) begin
                    s_a_valid   <= 0;
                    s_a_address <= 0;
                    s_a_data    <= 0;
                    s_a_opcode  <= 0;
                    s_a_param   <= 0;
                    s_a_size    <= 0;
                    s_a_mask    <= 0;
                    s_a_source  <= 0;                    
                end
                else begin
                    s_a_valid   <= s_a_valid;
                    s_a_address <= s_a_address;
                    s_a_data    <= s_a_data;
                    s_a_opcode  <= s_a_opcode;
                    s_a_param   <= s_a_param;
                    s_a_size    <= s_a_size;
                    s_a_mask    <= s_a_mask;
                    s_a_source  <= s_a_source;                    
                end
                end
            end else begin
                s_a_valid   <= 0;
                s_a_address <= 0;
                s_a_data    <= 0;
                s_a_opcode  <= 0;
                s_a_param   <= 0;
                s_a_size    <= 0;
                s_a_mask    <= 0;
                s_a_source  <= 0;
                selected_slave <= selected_slave;
            end
        end
    end

    // D Channel
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            m_d_opcode <= 0;
            m_d_param  <= 0;
            m_d_size   <= 0;
            m_d_sink   <= 0;
            m_d_source <= 0;
            m_d_data   <= 0;
            m_d_error  <= 1'b0;
            m_d_valid  <= 1'b0;                     
        end
        else begin
            if(state_response) begin
                if(s_d_valid) begin
                    m_d_opcode <= s_d_opcode;
                    m_d_param  <= s_d_param;
                    m_d_size   <= s_d_size;
                    m_d_sink   <= s_d_sink;
                    m_d_source <= s_d_source;
                    m_d_data   <= s_d_data;
                    m_d_error  <= s_d_error;
                    m_d_valid  <= s_d_valid;
                end
            end
            else if(state_done) begin
                if(m_d_ready) begin
                    m_d_opcode <= 0;
                    m_d_param  <= 0;
                    m_d_size   <= 0;
                    m_d_sink   <= 0;
                    m_d_source <= 0;
                    m_d_data   <= 0;
                    m_d_error  <= 1'b0;
                    m_d_valid  <= 1'b0;  
                end
                else begin
                    m_d_opcode <= m_d_opcode;
                    m_d_param  <= m_d_param;
                    m_d_size   <= m_d_size;
                    m_d_sink   <= m_d_sink;
                    m_d_source <= m_d_source;
                    m_d_data   <= m_d_data;
                    m_d_error  <= m_d_error;
                    m_d_valid  <= m_d_valid;                    
                end
            end
        end
    end


    always @(posedge clk or posedge rst) begin
        if (rst) begin
            r_m_a_valid   <= 1'b0;
            r_m_a_address <= {ADDR_WIDTH{1'b0}};
            r_m_a_data    <= {DATA_WIDTH{1'b0}};
            r_m_a_opcode  <= {OPCODE_WIDTH{1'b0}};
            r_m_a_param   <= {PARAM_WIDTH{1'b0}};
            r_m_a_size    <= {SIZE_WIDTH{1'b0}};
            r_m_a_mask    <= {MASK_WIDTH{1'b0}};
            r_m_a_source  <= {SOURCE_WIDTH{1'b0}};
        end 
        // ✅ FIX: Load from LATCH only when starting new transaction, not during REQUEST
        else if ((a_latch_valid & state_idle) | state_slave_select) begin
            r_m_a_valid   <= a_latch_valid;
            r_m_a_address <= a_latch_address;
            r_m_a_data    <= a_latch_data;
            r_m_a_opcode  <= a_latch_opcode;
            r_m_a_param   <= a_latch_param;
            r_m_a_size    <= a_latch_size;
            r_m_a_mask    <= a_latch_mask;
            r_m_a_source  <= a_latch_source;
        end
        // During REQUEST state, hold the values (don't reload from latch)
        else if (state_request) begin
            r_m_a_valid   <= r_m_a_valid;
            r_m_a_address <= r_m_a_address;
            r_m_a_data    <= r_m_a_data;
            r_m_a_opcode  <= r_m_a_opcode;
            r_m_a_param   <= r_m_a_param;
            r_m_a_size    <= r_m_a_size;
            r_m_a_mask    <= r_m_a_mask;
            r_m_a_source  <= r_m_a_source;
        end
        else begin
            r_m_a_valid   <= 1'b0;
            r_m_a_address <= {ADDR_WIDTH{1'b0}};
            r_m_a_data    <= {DATA_WIDTH{1'b0}};
            r_m_a_opcode  <= {OPCODE_WIDTH{1'b0}};
            r_m_a_param   <= {PARAM_WIDTH{1'b0}};
            r_m_a_size    <= {SIZE_WIDTH{1'b0}};
            r_m_a_mask    <= {MASK_WIDTH{1'b0}};
            r_m_a_source  <= {SOURCE_WIDTH{1'b0}};        
        end
    end

    tilelink_uh_decoder #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .NUM_SLAVES (NUM_SLAVES),
        .DEPTH      (DEPTH)
    ) u_tilelink_ul_decoder (
        .clk         (clk),
        .rst         (rst),
        .tl_in_a_valid (a_latch_valid),    // ✅ Use latch, not direct input
        .tl_in_addr  (a_latch_address),    // ✅ Use latch, not direct input
        .tl_out      (w_selected_slave),
        .tl_out_valid(decoder_valid)
    );

// DEBUG: Monitor slave D-channel input
always @(posedge clk) begin
    if (s_d_valid) begin
        $display("[INTERCONNECT RX] Time=%0t s_d_valid=%b s_d_ready=%b s_d_opcode=%d s_d_data=0x%h s_d_source=%d state=%d",
                 $time, s_d_valid, s_d_ready, s_d_opcode, s_d_data, s_d_source, interconnect_state);
    end
end

// DEBUG: Monitor master D-channel output
always @(posedge clk) begin
    if (m_d_valid) begin
        $display("[INTERCONNECT TX] Time=%0t m_d_valid=%b m_d_ready=%b m_d_opcode=%d m_d_data=0x%h m_d_source=%d",
                 $time, m_d_valid, m_d_ready, m_d_opcode, m_d_data, m_d_source);
    end
end

// DEBUG: Monitor ready signal changes
reg prev_m_a_ready;
always @(posedge clk) begin
    if (rst) begin
        prev_m_a_ready <= 0;
    end else begin
        prev_m_a_ready <= m_a_ready;
        
        // Print when ready changes
        if (m_a_ready != prev_m_a_ready) begin
            $display("[INT-READY] Time=%0t m_a_ready: %b->%b (latch_valid=%b)", 
                     $time, prev_m_a_ready, m_a_ready, a_latch_valid);
        end
    end
end

endmodule


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

