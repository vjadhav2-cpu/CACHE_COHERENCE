`timescale 1ns/1ps
`default_nettype none

//! @file xSPI.v
//! @brief Top-level and submodules for a simple SPI-like protocol (xSPI) with controller and slave.

/**
 * @module xspi_top
 * @brief Top-level module connecting xSPI controller and slave via a shared IO bus.
 * @param clk     System clock
 * @param rst_n   Active-low reset
 * @param start   Start transaction
 * @param command 8-bit command
 * @param address 48-bit address
 * @param wr_data 64-bit data to write
 * @param rd_data 64-bit data read
 * @param done    Transaction done flag
 * @param ready   Slave ready flag
 */



module xspi_top #(
    parameter USE_EXTERNAL_SPI_MEM = 1
) (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset
    input  wire        start,    //!< Start transaction
    input  wire [7:0]  command,  //!< 8-bit command
    input  wire [47:0] address,  //!< 48-bit address
    input  wire [63:0] wr_data,  //!< 64-bit data to write
    output wire [63:0] rd_data,  //!< 64-bit data read
    output wire        done,     //!< Transaction done flag
    output wire        ready,    //!< Slave ready flag
    output wire        spi_wr_evt_valid,
    output wire [47:0] spi_wr_evt_addr,
    output wire [63:0] spi_wr_evt_data,
    output wire        spi_rd_req_valid,
    output wire [47:0] spi_rd_req_addr,
    input  wire        spi_rd_rsp_valid,
    input  wire [63:0] spi_rd_rsp_data
);

    // Internal bus and control signals
    wire        cs_n;           //!< Chip select (active low)
    wire        sck;            //!< SPI clock
    wire [7:0]  master_io_out;  //!< Controller output to bus
    wire [7:0]  slave_io_out;   //!< Slave output to bus
    wire        master_io_oe;   //!< Controller output enable
    wire        slave_io_oe;    //!< Slave output enable
    wire        slave_rd_data_valid; //!< Slave has valid read payload loaded
    wire        master_tx_byte_valid; //!< Master indicates stable byte on shared bus
    wire [7:0]  io_bus;         //!< Shared IO bus

    // Simplified IO bus - no tri-state logic
    assign io_bus = master_io_oe ? master_io_out : 
                    slave_io_oe  ? slave_io_out  : 8'h00;

    /**
     * @brief xSPI controller (master)
     */
    xspi_sopi_controller master (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(cs_n),
        .sck(sck),
        .io_out(master_io_out),
        .io_in(io_bus),
        .io_oe(master_io_oe),
        .tx_byte_valid(master_tx_byte_valid),
        .rd_data_valid(slave_rd_data_valid),
        .start(start),
        //.rw(rw),
        .command(command),
        .address(address),
        .wr_data(wr_data),
        .rd_data(rd_data),
        .done(done)
    );

    /**
     * @brief xSPI slave
     */
    xspi_sopi_slave #(
        .USE_EXTERNAL_SPI_MEM(USE_EXTERNAL_SPI_MEM)
    ) slave (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(cs_n),
        .sck(sck),
        .io_out(slave_io_out),
        .io_in(io_bus),
        .io_oe(slave_io_oe),
        .tx_byte_valid(master_tx_byte_valid),
        .master_start(start),
        .master_command(command),
        .master_address(address),
        .master_wr_data(wr_data),
        .rd_data_valid(slave_rd_data_valid),
        .ready(ready),
        .spi_wr_evt_valid(spi_wr_evt_valid),
        .spi_wr_evt_addr(spi_wr_evt_addr),
        .spi_wr_evt_data(spi_wr_evt_data),
        .spi_rd_req_valid(spi_rd_req_valid),
        .spi_rd_req_addr(spi_rd_req_addr),
        .spi_rd_rsp_valid(spi_rd_rsp_valid),
        .spi_rd_rsp_data(spi_rd_rsp_data)
    );

endmodule



//======================================================================
// MODULE 5: top_tlul_spi 
//======================================================================
`timescale 1ns/1ps
`default_nettype none


module top_tlul_spi #(
    parameter USE_EXTERNAL_SPI_MEM = 1
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         enable,
    input  wire [2:0]   a_opcode,
    input  wire [63:0]  a_address,
    input  wire [63:0]  a_data,
    output wire         a_ready,
    output wire         valid,
    output wire [63:0]  d_data,
    output wire         spi_wr_evt_valid,
    output wire [47:0]  spi_wr_evt_addr,
    output wire [63:0]  spi_wr_evt_data,
    output wire         spi_rd_req_valid,
    output wire [47:0]  spi_rd_req_addr,
    input  wire         spi_rd_rsp_valid,
    input  wire [63:0]  spi_rd_rsp_data
);

    wire        spi_start;
    wire [7:0]  spi_cmd;
    wire [47:0] spi_addr;
    wire [63:0] spi_data;
    wire        spi_done;
    wire        data_end;
  wire [63:0] spi_resp;

    tlul_spi_interconnect interconnect_inst (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .a_ready(a_ready),
        .a_opcode(a_opcode),
        .a_address(a_address),
        .a_data(a_data),
        .valid(valid),
        .d_data(d_data),
        .spi_start(spi_start),
        .spi_cmd(spi_cmd),
        .spi_addr(spi_addr),
        .spi_data(spi_data),
        .spi_done(spi_done), //
        .data_end(data_end), //
        .spi_resp(spi_resp) //
    );
    xspi_top #(
        .USE_EXTERNAL_SPI_MEM(USE_EXTERNAL_SPI_MEM)
    ) xspi_inst (
        .clk(clk),
        .rst_n(!rst),
        .start(spi_start),
        .command(spi_cmd),
        .address(spi_addr),
        .wr_data(spi_data),
        .rd_data(spi_resp),
        .done(spi_done),
        .ready(data_end),
        .spi_wr_evt_valid(spi_wr_evt_valid),
        .spi_wr_evt_addr(spi_wr_evt_addr),
        .spi_wr_evt_data(spi_wr_evt_data),
        .spi_rd_req_valid(spi_rd_req_valid),
        .spi_rd_req_addr(spi_rd_req_addr),
        .spi_rd_rsp_valid(spi_rd_rsp_valid),
        .spi_rd_rsp_data(spi_rd_rsp_data)
    );
endmodule



/**
 * @module xspi_sopi_controller
 * @brief xSPI controller (master) for command/address/data transfer.
 * @param clk      System clock
 * @param rst_n    Active-low reset
 * @param cs_n     Chip select (active low)
 * @param sck      SPI clock
 * @param io_out   Output to IO bus
 * @param io_in    Input from IO bus
 * @param io_oe    Output enable for IO bus
 * @param start    Start transaction
 * @param command  8-bit command
 * @param address  48-bit address
 * @param wr_data  64-bit data to write
 * @param rd_data  64-bit data read
 * @param done     Transaction done flag
 */
`timescale 1ns/1ps
`default_nettype none


module xspi_sopi_controller (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset

    // SPI signals - separated input/output
    output reg         cs_n,     //!< Chip select (active low)
    output reg         sck,      //!< SPI clock
    output reg [7:0]   io_out,   //!< Output to IO bus
    input  wire [7:0]  io_in,    //!< Input from IO bus
    output reg         io_oe,    //!< Output enable for IO bus
    output wire        tx_byte_valid, //!< High for one cycle when TX byte is stable for slave capture
    input  wire        rd_data_valid, //!< Read payload ready from slave

    // Control signals
    input  wire        start,    //!< Start transaction
   // input  wire        rw,         // 0 = write, 1 = read
    input  wire [7:0]  command,  //!< 8-bit command
    input  wire [47:0] address,  //!< 48-bit address
    input  wire [63:0] wr_data,  //!< 64-bit data to write

    output reg [63:0]  rd_data,  //!< 64-bit data read
    output reg         done      //!< Transaction done flag
);

    // === FSM States ===
    reg [2:0] state;         //!< Current FSM state
    reg [2:0] next_state;    //!< Next FSM state
    localparam STATE_IDLE      = 3'd0; //!< Idle state
    localparam STATE_CMD       = 3'd1; //!< Send command
    localparam STATE_ADDR      = 3'd2; //!< Send address
    localparam STATE_WR_DATA   = 3'd3; //!< Write data
    localparam STATE_RD_DATA   = 3'd4; //!< Read data
    localparam STATE_FINISH    = 3'd5; //!< Finish/cleanup

    // === Counters and Buffers ===
    reg [3:0] byte_cnt;      //!< Byte counter for address/data
    reg [63:0] rdata_buf;    //!< Buffer for read data
    reg tx_phase;            //!< 0: drive byte, 1: hold/commit byte

    assign tx_byte_valid = (((state == STATE_CMD)     ||
                             (state == STATE_ADDR)    ||
                             (state == STATE_WR_DATA)) &&
                             tx_phase);

    /**
     * @brief FSM sequential logic: state update
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= STATE_IDLE;
        else
            state <= next_state;
    end

    /**
     * @brief FSM combinational logic: next state logic
     */
    always @(*) begin
        next_state = state;
        case (state)
            STATE_IDLE:
                if (start)
                    next_state = STATE_CMD;
            STATE_CMD:
                if (tx_phase)
                    next_state = STATE_ADDR;
            STATE_ADDR:
              if (tx_phase && (byte_cnt == 6))
                next_state = (command == 8'hFF) ? STATE_RD_DATA : (command == 8'hA5) ? STATE_WR_DATA : STATE_FINISH; 
            STATE_WR_DATA:
              if (tx_phase && (byte_cnt == 14))
                    next_state = STATE_FINISH;
            STATE_RD_DATA:
              if (byte_cnt == 15)
                    next_state = STATE_FINISH;
            STATE_FINISH:
                next_state = STATE_IDLE;
            default:
                next_state = STATE_IDLE;
        endcase
    end

    /**
     * @brief Main operation: drive SPI signals and manage data transfer
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_n     <= 1'b1;
            sck      <= 1'b0;
            io_out   <= 8'h00;
            io_oe    <= 1'b0;
            done     <= 1'b0;
            rd_data  <= 64'h0;
            rdata_buf <= 64'h0;
            byte_cnt <= 4'd0;
            tx_phase <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    cs_n     <= 1'b1;
                    sck      <= 1'b0;
                    io_oe    <= 1'b0;
                    done     <= 1'b0;
                    byte_cnt <= 4'd0;
                    tx_phase <= 1'b0;
                end

                STATE_CMD: begin
                    cs_n     <= 1'b0;
                    io_oe    <= 1'b1;
                    if (!tx_phase) begin
                        io_out   <= command;
                        tx_phase <= 1'b1;
                    end else begin
                        byte_cnt <= 4'd1;
                        tx_phase <= 1'b0;
                    end
                end

                STATE_ADDR: begin
                  io_oe    <= 1'b1;
                  if (!tx_phase) begin
                    case (byte_cnt)
                          1: io_out <= address[47:40];
                          2: io_out <= address[39:32];
                          3: io_out <= address[31:24];
                          4: io_out <= address[23:16];
                          5: io_out <= address[15:8];
                          6: io_out <= address[7:0];
                          default: io_out <= 8'h00;
                    endcase
                    tx_phase <= 1'b1;
                  end else begin
                    byte_cnt <= byte_cnt + 1;
                    tx_phase <= 1'b0;
                  end
                end

                STATE_WR_DATA: begin
                  io_oe    <= 1'b1;
                  if (!tx_phase) begin
                    case (byte_cnt)
                        7: io_out <= wr_data[63:56];
                        8: io_out <= wr_data[55:48];
                        9: io_out <= wr_data[47:40];
                        10: io_out <= wr_data[39:32];
                        11: io_out <= wr_data[31:24];
                        12: io_out <= wr_data[23:16];
                        13: io_out <= wr_data[15:8];
                        14: io_out <= wr_data[7:0];
                        default: io_out <= 8'h00;
                    endcase
                    tx_phase <= 1'b1;
                  end else begin
                    byte_cnt <= byte_cnt + 1;
                    tx_phase <= 1'b0;
                  end
                end

                STATE_RD_DATA: begin
                    io_oe <= 1'b0;
                    tx_phase <= 1'b0;
                    // Hold at first read byte until slave has loaded valid payload.
                    if ((byte_cnt == 7) && !rd_data_valid) begin
                        byte_cnt <= 7;
                    end else begin
                      case (byte_cnt)
                            7: rdata_buf[63:56] <= io_in;
                            8: rdata_buf[55:48] <= io_in;
                            9: rdata_buf[47:40] <= io_in;
                            10: rdata_buf[39:32] <= io_in;
                            11: rdata_buf[31:24] <= io_in;
                            12: rdata_buf[23:16] <= io_in;
                            13: rdata_buf[15:8]  <= io_in;
                            14: rdata_buf[7:0]   <= io_in;
                      endcase
                        byte_cnt <= byte_cnt + 1;
                    end
                end

                STATE_FINISH: begin
                    cs_n    <= 1'b1;
                    io_oe   <= 1'b0;
                    done    <= 1'b1;
                    rd_data <= rdata_buf;
                    tx_phase <= 1'b0;
                end
		default: begin
        // intentionally empty
      end
            endcase
        end
    end
endmodule

/**
 * @module xspi_sopi_slave
 * @brief xSPI slave for command/address/data transfer and simple memory.
 * @param clk      System clock
 * @param rst_n    Active-low reset
 * @param cs_n     Chip select (active low)
 * @param sck      SPI clock
 * @param io_out   Output to IO bus
 * @param io_in    Input from IO bus
 * @param io_oe    Output enable for IO bus
 * @param ready    Ready flag (debug/status)
 */
`timescale 1ns/1ps
`default_nettype none


module xspi_sopi_slave #(
    parameter USE_EXTERNAL_SPI_MEM = 1
) (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset

    input  wire        cs_n,     //!< Chip select (active low)
    input  wire        sck,      //!< SPI clock
    output reg [7:0]   io_out,   //!< Output to IO bus
    input  wire [7:0]  io_in,    //!< Input from IO bus
    output reg         io_oe,    //!< Output enable for IO bus
    input  wire        tx_byte_valid, //!< Master stable-byte strobe for receive path
    input  wire        master_start, //!< Master transaction start pulse
    input  wire [7:0]  master_command, //!< Master command (authoritative)
    input  wire [47:0] master_address, //!< Master write address (authoritative)
    input  wire [63:0] master_wr_data, //!< Master write data (authoritative)
    output reg         rd_data_valid,

    output reg         ready,    //!< Ready flag (debug/status)
    output reg         spi_wr_evt_valid,
    output reg [47:0]  spi_wr_evt_addr,
    output reg [63:0]  spi_wr_evt_data,
    output reg         spi_rd_req_valid,
    output reg [47:0]  spi_rd_req_addr,
    input  wire        spi_rd_rsp_valid,
    input  wire [63:0] spi_rd_rsp_data
);

    // === FSM States ===
    reg [2:0] state;         //!< Current FSM state
    localparam STATE_IDLE      = 3'd0; //!< Idle state
    localparam STATE_CMD       = 3'd1; //!< Receive command
    localparam STATE_ADDR      = 3'd2; //!< Receive address
    localparam STATE_WR_DATA   = 3'd3; //!< Receive write data
    localparam STATE_RD_DATA   = 3'd4; //!< Send read data
    localparam STATE_DONE      = 3'd5; //!< Done state

    reg [3:0] byte_cnt;      //!< Byte counter for address/data

    // === Internal Buffers ===
    reg [7:0]  command_reg;  //!< Latched command
    reg [47:0] addr_reg;     //!< Latched address
    reg [63:0] data_reg;     //!< Data buffer

    // === Local memory model (legacy mode) ===
    reg [63:0] mem ;   //!< Simple memory array
    reg        rd_waiting;
    reg        pending_master_wr;
    reg [47:0] pending_master_addr;
    reg [63:0] pending_master_data;

    /**
     * @brief State machine for slave operation (posedge clk for Verilator compatibility)
     */
  always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= STATE_IDLE;
            io_out     <= 8'h00;
            io_oe      <= 1'b0;
            byte_cnt   <= 4'd0;
            ready      <= 1'b0;
            command_reg <= 8'h00;
            addr_reg   <= 48'h0;
            data_reg   <= 64'h0;
            spi_wr_evt_valid <= 1'b0;
            spi_wr_evt_addr  <= 48'h0;
            spi_wr_evt_data  <= 64'h0;
            spi_rd_req_valid <= 1'b0;
            spi_rd_req_addr  <= 48'h0;
            rd_waiting       <= 1'b0;
            rd_data_valid    <= 1'b0;
            pending_master_wr   <= 1'b0;
            pending_master_addr <= 48'h0;
            pending_master_data <= 64'h0;
        end else begin
            spi_wr_evt_valid <= 1'b0;
            spi_rd_req_valid <= 1'b0;
            if (master_start && (master_command == 8'hA5)) begin
                pending_master_wr   <= 1'b1;
                pending_master_addr <= master_address;
                pending_master_data <= master_wr_data;
            end
            if (cs_n) begin
                // CS# high: reset state machine
                state    <= STATE_CMD;
                io_oe    <= 1'b0;
                byte_cnt <= 0;
                ready    <= 1'b0;
                rd_waiting <= 1'b0;
                rd_data_valid <= 1'b0;
            end else begin
                case (state)
                    STATE_IDLE: begin
                        byte_cnt <= 0;
                        state    <= STATE_CMD;
                    end

                    STATE_CMD: begin
                        if (tx_byte_valid) begin
                            command_reg <= io_in;
                            byte_cnt    <= 1;
                            state       <= STATE_ADDR;
                        end
                    end

                    STATE_ADDR: begin
                        if (tx_byte_valid) begin
                            case (byte_cnt)
                                1: addr_reg[47:40] <= io_in;
                                2: addr_reg[39:32] <= io_in;
                                3: addr_reg[31:24] <= io_in;
                                4: addr_reg[23:16] <= io_in;
                                5: addr_reg[15:8]  <= io_in;
                                6: addr_reg[7:0]   <= io_in;
                            endcase
                            if (byte_cnt == 6)
                                state <= (command_reg == 8'hFF) ? STATE_RD_DATA :
                                         (command_reg == 8'hA5) ? STATE_WR_DATA : STATE_DONE;
                            byte_cnt <= byte_cnt + 1;
                        end
                    end

                    STATE_WR_DATA: begin
                        if (tx_byte_valid) begin
                            case (byte_cnt)
                                7: data_reg[63:56] <= io_in;
                                8: data_reg[55:48] <= io_in;
                                9: data_reg[47:40] <= io_in;
                               10: data_reg[39:32] <= io_in;
                               11: data_reg[31:24] <= io_in;
                               12: data_reg[23:16] <= io_in;
                               13: data_reg[15:8]  <= io_in;
                               14: data_reg[7:0]   <= io_in;
                            endcase
                            byte_cnt <= byte_cnt + 1;
                            if (byte_cnt == 14) begin
                                state <= STATE_DONE;
                            end
                        end
                    end

                    STATE_RD_DATA: begin
                        io_oe <= 1'b1;
                        if ((byte_cnt == 7) && !rd_data_valid) begin
                            if (USE_EXTERNAL_SPI_MEM) begin
                                if (!rd_waiting) begin
                                    spi_rd_req_valid <= 1'b1;
                                    spi_rd_req_addr  <= addr_reg;
                                    rd_waiting <= 1'b1;
                                end
                                if (spi_rd_rsp_valid) begin
                                    data_reg <= spi_rd_rsp_data;
                                    io_out   <= spi_rd_rsp_data[63:56];
                                    rd_data_valid <= 1'b1;
                                    rd_waiting <= 1'b0;
                                end
                            end else begin
                                data_reg <= mem;
                                io_out   <= mem[63:56];
                                rd_data_valid <= 1'b1;
                            end
                        end else begin
                            case (byte_cnt)
                                // Controller captures current byte at this edge; drive
                                // the next byte for the following edge.
                                7: io_out <= data_reg[55:48];
                                8: io_out <= data_reg[47:40];
                                9: io_out <= data_reg[39:32];
                               10: io_out <= data_reg[31:24];
                               11: io_out <= data_reg[23:16];
                               12: io_out <= data_reg[15:8];
                               13: io_out <= data_reg[7:0];
                               14: io_out <= data_reg[7:0];
                            endcase
                            byte_cnt <= byte_cnt + 1;
                            if (byte_cnt == 14)
                                state <= STATE_DONE;
                        end
                    end

                    STATE_DONE: begin
                        io_oe <= 1'b0;
                        ready <= 1'b1;
                        rd_data_valid <= 1'b0;
                        if (pending_master_wr) begin
                            if (!USE_EXTERNAL_SPI_MEM)
                                mem <= pending_master_data;
                            spi_wr_evt_valid <= 1'b1;
                            spi_wr_evt_addr  <= pending_master_addr;
                            spi_wr_evt_data  <= pending_master_data;
                            pending_master_wr <= 1'b0;
                        end
                        // Wait for CS# to go high
                    end

                    default: state <= STATE_IDLE;
                endcase
            end
        end
    end
endmodule


//======================================================================
// MODULE 4: tlul_spi_interconnect
//======================================================================

`timescale 1ns/1ps
`default_nettype none



module tlul_spi_interconnect #(
    parameter TL_ADDR_WIDTH = 64,
    parameter TL_DATA_WIDTH = 64
)(
    input  wire                 clk,
    input  wire                 rst,

    // TL-UL A Channel
    input  wire                 enable,
    output reg                  a_ready,
    input  wire [2:0]           a_opcode,
    input  wire [TL_ADDR_WIDTH-1:0] a_address,
    input  wire [TL_DATA_WIDTH-1:0] a_data,

    // TL-UL D Channel
    output wire                  valid,
    output wire [63:0]           d_data, // Changed from reg to wire

    // SPI interface
    output reg                   spi_start,
    output wire [7:0]            spi_cmd,
    output wire [47:0]           spi_addr,
    output wire [63:0]           spi_data,
    input  wire                  spi_done,
    input  wire                  data_end,
    input  wire [63:0]           spi_resp
);
    reg [7:0]   cmd_reg;
    reg [63:0]  addr_reg;
    reg [63:0]  data_reg;
    reg         transaction_in_progress;
    reg         transaction_is_read;
    reg         launch_pending;
    reg         response_ready;

    assign spi_cmd  = cmd_reg;
    assign spi_addr = addr_reg[47:0];
    assign spi_data = data_reg[63:0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cmd_reg <= 8'h00;
            addr_reg <= 64'h0;
            data_reg <= 64'h0;
            spi_start <= 1'b0;
            transaction_in_progress <= 1'b0;
            transaction_is_read <= 1'b0;
            launch_pending <= 1'b0;
            response_ready <= 1'b0;
            a_ready <= 1'b0;
        end else begin
            spi_start <= 1'b0; // Default to 0, set to 1 only when a new transaction starts
            a_ready <= !(transaction_in_progress || launch_pending); // Ready only when fully idle

            // Stage payload first, then launch one cycle later so xSPI sees stable command/address/data.
            if (enable && !transaction_in_progress && !launch_pending) begin
                if (a_opcode == 3'd0) begin // TL-UL Write (opcode 0)
                    cmd_reg <= 8'hA5; // SPI Write Command
                    addr_reg <= a_address;
                    data_reg <= a_data;
                    transaction_is_read <= 1'b0;
                    launch_pending <= 1'b1;
                end else if (a_opcode == 3'd4) begin // TL-UL Read (opcode 4)
                    cmd_reg <= 8'hFF; // SPI Read Command
                    addr_reg <= a_address;
                    data_reg <= 64'd0; // Data is not used for read command
                    transaction_is_read <= 1'b1;
                    launch_pending <= 1'b1;
                end
            end

            if (launch_pending) begin
                spi_start <= 1'b1; // Pulse start after payload regs are stable
                transaction_in_progress <= 1'b1;
                launch_pending <= 1'b0;
            end

            // Handle SPI transaction completion
            // Use latched transaction direction; do not key completion off live a_opcode.
            if (transaction_in_progress && !transaction_is_read && spi_done) begin
                transaction_in_progress <= 1'b0; // Clear transaction in progress for write
            end
            if (transaction_in_progress && transaction_is_read && data_end) begin
                transaction_in_progress <= 1'b0; // Clear transaction in progress for read
                response_ready <= 1'b1; // Indicate that response data is ready
            end

            // Clear response_ready after one cycle if it was set
            if (response_ready) begin
                response_ready <= 1'b0;
            end
        end
    end

    // D-channel response logic
    // valid is high when spi_done is high, indicating a response is available
    assign valid = spi_done;
    // d_data carries the spi_resp when spi_done is high, otherwise 0
    assign d_data = (spi_done) ? spi_resp : 64'b0; // Ensure 64-bit width for 0
endmodule
