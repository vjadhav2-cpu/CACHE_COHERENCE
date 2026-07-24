// =============================================================================
// tlc_slave_mem_manager.v  —  TL-C slave manager with raw mem_* southbound port
// =============================================================================
//
// Purpose
//   Presents a coherent TL-C slave interface toward tlc_xbar_top while driving
//   a simple raw memory port internally. This keeps coherence handling at the
//   crossbar boundary and avoids any TL-C or TL-UH requirements in the memory
//   array/controller itself.
//
// Southbound memory contract
//   - mem_req / mem_we / mem_addr / mem_wdata / mem_wmask define a request.
//   - Hold mem_req asserted until mem_ready acknowledges the request.
//   - For writes, completion is mem_ready on the write request.
//   - For reads, mem_ready acknowledges request acceptance and mem_rvalid later
//     returns mem_rdata.
//
// Supported upstream TL-C traffic
//   - Coherent: AcquireBlock / AcquirePerm, ProbeAck / ProbeAckData,
//               Release / ReleaseData, Grant / GrantData / ReleaseAck.
//   - Uncached: Get / PutFullData / PutPartialData, AccessAck / AccessAckData.
// =============================================================================
`timescale 1ns/10ps
`default_nettype none

module tlc_slave_mem_manager #(
    parameter integer SLAVE_ID   = 0,
    parameter integer ADDR_W     = 32,
    parameter integer DATA_W     = 64,
    parameter integer SOURCE_W   = 4,
    parameter integer M_SOURCE_W = 6,
    parameter integer SINK_W     = 4,
    parameter integer N_MASTERS  = 3,
    parameter integer MST_ID_W   = 2,
    parameter integer DIR_DEPTH  = 32
) (
    input  wire                    clk,
    input  wire                    rst_n,

    // TL-C A
    input  wire                    a_valid,
    output wire                    a_ready,
    input  wire [2:0]              a_opcode,
    input  wire [2:0]              a_param,
    input  wire [3:0]              a_size,
    input  wire [M_SOURCE_W-1:0]   a_source,
    input  wire [ADDR_W-1:0]       a_address,
    input  wire [DATA_W/8-1:0]     a_mask,
    input  wire [DATA_W-1:0]       a_data,
    input  wire                    a_corrupt,

    // TL-C B
    output reg                     b_valid,
    input  wire                    b_ready,
    output reg  [2:0]              b_opcode,
    output reg  [2:0]              b_param,
    output reg  [3:0]              b_size,
    output reg  [SOURCE_W-1:0]     b_source,
    output reg  [ADDR_W-1:0]       b_address,
    output reg  [DATA_W/8-1:0]     b_mask,
    output reg  [DATA_W-1:0]       b_data,
    output reg                     b_corrupt,
    output reg  [MST_ID_W-1:0]     b_dest,

    // TL-C C
    input  wire                    c_valid,
    output wire                    c_ready,
    input  wire [2:0]              c_opcode,
    input  wire [2:0]              c_param,
    input  wire [3:0]              c_size,
    input  wire [M_SOURCE_W-1:0]   c_source,
    input  wire [ADDR_W-1:0]       c_address,
    input  wire [DATA_W-1:0]       c_data,
    input  wire                    c_corrupt,

    // TL-C D
    output reg                     d_valid,
    input  wire                    d_ready,
    output reg  [2:0]              d_opcode,
    output reg  [1:0]              d_param,
    output reg  [3:0]              d_size,
    output reg  [M_SOURCE_W-1:0]   d_source,
    output reg  [SINK_W-1:0]       d_sink,
    output reg                     d_denied,
    output reg  [DATA_W-1:0]       d_data,
    output reg                     d_corrupt,

    // TL-C E
    input  wire                    e_valid,
    output wire                    e_ready,
    input  wire [SINK_W-1:0]       e_sink,

    // Raw memory southbound interface
    output reg                     mem_req,
    output reg                     mem_we,
    output reg  [ADDR_W-1:0]       mem_addr,
    output reg  [DATA_W-1:0]       mem_wdata,
    output reg  [DATA_W/8-1:0]     mem_wmask,
    input  wire [DATA_W-1:0]       mem_rdata,
    input  wire                    mem_rvalid,
    input  wire                    mem_ready
);

    localparam [2:0] TL_A_PUT_FULL_DATA    = 3'd0;
    localparam [2:0] TL_A_PUT_PARTIAL_DATA = 3'd1;
    localparam [2:0] TL_A_GET              = 3'd4;
    localparam [2:0] TL_A_ACQUIRE_BLOCK    = 3'd6;
    localparam [2:0] TL_A_ACQUIRE_PERM     = 3'd7;

    localparam [2:0] TL_B_PROBE            = 3'd6;

    localparam [2:0] TL_C_PROBE_ACK        = 3'd4;
    localparam [2:0] TL_C_PROBE_ACK_DATA   = 3'd5;
    localparam [2:0] TL_C_RELEASE          = 3'd6;
    localparam [2:0] TL_C_RELEASE_DATA     = 3'd7;

    localparam [2:0] TL_D_ACCESS_ACK       = 3'd0;
    localparam [2:0] TL_D_ACCESS_ACK_DATA  = 3'd1;
    localparam [2:0] TL_D_GRANT            = 3'd4;
    localparam [2:0] TL_D_GRANT_DATA       = 3'd5;
    localparam [2:0] TL_D_RELEASE_ACK      = 3'd6;

    localparam [1:0] COH_I = 2'b00;
    localparam [1:0] COH_B = 2'b01;
    localparam [1:0] COH_T = 2'b10;

    localparam [2:0] PARAM_NtoB = 3'd0;
    localparam [2:0] PARAM_NtoT = 3'd1;
    localparam [2:0] PARAM_BtoT = 3'd2;

    localparam [3:0] S_IDLE          = 4'd0;
    localparam [3:0] S_DIR_LOOKUP    = 4'd1;
    localparam [3:0] S_PROBE_SEND    = 4'd2;
    localparam [3:0] S_PROBE_WAIT    = 4'd3;
    localparam [3:0] S_MEM_REQ       = 4'd4;
    localparam [3:0] S_MEM_WAIT      = 4'd5;
    localparam [3:0] S_D_SEND        = 4'd6;
    localparam [3:0] S_GRANTACK_WAIT = 4'd7;

    localparam DIR_IDX_W = $clog2(DIR_DEPTH);
    localparam DIR_TAG_W = ADDR_W - DIR_IDX_W - 3;

    reg [3:0] state;

    reg [M_SOURCE_W-1:0] req_source;
    reg [ADDR_W-1:0]     req_addr;
    reg [2:0]            req_opcode;
    reg [2:0]            req_param;
    reg [3:0]            req_size;
    reg [DATA_W-1:0]     req_data;
    reg [DATA_W/8-1:0]   req_mask;

    reg [N_MASTERS-1:0]  probe_pending;
    reg [SINK_W-1:0]     sink_cnt;

    reg                  pending_mem_read;
    reg                  pending_expect_grantack;
    reg                  pending_dir_update;
    reg [2:0]            pending_d_opcode;
    reg [1:0]            pending_d_param;
    reg [DATA_W-1:0]     pending_d_data;

    reg                  dir_v    [0:DIR_DEPTH-1];
    reg [DIR_TAG_W-1:0]  dir_tag  [0:DIR_DEPTH-1];
    reg [1:0]            dir_coh  [0:DIR_DEPTH-1];
    reg [N_MASTERS-1:0]  dir_pres [0:DIR_DEPTH-1];

    wire [DIR_IDX_W-1:0] req_dir_idx = req_addr[DIR_IDX_W+2 : 3];
    wire [DIR_TAG_W-1:0] req_dir_tag = req_addr[ADDR_W-1 : DIR_IDX_W+3];
    wire                 dir_hit = dir_v[req_dir_idx] && (dir_tag[req_dir_idx] == req_dir_tag);
    wire [MST_ID_W-1:0]  req_mst_id = req_source[M_SOURCE_W-1:SOURCE_W];
    wire [N_MASTERS-1:0] req_mst_mask = {{(N_MASTERS-1){1'b0}}, 1'b1} << req_mst_id;
    wire [N_MASTERS-1:0] dir_pres_now = dir_hit ? dir_pres[req_dir_idx] : {N_MASTERS{1'b0}};
    wire [1:0]           dir_coh_now  = dir_hit ? dir_coh[req_dir_idx]  : COH_I;
    wire                 req_is_acquire = (req_opcode == TL_A_ACQUIRE_BLOCK) || (req_opcode == TL_A_ACQUIRE_PERM);
    wire                 req_is_get     = (req_opcode == TL_A_GET);
    wire                 req_is_put     = (req_opcode == TL_A_PUT_FULL_DATA) || (req_opcode == TL_A_PUT_PARTIAL_DATA);
    wire                 req_needs_mem  = req_is_get || req_is_put || (req_opcode == TL_A_ACQUIRE_BLOCK);

    wire [N_MASTERS-1:0] probes_excl   = dir_pres_now & ~req_mst_mask;
    wire [N_MASTERS-1:0] probes_shared = (dir_coh_now == COH_T) ? probes_excl : {N_MASTERS{1'b0}};
    wire [N_MASTERS-1:0] probes_needed = req_is_put
                                       ? dir_pres_now
                                       : (((req_param == PARAM_NtoT) || (req_param == PARAM_BtoT))
                                          ? probes_excl : probes_shared);

    wire [N_MASTERS-1:0] probe_next_oh;
    genvar gp;
    generate
        for (gp = 0; gp < N_MASTERS; gp = gp + 1) begin : g_probe_pri
            if (gp == 0)
                assign probe_next_oh[0] = probe_pending[0];
            else
                assign probe_next_oh[gp] = probe_pending[gp] && !(|(probe_pending[gp-1:0]));
        end
    endgenerate

    integer k;
    reg [MST_ID_W-1:0] probe_next_idx;
    always @(*) begin
        probe_next_idx = {MST_ID_W{1'b0}};
        for (k = 0; k < N_MASTERS; k = k + 1)
            if (probe_next_oh[k])
                probe_next_idx = k[MST_ID_W-1:0];
    end

    assign a_ready = (state == S_IDLE);
    assign c_ready = (state == S_IDLE) || (state == S_PROBE_WAIT);
    assign e_ready = (state == S_GRANTACK_WAIT);

    integer ri;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                  <= S_IDLE;
            req_source             <= {M_SOURCE_W{1'b0}};
            req_addr               <= {ADDR_W{1'b0}};
            req_opcode             <= 3'b0;
            req_param              <= 3'b0;
            req_size               <= 4'b0;
            req_data               <= {DATA_W{1'b0}};
            req_mask               <= {DATA_W/8{1'b0}};
            probe_pending          <= {N_MASTERS{1'b0}};
            sink_cnt               <= {SINK_W{1'b0}};
            pending_mem_read       <= 1'b0;
            pending_expect_grantack<= 1'b0;
            pending_dir_update     <= 1'b0;
            pending_d_opcode       <= 3'b0;
            pending_d_param        <= 2'b0;
            pending_d_data         <= {DATA_W{1'b0}};

            b_valid   <= 1'b0;
            b_opcode  <= 3'b0;
            b_param   <= 3'b0;
            b_size    <= 4'b0;
            b_source  <= {SOURCE_W{1'b0}};
            b_address <= {ADDR_W{1'b0}};
            b_mask    <= {DATA_W/8{1'b1}};
            b_data    <= {DATA_W{1'b0}};
            b_corrupt <= 1'b0;
            b_dest    <= {MST_ID_W{1'b0}};

            d_valid   <= 1'b0;
            d_opcode  <= 3'b0;
            d_param   <= 2'b0;
            d_size    <= 4'b0;
            d_source  <= {M_SOURCE_W{1'b0}};
            d_sink    <= {SINK_W{1'b0}};
            d_denied  <= 1'b0;
            d_data    <= {DATA_W{1'b0}};
            d_corrupt <= 1'b0;

            mem_req   <= 1'b0;
            mem_we    <= 1'b0;
            mem_addr  <= {ADDR_W{1'b0}};
            mem_wdata <= {DATA_W{1'b0}};
            mem_wmask <= {DATA_W/8{1'b0}};

            for (ri = 0; ri < DIR_DEPTH; ri = ri + 1) begin
                dir_v[ri]    <= 1'b0;
                dir_tag[ri]  <= {DIR_TAG_W{1'b0}};
                dir_coh[ri]  <= COH_I;
                dir_pres[ri] <= {N_MASTERS{1'b0}};
            end
        end else begin
            b_valid <= 1'b0;
            d_valid <= d_valid;

            case (state)
                S_IDLE: begin
                    d_valid  <= 1'b0;
                    mem_req  <= 1'b0;
                    if (a_valid) begin
                        req_source <= a_source;
                        req_addr   <= a_address;
                        req_opcode <= a_opcode;
                        req_param  <= a_param;
                        req_size   <= a_size;
                        req_data   <= a_data;
                        req_mask   <= a_mask;
                        state      <= S_DIR_LOOKUP;
                    end else if (c_valid &&
                                 (c_opcode == TL_C_RELEASE || c_opcode == TL_C_RELEASE_DATA)) begin
                        req_source <= c_source;
                        req_addr   <= c_address;
                        req_size   <= c_size;
                        req_opcode <= c_opcode;
                        req_data   <= c_data;
                        req_mask   <= {DATA_W/8{1'b1}};

                        if (c_opcode == TL_C_RELEASE_DATA) begin
                            $display("[SLV_REL_RECV][%0t] slave=%0d addr=0x%08h data=0x%016h src=0x%0h", $time, SLAVE_ID, c_address, c_data, c_source);
                            mem_req   <= 1'b1;
                            mem_we    <= 1'b1;
                            mem_addr  <= c_address;
                            mem_wdata <= c_data;
                            mem_wmask <= {DATA_W/8{1'b1}};
                            state     <= S_MEM_REQ;
                            pending_mem_read        <= 1'b0;
                            pending_expect_grantack <= 1'b0;
                            pending_dir_update      <= 1'b0;
                            pending_d_opcode        <= TL_D_RELEASE_ACK;
                            pending_d_param         <= 2'b00;
                            pending_d_data          <= {DATA_W{1'b0}};
                        end else begin
                            pending_mem_read        <= 1'b0;
                            pending_expect_grantack <= 1'b0;
                            pending_dir_update      <= 1'b0;
                            pending_d_opcode        <= TL_D_RELEASE_ACK;
                            pending_d_param         <= 2'b00;
                            pending_d_data          <= {DATA_W{1'b0}};
                            state                   <= S_D_SEND;
                        end

                        if (dir_v[c_address[DIR_IDX_W+2:3]] &&
                            dir_tag[c_address[DIR_IDX_W+2:3]] == c_address[ADDR_W-1:DIR_IDX_W+3]) begin
                            dir_pres[c_address[DIR_IDX_W+2:3]] <=
                                dir_pres[c_address[DIR_IDX_W+2:3]] &
                                ~({{(N_MASTERS-1){1'b0}}, 1'b1} << c_source[M_SOURCE_W-1:SOURCE_W]);
                            if ((dir_pres[c_address[DIR_IDX_W+2:3]] &
                                 ~({{(N_MASTERS-1){1'b0}}, 1'b1} << c_source[M_SOURCE_W-1:SOURCE_W])) ==
                                {N_MASTERS{1'b0}}) begin
                                dir_v[c_address[DIR_IDX_W+2:3]]   <= 1'b0;
                                dir_tag[c_address[DIR_IDX_W+2:3]] <= {DIR_TAG_W{1'b0}};
                                dir_coh[c_address[DIR_IDX_W+2:3]] <= COH_I;
                            end
                        end
                    end
                end

                S_DIR_LOOKUP: begin
                    probe_pending <= probes_needed;
                    if (probes_needed != {N_MASTERS{1'b0}}) begin
                        state <= S_PROBE_SEND;
                    end else if (req_needs_mem) begin
                        mem_req   <= 1'b1;
                        mem_we    <= req_is_put;
                        mem_addr  <= req_addr;
                        mem_wdata <= req_data;
                        mem_wmask <= req_mask;
                        state     <= S_MEM_REQ;
                        pending_mem_read        <= req_is_get || (req_opcode == TL_A_ACQUIRE_BLOCK);
                        pending_expect_grantack <= req_is_acquire;
                        pending_dir_update      <= req_is_acquire;
                        pending_d_opcode        <= req_is_get ? TL_D_ACCESS_ACK_DATA :
                                                   req_is_put ? TL_D_ACCESS_ACK :
                                                   (req_opcode == TL_A_ACQUIRE_PERM) ? TL_D_GRANT :
                                                   TL_D_GRANT_DATA;
                        pending_d_param         <= req_is_acquire
                                                   ? ((req_param == PARAM_NtoB) ? 2'b01 : 2'b00)
                                                   : 2'b00;
                    end else begin
                        pending_mem_read        <= 1'b0;
                        pending_expect_grantack <= req_is_acquire;
                        pending_dir_update      <= req_is_acquire;
                        pending_d_opcode        <= TL_D_GRANT;
                        pending_d_param         <= (req_param == PARAM_NtoB) ? 2'b01 : 2'b00;
                        pending_d_data          <= {DATA_W{1'b0}};
                        state                   <= S_D_SEND;
                    end
                end

                S_PROBE_SEND: begin
                    b_valid   <= 1'b1;
                    b_opcode  <= TL_B_PROBE;
                    b_param   <= 3'd1;
                    b_size    <= req_size;
                    b_source  <= {SOURCE_W{1'b0}};
                    b_address <= req_addr;
                    b_mask    <= {DATA_W/8{1'b1}};
                    b_data    <= {DATA_W{1'b0}};
                    b_corrupt <= 1'b0;
                    b_dest    <= probe_next_idx;
                    if (b_ready) begin
                        probe_pending[probe_next_idx] <= 1'b0;
                        state <= S_PROBE_WAIT;
                    end
                end

                S_PROBE_WAIT: begin
                    if (c_valid && (c_opcode == TL_C_PROBE_ACK || c_opcode == TL_C_PROBE_ACK_DATA)) begin
                        if (c_opcode == TL_C_PROBE_ACK_DATA) begin
                            mem_req   <= 1'b1;
                            mem_we    <= 1'b1;
                            mem_addr  <= req_addr;
                            mem_wdata <= c_data;
                            mem_wmask <= {DATA_W/8{1'b1}};
                            state     <= S_MEM_REQ;
                            pending_mem_read        <= req_needs_mem && (req_is_get || (req_opcode == TL_A_ACQUIRE_BLOCK));
                            pending_expect_grantack <= req_is_acquire;
                            pending_dir_update      <= req_is_acquire;
                            pending_d_opcode        <= req_is_get ? TL_D_ACCESS_ACK_DATA :
                                                       req_is_put ? TL_D_ACCESS_ACK :
                                                       (req_opcode == TL_A_ACQUIRE_PERM) ? TL_D_GRANT :
                                                       TL_D_GRANT_DATA;
                            pending_d_param         <= req_is_acquire
                                                       ? ((req_param == PARAM_NtoB) ? 2'b01 : 2'b00)
                                                       : 2'b00;
                        end else if (probe_pending != {N_MASTERS{1'b0}}) begin
                            state <= S_PROBE_SEND;
                        end else if (req_needs_mem) begin
                            mem_req   <= 1'b1;
                            mem_we    <= req_is_put;
                            mem_addr  <= req_addr;
                            mem_wdata <= req_data;
                            mem_wmask <= req_mask;
                            state     <= S_MEM_REQ;
                            pending_mem_read        <= req_is_get || (req_opcode == TL_A_ACQUIRE_BLOCK);
                            pending_expect_grantack <= req_is_acquire;
                            pending_dir_update      <= req_is_acquire;
                            pending_d_opcode        <= req_is_get ? TL_D_ACCESS_ACK_DATA :
                                                       req_is_put ? TL_D_ACCESS_ACK :
                                                       (req_opcode == TL_A_ACQUIRE_PERM) ? TL_D_GRANT :
                                                       TL_D_GRANT_DATA;
                            pending_d_param         <= req_is_acquire
                                                       ? ((req_param == PARAM_NtoB) ? 2'b01 : 2'b00)
                                                       : 2'b00;
                        end else begin
                            pending_mem_read        <= 1'b0;
                            pending_expect_grantack <= req_is_acquire;
                            pending_dir_update      <= req_is_acquire;
                            pending_d_opcode        <= TL_D_GRANT;
                            pending_d_param         <= (req_param == PARAM_NtoB) ? 2'b01 : 2'b00;
                            pending_d_data          <= {DATA_W{1'b0}};
                            state                   <= S_D_SEND;
                        end
                    end
                end

                S_MEM_REQ: begin
                    mem_req <= 1'b1;
                    if (mem_ready) begin
                        $display("[SLV_MEM_ACC][%0t] slave=%0d we=%0b addr=0x%08h wdata=0x%016h opcode=%0d", $time, SLAVE_ID, mem_we, mem_addr, mem_wdata, req_opcode);
                        mem_req <= 1'b0;
                        if (pending_mem_read)
                            state <= S_MEM_WAIT;
                        else begin
                            pending_d_data <= {DATA_W{1'b0}};
                            state <= S_D_SEND;
                        end
                    end
                end

                S_MEM_WAIT: begin
                    if (mem_rvalid) begin
                        $display("[SLV_MEM_RSP][%0t] slave=%0d addr=0x%08h rdata=0x%016h", $time, SLAVE_ID, req_addr, mem_rdata);
                        pending_d_data <= mem_rdata;
                        state <= S_D_SEND;
                    end
                end

                S_D_SEND: begin
                    if (!d_valid) begin
                        d_valid   <= 1'b1;
                        d_opcode  <= pending_d_opcode;
                        d_param   <= pending_d_param;
                        d_size    <= req_size;
                        d_source  <= req_source;
                        d_sink    <= sink_cnt;
                        d_denied  <= 1'b0;
                        d_data    <= pending_d_data;
                        d_corrupt <= a_corrupt | c_corrupt;
                        sink_cnt  <= sink_cnt + 1'b1;

                        if (pending_dir_update) begin
                            dir_v[req_dir_idx]   <= 1'b1;
                            dir_tag[req_dir_idx] <= req_dir_tag;
                            if (req_param == PARAM_NtoB) begin
                                dir_coh[req_dir_idx]  <= COH_B;
                                dir_pres[req_dir_idx] <= (dir_hit ? dir_pres[req_dir_idx] : {N_MASTERS{1'b0}}) | req_mst_mask;
                            end else begin
                                dir_coh[req_dir_idx]  <= COH_T;
                                dir_pres[req_dir_idx] <= req_mst_mask;
                            end
                        end else if (req_is_put || req_is_get) begin
                            dir_v[req_dir_idx]   <= 1'b0;
                            dir_tag[req_dir_idx] <= {DIR_TAG_W{1'b0}};
                            dir_coh[req_dir_idx] <= COH_I;
                            dir_pres[req_dir_idx]<= {N_MASTERS{1'b0}};
                        end
                    end else if (d_ready) begin
                        d_valid <= 1'b0;
                        state   <= pending_expect_grantack ? S_GRANTACK_WAIT : S_IDLE;
                    end
                end

                S_GRANTACK_WAIT: begin
                    if (e_valid)
                        state <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
