// L1 controller request types
localparam [2:0] L1_REQ_READ_MISS      = 3'd0;
localparam [2:0] L1_REQ_WRITE_MISS     = 3'd1;
localparam [2:0] L1_REQ_WRITE_BACK     = 3'd2;
localparam [2:0] L1_REQ_UNCACHED_READ  = 3'd3;
localparam [2:0] L1_REQ_UNCACHED_WRITE = 3'd4;

// TileLink channel opcodes
localparam [2:0] A_OPCODE_PUT_FULL_DATA  = 3'd0;
localparam [2:0] A_OPCODE_GET            = 3'd4;
localparam [2:0] A_OPCODE_ACQUIRE_BLOCK  = 3'd6;

localparam [2:0] B_OPCODE_PROBE_BLOCK    = 3'd6;

localparam [2:0] C_OPCODE_PROBE_ACK      = 3'd4;
localparam [2:0] C_OPCODE_PROBE_ACK_DATA = 3'd5;
localparam [2:0] C_OPCODE_RELEASE        = 3'd6;
localparam [2:0] C_OPCODE_RELEASE_DATA   = 3'd7;

localparam [2:0] D_OPCODE_ACCESS_ACK      = 3'd0;
localparam [2:0] D_OPCODE_ACCESS_ACK_DATA = 3'd1;
localparam [2:0] D_OPCODE_GRANT           = 3'd4;
localparam [2:0] D_OPCODE_GRANT_DATA      = 3'd5;
localparam [2:0] D_OPCODE_RELEASE_ACK     = 3'd6;

// TileLink permissions / reports
localparam [2:0] PARAM_NtoB = 3'd0;
localparam [2:0] PARAM_NtoT = 3'd1;
localparam [2:0] PARAM_BtoT = 3'd2;

localparam [2:0] PARAM_TtoB = 3'd0;
localparam [2:0] PARAM_TtoN = 3'd1;
localparam [2:0] PARAM_BtoN = 3'd2;
localparam [2:0] PARAM_TtoT = 3'd3;
localparam [2:0] PARAM_BtoB = 3'd4;
localparam [2:0] PARAM_NtoN = 3'd5;

