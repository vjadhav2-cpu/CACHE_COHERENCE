`ifndef TL_SRAM_CTRL_PKG_VH
`define TL_SRAM_CTRL_PKG_VH

// ============================================================================
// TileLink TL-UH SRAM Memory Controller Configuration
// Target: 8MB Main Memory (4 banks × 2MB each)
// ============================================================================

// TileLink Bus Parameters
parameter TL_AW  = 64;           // Address width
parameter TL_DW  = 64;           // Data width (64-bit / 8-byte beats)
parameter TL_DBW = TL_DW / 8;    // Data byte width (8)
parameter TL_SZW = 8;            // Size field width (supports up to 2^7 = 128 bytes)
parameter TL_AIW = 4;            // Source ID width (supports 16 outstanding per master)
parameter TL_DIW = 1;            // Sink ID width

// Memory Organization
// 8MB total = 4 banks × 2MB per bank
// 2MB per bank = 262144 entries × 8 bytes
parameter NUM_BANKS      = 4;
parameter BANK_DEPTH     = 262144;  // 2MB per bank (262144 × 8 bytes)
parameter BANK_AW        = 18;      // log2(262144) = 18 bits
parameter BANK_SEL_BITS  = 2;       // log2(4 banks) = 2 bits
parameter BANK_SEL_LO    = 21;      // Bank select starts at bit 21
parameter BANK_SEL_HI    = 22;      // Bank select ends at bit 22
// Address mapping: [31:23] unused | [22:21] bank | [20:3] offset | [2:0] byte

// Outstanding Transaction Support
parameter OUTSTANDING_REQS = 16;    // Per-source outstanding requests

// Cache Line Configuration (for LLC writebacks)
parameter CACHE_LINE_BYTES = 64;    // 64-byte cache lines
parameter CACHE_LINE_BEATS = 8;     // 64 bytes / 8 bytes per beat = 8 beats

// ============================================================================
// TileLink TL-UH Opcodes (Channel A - Request)
// ============================================================================
localparam [2:0] TL_A_PUT_FULL    = 3'd0;  // Full data write
localparam [2:0] TL_A_PUT_PARTIAL = 3'd1;  // Partial data write (with mask)
localparam [2:0] TL_A_ARITHMETIC  = 3'd2;  // Atomic arithmetic (ADD, MIN, MAX, etc.)
localparam [2:0] TL_A_LOGICAL     = 3'd3;  // Atomic logical (AND, OR, XOR, SWAP)
localparam [2:0] TL_A_GET         = 3'd4;  // Read request
localparam [2:0] TL_A_INTENT      = 3'd5;  // Prefetch hint
localparam [2:0] TL_A_ACQUIRE_BLK = 3'd6;  // TL-C only (not used in TL-UH)
localparam [2:0] TL_A_ACQUIRE_PRM = 3'd7;  // TL-C only (not used in TL-UH)

// ============================================================================
// TileLink TL-UH Opcodes (Channel D - Response)
// ============================================================================
localparam [2:0] TL_D_ACCESS_ACK      = 3'd0;  // Write acknowledgment
localparam [2:0] TL_D_ACCESS_ACK_DATA = 3'd1;  // Read data response
localparam [2:0] TL_D_HINT_ACK        = 3'd2;  // Prefetch hint acknowledgment

// ============================================================================
// Atomic Operation Parameters (ArithmeticData)
// ============================================================================
localparam [2:0] TL_PARAM_MIN  = 3'd0;  // Signed minimum
localparam [2:0] TL_PARAM_MAX  = 3'd1;  // Signed maximum
localparam [2:0] TL_PARAM_MINU = 3'd2;  // Unsigned minimum
localparam [2:0] TL_PARAM_MAXU = 3'd3;  // Unsigned maximum
localparam [2:0] TL_PARAM_ADD  = 3'd4;  // Addition

// ============================================================================
// Atomic Operation Parameters (LogicalData)
// ============================================================================
localparam [2:0] TL_PARAM_XOR  = 3'd0;  // Bitwise XOR
localparam [2:0] TL_PARAM_OR   = 3'd1;  // Bitwise OR
localparam [2:0] TL_PARAM_AND  = 3'd2;  // Bitwise AND
localparam [2:0] TL_PARAM_SWAP = 3'd3;  // Swap (exchange)

// ============================================================================
// Intent (Prefetch) Parameters
// ============================================================================
localparam [2:0] TL_PARAM_PREFETCH_READ  = 3'd0;
localparam [2:0] TL_PARAM_PREFETCH_WRITE = 3'd1;

`endif
