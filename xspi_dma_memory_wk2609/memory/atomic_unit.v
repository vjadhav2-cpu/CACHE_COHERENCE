`include "tl_sram_ctrl_pkg.vh"

module atomic_unit #(
    parameter DW = 64
) (
    input  wire [   2:0] opcode,
    input  wire [   2:0] param,
    input  wire [DW-1:0] operand_a,
    input  wire [DW-1:0] operand_b,
    output reg  [DW-1:0] result,
    output wire [DW-1:0] original
);

  assign original = operand_a;

  wire signed [DW-1:0] a_signed = operand_a;
  wire signed [DW-1:0] b_signed = operand_b;

  always @(*) begin
    result = operand_a;

    case (opcode)
      TL_A_ARITHMETIC: begin
        case (param)
          TL_PARAM_MIN: begin
            result = (a_signed < b_signed) ? operand_a : operand_b;
          end
          TL_PARAM_MAX: begin
            result = (a_signed > b_signed) ? operand_a : operand_b;
          end
          TL_PARAM_MINU: begin
            result = (operand_a < operand_b) ? operand_a : operand_b;
          end
          TL_PARAM_MAXU: begin
            result = (operand_a > operand_b) ? operand_a : operand_b;
          end
          TL_PARAM_ADD: begin
            result = operand_a + operand_b;
          end
          default: begin
            result = operand_a;
          end
        endcase
      end

      TL_A_LOGICAL: begin
        case (param)
          TL_PARAM_XOR: begin
            result = operand_a ^ operand_b;
          end
          TL_PARAM_OR: begin
            result = operand_a | operand_b;
          end
          TL_PARAM_AND: begin
            result = operand_a & operand_b;
          end
          TL_PARAM_SWAP: begin
            result = operand_b;
          end
          default: begin
            result = operand_a;
          end
        endcase
      end

      default: begin
        result = operand_a;
      end
    endcase
  end

endmodule
