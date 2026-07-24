`timescale 1ns/1ps

module day14 #(
    parameter NUM_PORTS = 3,
    parameter NUM_MASTERS = 3   // Alias for NUM_PORTS
)(
    input  wire [NUM_PORTS-1:0] req_i,
    output wire [NUM_PORTS-1:0] gnt_o
);

    // 3-bit priority encoder without generate
    assign gnt_o[0] = req_i[0];
    assign gnt_o[1] = req_i[1] & ~req_i[0];
    assign gnt_o[2] = req_i[2] & ~(req_i[1] | req_i[0]);

endmodule