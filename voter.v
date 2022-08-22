`timescale 1 ns / 1 ps

module voter #(
    parameter [4:0] N = 1,
    parameter [0:0] ERROR = 0
) (
    input  [N-1:0] A,
    input  [N-1:0] B,
    input  [N-1:0] C,
    output [N-1:0] OUT,
);

    assign OUT = (A & B) | (A & C) | (B & C);

endmodule