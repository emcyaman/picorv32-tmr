`timescale 1 ns / 1 ps

module word_voter #(
    // Width of the word (1-32)
    parameter N = 1
) (
    input  [N-1:0] IN [3],
`ifdef TMR_INJECT_ERR
    input  [  2:0] ERR_INJECT,
`endif
    output [N-1:0] OUT,
    output [  2:0] ERROR
);
    wire [N-1:0] A, B, C;

    wire match_A_B;
    wire match_A_C;
    wire match_B_C;

`ifdef TMR_INJECT_ERR
    assign A = ERR_INJECT [0] ? ~(IN [0]) : IN [0];
    assign B = ERR_INJECT [1] ? ~(IN [1]) : IN [1];
    assign C = ERR_INJECT [2] ? ~(IN [2]) : IN [2];
`endif
`ifndef TMR_INJECT_ERR
    assign A = IN [0];
    assign B = IN [1];
    assign C = IN [2];
`endif

    match #(.N(N)) comp_AB (.A(A), .B(B), .OUT(match_A_B));
    match #(.N(N)) comp_AC (.A(A), .B(C), .OUT(match_A_C));
    match #(.N(N)) comp_BC (.A(B), .B(C), .OUT(match_B_C));

    assign OUT = match_A_C ? A : B;
    assign ERROR = {~(match_A_C | match_B_C), ~(match_A_B | match_B_C), ~(match_A_B | match_A_C)};

endmodule

module match #(
    // Width of the word (1-32)
    parameter N = 1
) (
    input   [N-1:0] A,
    input   [N-1:0] B,
    output  OUT
);
    wire [N-1:0] w0;

    assign w0 = ~(A ^ B);
    assign OUT = &w0;

endmodule
