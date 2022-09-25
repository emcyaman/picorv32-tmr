`timescale 1 ns / 1 ps

module word_voter_tb;
    reg clk;
    reg [31:0] a;
    reg [31:0] b;
    reg [31:0] c;
    reg [31:0] a_reg;
    reg [31:0] b_reg;
    reg [31:0] c_reg;
    wire [31:0] out;
    wire [ 2:0] err;

    initial begin
        clk = 1'b0;
        forever begin
            #1 clk = ~clk;
        end
    end

    initial begin
        $dumpfile("word_voter_tb.vcd");
        $dumpvars(0, word_voter_tb);
        a = $random;
        b = a;
        c = a;
        #5
        $display("a = %x, b = %x, c = %x, out = %x, err = %b", a, b, c, out, err);
        if (out != a || err != 3'b000) begin
            $display("ERROR! Invalid output");
            $finish;
        end
        a = $random;
        b = a;
        c = $random;
        #5
        $display("a = %x, b = %x, c = %x, out = %x, err = %b", a, b, c, out, err);
        if (out != a || err != 3'b100) begin
            $display("ERROR! Invalid output");
            $finish;
        end
        a = $random;
        b = $random;
        c = a;
        #5
        $display("a = %x, b = %x, c = %x, out = %x, err = %b", a, b, c, out, err);
        if (out != a || err != 3'b010) begin
            $display("ERROR! Invalid output");
            $finish;
        end
        a = $random;
        b = $random;
        c = b;
        #5
        $display("a = %x, b = %x, c = %x, out = %x, err = %b", a, b, c, out, err);
        if (out != b || err != 3'b001) begin
            $display("ERROR! Invalid output");
            $finish;
        end
        a = $random;
        b = $random;
        c = $random;
        #5
        $display("a = %x, b = %x, c = %x, out = %x, err = %b", a, b, c, out, err);
        if (err != 3'b111) begin
            $display("ERROR! Invalid output");
            $finish;
        end
        #5
        $display("Completed successfully!");
        $finish;
    end

    always @(posedge clk) begin
        a_reg <= a;
        b_reg <= b;
        c_reg <= c;
    end

    word_voter #(32) uut (a_reg, b_reg, c_reg, out, err);

endmodule
