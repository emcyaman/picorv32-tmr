// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

`timescale 1 ns / 1 ps

`ifndef VERILATOR
module testbench #(
    parameter AXI_TEST = 0,
    parameter VERBOSE = 0
);
    reg clk = 1;
    reg resetn = 0;
    wire trap;

    always #5 clk = ~clk;

    // integer cntr = 0;
    // always @(posedge clk) begin
    //     if (cntr % 1000 == 0) begin
    //         $display("Counter cycle: %d", cntr);
    //     end
    //     cntr = cntr + 1;
    // end

    initial begin
        repeat (100) @(posedge clk);
        resetn <= 1;
    end

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("testbench_tmr.vcd");
            $dumpvars(0, testbench);
        end
        repeat (1000000) @(posedge clk);
        $display("TIMEOUT");
        $finish;
    end

    wire trace_valid;
    wire [35:0] trace_data;
    integer trace_file;

    initial begin
        if ($test$plusargs("trace")) begin
            trace_file = $fopen("testbench_tmr.trace", "w");
            repeat (10) @(posedge clk);
            while (!trap) begin
                @(posedge clk);
                if (trace_valid)
                    $fwrite(trace_file, "%x\n", trace_data);
            end
            $fclose(trace_file);
            $display("Finished writing testbench_tmr.trace.");
        end
    end

    picorv32_wrapper #(
        .AXI_TEST (AXI_TEST),
        .VERBOSE  (VERBOSE)
    ) top (
        .clk(clk),
        .resetn(resetn),
        .trap(trap),
        .trace_valid(trace_valid),
        .trace_data(trace_data)
    );
endmodule
`endif

module picorv32_wrapper #(
    parameter AXI_TEST = 0,
    parameter VERBOSE = 0
) (
    input clk,
    input resetn,
    output trap,
    output trace_valid,
    output [35:0] trace_data
);
    wire tests_passed;
    reg [31:0] irq = 0;

    reg [15:0] count_cycle = 0;
    always @(posedge clk) count_cycle <= resetn ? count_cycle + 1 : 0;

    always @* begin
        irq = 0;
        irq[4] = &count_cycle[12:0];
        irq[5] = &count_cycle[15:0];
    end

    wire        mem_valid;
    wire        mem_ready;
    
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [ 3:0] mem_wstrb;
    wire [31:0] mem_rdata;

    memory #(
        .SIZE 	   (128*1024/4),
        .VERBOSE   (VERBOSE)
    ) mem (
        .clk  	   (clk),
        
        .mem_valid (mem_valid),
        .mem_ready (mem_ready),

        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_wstrb (mem_wstrb),
        .mem_rdata (mem_rdata),

        .tests_passed (tests_passed)
    );

    picorv32 #(
`ifndef SYNTH_TEST
`ifdef SP_TEST
        .ENABLE_REGS_DUALPORT(0),
`endif
`ifdef COMPRESSED_ISA
        .COMPRESSED_ISA(1),
`endif
        .ENABLE_MUL(1),
        .ENABLE_DIV(1),
        .ENABLE_IRQ(1),
        .ENABLE_TRACE(1)
`endif
    ) uut (
        .clk            (clk            ),
        .resetn         (resetn         ),
        .trap           (trap           ),
        .mem_valid      (mem_valid      ),
        .mem_ready      (mem_ready      ),
        .mem_addr       (mem_addr       ),
        .mem_wdata      (mem_wdata      ),
        .mem_wstrb      (mem_wstrb      ),
        .mem_rdata      (mem_rdata      ),
        .irq            (irq            ),
        .trace_valid    (trace_valid    ),
        .trace_data     (trace_data     )
    );

    reg [1023:0] firmware_file;
    initial begin
        if (!$value$plusargs("firmware=%s", firmware_file))
            firmware_file = "firmware/firmware.hex";
        $readmemh(firmware_file, mem.memory);
    end

    integer cycle_counter;
    always @(posedge clk) begin
        cycle_counter <= resetn ? cycle_counter + 1 : 0;
        if (resetn && trap) begin
`ifndef VERILATOR
            repeat (10) @(posedge clk);
`endif
            $display("TRAP after %1d clock cycles", cycle_counter);
            if (tests_passed) begin
                $display("ALL TESTS PASSED.");
                $finish;
            end else begin
                $display("ERROR!");
                if ($test$plusargs("noerror"))
                    $finish;
                $stop;
            end
        end
    end
endmodule

module memory #(
    parameter SIZE = 1024,
    parameter VERBOSE = 0
)
(
    input clk,
    
    input mem_valid,
    output reg mem_ready,

    input [31:0] mem_addr,
    input [31:0] mem_wdata,
    input [ 3:0] mem_wstrb,
    output reg [31:0] mem_rdata,

    output reg tests_passed
);

    reg [31:0] memory [0:SIZE-1];
	reg verbose;
	initial verbose = $test$plusargs("verbose") || VERBOSE;

    initial begin
        mem_ready = 0;
        mem_rdata = 0;
        tests_passed = 0;
    end

    always @(posedge clk) begin
        mem_ready <= 0;
        if (mem_valid && !mem_ready) begin
            // SRAM access
            if (mem_addr < SIZE * 4) begin
                // Read from memory
                mem_rdata <= memory[mem_addr >> 2];
                // Write to memory
                if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
                if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
                if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
                if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
            end
            // Console access
            else if (mem_addr == 32'h1000_0000) begin
                if (verbose) begin
                    // Write a character or whole word to the console
                    if (32 <= mem_addr && mem_addr < 128)
                        $display("OUT: '%c'", mem_wdata[7:0]);
                    else
                        $display("OUT: %3d", mem_wdata);
                end
                else
                    $write("%c", mem_wdata[7:0]);
            end
            // Test end signalling
            else if (mem_addr == 32'h2000_0000) begin
                if (mem_wdata == 123456789)
                    tests_passed = 1;
            end
            // Invalid memory access
            else begin
                if (mem_wstrb != 0)
                    $display("OUT-OF-BOUND MEMORY WRITE TO %08x", mem_addr);
                else
                    $display("OUT-OF-BOUND MEMORY READ FROM %08x", mem_addr);
                $finish;
            end
            mem_ready <= 1;
        end
    end

endmodule