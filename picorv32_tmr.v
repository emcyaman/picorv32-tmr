/*
 *  PicoRV32 -- A Small RISC-V (RV32I) Processor Core
 *
 *  Copyright (C) 2015  Claire Xenia Wolf <claire@yosyshq.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/* verilator lint_off WIDTH */
/* verilator lint_off PINMISSING */
/* verilator lint_off CASEOVERLAP */
/* verilator lint_off CASEINCOMPLETE */

`timescale 1 ns / 1 ps
// `default_nettype none
// `define DEBUGNETS
// `define DEBUGREGS
// `define DEBUGASM
// `define DEBUG

`ifdef DEBUG
  `define debug(debug_command) debug_command
`else
  `define debug(debug_command)
`endif

`ifdef FORMAL
  `define FORMAL_KEEP (* keep *)
  `define assert(assert_expr) assert(assert_expr)
`else
  `ifdef DEBUGNETS
    `define FORMAL_KEEP (* keep *)
  `else
    `define FORMAL_KEEP
  `endif
  `define assert(assert_expr) empty_statement
`endif

// uncomment this for register file in extra module
// `define PICORV32_REGS picorv32_regs

// this macro can be used to check if the verilog files in your
// design are read in the correct order.
`define PICORV32_V


/***************************************************************
 * picorv32
 ***************************************************************/

module picorv32_tmr #(
    parameter [ 0:0] ENABLE_COUNTERS = 1,
    parameter [ 0:0] ENABLE_COUNTERS64 = 1,
    parameter [ 0:0] ENABLE_REGS_16_31 = 1,
    parameter [ 0:0] ENABLE_REGS_DUALPORT = 1,
    parameter [ 0:0] LATCHED_MEM_RDATA = 0,
    parameter [ 0:0] TWO_STAGE_SHIFT = 1,
    parameter [ 0:0] BARREL_SHIFTER = 0,
    parameter [ 0:0] TWO_CYCLE_COMPARE = 0,
    parameter [ 0:0] TWO_CYCLE_ALU = 0,
    parameter [ 0:0] COMPRESSED_ISA = 0,
    parameter [ 0:0] CATCH_MISALIGN = 1,
    parameter [ 0:0] CATCH_ILLINSN = 1,
    parameter [ 0:0] ENABLE_PCPI = 0,
    parameter [ 0:0] ENABLE_MUL = 0,
    parameter [ 0:0] ENABLE_FAST_MUL = 0,
    parameter [ 0:0] ENABLE_DIV = 0,
    parameter [ 0:0] ENABLE_IRQ = 0,
    parameter [ 0:0] ENABLE_IRQ_QREGS = 1,
    parameter [ 0:0] ENABLE_IRQ_TIMER = 1,
    parameter [ 0:0] ENABLE_TRACE = 0,
    parameter [ 0:0] REGS_INIT_ZERO = 0,
    parameter [31:0] MASKED_IRQ = 32'h 0000_0000,
    parameter [31:0] LATCHED_IRQ = 32'h ffff_ffff,
    parameter [31:0] PROGADDR_RESET = 32'h 0000_0000,
    parameter [31:0] PROGADDR_IRQ = 32'h 0000_0010,
    parameter [31:0] STACKADDR = 32'h ffff_ffff
) (
    input clk, resetn,
    output reg trap,

    output reg        mem_valid,
    output reg        mem_instr,
    input             mem_ready,

    output reg [31:0] mem_addr,
    output reg [31:0] mem_wdata,
    output reg [ 3:0] mem_wstrb,
    input      [31:0] mem_rdata,

    // Look-Ahead Interface
    output            mem_la_read,
    output            mem_la_write,
    output     [31:0] mem_la_addr,
    output reg [31:0] mem_la_wdata,
    output reg [ 3:0] mem_la_wstrb,

    // Pico Co-Processor Interface (PCPI)
    output reg        pcpi_valid,
    output reg [31:0] pcpi_insn,
    output     [31:0] pcpi_rs1,
    output     [31:0] pcpi_rs2,
    input             pcpi_wr,
    input      [31:0] pcpi_rd,
    input             pcpi_wait,
    input             pcpi_ready,

    // IRQ Interface
    input      [31:0] irq,
    output reg [31:0] eoi,

    // Trace Interface
    output reg        trace_valid,
    output reg [35:0] trace_data
);

    wire resetn_a, resetn_b, resetn_c;
    wire trap_a, trap_b, trap_c;

    wire mem_valid_a, mem_valid_b, mem_valid_c;
    wire mem_instr_a, mem_instr_b, mem_instr_c;

    wire [31:0] mem_addr_a, mem_addr_b, mem_addr_c;
    wire [31:0] mem_wdata_a, mem_wdata_b, mem_wdata_c;
    wire [ 3:0] mem_wstrb_a, mem_wstrb_b, mem_wstrb_c;

    wire        mem_la_read_a, mem_la_read_b, mem_la_read_c;
    wire        mem_la_write_a, mem_la_write_b, mem_la_write_c;
    wire [31:0] mem_la_addr_a, mem_la_addr_b, mem_la_addr_c;
    wire [31:0] mem_la_wdata_a, mem_la_wdata_b, mem_la_wdata_c;
    wire [ 3:0] mem_la_wstrb_a, mem_la_wstrb_b, mem_la_wstrb_c;

    picorv32 #(
    .ENABLE_COUNTERS,
    .ENABLE_COUNTERS64,
    .ENABLE_REGS_16_31,
    .ENABLE_REGS_DUALPORT,
    .LATCHED_MEM_RDATA,
    .TWO_STAGE_SHIFT,
    .BARREL_SHIFTER,
    .TWO_CYCLE_COMPARE,
    .TWO_CYCLE_ALU,
    .COMPRESSED_ISA,
    .CATCH_MISALIGN,
    .CATCH_ILLINSN,
    .ENABLE_PCPI,
    .ENABLE_MUL,
    .ENABLE_FAST_MUL,
    .ENABLE_DIV,
    .ENABLE_IRQ,
    .ENABLE_IRQ_QREGS,
    .ENABLE_IRQ_TIMER,
    .ENABLE_TRACE,
    .REGS_INIT_ZERO,
    .MASKED_IRQ,
    .LATCHED_IRQ,
    .PROGADDR_RESET,
    .PROGADDR_IRQ,
    .STACKADDR
    ) core_a (
        .clk          (clk         ),
        .resetn       (resetn_a    ),
        .trap         (trap_a      ),
        
        // Memory Interface
        .mem_valid    (mem_valid_a ),
        .mem_instr    (mem_instr_a ),
        .mem_ready    (mem_ready   ),
        .mem_addr     (mem_addr_a  ),
        .mem_wdata    (mem_wdata_a ),
        .mem_wstrb    (mem_wstrb_a ),
        .mem_rdata    (mem_rdata   ),
        
        // Pico Co-Processor Interface (PCPI)
        .pcpi_valid  (pcpi_valid_a ),
        .pcpi_insn   (pcpi_insn_a  ),
        .pcpi_rs1    (pcpi_rs1_a   ),
        .pcpi_rs2    (pcpi_rs2_a   ),
        .pcpi_wr     (pcpi_wr      ),
        .pcpi_rd     (pcpi_rd      ),
        .pcpi_wait   (pcpi_wait    ),
        .pcpi_ready  (pcpi_ready   ),

        // IRQ Interface
        .irq         (irq          ),
        .eoi         (eoi_a        ),

        // Trace Interface
        .trace_valid (trace_valid_a),
        .trace_data  (trace_data_a )
    );

endmodule
