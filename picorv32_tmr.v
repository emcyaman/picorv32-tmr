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
    output trap,

    output        mem_valid,
    output        mem_instr,
    input         mem_ready,

    output [31:0] mem_addr,
    output [31:0] mem_wdata,
    output [ 3:0] mem_wstrb,
    input  [31:0] mem_rdata,

    // Look-Ahead Interface
    output        mem_la_read,
    output        mem_la_write,
    output [31:0] mem_la_addr,
    output [31:0] mem_la_wdata,
    output [ 3:0] mem_la_wstrb,

    // Pico Co-Processor Interface (PCPI)
    output        pcpi_valid,
    output [31:0] pcpi_insn,
    output [31:0] pcpi_rs1,
    output [31:0] pcpi_rs2,
    input         pcpi_wr,
    input  [31:0] pcpi_rd,
    input         pcpi_wait,
    input         pcpi_ready,

    // IRQ Interface
    input  [31:0] irq,
    output [31:0] eoi,

    // Trace Interface
    output        trace_valid,
    output [35:0] trace_data,

    // TMR related interfaces
`ifdef TMR_INJECT_ERR
    input  [ 2:0] tmr_inject [14],
`endif
    output [ 2:0] tmr_errors [14]
);

    wire        resetn_tmr          [3];
    wire        trap_tmr            [3];

    wire        mem_valid_tmr       [3];
    wire        mem_instr_tmr       [3];

    wire [31:0] mem_addr_tmr        [3];
    wire [31:0] mem_wdata_tmr       [3];
    wire [ 3:0] mem_wstrb_tmr       [3];

    wire        mem_la_read_tmr     [3];
    wire        mem_la_write_tmr    [3];
    wire [31:0] mem_la_addr_tmr     [3];
    wire [31:0] mem_la_wdata_tmr    [3];
    wire [ 3:0] mem_la_wstrb_tmr    [3];

    wire        pcpi_valid_tmr      [3];
    wire [31:0] pcpi_insn_tmr       [3];
    wire [31:0] pcpi_rs1_tmr        [3];
    wire [31:0] pcpi_rs2_tmr        [3];
    
    wire [31:0] eoi_tmr             [3];

    wire        trace_valid_tmr     [3];
    wire [35:0] trace_data_tmr      [3];

    wire [5:0]  mem_signals         [3];
    wire [3:0]  mem_la_wstrb_signal [3];
    wire [5:0]  mem_la_signals      [3];

    genvar i;

    generate
      for (i = 0; i < 3; i = i+1) begin

        picorv32 #(
          ENABLE_COUNTERS,
          ENABLE_COUNTERS64,
          ENABLE_REGS_16_31,
          ENABLE_REGS_DUALPORT,
          LATCHED_MEM_RDATA,
          TWO_STAGE_SHIFT,
          BARREL_SHIFTER,
          TWO_CYCLE_COMPARE,
          TWO_CYCLE_ALU,
          COMPRESSED_ISA,
          CATCH_MISALIGN,
          CATCH_ILLINSN,
          ENABLE_PCPI,
          ENABLE_MUL,
          ENABLE_FAST_MUL,
          ENABLE_DIV,
          ENABLE_IRQ,
          ENABLE_IRQ_QREGS,
          ENABLE_IRQ_TIMER,
          ENABLE_TRACE,
          REGS_INIT_ZERO,
          MASKED_IRQ,
          LATCHED_IRQ,
          PROGADDR_RESET,
          PROGADDR_IRQ,
          STACKADDR
        ) core_u0 (
            .clk          (clk                ),
            .resetn       (resetn             ),
            .trap         (trap_tmr   [i]     ),
            
            // Memory Interface
            .mem_valid    (mem_valid_tmr [i]  ),
            .mem_instr    (mem_instr_tmr [i]  ),
            .mem_ready    (mem_ready          ),
            .mem_addr     (mem_addr_tmr  [i]  ),
            .mem_wdata    (mem_wdata_tmr [i]  ),
            .mem_wstrb    (mem_wstrb_tmr [i]  ),
            .mem_rdata    (mem_rdata          ),
            
            // Pico Co-Processor Interface (PCPI)
            .pcpi_valid   (pcpi_valid_tmr [i] ),
            .pcpi_insn    (pcpi_insn_tmr  [i] ),
            .pcpi_rs1     (pcpi_rs1_tmr   [i] ),
            .pcpi_rs2     (pcpi_rs2_tmr   [i] ),
            .pcpi_wr      (pcpi_wr            ),
            .pcpi_rd      (pcpi_rd            ),
            .pcpi_wait    (pcpi_wait          ),
            .pcpi_ready   (pcpi_ready         ),

            // IRQ Interface
            .irq          (irq                ),
            .eoi          (eoi_tmr [i]        ),

            // Trace Interface
            .trace_valid  (trace_valid_tmr [i]),
            .trace_data   (trace_data_tmr  [i])
        );

        assign mem_signals[i] = {mem_valid_tmr [i], mem_instr_tmr [i], mem_wstrb_tmr [i]};
        assign mem_la_wstrb_signal[i] = mem_la_wstrb_tmr[i];
        assign mem_la_signals[i] = {mem_la_read_tmr[i], mem_la_write_tmr[i], mem_la_wstrb_signal[i]};

      end
    endgenerate

`ifndef TMR_INJECT_ERR
    word_voter #(1)  voter_trap (trap_tmr, trap, tmr_errors [0]);

    word_voter #(6)  voter_mem_signals (mem_signals, {mem_valid, mem_instr, mem_wstrb}, tmr_errors [1]);
    word_voter #(32) voter_mem_wdata (mem_addr_tmr, mem_addr, tmr_errors [2]);
    word_voter #(32) voter_mem_addr (mem_wdata_tmr, mem_wdata, tmr_errors [3]);

    word_voter #(6)  voter_mem_la_signals (mem_la_signals, {mem_la_read, mem_la_write, mem_la_wstrb}, tmr_errors [4]);
    word_voter #(32) voter_mem_la_addr (mem_la_addr_tmr, mem_la_addr, tmr_errors [5]);
    word_voter #(32) voter_mem_la_wdata (mem_la_wdata_tmr, mem_la_wdata, tmr_errors [6]);

    word_voter #(1)  voter_pcpi_valid (pcpi_valid_tmr, pcpi_valid, tmr_errors [7]);
    word_voter #(32) voter_pcpi_insn (pcpi_insn_tmr, pcpi_insn, tmr_errors [8]);
    word_voter #(32) voter_pcpi_rs1 (pcpi_rs1_tmr, pcpi_rs1, tmr_errors [9]);
    word_voter #(32) voter_pcpi_rs2 (pcpi_rs2_tmr, pcpi_rs2, tmr_errors [10]);

    word_voter #(32) voter_eoi (eoi_tmr, eoi, tmr_errors [11]);

    word_voter #(1)  voter_trace_valid (trace_valid_tmr, trace_valid, tmr_errors [12]);
    word_voter #(36) voter_trace_data  (trace_data_tmr, trace_data, tmr_errors [13]);
`endif 

`ifdef TMR_INJECT_ERR
    word_voter #(1)  voter_trap (trap_tmr, tmr_inject [0], trap, tmr_errors [0]);

    word_voter #(6)  voter_mem_signals (mem_signals, tmr_inject [1], {mem_valid, mem_instr, mem_wstrb}, tmr_errors [1]);
    word_voter #(32) voter_mem_wdata (mem_addr_tmr, tmr_inject [2], mem_addr, tmr_errors [2]);
    word_voter #(32) voter_mem_addr (mem_wdata_tmr, tmr_inject [3], mem_wdata, tmr_errors [3]);

    word_voter #(6)  voter_mem_la_signals (mem_la_signals, tmr_inject [4], {mem_la_read, mem_la_write, mem_la_wstrb}, tmr_errors [4]);
    word_voter #(32) voter_mem_la_addr (mem_la_addr_tmr, tmr_inject [5], mem_la_addr, tmr_errors [5]);
    word_voter #(32) voter_mem_la_wdata (mem_la_wdata_tmr, tmr_inject [6], mem_la_wdata, tmr_errors [6]);

    word_voter #(1)  voter_pcpi_valid (pcpi_valid_tmr, tmr_inject [7], pcpi_valid, tmr_errors [7]);
    word_voter #(32) voter_pcpi_insn (pcpi_insn_tmr, tmr_inject [8], pcpi_insn, tmr_errors [8]);
    word_voter #(32) voter_pcpi_rs1 (pcpi_rs1_tmr, tmr_inject [9], pcpi_rs1, tmr_errors [9]);
    word_voter #(32) voter_pcpi_rs2 (pcpi_rs2_tmr, tmr_inject [10], pcpi_rs2, tmr_errors [10]);

    word_voter #(32) voter_eoi (eoi_tmr, tmr_inject [11], eoi, tmr_errors [11]);

    word_voter #(1)  voter_trace_valid (trace_valid_tmr, tmr_inject [12], trace_valid, tmr_errors [12]);
    word_voter #(36) voter_trace_data  (trace_data_tmr, tmr_inject [13], trace_data, tmr_errors [13]);
`endif

endmodule
