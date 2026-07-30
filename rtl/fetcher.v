`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module fetcher(
  input  logic clk,
  input  logic reset,
  // inputs
  input  logic [31:0] pc,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  // outputs
  output fetcher_output out
);

  // ADR-0003: dual-word combinational fetch window. A 32-bit instruction can
  // straddle a 4-byte boundary when pc[1] is set (i.e. it immediately
  // follows a compressed instruction), so fetch always reads the
  // word-aligned pair straddling pc and windows the 32 bits starting at pc
  // out of the two -- stateless, so a straddle costs nothing: no aligner
  // FSM, no buffer, no stall (CLAUDE.md invariant 1).
  assign imem_addr  = {pc[31:2], 2'b00};
  assign imem_addr2 = imem_addr + 4;

  // Named continuous assigns, not part-selects inside the always_comb below
  // (CLAUDE.md's documented `sorry:` exception is rtl/executor.v only).
  logic [63:0] fetch_pair;
  assign fetch_pair = {imem_data2, imem_data} >> (pc[1] ? 16 : 0);
  logic [31:0] windowed_instr;
  assign windowed_instr = fetch_pair[31:0];

  always_comb begin
    if (reset) begin
      out.valid = 1'b0;
      out.pc = 32'b0;
      out.instr = 32'b0;
    end else begin
      // Fetch never presents a wrong-path instruction (CLAUDE.md invariant 1),
      // so it is valid on every non-reset cycle.
      out.valid = 1'b1;
      out.instr = windowed_instr;
      out.pc = pc;
    end
  end

endmodule
