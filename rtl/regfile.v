`timescale 1 ns / 1 ps
`default_nettype none
module regfile(
  input  logic        clk,
  input  logic [4:0]  rs1,
  input  logic [4:0]  rs2,
  output logic [31:0] reg_rs1,
  output logic [31:0] reg_rs2,
  input  logic        wen,
  input  logic [4:0]  waddr,
  input  logic [31:0] wdata
);
  // Two arrays, one architectural register file: an ice40 EBR has one read
  // port, so a second read port means a second copy of the state. Both are
  // written from the same address and data word every time, and
  // `regs_a[i] == regs_b[i]` is asserted in test/regfile_tb.v because nothing
  // else would notice them drifting -- test/cosim.cc reads regs_a alone, and
  // reg_rs2 is the only consumer of regs_b. yosys infers the EBRs from these
  // plain arrays with no attribute (ADR-0042).
  logic [31:0] regs_a[31:0];
  logic [31:0] regs_b[31:0];
  logic [31:0] read_a;
  logic [31:0] read_b;
  logic [4:0]  held_rs1;
  logic [4:0]  held_rs2;

  // The read is registered, so the operand for the address presented in cycle N
  // appears in cycle N+1 -- a cycle after decode needs it. Decode therefore
  // presents the address, bubbles, and issues on the next cycle
  // (`operand_stall` in rtl/decoder.v).
  //
  // The write-first term is the only forwarding here, and it is what makes one
  // stalled cycle enough for a register being written this cycle: the writer is
  // in the decode scoreboard while it is in writeback, so the reader waits, and
  // the edge it waits over is the one this term captures. Without it the operand
  // would be exactly one writeback stale with nothing left to catch it. An ice40
  // EBR has no write-first mode, so yosys builds this comparator and mux in
  // fabric.
  always_ff @(posedge clk) begin
    read_a   <= (wen && waddr == rs1) ? wdata : regs_a[rs1];
    read_b   <= (wen && waddr == rs2) ? wdata : regs_b[rs2];
    held_rs1 <= rs1;
    held_rs2 <= rs2;
    if (wen && waddr != 5'd0) begin
      regs_a[waddr] <= wdata;
      regs_b[waddr] <= wdata;
    end
  end

  // x0 always reads 0, regardless of wen/waddr, and the array is never written
  // at address 0 -- so this is the whole of the read path. Everything else
  // arrives already registered, which is what keeps the writeback stage's data
  // out of the branch comparator and out of the fetch loop behind it.
  //
  // The zero test reads the held pair, not `rs1`/`rs2`. `rs1` is instruction
  // bits, so selecting on it would put this comparator after the fetched word,
  // where the whole decode head already is. The held pair is the pair the
  // instruction now issuing reads because `operand_stall` in rtl/decoder.v lets
  // nothing issue until it is; the pair presented in the same cycle is a guess
  // at the next instruction's and is deliberately something else.
  always_comb begin
    reg_rs1 = (held_rs1 == 5'd0) ? 32'b0 : read_a;
    reg_rs2 = (held_rs2 == 5'd0) ? 32'b0 : read_b;
  end
endmodule
