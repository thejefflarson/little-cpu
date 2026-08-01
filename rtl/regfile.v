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
  // TWO ARRAYS, ONE ARCHITECTURAL REGISTER FILE (ADR-0042). An ice40 EBR has
  // one read port, so a second read port means a second copy of the state. The
  // duplication is forced by the storage element, not chosen: both arrays are
  // written from the same address and the same data word, every time, and
  // `regs_a[i] == regs_b[i]` is an invariant test/regfile_tb.v asserts
  // directly. Nothing else would notice them drifting -- test/cosim.cc reads
  // regs_a alone, and reg_rs2 is the only consumer of regs_b.
  //
  // Before ADR-0042 this was one flip-flop array with a combinational 32:1 read
  // mux, and nextpnr placed the core at 6971 logic cells, 132% of an up5k.
  // Moving these two arrays into block RAM is what takes it to 79%; yosys
  // infers the EBRs from the plain arrays below with no attribute and no
  // explicit primitive.
  logic [31:0] regs_a[31:0];
  logic [31:0] regs_b[31:0];
  logic [31:0] read_a;
  logic [31:0] read_b;

  // THE READ IS REGISTERED, so the operand for the address presented in cycle N
  // appears on reg_rs1/reg_rs2 in cycle N+1 -- a whole cycle after decode needs
  // it, because decode computes branch targets and load/store addresses from
  // these values in the cycle it issues. So decode presents the address,
  // bubbles for one cycle, and issues on the next: `operand_stall` in
  // rtl/decoder.v. CLAUDE.md invariant 9 is the rule that makes it correct, and
  // it is not local to this file.
  //
  // The write-first term is not decoration. Without it, a write committed at
  // the very posedge that captures the read would be missed and the operand
  // would be exactly one writeback stale -- the ADR-0004 defect, re-introduced
  // one cycle earlier where the scoreboard cannot see it. An ice40 EBR has no
  // write-first mode, so yosys builds this comparator and mux in fabric, which
  // is the right place for it: it is the part a reader most needs to see.
  always_ff @(posedge clk) begin
    read_a <= (wen && waddr == rs1) ? wdata : regs_a[rs1];
    read_b <= (wen && waddr == rs2) ? wdata : regs_b[rs2];
    if (wen && waddr != 5'd0) begin
      regs_a[waddr] <= wdata;
      regs_b[waddr] <= wdata;
    end
  end

  // Write-through bypass (ADR-0004, CLAUDE.md invariant 6), unchanged in intent
  // from the flip-flop regfile: a write presented in the cycle the operand is
  // USED forwards wdata directly instead of the value captured a cycle ago.
  //
  // The two forwarding points together -- write-first above, this bypass here
  // -- are what make the operand the cycle-N architectural value *including* a
  // cycle-N writeback, which is the whole observable content of invariant 6 and
  // the reason ADR-0004's stall-only scoreboard needs no forwarding path for
  // the writeback slot. Deleting either one is a direct invariant-6 violation;
  // deleting the rs2 half of this one is the ladder's liveness probe (ADR-0040).
  // x0 always reads 0, regardless of wen/waddr.
  always_comb begin
    reg_rs1 = (rs1 == 5'd0) ? 32'b0 : (wen && waddr == rs1) ? wdata : read_a;
    reg_rs2 = (rs2 == 5'd0) ? 32'b0 : (wen && waddr == rs2) ? wdata : read_b;
  end
endmodule
