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

  // The read is registered, so the operand for the address presented in cycle N
  // appears in cycle N+1 -- a cycle after decode needs it. Decode therefore
  // presents the address, bubbles, and issues on the next cycle
  // (`operand_stall` in rtl/decoder.v, CLAUDE.md invariant 9).
  //
  // Without the write-first term a write committed at the very posedge that
  // captures the read would be missed, leaving the operand exactly one writeback
  // stale -- the ADR-0004 defect, one cycle earlier where the scoreboard cannot
  // see it. An ice40 EBR has no write-first mode, so yosys builds this
  // comparator and mux in fabric.
  always_ff @(posedge clk) begin
    read_a <= (wen && waddr == rs1) ? wdata : regs_a[rs1];
    read_b <= (wen && waddr == rs2) ? wdata : regs_b[rs2];
    if (wen && waddr != 5'd0) begin
      regs_a[waddr] <= wdata;
      regs_b[waddr] <= wdata;
    end
  end

  // Write-through bypass: a write presented in the cycle the operand is used
  // forwards wdata directly instead of the value captured a cycle ago. Together
  // with the write-first term above it makes the operand the cycle-N
  // architectural value including a cycle-N writeback, which is the whole
  // observable content of CLAUDE.md invariant 6 and the reason ADR-0004's
  // stall-only scoreboard needs no forwarding path for the writeback slot.
  // Deleting either point violates invariant 6; deleting the rs2 half of this
  // one is the ladder's liveness probe for reg_ch0 (ADR-0040).
  // x0 always reads 0, regardless of wen/waddr.
  always_comb begin
    reg_rs1 = (rs1 == 5'd0) ? 32'b0 : (wen && waddr == rs1) ? wdata : read_a;
    reg_rs2 = (rs2 == 5'd0) ? 32'b0 : (wen && waddr == rs2) ? wdata : read_b;
  end
endmodule
