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
  logic [31:0] regs[31:0];

  // Combinational read with write-through bypass (ADR-0004, CLAUDE.md invariant
  // 6): a same-cycle write to the address currently being read forwards wdata
  // directly instead of the stale pre-write contents, so the caller sees this
  // cycle's write, not the previous cycle's answer to a different question. x0
  // always reads 0, regardless of wen/waddr.
  always_comb begin
    reg_rs1 = (rs1 == 5'd0) ? 32'b0 : (wen && waddr == rs1) ? wdata : regs[rs1];
    reg_rs2 = (rs2 == 5'd0) ? 32'b0 : (wen && waddr == rs2) ? wdata : regs[rs2];
  end

  always_ff @(posedge clk) begin
    if (wen && waddr != 5'd0) regs[waddr] <= wdata;
  end
endmodule
