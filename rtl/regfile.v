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
  logic [31:0] regs[30:0];

  always_ff @(posedge clk) begin
    reg_rs1 <= rs1 > 0 ? regs[rs1 - 1] : 0;
    reg_rs2 <= rs2 > 0 ? regs[rs2 - 1] : 0;
    if (wen) begin
      if(waddr > 0) regs[waddr - 1] <= wdata;
    end
  end
endmodule
