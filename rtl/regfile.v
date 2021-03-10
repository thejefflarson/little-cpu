`default_nettype none
module regfile(
  input  var logic         clk,
  input  var logic [4:0]   rs1,
  input  var logic [4:0]   rs2,
  output var logic [31:0] reg_rs1,
  output var logic [31:0] reg_rs2,
  input  var logic         wen,
  input  var logic [4:0]   waddr,
  input  var logic [31:0]  wdata
);
  logic [31:0] regs[0:31];

  always_ff @(posedge clk) begin
    reg_rs1 <= regs[rs1];
    reg_rs2 <= regs[rs2];
    if (wen) begin
      regs[waddr] <= wdata;
    end
  end
endmodule
