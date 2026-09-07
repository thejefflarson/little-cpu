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
  // Two copies of one register file: an ice40 EBR has one read port.
  logic [31:0] regs_a[31:0];
  logic [31:0] regs_b[31:0];
  logic [31:0] read_a;
  logic [31:0] read_b;
  logic [4:0]  held_rs1;
  logic [4:0]  held_rs2;

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

  always_comb begin
    reg_rs1 = (held_rs1 == 5'd0) ? 32'b0 : (wen && waddr == held_rs1) ? wdata : read_a;
    reg_rs2 = (held_rs2 == 5'd0) ? 32'b0 : (wen && waddr == held_rs2) ? wdata : read_b;
  end
endmodule
