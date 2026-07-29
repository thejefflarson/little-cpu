`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// Decode-vector bench (see `eb18320`): confirms the SLTI/SLTIU/XORI
// immediate-source fix (decoder.v's `instr_shift`) and the funct12-exact EBREAK
// fix, directly against the decoder — no full pipeline needed for either check.
module decoder_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  fetcher_output in;
  logic [31:0] reg_rs1, reg_rs2;
  logic [31:0] pc;
  logic [4:0] rs1, rs2;
  decoder_output out;
  // No in-flight producer at the executor stage and no divide in progress:
  // this bench exercises decode vectors in isolation, not the hazard
  // scoreboard (see test/regfile_tb.v and test/asm/hazard.S for that).
  executor_output executor_out = '0;
  logic divider_stall = 1'b0;
  logic accessor_stall = 1'b0;
  logic accessor_pending_valid = 1'b0;
  logic [4:0] accessor_pending_rd = 5'b0;

  decoder dut (
    .clk(clk),
    .reset(reset),
    .in(in),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stall),
    .accessor_stall(accessor_stall),
    .accessor_pending_valid(accessor_pending_valid),
    .accessor_pending_rd(accessor_pending_rd),
    .pc(pc),
    .rs1(rs1),
    .rs2(rs2),
    .out(out)
  );

  int errors = 0;

  task automatic check_hex(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_bit(input string what, input logic got, input logic expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%b expected=%b", what, got, expected);
        errors++;
      end
    end
  endtask

  initial begin
    reset = 1;
    in = '0;
    reg_rs1 = 0;
    reg_rs2 = 0;
    repeat (2) @(posedge clk);
    #1;
    reset = 0;

    // xori x1, x2, -1  =>  imm=0xfff (sign -1), rs1=x2, funct3=100, rd=x1,
    // opcode=0010011 (I-type math-immediate).
    in.instr = 32'hfff14093;
    in.pc = 32'h0;
    #1; // math_arg is purely combinational off `in.instr`; no clock edge needed
    check_hex("xori math_arg", dut.math_arg, 32'hffffffff);
    @(posedge clk);
    #1;
    check_hex("xori out.rs2 (registered math_arg)", out.rs2, 32'hffffffff);

    // ebreak: 0x00100073 (funct12 == 1) must set is_ebreak.
    in.instr = 32'h00100073;
    @(posedge clk);
    #1;
    check_bit("ebreak sets is_ebreak", out.is_ebreak, 1'b1);

    // mret: 0x30200073 (funct12 == 0x302) must NOT set is_ebreak.
    in.instr = 32'h30200073;
    @(posedge clk);
    #1;
    check_bit("mret does not set is_ebreak", out.is_ebreak, 1'b0);

    // wfi: 0x10500073 (funct12 == 0x105) must NOT set is_ebreak.
    in.instr = 32'h10500073;
    @(posedge clk);
    #1;
    check_bit("wfi does not set is_ebreak", out.is_ebreak, 1'b0);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: decode vectors (xori immediate, ebreak/mret/wfi)");
      $finish;
    end
  end
endmodule
