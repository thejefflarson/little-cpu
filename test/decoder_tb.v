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
  // ADR-0026's drain slot: nothing is in flight in this bench, so the pipe
  // always reads as drained and a CSR instruction never serializes here.
  logic accessor_out_valid = 1'b0;
  // The CSR file (rtl/csrs.v) is a sibling of the decoder, not part of it,
  // so this bench stubs its read port. `csr_implemented` is driven per
  // vector below, because it is what decides whether a Zicsr encoding is a
  // recognised instruction at all (ADR-0005).
  logic [31:0] csr_rdata = 32'b0;
  logic csr_implemented = 1'b0;
  logic [11:0] csr_addr;
  logic csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata;

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
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .pc(pc),
    .rs1(rs1),
    .rs2(rs2),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .instret(instret),
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
    // rtl/fetcher.v drives this to exactly !reset (CLAUDE.md invariant 1),
    // so the real decoder never sees a non-reset cycle with it low.
    in.valid = 1'b1;
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

    // ---- Zicsr (ADR-0005) ------------------------------------------------
    // The three immediate forms used to be folded straight into the register
    // forms, which lost the zimm-vs-rs1 distinction. These vectors are the
    // regression for that split; test/csr_tb.v covers the CSR file itself.
    csr_implemented = 1'b1;

    // csrrw a1, mscratch, a0  ->  0x340515f3
    in.instr = 32'h340515f3;
    reg_rs1 = 32'hdeadbeef;
    csr_rdata = 32'h0000cafe;
    #1;
    check_hex("csrrw csr_addr", {20'b0, dut.csr_addr}, 32'h340);
    check_bit("csrrw is not an immediate form", dut.is_csr_imm, 1'b0);
    check_hex("csrrw operand is rs1", dut.csr_arg, 32'hdeadbeef);
    check_hex("csrrw wdata is the operand", dut.csr_wdata, 32'hdeadbeef);
    check_bit("csrrw writes", dut.csr_write_op, 1'b1);
    check_bit("csrrw with rd != x0 reads", dut.csr_read_op, 1'b1);
    check_bit("csrrw uses rs1", dut.uses_rs1, 1'b1);
    check_bit("an implemented CSR is a valid instruction", dut.instr_valid, 1'b1);

    // CLAUDE.md invariant 5 / ADR-0026: it does NOT issue yet. The previous
    // vector is still sitting in decoder_out, so the pipe is not drained and
    // this bubbles instead -- which is bubble-shaped, not hold-shaped: `out`
    // goes to zero rather than staying put.
    check_bit("a CSR instruction serializes while the pipe is busy",
              dut.csr_serialize, 1'b1);
    check_bit("...and that is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_bit("...and no CSR write commits", dut.csr_wen, 1'b0);
    @(posedge clk);
    #1;
    check_bit("the serializing cycle bubbles decoder_out", out.valid, 1'b0);
    check_bit("...which drains the pipe", dut.pipe_drained, 1'b1);
    check_bit("...so it issues now", dut.issuing, 1'b1);
    check_bit("...committing the write", dut.csr_wen, 1'b1);
    @(posedge clk);
    #1;
    check_hex("csr read rides the is_add pass-through", out.rs1, 32'h0000cafe);
    check_hex("...with rs2 zeroed", out.rs2, 32'b0);
    check_bit("...as an add", out.is_add, 1'b1);
    check_hex("...to the encoded rd", {27'b0, out.rd}, 32'd11);

    // csrrsi a0, mscratch, 0x1f  ->  0x340fe573. The zimm is not a register
    // index: no rs1 interlock, and the operand comes out of the encoding.
    in.instr = 32'h340fe573;
    executor_out.valid = 1'b1;
    executor_out.rd = 5'd31; // == the zimm bits, if they were read as rs1
    #1;
    check_bit("csrrsi is an immediate form", dut.is_csr_imm, 1'b1);
    check_hex("csrrsi operand is the zimm", dut.csr_arg, 32'h1f);
    check_hex("csrrsi wdata sets the zimm bits", dut.csr_wdata, 32'h0000caff);
    check_bit("csrrsi does not use rs1", dut.uses_rs1, 1'b0);
    check_bit("...so the zimm raises no interlock", dut.hazard_rs1, 1'b0);

    // csrrs a0, mscratch, x31 -- the register form of the same five bits
    // DOES interlock (0x340fa573).
    in.instr = 32'h340fa573;
    #1;
    check_bit("csrrs x31 uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and interlocks on it", dut.hazard_rs1, 1'b1);
    executor_out.valid = 1'b0;

    // csrr a0, misa  ==  csrrs a0, misa, x0 (0x30102573): rs1 == x0
    // suppresses the write, which is what makes it legal on a read-only CSR.
    in.instr = 32'h30102573;
    #1;
    check_bit("csrrs with rs1 == x0 suppresses the write", dut.csr_write_op, 1'b0);
    check_bit("...and still reads", dut.csr_read_op, 1'b1);

    // csrw mscratch, a0  ==  csrrw x0, mscratch, a0 (0x34051073): rd == x0
    // suppresses the read.
    in.instr = 32'h34051073;
    #1;
    check_bit("csrrw with rd == x0 suppresses the read", dut.csr_read_op, 1'b0);
    check_bit("...and still writes", dut.csr_write_op, 1'b1);

    // An unimplemented CSR stays an unrecognised instruction (ADR-0005): the
    // trap for it lands with trap entry, not here.
    csr_implemented = 1'b0;
    #1;
    check_bit("an unimplemented CSR is not a valid instruction", dut.instr_valid, 1'b0);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: decode vectors (xori immediate, ebreak/mret/wfi, Zicsr)");
      $finish;
    end
  end
endmodule
