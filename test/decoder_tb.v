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
  // ADR-0028's trap-entry pair. rtl/csrs.v is a sibling of the decoder, not
  // part of it, so these are stubbed the same way the read port above is:
  // mtvec/mepc are driven to recognisable constants and the trap-commit
  // outputs are observed.
  logic [31:0] mtvec = 32'h0000_0100;
  logic [31:0] mepc  = 32'h0000_0244;
  logic trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc;

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
    .mtvec(mtvec),
    .mepc(mepc),
    .pc(pc),
    .rs1(rs1),
    .rs2(rs2),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .instret(instret),
    .trap_entry(trap_entry),
    .trap_cause(trap_cause),
    .trap_epc(trap_epc),
    .mret_entry(mret_entry),
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

    // ebreak is funct12 == 1 EXACTLY; mret (0x302) and wfi (0x105) are
    // different SYSTEM instructions sharing funct3 == 0 with it, and folding
    // them into ebreak was the original defect here.
    //
    // Read off the decode flag rather than off `out.is_ebreak`, because since
    // trap entry landed `ebreak` traps (cause 3) and ADR-0028 suppresses every
    // execution flag on a trapping issue -- so the registered flag is now 0
    // for all three and the vector would pass vacuously.
    in.instr = 32'h00100073;
    #1;
    check_bit("ebreak sets instr_ebreak", dut.instr_ebreak, 1'b1);
    in.instr = 32'h30200073;
    #1;
    check_bit("mret does not set instr_ebreak", dut.instr_ebreak, 1'b0);
    check_bit("...it sets instr_mret", dut.instr_mret, 1'b1);
    in.instr = 32'h10500073;
    #1;
    check_bit("wfi does not set instr_ebreak", dut.instr_ebreak, 1'b0);
    check_bit("...it sets instr_wfi", dut.instr_wfi, 1'b1);
    @(posedge clk);
    #1;

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

    // An unimplemented CSR is not a recognised instruction, and that is now
    // how it becomes an illegal-instruction TRAP (ADR-0005) rather than a
    // silently suppressed no-op.
    csr_implemented = 1'b0;
    #1;
    check_bit("an unimplemented CSR is not a valid instruction", dut.instr_valid, 1'b0);
    check_bit("...so it is illegal", dut.instr_illegal, 1'b1);
    check_hex("...with cause 2", trap_cause, 32'd2);
    csr_implemented = 1'b1;

    // ---- M3 traps (CLAUDE.md invariant 2 / ADR-0005 / ADR-0030) ----------
    // Every cause, taken directly against the decoder. The `.S` suite takes
    // the same five through a real handler; this is where the CAUSE ENCODER
    // itself is pinned, including the cases the suite cannot reach because
    // they never co-occur with anything observable.

    // The four M3 additions to instr_valid. Until trap entry landed these were
    // merely unrecognised, which was harmless only while unrecognised meant
    // "no trap" -- a legal `fence` would fault the moment that changed
    // (ADR-0034 recorded this as the change that must fix it).
    in.instr = 32'h0ff0000f;   // fence iorw, iorw
    #1;
    check_bit("fence is a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h0000100f;   // fence.i
    #1;
    check_bit("fence.i is a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h10500073;   // wfi
    #1;
    check_bit("wfi is a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h30200073;   // mret
    #1;
    check_bit("mret is a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    check_bit("...and is not an ecall", dut.instr_ecall, 1'b0);

    // Cause 2, the canonical illegal instruction: the all-zero word.
    in.instr = 32'h00000000;
    #1;
    check_bit("the all-zero word is illegal", dut.instr_illegal, 1'b1);
    check_hex("...cause 2", trap_cause, 32'd2);
    check_bit("...and traps", dut.trap_pending, 1'b1);

    // Cause 3 and cause 11: ebreak and ecall.
    in.instr = 32'h00100073;
    #1;
    check_hex("ebreak is cause 3", trap_cause, 32'd3);
    in.instr = 32'h00000073;
    #1;
    check_hex("ecall is cause 11", trap_cause, 32'd11);

    // Cause 4: a misaligned load. `lw a1, 4(a0)` (0x00452583) with a0 holding
    // an odd address -- the effective address is what is tested, not the
    // register and not the immediate.
    in.instr = 32'h00452583;
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("misaligned lw is cause 4", trap_cause, 32'd4);
    check_bit("...and traps", dut.trap_pending, 1'b1);
    reg_rs1 = 32'h0001_0000;
    #1;
    check_bit("an aligned lw does not trap", dut.trap_pending, 1'b0);
    // 2-byte alignment is not enough for a word access.
    reg_rs1 = 32'h0001_0002;
    #1;
    check_bit("a 2-aligned lw still traps", dut.trap_pending, 1'b1);

    // Cause 6: a misaligned store. `sh a1, 0(a0)` (0x00b51023) needs only
    // 2-byte alignment, so an odd address traps and an even one does not.
    in.instr = 32'h00b51023;
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("misaligned sh is cause 6", trap_cause, 32'd6);
    reg_rs1 = 32'h0001_0002;
    #1;
    check_bit("a 2-aligned sh does not trap", dut.trap_pending, 1'b0);

    // Byte accesses are aligned by construction and can never raise either
    // cause -- which is what made the riscv-formal attribution checkable:
    // insn_lb/insn_lbu/insn_sb passed while the nine word/halfword checks
    // failed. `sb a1, 0(a0)` (0x00b50023) at the most awkward address there is.
    in.instr = 32'h00b50023;
    reg_rs1 = 32'h0001_0003;
    #1;
    check_bit("a byte store never traps", dut.trap_pending, 1'b0);
    // `lb a1, 0(a0)` (0x00050583)
    in.instr = 32'h00050583;
    #1;
    check_bit("a byte load never traps", dut.trap_pending, 1'b0);
    reg_rs1 = 32'b0;

    // Cause 2 again, the other illegal-CSR rule: a WRITE to a CSR that is
    // read-only by address. mvendorid is 0xF11, so addr[11:10] == 2'b11.
    // `csrw mvendorid, a0` == csrrw x0, mvendorid, a0 (0xf1151073).
    in.instr = 32'hf1151073;
    #1;
    check_bit("an implemented read-only CSR is still a valid encoding",
              dut.instr_valid, 1'b1);
    check_bit("...but writing it is illegal", dut.csr_readonly_write, 1'b1);
    check_hex("...cause 2", trap_cause, 32'd2);
    // ...while READING the same CSR is legal, because CSRRS with rs1 == x0
    // suppresses the write. `csrr a0, mvendorid` (0xf1102573).
    in.instr = 32'hf1102573;
    #1;
    check_bit("reading a read-only CSR is not a write", dut.csr_readonly_write, 1'b0);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);

    // ADR-0027: a trapping issue commits nothing outside the pipeline
    // registers -- no minstret increment and no CSR access. Checked on the
    // illegal CSR write above, which is both a trap AND a CSR instruction, so
    // a gate that only looked at `instr_valid` (the pre-M3 spelling) would
    // still let its write through.
    //
    // A CSR instruction serializes whether or not it is legal, so this needs a
    // drained pipe to issue at all -- and the checks below would otherwise be
    // satisfied by the stall rather than by the trap, which is exactly the
    // vacuous pass worth avoiding here.
    in.instr = 32'hf1151073;
    @(posedge clk);
    #1;
    check_bit("...it issues once the pipe drains", dut.issuing, 1'b1);
    check_bit("a trapping issue does not count in minstret", instret, 1'b0);
    check_bit("...and commits no CSR write", csr_wen, 1'b0);
    check_bit("...and no CSR read", csr_ren, 1'b0);
    check_bit("...but it does commit a trap", trap_entry, 1'b1);

    // ADR-0028: the trapping instruction retires having architecturally done
    // nothing -- pc goes to mtvec, rd is x0, and no memory flag survives, so
    // rtl/accessor.v issues no bus request and rvfi_mem_* come out zero.
    // Checked after the edge, on the published decoder_output.
    in.instr = 32'h00452583;   // the misaligned lw again
    reg_rs1 = 32'h0001_0001;
    in.pc = 32'h0000_0080;
    #1;
    check_hex("trap_epc is the FAULTING pc, not the next one", trap_epc, 32'h0000_0080);
    @(posedge clk);
    #1;
    check_hex("a trap redirects pc to mtvec", pc, 32'h0000_0100);
    check_bit("...the trapping instruction still retires", out.valid, 1'b1);
    check_hex("...writing no register", {27'b0, out.rd}, 32'b0);
    check_bit("...and issuing no load", out.is_lw, 1'b0);
    reg_rs1 = 32'b0;

    // mret is a branch to mepc, taken by the same mechanism. It serializes
    // (CLAUDE.md invariant 5), so it needs a drained pipe to issue: the
    // previous vector is still in decoder_out, which bubbles this cycle and
    // drains it.
    in.instr = 32'h30200073;
    #1;
    check_bit("mret serializes while the pipe is busy", dut.csr_serialize, 1'b1);
    check_bit("...so it does not commit yet", mret_entry, 1'b0);
    @(posedge clk);
    #1;
    check_bit("...the serializing cycle bubbles decoder_out", out.valid, 1'b0);
    check_bit("...which drains the pipe", dut.pipe_drained, 1'b1);
    check_bit("...so it commits now", mret_entry, 1'b1);
    check_bit("...and is not a trap", trap_entry, 1'b0);
    @(posedge clk);
    #1;
    check_hex("mret redirects pc to mepc", pc, 32'h0000_0244);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: decode vectors (xori immediate, ebreak/mret/wfi, Zicsr, M3 traps)");
      $finish;
    end
  end
endmodule
