`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// Decode vectors, driven straight at rtl/decoder.v with no pipeline around it.
//
// The one timing rule: a combinational decode flag is readable the instant
// `in.instr` settles, but anything about issuing, publishing or committing has
// to be checked after the operand-fetch cycle is spent.
module decoder_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  fetcher_output in;
  logic [31:0] reg_rs1, reg_rs2;
  logic [31:0] pc;
  logic [31:0] next_pc;
  logic [4:0] rs1, rs2;
  decoder_output out;
  // Driven high by the vectors that need something in flight to serialize
  // against; the hazard scoreboard is test/regfile_tb.v's and hazard.S's.
  executor_output executor_out = '0;
  logic divider_stall = 1'b0;
  logic accessor_stall = 1'b0;
  logic fetch_stall = 1'b0;
  logic accessor_pending_valid = 1'b0;
  logic [4:0] accessor_pending_rd = 5'b0;
  logic accessor_out_valid = 1'b0;
  // rtl/csrs.v is a sibling of the decoder, not part of it, so it is stubbed.
  logic [31:0] csr_rdata = 32'b0;
  logic csr_implemented = 1'b0;
  logic [11:0] csr_addr;
  logic csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata;
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
    .fetch_stall(fetch_stall),
    .accessor_pending_valid(accessor_pending_valid),
    .accessor_pending_rd(accessor_pending_rd),
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(mtvec),
    .mepc(mepc),
    .pc(pc),
    .next_pc(next_pc),
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

  // pc must have exactly one driver, and it must be next_pc. The memory latches
  // its address off next_pc one cycle before the fetch that reads it. Give pc a
  // second driver and the memory runs a cycle out of step with decode, so the
  // core executes whatever is at the wrong addresses. Nothing on the
  // riscv-formal ladder reads that port.
  //
  // Checked on every edge rather than in one vector, because a change like that
  // shows up on some instructions and not on others.
  logic [31:0] prev_next_pc;
  logic        prev_next_pc_valid = 1'b0;
  always @(posedge clk) begin
    if (prev_next_pc_valid && pc !== prev_next_pc) begin
      $display("MISMATCH next_pc predicted %08x but pc became %08x", prev_next_pc, pc);
      errors++;
    end
    prev_next_pc <= next_pc;
    prev_next_pc_valid <= 1'b1;
  end

  task automatic operand_fetch_cycle();
    begin
      @(posedge clk);
      #1;
    end
  endtask

  // The addi x0, x0, 0 matters. operand_stall compares rs1 and rs2 against
  // whatever they were at the last clock edge, and most vectors here take no
  // edge at all. Without parking on x0 first, an earlier vector can leave the
  // same pair behind, no fetch cycle happens, and every check below lands a
  // cycle early.
  task automatic present_and_fetch(input logic [31:0] instr);
    begin
      in.instr = 32'h00000013;
      @(posedge clk);
      #1;
      in.instr = instr;
      #1;
      operand_fetch_cycle();
    end
  endtask

  initial begin
    reset = 1;
    in = '0;
    // rtl/fetcher.v drives this to exactly !reset, so the real decoder never
    // sees a non-reset cycle with it low.
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
    #1;
    check_hex("xori math_arg", dut.math_arg, 32'hffffffff);
    check_bit("a newly presented instruction stalls for its operands",
              dut.operand_stall, 1'b1);
    // Two separate things to get wrong, both silent. Drop the stall and the
    // instruction issues with a register the file has not read yet. Drop the
    // bubble and it publishes during its own fetch cycle. Either way every
    // other vector here still passes.
    check_bit("...so it does not issue in that cycle", dut.issuing, 1'b0);
    operand_fetch_cycle();
    check_bit("...and the fetch cycle bubbled decoder_out", out.valid, 1'b0);
    check_bit("...and is eligible to issue on the very next cycle",
              dut.operand_stall, 1'b0);
    check_bit("...which it does", dut.issuing, 1'b1);
    @(posedge clk);
    #1;
    check_hex("xori out.rs2 (registered math_arg)", out.rs2, 32'hffffffff);

    // Read off the decode flag, not `out.is_ebreak`: a trapping issue
    // suppresses every execution flag, so the registered flag is 0 for all
    // three of these and the vector would pass vacuously.
    in.instr = 32'h00100073;   // ebreak
    #1;
    check_bit("ebreak sets instr_ebreak", dut.instr_ebreak, 1'b1);
    in.instr = 32'h30200073;   // mret
    #1;
    check_bit("mret does not set instr_ebreak", dut.instr_ebreak, 1'b0);
    check_bit("...it sets instr_mret", dut.instr_mret, 1'b1);
    in.instr = 32'h10500073;   // wfi
    #1;
    check_bit("wfi does not set instr_ebreak", dut.instr_ebreak, 1'b0);
    check_bit("...it sets instr_wfi", dut.instr_wfi, 1'b1);
    @(posedge clk);
    #1;

    csr_implemented = 1'b1;

    // csrrw a1, mscratch, a0  ->  0x340515f3
    in.instr = 32'h340515f3;
    reg_rs1 = 32'hdeadbeef;   // re-presented below through present_and_fetch
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

    executor_out.valid = 1'b1;
    executor_out.rd = 5'd0;   // x0 raises no RAW hazard of its own
    present_and_fetch(32'h340515f3);
    check_bit("a CSR instruction serializes while the pipe is busy",
              dut.serialize, 1'b1);
    check_bit("...and that is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_bit("...and no CSR write commits", dut.csr_wen, 1'b0);
    check_bit("...and it bubbles decoder_out rather than holding it",
              out.valid, 1'b0);
    executor_out.valid = 1'b0;
    #1;
    check_bit("the drained pipe releases it", dut.pipe_drained, 1'b1);
    check_bit("...so it issues now", dut.issuing, 1'b1);
    check_bit("...committing the write", dut.csr_wen, 1'b1);
    @(posedge clk);
    #1;
    check_hex("csr read rides the is_add pass-through", out.rs1, 32'h0000cafe);
    check_hex("...with rs2 zeroed", out.rs2, 32'b0);
    check_bit("...as an add", out.is_add, 1'b1);
    check_hex("...to the encoded rd", {27'b0, out.rd}, 32'd11);

    in.instr = 32'h340fe573;   // csrrsi a0, mscratch, 0x1f
    executor_out.valid = 1'b1;
    executor_out.rd = 5'd31; // == the zimm bits, if they were read as rs1
    #1;
    check_bit("csrrsi is an immediate form", dut.is_csr_imm, 1'b1);
    check_hex("csrrsi operand is the zimm", dut.csr_arg, 32'h1f);
    check_hex("csrrsi wdata sets the zimm bits", dut.csr_wdata, 32'h0000caff);
    check_bit("csrrsi does not use rs1", dut.uses_rs1, 1'b0);
    check_bit("...so the zimm raises no interlock", dut.hazard_rs1, 1'b0);

    in.instr = 32'h340fa573;   // csrrs a0, mscratch, x31: the same five bits
    #1;
    check_bit("csrrs x31 uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and interlocks on it", dut.hazard_rs1, 1'b1);
    executor_out.valid = 1'b0;

    in.instr = 32'h30102573;   // csrr a0, misa == csrrs a0, misa, x0
    #1;
    check_bit("csrrs with rs1 == x0 suppresses the write", dut.csr_write_op, 1'b0);
    check_bit("...and still reads", dut.csr_read_op, 1'b1);

    in.instr = 32'h34051073;   // csrw mscratch, a0 == csrrw x0, mscratch, a0
    #1;
    check_bit("csrrw with rd == x0 suppresses the read", dut.csr_read_op, 1'b0);
    check_bit("...and still writes", dut.csr_write_op, 1'b1);

    csr_implemented = 1'b0;
    #1;
    check_bit("an unimplemented CSR is not a valid instruction", dut.instr_valid, 1'b0);
    check_bit("...so it is illegal", dut.instr_illegal, 1'b1);
    check_hex("...with cause 2", trap_cause, 32'd2);
    csr_implemented = 1'b1;

    // Anything the decoder does not recognise traps, so a legal encoding left
    // out of instr_valid faults.
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

    in.instr = 32'h00000000;
    #1;
    check_bit("the all-zero word is illegal", dut.instr_illegal, 1'b1);
    check_hex("...cause 2", trap_cause, 32'd2);
    check_bit("...and traps", dut.trap_pending, 1'b1);

    in.instr = 32'h00100073;
    #1;
    check_hex("ebreak is cause 3", trap_cause, 32'd3);
    in.instr = 32'h00000073;
    #1;
    check_hex("ecall is cause 11", trap_cause, 32'd11);

    // The EFFECTIVE address decides, not rs1 and not the immediate.
    in.instr = 32'h00452583;   // lw a1, 4(a0)
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("misaligned lw is cause 4", trap_cause, 32'd4);
    check_bit("...and traps", dut.trap_pending, 1'b1);
    reg_rs1 = 32'h0001_0000;
    #1;
    check_bit("an aligned lw does not trap", dut.trap_pending, 1'b0);
    reg_rs1 = 32'h0001_0002;
    #1;
    check_bit("a 2-aligned lw still traps", dut.trap_pending, 1'b1);

    in.instr = 32'h00b51023;   // sh a1, 0(a0): only 2-byte alignment needed
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("misaligned sh is cause 6", trap_cause, 32'd6);
    reg_rs1 = 32'h0001_0002;
    #1;
    check_bit("a 2-aligned sh does not trap", dut.trap_pending, 1'b0);

    in.instr = 32'h00b50023;   // sb a1, 0(a0), at the most awkward address
    reg_rs1 = 32'h0001_0003;
    #1;
    check_bit("a byte store never traps", dut.trap_pending, 1'b0);
    in.instr = 32'h00050583;   // lb a1, 0(a0)
    #1;
    check_bit("a byte load never traps", dut.trap_pending, 1'b0);
    reg_rs1 = 32'b0;

    // The other illegal-CSR rule: read-only by address, addr[11:10] == 2'b11.
    in.instr = 32'hf1151073;   // csrw mvendorid, a0
    #1;
    check_bit("an implemented read-only CSR is still a valid encoding",
              dut.instr_valid, 1'b1);
    check_bit("...but writing it is illegal", dut.csr_readonly_write, 1'b1);
    check_hex("...cause 2", trap_cause, 32'd2);
    in.instr = 32'hf1102573;   // csrr a0, mvendorid
    #1;
    check_bit("reading a read-only CSR is not a write", dut.csr_readonly_write, 1'b0);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);

    // Run again with nothing in the pipeline. A CSR instruction waits either
    // way, legal or not, so otherwise the checks below pass because it stalled
    // rather than because it trapped.
    present_and_fetch(32'hf1151073);
    check_bit("...it issues once the pipe drains", dut.issuing, 1'b1);
    check_bit("a trapping issue does not count in minstret", instret, 1'b0);
    check_bit("...and commits no CSR write", csr_wen, 1'b0);
    check_bit("...and no CSR read", csr_ren, 1'b0);
    check_bit("...but it does commit a trap", trap_entry, 1'b1);

    reg_rs1 = 32'h0001_0001;
    in.pc = 32'h0000_0080;
    present_and_fetch(32'h00452583);   // the misaligned lw again
    check_hex("trap_epc is the FAULTING pc, not the next one", trap_epc, 32'h0000_0080);
    @(posedge clk);
    #1;
    check_hex("a trap redirects pc to mtvec", pc, 32'h0000_0100);
    check_bit("...the trapping instruction still retires", out.valid, 1'b1);
    check_hex("...writing no register", {27'b0, out.rd}, 32'b0);
    check_bit("...and issuing no load", out.is_lw, 1'b0);
    reg_rs1 = 32'b0;

    executor_out.valid = 1'b1;
    executor_out.rd = 5'd0;
    present_and_fetch(32'h30200073);
    check_bit("mret serializes while the pipe is busy", dut.serialize, 1'b1);
    check_bit("...so it does not commit yet", mret_entry, 1'b0);
    check_bit("...and it bubbles decoder_out rather than holding it",
              out.valid, 1'b0);
    executor_out.valid = 1'b0;
    #1;
    check_bit("the drained pipe releases it", dut.pipe_drained, 1'b1);
    check_bit("...so it commits now", mret_entry, 1'b1);
    check_bit("...and is not a trap", trap_entry, 1'b0);
    @(posedge clk);
    #1;
    check_hex("mret redirects pc to mepc", pc, 32'h0000_0244);

    // Issued once so decoder_out holds something a bubble would visibly destroy.
    present_and_fetch(32'h00100093);   // addi x1, x0, 1
    @(posedge clk);
    #1;
    check_bit("the instruction issued", out.valid, 1'b1);
    check_hex("...into decoder_out", {27'b0, out.rd}, 32'd1);

    fetch_stall = 1'b1;
    #1;
    check_bit("a stolen fetch window is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_hex("...and the pc holds, so the instruction is presented again",
              next_pc, pc);
    check_bit("...and no trap is committed out of the stolen window", trap_entry, 1'b0);

    // A freeze holds decoder_out; a steal clears it. On a cycle with both,
    // holding has to win or the held instruction is lost. Only the order of the
    // arms in the publish block decides that, and swapping them is silent.
    accessor_stall = 1'b1;
    #1;
    @(posedge clk);
    #1;
    check_bit("a steal coinciding with an accessor freeze holds decoder_out",
              out.valid, 1'b1);
    check_hex("...unchanged", {27'b0, out.rd}, 32'd1);
    accessor_stall = 1'b0;
    divider_stall = 1'b1;
    #1;
    @(posedge clk);
    #1;
    check_bit("...and so does one coinciding with a divide", out.valid, 1'b1);
    divider_stall = 1'b0;
    #1;
    @(posedge clk);
    #1;
    check_bit("a steal on its own bubbles decoder_out instead", out.valid, 1'b0);
    fetch_stall = 1'b0;
    #1;
    @(posedge clk);
    #1;
    check_bit("...and the same instruction issues once the window is back",
              out.valid, 1'b1);
    check_hex("...as itself", {27'b0, out.rd}, 32'd1);

    // Text is writable, so a store just before a fence.i can change the words
    // being fetched right behind it. Waiting is what puts that store's write
    // ahead of the next fetch address.
    executor_out.valid = 1'b1;
    executor_out.rd = 5'd0;
    present_and_fetch(32'h0000100f);
    check_bit("fence.i serializes while the pipe is busy", dut.serialize, 1'b1);
    check_bit("...and that is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_bit("...and it bubbles decoder_out rather than holding it",
              out.valid, 1'b0);
    executor_out.valid = 1'b0;
    #1;
    check_bit("the drained pipe releases it", dut.pipe_drained, 1'b1);
    check_bit("...so it issues now", dut.issuing, 1'b1);

    // A plain `fence` has nothing to wait for: one bus, one access in flight.
    executor_out.valid = 1'b1;
    in.instr = 32'h0ff0000f;
    #1;
    check_bit("a plain fence does not serialize", dut.serialize, 1'b0);
    executor_out.valid = 1'b0;

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: decode vectors (xori immediate, ebreak/mret/wfi, Zicsr, M3 traps)");
      $finish;
    end
  end
endmodule
