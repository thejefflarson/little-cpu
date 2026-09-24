`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// X vectors, driven straight at rtl/executor.v with no decoder or pipeline around it,
// the same way test/exec_tb.v drives the arithmetic. This bench is the arithmetic
// bench's counterpart for everything else the D/X split moved into X: branch and jump
// resolution, the effective address and its region test, the trap-cause priority chain,
// CSR access, atomic address and fault, and the one-cycle interrupt bubble's commit.
// `in` is built by hand, the way exec_tb.v builds it, rather than decoded from a raw
// instruction word -- X never sees one; it consumes the flags D already decided.
module executor_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  dx_output in;
  logic [31:0] reg_rs1, reg_rs2;
  logic x_busy;
  logic [31:0] atomic_addr;
  logic atomic_supported;
  logic [11:0] csr_addr;
  logic csr_ren, csr_wen;
  logic [31:0] csr_wdata;
  logic [31:0] csr_rdata;
  logic csr_implemented;
  logic instret;
  logic trap_entry;
  logic [31:0] trap_cause, trap_epc, trap_tval;
  logic mret_entry;
  logic [31:0] mtvec = 32'h0000_0100;
  logic [31:0] mepc  = 32'h0000_0244;
  logic redirect;
  logic [31:0] redirect_target;
  decoder_output launch;
  executor_output out;

  executor dut (
    .clk(clk),
    .reset(reset),
    .in(in),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .x_busy(x_busy),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .instret(instret),
    .trap_entry(trap_entry),
    .trap_cause(trap_cause),
    .trap_epc(trap_epc),
    .trap_tval(trap_tval),
    .mret_entry(mret_entry),
    .mtvec(mtvec),
    .mepc(mepc),
    .redirect(redirect),
    .redirect_target(redirect_target),
    .launch(launch),
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

  // `x_busy` is exactly the OR of the two reasons the divider and the region wait, the
  // counterpart of test/decoder_tb.v's own OR-identity check on D's side of the split.
  always @(clk) begin
    if (x_busy !== (dut.divider_busy || dut.region_stall)) begin
      $display("MISMATCH x_busy is not the OR of divider_busy and region_stall: x_busy=%b divider_busy=%b region_stall=%b",
               x_busy, dut.divider_busy, dut.region_stall);
      errors++;
    end
  end

  task automatic clear_in;
    begin
      in = '0;
      in.valid = 1'b1;
      in.pc = 32'h0000_0000;
      // Low bits '11': an uncompressed word, so pc_inc defaults to 4 unless a vector
      // deliberately drives a compressed one to check the +2 arm instead.
      in.instr = 32'h0000_0003;
      in.rd = 5'd1;
      in.rs1 = 5'd2;
      in.rs2 = 5'd3;
      reg_rs1 = 32'b0;
      reg_rs2 = 32'b0;
      csr_rdata = 32'b0;
      csr_implemented = 1'b1;
      atomic_supported = 1'b1;
    end
  endtask

  initial begin
    reset = 1;
    clear_in();
    repeat (2) @(posedge clk);
    #1;
    reset = 0;

    // --- Branch and jump resolution: both arms of the chain that add to the fetched pc,
    // and the one that adds neither. ---
    clear_in();
    in.is_add = 1'b1;
    in.pc = 32'h0000_00a0;
    in.instr = 32'h0000_0013;   // an uncompressed word: pc_inc must read 4
    #1;
    check_hex("a non-branching uncompressed instruction resolves to pc+4",
              redirect_target, 32'h0000_00a4);
    check_bit("...which is not a redirect (D's own guess already matches)",
              redirect, 1'b0);
    in.instr = 32'h0000_0001;   // a compressed word: pc_inc must read 2
    #1;
    check_hex("a compressed instruction resolves to pc+2", redirect_target, 32'h0000_00a2);

    clear_in();
    in.pc = 32'h0000_00a0;
    in.is_jal = 1'b1;
    in.immediate = 32'd8;
    #1;
    check_hex("jal adds its immediate to the fetched pc", redirect_target, 32'h0000_00a8);
    check_bit("...which IS a redirect", redirect, 1'b1);

    clear_in();
    in.pc = 32'h0000_00a0;
    in.is_jalr = 1'b1;
    in.immediate = 32'd5;       // odd, to prove the low bit is masked
    reg_rs1 = 32'h0000_1000;
    #1;
    check_hex("jalr adds its immediate to rs1 and clears bit 0",
              redirect_target, 32'h0000_1004);

    clear_in();
    in.pc = 32'h0000_00a0;
    in.is_beq = 1'b1;
    in.immediate = 32'd8;
    reg_rs1 = 32'd5;
    reg_rs2 = 32'd5;
    #1;
    check_hex("a taken branch adds its own immediate", redirect_target, 32'h0000_00a8);
    reg_rs2 = 32'd6;
    #1;
    check_hex("an untaken branch steps sequentially instead", redirect_target, 32'h0000_00a4);
    in.is_bne = 1'b1; in.is_beq = 1'b0;
    #1;
    check_hex("bne is the complement of beq", redirect_target, 32'h0000_00a8);
    in.is_bne = 1'b0; in.is_blt = 1'b1;
    reg_rs1 = -32'd1; reg_rs2 = 32'd1;   // -1 < 1 signed, but -1 > 1 unsigned
    #1;
    check_hex("blt compares signed", redirect_target, 32'h0000_00a8);
    in.is_blt = 1'b0; in.is_bltu = 1'b1;
    #1;
    check_hex("bltu compares unsigned, so the same operands do not take it",
              redirect_target, 32'h0000_00a4);

    // --- mret and a trap both redirect off the resolved-target chain, mret to mepc and a
    // trap to mtvec, both same-cycle claims (X owns no registered pc of its own). ---
    clear_in();
    in.is_mret = 1'b1;
    #1;
    check_hex("mret resolves to mepc", redirect_target, mepc);
    check_bit("...and commits as mret", mret_entry, 1'b1);
    check_bit("...not a trap", trap_entry, 1'b0);

    clear_in();
    in.pc = 32'h0000_0080;
    in.instr = 32'h0000_0000;   // the all-zero word: illegal
    #1;
    check_bit("the all-zero word is illegal", instret, 1'b0);
    check_hex("...cause 2", trap_cause, 32'd2);
    check_hex("...and it redirects to mtvec", redirect_target, mtvec);
    check_bit("...as a redirect", redirect, 1'b1);
    check_bit("...and a trap entry", trap_entry, 1'b1);

    // --- The trap-cause priority chain, one term isolated at a time. ---
    clear_in();
    in.imem_fault = 1'b1;
    in.instr = 32'h0000_0073;   // looks like ecall, but the fetch itself was refused
    #1;
    check_hex("a fetch fault outranks anything the unfetched word decodes to",
              trap_cause, 32'd1);
    check_hex("...reporting the faulting pc as tval", trap_tval, in.pc);

    clear_in();
    in.is_csr_access = 1'b1;
    in.is_csrrw = 1'b1;
    in.instr = 32'h34051073;   // csrw mscratch, a0 -- a CSR the platform does not implement
    csr_implemented = 1'b0;
    #1;
    check_hex("an unimplemented CSR is illegal", trap_cause, 32'd2);
    check_hex("...reporting the instruction word", trap_tval, in.instr);

    clear_in();
    in.is_csr_access = 1'b1;
    in.is_csrrw = 1'b1;
    in.instr = 32'hf1151073;   // csrw mvendorid, a0 -- addr 0xf11 is read-only (top 2 bits 11)
    #1;
    check_bit("writing a read-only CSR is illegal", instret, 1'b0);
    check_hex("...cause 2", trap_cause, 32'd2);

    clear_in();
    in.is_ebreak = 1'b1;
    #1;
    check_hex("ebreak is cause 3", trap_cause, 32'd3);
    check_hex("...and reports nothing: mepc already has the address", trap_tval, 32'h0);

    clear_in();
    in.is_ecall = 1'b1;
    #1;
    check_hex("ecall is cause 11", trap_cause, 32'd11);
    check_hex("...and reports nothing either", trap_tval, 32'h0);

    // Addresses in this block are all deep inside a RAM block (region_stall settles
    // combinationally), so `trap_entry` reads its real committed value with no wait.
    clear_in();
    in.is_lw = 1'b1;
    in.immediate = 32'd4;
    reg_rs1 = 32'h0001_2001;
    #1;
    check_hex("a misaligned lw is cause 4", trap_cause, 32'd4);
    check_hex("...reporting the address it computed", trap_tval, 32'h0001_2005);
    reg_rs1 = 32'h0001_2000;
    #1;
    check_bit("an aligned lw does not trap", trap_entry, 1'b0);
    reg_rs1 = 32'h0001_2002;
    #1;
    check_bit("a 2-aligned lw still traps", trap_entry, 1'b1);

    clear_in();
    in.is_sh = 1'b1;
    in.immediate = 32'd0;
    reg_rs1 = 32'h0001_2001;
    #1;
    check_hex("a misaligned sh is cause 6", trap_cause, 32'd6);
    reg_rs1 = 32'h0001_2002;
    #1;
    check_bit("a 2-aligned sh does not trap", trap_entry, 1'b0);

    clear_in();
    in.is_sb = 1'b1;
    reg_rs1 = 32'h0001_2003;
    #1;
    check_bit("a byte store never traps on alignment", trap_entry, 1'b0);
    clear_in();
    in.is_lb = 1'b1;
    reg_rs1 = 32'h0001_2003;
    #1;
    check_bit("a byte load never traps on alignment", trap_entry, 1'b0);

    // --- fence, fence.i and wfi are valid and never trap on their own. ---
    clear_in();
    in.is_fence = 1'b1;
    #1;
    check_bit("fence does not trap", trap_entry, 1'b0);
    clear_in();
    in.is_fencei = 1'b1;
    #1;
    check_bit("fence.i does not trap", trap_entry, 1'b0);
    clear_in();
    in.is_wfi = 1'b1;
    #1;
    check_bit("wfi does not trap", trap_entry, 1'b0);

    // --- CSR read/write suppression: Zicsr's own rules make csrr(ci) legal on a
    // read-only register by skipping the write, and csrw legal with no destination by
    // skipping the read. ---
    clear_in();
    in.is_csr_access = 1'b1;
    in.is_csrrs = 1'b1;
    in.instr = 32'h30102573;   // csrrs a0, misa, x0 == csrr a0, misa
    csr_rdata = 32'hdead_beef;
    #1;
    check_bit("csrrs with rs1 == x0 suppresses the write", csr_wen, 1'b0);
    check_bit("...and still reads", csr_ren, 1'b1);
    check_hex("...passing the read value through as an add", launch.rs1, 32'hdead_beef);

    clear_in();
    in.is_csr_access = 1'b1;
    in.is_csrrw = 1'b1;
    in.rd = 5'd0;
    in.instr = 32'h34051073;   // csrw mscratch, a0 == csrrw x0, mscratch, a0
    reg_rs1 = 32'hcafe_babe;
    #1;
    check_bit("csrrw with rd == x0 suppresses the read", csr_ren, 1'b0);
    check_bit("...and still writes", csr_wen, 1'b1);
    check_hex("...the operand value", csr_wdata, 32'hcafe_babe);

    clear_in();
    in.is_csr_access = 1'b1;
    in.is_csrrw = 1'b1;
    in.is_csr_imm = 1'b1;
    in.instr = 32'h340fd573;   // csrrwi a0, mscratch, 0x1f -- zimm in the rs1 field
    #1;
    check_hex("an immediate CSR form reads its operand off the instruction word",
              csr_wdata, 32'h1f);

    clear_in();
    in.is_csr_access = 1'b1;
    in.is_csrrs = 1'b1;
    in.instr = 32'h340fa573;   // csrrs a0, mscratch, x31
    csr_rdata = 32'h0000cafe;
    reg_rs1 = 32'h0000001f;
    #1;
    check_hex("a register-form CSR sets bits with the read value ORed against rs1",
              csr_wdata, 32'h0000caff);

    // --- Atomics: the effective address is rs1 verbatim, no adder, and the platform's
    // refusal is a same-cycle fault since it arrives with the address. ---
    clear_in();
    in.is_amoadd = 1'b1;
    reg_rs1 = 32'h0001_0000;
    #1;
    check_hex("an atomic's effective address is rs1 alone", atomic_addr, 32'h0001_0000);
    check_hex("...matching what launch hands the accessor", launch.mem_addr, 32'h0001_0000);

    clear_in();
    atomic_supported = 1'b0;
    in.is_lr = 1'b1;
    reg_rs1 = 32'h0004_0000;
    #1;
    check_bit("an lr.w the platform does not answer traps", trap_entry, 1'b1);
    check_hex("...as a LOAD access fault", trap_cause, 32'd5);
    clear_in();
    atomic_supported = 1'b0;
    in.is_amoadd = 1'b1;
    reg_rs1 = 32'h0004_0000;
    #1;
    check_hex("an AMO there is a STORE/AMO access fault", trap_cause, 32'd7);
    clear_in();
    atomic_supported = 1'b0;
    in.is_sc = 1'b1;
    reg_rs1 = 32'h0004_0000;
    #1;
    check_hex("...and so is sc.w", trap_cause, 32'd7);

    clear_in();
    in.is_amoadd = 1'b1;
    reg_rs1 = 32'h0001_0002;
    #1;
    check_hex("a misaligned AMO is a store misalignment", trap_cause, 32'd6);
    clear_in();
    in.is_sc = 1'b1;
    reg_rs1 = 32'h0001_0002;
    #1;
    check_hex("...and so is a misaligned sc.w", trap_cause, 32'd6);
    clear_in();
    in.is_lr = 1'b1;
    reg_rs1 = 32'h0001_0002;
    #1;
    check_hex("a misaligned lr.w is a LOAD misalignment instead", trap_cause, 32'd4);

    clear_in();
    in.is_amoadd = 1'b1;
    reg_rs1 = 32'h0001_0000;
    #1;
    check_bit("an aligned, answered AMO does not trap", trap_entry, 1'b0);

    // A refused AMO retires with no rd, no operation flag, and no bus transaction: the
    // whole point of gating launch.valid on the trap.
    clear_in();
    in.is_amoadd = 1'b1;
    reg_rs1 = 32'h0001_0002;   // misaligned -> a trap
    #1;
    check_bit("the trapping AMO still retires (launch.valid)", launch.valid, 1'b1);
    check_hex("...writing no register", {27'b0, launch.rd}, 32'd0);
    check_bit("...raising no AMO or atomic flag", launch.is_amo || launch.is_lr ||
              launch.is_sc, 1'b0);

    // --- Region test: the fast arm answers same-cycle deep inside text or RAM; a block
    // within 2 KB of an edge waits a cycle for the deferred answer instead. ---
    clear_in();
    in.is_lw = 1'b1;
    reg_rs1 = 32'h0001_1000;   // deep inside the 64 KB RAM
    #1;
    check_bit("a load deep inside RAM waits for nothing", dut.region_stall, 1'b0);
    check_bit("...and does not fault", trap_entry, 1'b0);

    clear_in();
    in.is_lw = 1'b1;
    reg_rs1 = 32'h0000_0800;   // deep inside the 8 KB text window
    #1;
    check_bit("a load deep inside text waits for nothing either", dut.region_stall, 1'b0);
    check_bit("...and does not fault", trap_entry, 1'b0);

    clear_in();
    in.is_lw = 1'b1;
    reg_rs1 = 32'h0001_0400;   // the RAM's first 2 KB block
    #1;
    check_bit("the RAM's first block does not reach the fast path", dut.region_stall, 1'b1);
    reg_rs1 = 32'h0001_0800;
    #1;
    check_bit("...and the block above it does", dut.region_stall, 1'b0);

    // The deferred-answer protocol: `in` held steady (D's job under x_busy is tested in
    // test/decoder_tb.v), the answer registers one cycle after the capture and is read
    // the cycle after that.
    clear_in();
    in.is_lw = 1'b1;
    reg_rs1 = 32'h0004_0000;   // claimed by nothing
    #1;
    check_bit("an out-of-map load waits for its region answer", dut.region_stall, 1'b1);
    check_bit("...which raises x_busy", x_busy, 1'b1);
    @(posedge clk);
    #1;
    check_bit("the answer is there on the next cycle", dut.ls_answer_valid, 1'b1);
    check_bit("...with the wait over", dut.region_stall, 1'b0);
    check_bit("...and it faults", trap_entry, 1'b1);
    check_hex("...as a LOAD access fault", trap_cause, 32'd5);

    clear_in();
    in.is_sw = 1'b1;
    reg_rs1 = 32'h0004_0000;
    #1;
    @(posedge clk);
    #1;
    check_hex("the same address STORES as a STORE/AMO access fault", trap_cause, 32'd7);

    clear_in();
    in.is_lw = 1'b1;
    reg_rs1 = 32'h0001_0000;   // the RAM's own base -- answered
    #1;
    @(posedge clk);
    #1;
    check_bit("a load the map answers does not fault", trap_entry, 1'b0);

    clear_in();
    in.is_lw = 1'b1;
    reg_rs1 = 32'h0002_0000;   // the timer's reserved window
    #1;
    @(posedge clk);
    #1;
    check_bit("a load from the timer's window is answered", trap_entry, 1'b0);
    clear_in();
    in.is_lw = 1'b1;
    reg_rs1 = 32'h0002_0020;   // the UART
    #1;
    @(posedge clk);
    #1;
    check_bit("...and so is one from the UART", trap_entry, 1'b0);
    clear_in();
    in.is_lw = 1'b1;
    reg_rs1 = 32'h0002_0028;   // the SPI controller
    #1;
    @(posedge clk);
    #1;
    check_bit("...and the SPI controller", trap_entry, 1'b0);

    // --- The interrupt bubble commits exactly like any other trap, off `in.is_interrupt`
    // alone: no register value is needed, so this is the one trap D can raise with no
    // help from X's operand ports. ---
    clear_in();
    in.is_interrupt = 1'b1;
    in.pc = 32'h0000_0300;
    #1;
    check_hex("the interrupt bubble's cause is the machine timer", trap_cause, 32'h8000_0007);
    check_hex("...vectoring to mtvec", redirect_target, mtvec);
    check_hex("...at the pc of the instruction it displaced", trap_epc, 32'h0000_0300);
    check_bit("...counting nothing in minstret", instret, 1'b0);
    check_bit("...committing no CSR write", csr_wen, 1'b0);
    check_bit("...and no CSR read", csr_ren, 1'b0);
    check_bit("...and it is not an mret", mret_entry, 1'b0);
    check_hex("...and reporting no tval", trap_tval, 32'h0);

    // --- x_busy is exactly the divider or the region wait, nothing else; this is the
    // structural half of the Zkt isolation claim (formal covers it as an assertion, this
    // is the same fact read off simulation). ---
    clear_in();
    #1;
    check_bit("an ordinary instruction raises neither", x_busy, 1'b0);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: X control vectors (branch/jump resolution, trap priority and tval, CSR suppression, atomic address/fault, the region test, the interrupt bubble)");
      $finish;
    end
  end
endmodule
