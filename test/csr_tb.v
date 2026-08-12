`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// rtl/csrs.v's access port, driven directly.
//
// Two things here are checked nowhere else. `implemented` feeds
// rtl/decoder.v's `instr_valid`, so an address wrongly accepted becomes an
// instruction the core executes and one wrongly rejected becomes an illegal
// instruction -- in a `.S` test either reads as an execution bug rather than a
// CSR one. And the WARL masks cannot be checked by riscv-formal at
// all: rvfi_csrw_check.sv compares the write against the value the core says it
// wrote, with no model of a register that legally keeps only some bits, so a
// correctly masked CSR fails it. That is why mtvec, mepc and mstatus are kept
// off the `[csrs]` list in formal/checks.cfg.
module csr_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  logic [11:0] addr;
  logic        ren, wen;
  logic [31:0] wdata;
  logic [31:0] rdata;
  logic        implemented;
  logic        instret;
  // The second write port, driven for exactly the cycle rtl/decoder.v commits
  // a trap or an mret.
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc;
  logic [31:0] mtvec_value, mepc_value;
  // The platform's timer line, and the one bit rtl/decoder.v reads back.
  logic        irq_timer;
  logic        interrupt_pending;
 `ifdef RISCV_FORMAL
  rvfi_csr64 rvfi_mcycle, rvfi_minstret;
  rvfi_csr32 rvfi_mscratch;
 `endif

  csrs dut (
    .clk(clk),
    .reset(reset),
    .addr(addr),
    .ren(ren),
    .wen(wen),
    .wdata(wdata),
    .rdata(rdata),
    .implemented(implemented),
    .instret(instret),
    .trap_entry(trap_entry),
    .trap_cause(trap_cause),
    .trap_epc(trap_epc),
    .mret_entry(mret_entry),
    .irq_timer(irq_timer),
    .mtvec_value(mtvec_value),
    .mepc_value(mepc_value),
    .interrupt_pending(interrupt_pending)
   `ifdef RISCV_FORMAL
    ,
    .rvfi_mcycle(rvfi_mcycle),
    .rvfi_minstret(rvfi_minstret),
    .rvfi_mscratch(rvfi_mscratch)
   `endif
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

  task automatic peek(input logic [11:0] a);
    begin
      addr = a;
      ren = 1'b0;
      wen = 1'b0;
      #1;
    end
  endtask

  task automatic poke(input logic [11:0] a, input logic [31:0] d);
    begin
      addr = a;
      ren = 1'b0;
      wen = 1'b1;
      wdata = d;
      @(posedge clk);
      #1;
      wen = 1'b0;
    end
  endtask

  task automatic check_read(input string what, input logic [11:0] a,
                            input logic [31:0] expected);
    begin
      peek(a);
      check_hex(what, rdata, expected);
      check_bit(what, implemented, 1'b1);
    end
  endtask

  task automatic take_trap(input logic [31:0] cause, input logic [31:0] epc);
    begin
      trap_cause = cause;
      trap_epc = epc;
      trap_entry = 1'b1;
      @(posedge clk);
      #1;
      trap_entry = 1'b0;
    end
  endtask

  task automatic take_mret;
    begin
      mret_entry = 1'b1;
      @(posedge clk);
      #1;
      mret_entry = 1'b0;
    end
  endtask

  logic [31:0] before_lo, before_hi;

  initial begin
    addr = 12'b0;
    ren = 1'b0;
    wen = 1'b0;
    wdata = 32'b0;
    instret = 1'b0;
    trap_entry = 1'b0;
    mret_entry = 1'b0;
    trap_cause = 32'b0;
    trap_epc = 32'b0;
    irq_timer = 1'b0;
    reset = 1'b1;
    repeat (2) @(posedge clk);
    #1;
    reset = 1'b0;

    check_read("mscratch resets to 0", 12'h340, 32'h0);
    check_read("mtvec resets to 0", 12'h305, 32'h0);
    check_read("mepc resets to 0", 12'h341, 32'h0);
    check_read("mcause resets to 0", 12'h342, 32'h0);
    check_read("mstatus resets to MPP=M", 12'h300, 32'h0000_1800);

    check_read("misa", 12'h301, 32'h4000_1104);
    check_read("mie resets to 0", 12'h304, 32'h0);
    check_read("mip reads 0 with no source asserting", 12'h344, 32'h0);
    check_read("mtval reads 0", 12'h343, 32'h0);
    check_read("mvendorid", 12'hF11, 32'h0);
    check_read("marchid", 12'hF12, 32'h0);
    check_read("mimpid", 12'hF13, 32'h0);
    check_read("mhartid", 12'hF14, 32'h0);
    check_read("minstret", 12'hB02, 32'h0);
    check_read("minstreth", 12'hB82, 32'h0);
    check_read("mcycleh", 12'hB80, 32'h0);

    // The privileged spec lists both unconditionally for RV32 machine mode, so
    // `implemented` low on either is a conformance failure, not a scope choice.
    check_read("mstatush reads 0", 12'h310, 32'h0);
    check_read("mconfigptr reads 0", 12'hF15, 32'h0);

    // Writable by encoding with no implemented fields, so a write is a legal
    // WARL no-op: neither trapping nor retaining, which are conformance bugs in
    // opposite directions.
    poke(12'h310, 32'hFFFF_FFFF);
    check_read("mstatush still reads 0 after a write", 12'h310, 32'h0);

    // Read-only by encoding, so the decoder rejects a write before rtl/csrs.v
    // is asked; only the read side is this module's half.
    check_read("mconfigptr reads 0 after an attempted write", 12'hF15, 32'h0);

    peek(12'h7C0); // a custom/unimplemented machine CSR
    check_bit("0x7c0 is not implemented", implemented, 1'b0);
    check_hex("...and reads 0", rdata, 32'h0);
    peek(12'h306); // mcounteren: a real CSR name this core does not have
    check_bit("mcounteren is not implemented", implemented, 1'b0);
    peek(12'hC00); // the unprivileged cycle alias; this core has no user mode
    check_bit("the unprivileged cycle alias is not implemented", implemented, 1'b0);
    peek(12'h000);
    check_bit("address 0 is not implemented", implemented, 1'b0);

    poke(12'h340, 32'hdead_beef);
    check_read("mscratch round-trips every bit", 12'h340, 32'hdead_beef);
    poke(12'h342, 32'h0000_000b);
    check_read("mcause round-trips", 12'h342, 32'h0000_000b);

    addr = 12'h340;
    wdata = 32'h0;
    ren = 1'b1;
    wen = 1'b0;
    @(posedge clk);
    #1;
    ren = 1'b0;
    check_read("wen low leaves mscratch alone", 12'h340, 32'hdead_beef);

    poke(12'h305, 32'h0000_0103);
    check_read("mtvec masks bits [1:0]", 12'h305, 32'h0000_0100);
    // Bit 1 of mepc is a legal value, because C makes 2-byte targets legal, so
    // masking it too would be a bug rather than extra safety.
    poke(12'h341, 32'h0000_0103);
    check_read("mepc masks bit 0 only", 12'h341, 32'h0000_0102);
    poke(12'h300, 32'h0000_0000);
    check_read("mstatus MPP is hardwired to M", 12'h300, 32'h0000_1800);
    poke(12'h300, 32'hffff_ffff);
    check_read("mstatus keeps only MIE, MPIE and MPP", 12'h300, 32'h0000_1888);
    poke(12'h300, 32'h0000_0008);
    check_read("mstatus MIE alone", 12'h300, 32'h0000_1808);

    // rtl/csrs.v never decides an access is illegal -- the read-only test on
    // the address is in rtl/decoder.v with every other trap cause -- so the
    // WARL fallback that swallows these is what makes the RVFI report tell the
    // truth about what landed.
    poke(12'h301, 32'hffff_ffff);
    check_read("misa ignores a write", 12'h301, 32'h4000_1104);
    poke(12'h344, 32'hffff_ffff);
    check_read("mip ignores a write -- MTIP is the platform's line", 12'h344, 32'h0);
    poke(12'h343, 32'hffff_ffff);
    check_read("mtval ignores a write", 12'h343, 32'h0);
    poke(12'hF14, 32'hffff_ffff);
    check_read("mhartid ignores a write", 12'hF14, 32'h0);

    peek(12'hB00);
    before_lo = rdata;
    @(posedge clk);
    peek(12'hB00);
    check_hex("mcycle advances every cycle", rdata, before_lo + 32'd1);

    peek(12'hB02);
    before_lo = rdata;
    instret = 1'b0;
    @(posedge clk);
    peek(12'hB02);
    check_hex("minstret holds when nothing issues", rdata, before_lo);
    instret = 1'b1;
    @(posedge clk);
    #1;
    instret = 1'b0;
    peek(12'hB02);
    check_hex("minstret counts an issue", rdata, before_lo + 32'd1);

    // Driven with instret high, the case that would otherwise land value+1.
    instret = 1'b1;
    poke(12'hB02, 32'h0000_0100);
    instret = 1'b0;
    check_read("an explicit minstret write beats the increment", 12'hB02, 32'h0000_0100);

    poke(12'hB82, 32'h0000_0007);
    check_read("minstreth round-trips", 12'hB82, 32'h0000_0007);
    poke(12'hB02, 32'h0000_0000);
    check_read("...and a low write leaves it alone", 12'hB82, 32'h0000_0007);

    peek(12'hB80);
    before_hi = rdata;
    poke(12'hB00, 32'h0000_0000);
    check_read("an explicit mcycle write beats the increment", 12'hB00, 32'h0000_0000);
    check_read("...and does not disturb mcycleh", 12'hB80, before_hi);

    // Do not drop these as duplicates of the two vectors above. A CSR write
    // takes precedence over that cycle's automatic increment for the whole
    // 64-bit counter, not just for the half the address names, so the carry
    // boundary is the one place the rule can be broken: with the low half at
    // 0xffff_ffff a write to it must also suppress the carry into the high
    // half. Nothing else reaches that cycle -- the generated CSR checks read
    // only what the core reports writing and never look at the register, and a
    // `.S` program lands a write there only by calibrating instruction spacing.
    poke(12'hB80, 32'h0000_0000);
    poke(12'hB00, 32'hffff_fffe);
    @(posedge clk);
    #1;
    check_read("mcycle free-runs up to the carry boundary", 12'hB00, 32'hffff_ffff);
    poke(12'hB00, 32'h0000_0000);
    check_read("a write at the boundary still beats the increment", 12'hB00, 32'h0000_0000);
    check_read("...and its discarded carry does not reach mcycleh", 12'hB80, 32'h0000_0000);

    poke(12'hB82, 32'h0000_0000);
    poke(12'hB02, 32'hffff_fffe);
    instret = 1'b1;
    @(posedge clk);
    #1;
    check_read("minstret counts up to the carry boundary", 12'hB02, 32'hffff_ffff);
    poke(12'hB02, 32'h0000_0000);
    instret = 1'b0;
    check_read("a write at the boundary still beats the increment", 12'hB02, 32'h0000_0000);
    check_read("...and its discarded carry does not reach minstreth", 12'hB82, 32'h0000_0000);

    //-----------------------------------------------------------------------
    // The 87 hardware performance monitor addresses. The spec asks for all 29
    // counters and their event selectors and permits both to be read-only
    // zero, so `implemented` high with a zero read is the whole contract --
    // and there is no state behind them for a `.S` program to observe. Each is
    // recognised by an address range, so BOTH ENDS of every range are read
    // here and so are the addresses just outside it: a range one address too
    // wide swallows a neighbour and nothing else in this file would say so.
    //-----------------------------------------------------------------------

    check_read("mhpmcounter3 reads 0", 12'hB03, 32'h0);
    check_read("mhpmcounter31 reads 0", 12'hB1F, 32'h0);
    check_read("mhpmcounter3h reads 0", 12'hB83, 32'h0);
    check_read("mhpmcounter31h reads 0", 12'hB9F, 32'h0);
    check_read("mhpmevent3 reads 0", 12'h323, 32'h0);
    check_read("mhpmevent31 reads 0", 12'h33F, 32'h0);

    // Writable by encoding with no implemented fields, so a write is a legal
    // WARL no-op on all three ranges, the way mstatush's is.
    poke(12'hB03, 32'hffff_ffff);
    check_read("mhpmcounter3 still reads 0 after a write", 12'hB03, 32'h0);
    poke(12'hB9F, 32'hffff_ffff);
    check_read("mhpmcounter31h still reads 0 after a write", 12'hB9F, 32'h0);
    poke(12'h33F, 32'hffff_ffff);
    check_read("mhpmevent31 still reads 0 after a write", 12'h33F, 32'h0);

    // One past the top of each range, and the counter numbers 0-2 inside each
    // window that belong to something else or to nothing.
    peek(12'hB20);
    check_bit("0xb20 is one past mhpmcounter31", implemented, 1'b0);
    peek(12'hBA0);
    check_bit("0xba0 is one past mhpmcounter31h", implemented, 1'b0);
    // Number 3 of the window ABOVE each range. A compare that took one bit too
    // few of the address would double every range's width, and these are the
    // only addresses that see it: the three above are number 0 of that window
    // and stay illegal on the counter-number test alone.
    peek(12'hB23);
    check_bit("0xb23 is number 3 of the window above the counters", implemented, 1'b0);
    peek(12'hBA3);
    check_bit("0xba3 is number 3 of the window above the high counters", implemented, 1'b0);
    peek(12'h345);
    check_bit("0x345 is inside the window above the events", implemented, 1'b0);
    peek(12'hB01);
    check_bit("0xb01 is inside the counter window and is not a counter", implemented, 1'b0);
    peek(12'hB81);
    check_bit("0xb81 is inside the high window and is not a counter", implemented, 1'b0);
    peek(12'h321);
    check_bit("0x321 is inside the event window and is not an event", implemented, 1'b0);
    peek(12'h322);
    check_bit("0x322 is one below mhpmevent3", implemented, 1'b0);

    // The implemented neighbours must still answer with their own register
    // rather than the ranges' zero. mscratch was written 0xdeadbeef above and
    // nothing since has touched it.
    poke(12'hB02, 32'h5a5a_5a5a);
    check_read("minstret still answers 0xb02, one below mhpmcounter3", 12'hB02, 32'h5a5a_5a5a);
    check_read("mscratch still answers 0x340, one past mhpmevent31", 12'h340, 32'hdead_beef);

    // Nothing in the generated riscv-formal checks sees any of this. Their
    // per-instruction checks drop every value assertion for a retire that
    // traps, and the CSRs a trap writes are the WARL ones the header explains
    // cannot go on the `[csrs]` list, so this bench and test/asm/trap.S are all
    // there is.
    poke(12'h305, 32'h0000_0100);   // mtvec = 0x100
    poke(12'h300, 32'h0000_0008);   // mstatus.MIE = 1, MPIE = 0
    check_hex("mtvec_value echoes mtvec for the decoder", mtvec_value, 32'h0000_0100);

    take_trap(32'd4, 32'h0000_0080);
    check_read("a trap records the cause", 12'h342, 32'd4);
    check_read("...and the faulting pc in mepc", 12'h341, 32'h0000_0080);
    check_read("...pushes MIE into MPIE and clears MIE", 12'h300, 32'h0000_1880);
    check_hex("mepc_value echoes mepc for the decoder", mepc_value, 32'h0000_0080);

    // mepc is not touched by mret; the handler's own `csrw mepc` moves it.
    take_mret();
    check_read("mret restores MIE from MPIE and sets MPIE", 12'h300, 32'h0000_1888);
    check_read("...and leaves mepc alone", 12'h341, 32'h0000_0080);
    check_read("...and leaves mcause alone", 12'h342, 32'd4);

    poke(12'h300, 32'h0000_0000);
    take_trap(32'd2, 32'h0000_0200);
    check_read("a trap with MIE clear pushes a clear MPIE", 12'h300, 32'h0000_1800);
    check_read("...and records the new cause", 12'h342, 32'd2);

    // A compressed instruction faulting at pc % 4 == 2 must record an mepc with
    // bit 1 set, or the handler resumes two bytes early. test/asm/trap.S faults
    // a real `c.lw` at that alignment for the same reason.
    take_trap(32'd4, 32'h0000_0146);
    check_read("mepc preserves bit 1 on a 2-aligned faulting pc", 12'h341, 32'h0000_0146);
    take_trap(32'd4, 32'h0000_0147);
    check_read("...and still masks bit 0", 12'h341, 32'h0000_0146);

    //-----------------------------------------------------------------------
    // mie, mip and the interrupt decision. No riscv-formal check at the pin
    // names any of these, so this bench and test/asm/mtimer*.S are the whole
    // oracle for them.
    //-----------------------------------------------------------------------

    // MTIE is the only writable bit. MSIE and MEIE name sources this platform
    // does not have, and a WARL field with no source behind it reads zero
    // however it was written -- so writing all ones is the vector that
    // separates "one writable bit" from "a writable register".
    poke(12'h304, 32'hffff_ffff);
    check_read("mie keeps only MTIE", 12'h304, 32'h0000_0080);
    poke(12'h304, 32'h0000_0000);
    check_read("...and MTIE clears again", 12'h304, 32'h0);

    // mip is read-only and reports the line. Software lowers MTIP by moving
    // mtimecmp, which is a store to rtl/timer.v and not a CSR write at all.
    irq_timer = 1'b1;
    #1;
    check_read("mip.MTIP follows the platform line", 12'h344, 32'h0000_0080);
    poke(12'h344, 32'h0000_0000);
    check_read("...and a write cannot clear it", 12'h344, 32'h0000_0080);

    // The three-term gate, one term at a time. Each of these has been a real
    // bug in somebody's core: an interrupt that fires with MIE clear, one that
    // ignores its enable bit, and one that fires with no source at all.
    poke(12'h300, 32'h0000_0000);   // mstatus.MIE = 0
    poke(12'h304, 32'h0000_0080);   // mie.MTIE = 1
    #1;
    check_bit("MTIE and a source are not enough with mstatus.MIE clear",
              interrupt_pending, 1'b0);
    poke(12'h300, 32'h0000_0008);   // mstatus.MIE = 1
    poke(12'h304, 32'h0000_0000);   // mie.MTIE = 0
    #1;
    check_bit("...nor MIE and a source with MTIE clear", interrupt_pending, 1'b0);
    poke(12'h304, 32'h0000_0080);
    irq_timer = 1'b0;
    #1;
    check_bit("...nor both enables with no source", interrupt_pending, 1'b0);
    irq_timer = 1'b1;
    #1;
    check_bit("all three together arm it", interrupt_pending, 1'b1);

    // This is what bounds interrupt entry: taking one clears MIE on the same
    // edge, so the pending bit is already down on the next cycle and nothing
    // re-arms until `mret`. Delete the `mstatus_mie <= 1'b0` in rtl/csrs.v's
    // trap block and the core takes an interrupt every cycle forever, with
    // this vector the only thing that says so.
    take_trap(32'h8000_0007, 32'h0000_0400);
    check_bit("entry disarms it, so there is no second entry", interrupt_pending, 1'b0);
    check_read("...recording the interrupt cause", 12'h342, 32'h8000_0007);
    check_read("...and mepc, which points AT the un-executed instruction",
               12'h341, 32'h0000_0400);
    check_read("...having pushed MIE into MPIE", 12'h300, 32'h0000_1880);

    take_mret();
    #1;
    check_bit("mret pops MIE back, so a still-asserting line re-arms",
              interrupt_pending, 1'b1);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: CSR file (read mux, implemented set, WARL, suppression, counters, trap entry)");
      $finish;
    end
  end
endmodule
