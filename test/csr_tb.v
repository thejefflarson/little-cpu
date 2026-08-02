`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// rtl/csrs.v's own bench: drive the access port directly rather than infer the
// CSR file's behaviour from a whole-pipeline run. Three things it covers that
// nothing else can.
//
//  1. The read mux, including which addresses `implemented` accepts. That
//     output feeds rtl/decoder.v's `instr_valid`, so an address wrongly
//     accepted becomes an instruction the core executes and one wrongly
//     rejected becomes an illegal instruction -- neither reads as a CSR bug in
//     a `.S` test.
//  2. WARL masking, per field, which the riscv-formal ladder structurally
//     cannot check: rvfi_csrw_check.sv has no WARL model, so mtvec/mepc/mstatus
//     are off formal/checks.cfg's [csrs] list on purpose.
//  3. Write suppression as seen from this side -- wen low means nothing
//     changes. Which instructions drive wen low is the decoder's business and
//     is checked in test/decoder_tb.v.
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
  // The second write port. rtl/decoder.v drives these for exactly the cycle it
  // commits a trap (or an mret); this bench drives them directly (ADR-0028).
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc;
  logic [31:0] mtvec_value, mepc_value;
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
    .mtvec_value(mtvec_value),
    .mepc_value(mepc_value)
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

  // Read `a` combinationally, without issuing anything.
  task automatic peek(input logic [11:0] a);
    begin
      addr = a;
      ren = 1'b0;
      wen = 1'b0;
      #1;
    end
  endtask

  // Commit one write on the next edge, the way rtl/decoder.v does.
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

  // Commit one trap entry on the next edge, the way rtl/decoder.v does.
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
    reset = 1'b1;
    repeat (2) @(posedge clk);
    #1;
    reset = 1'b0;

    check_read("mscratch resets to 0", 12'h340, 32'h0);
    // 0 rather than an unmapped address: the harness, not the RTL, is what makes
    // a trap taken before a handler is installed loud (ADR-0029).
    check_read("mtvec resets to 0", 12'h305, 32'h0);
    check_read("mepc resets to 0", 12'h341, 32'h0);
    check_read("mcause resets to 0", 12'h342, 32'h0);
    // MPP is hardwired to M-mode out of reset, MIE/MPIE clear.
    check_read("mstatus resets to MPP=M", 12'h300, 32'h0000_1800);

    check_read("misa", 12'h301, 32'h4000_1104);
    check_read("mie reads 0", 12'h304, 32'h0);
    check_read("mip reads 0", 12'h344, 32'h0);
    check_read("mtval reads 0", 12'h343, 32'h0);
    check_read("mvendorid", 12'hF11, 32'h0);
    check_read("marchid", 12'hF12, 32'h0);
    check_read("mimpid", 12'hF13, 32'h0);
    check_read("mhartid", 12'hF14, 32'h0);
    check_read("minstret", 12'hB02, 32'h0);
    check_read("minstreth", 12'hB82, 32'h0);
    check_read("mcycleh", 12'hB80, 32'h0);

    // The privileged spec lists both unconditionally for RV32 machine mode, so
    // `implemented` low on either is a conformance failure rather than a scope
    // choice -- ADR-0005's set is a floor, not a closed list.
    check_read("mstatush reads 0", 12'h310, 32'h0);
    check_read("mconfigptr reads 0", 12'hF15, 32'h0);

    // mstatush is writable by encoding (addr[11:10] == 2'b00) and has no
    // implemented fields, so a write is a legal WARL no-op: it must neither trap
    // nor retain, which are conformance bugs in opposite directions.
    poke(12'h310, 32'hFFFF_FFFF);
    check_read("mstatush still reads 0 after a write", 12'h310, 32'h0);

    // mconfigptr is read-only by encoding (addr[11:10] == 2'b11), so the
    // decoder rejects a write before rtl/csrs.v is asked. Only the read side is
    // this module's half.
    check_read("mconfigptr reads 0 after an attempted write", 12'hF15, 32'h0);

    // Addresses this core does not have. `implemented` low is what makes
    // rtl/decoder.v call the instruction unrecognised.
    peek(12'h7C0); // a custom/unimplemented machine CSR
    check_bit("0x7c0 is not implemented", implemented, 1'b0);
    check_hex("...and reads 0", rdata, 32'h0);
    peek(12'h306); // mcounteren: a real CSR name this core does not have
    check_bit("mcounteren is not implemented", implemented, 1'b0);
    peek(12'hC00); // the unprivileged cycle alias: M-mode only (ADR-0005)
    check_bit("the unprivileged cycle alias is not implemented", implemented, 1'b0);
    peek(12'h000);
    check_bit("address 0 is not implemented", implemented, 1'b0);

    poke(12'h340, 32'hdead_beef);
    check_read("mscratch round-trips every bit", 12'h340, 32'hdead_beef);
    poke(12'h342, 32'h0000_000b);
    check_read("mcause round-trips", 12'h342, 32'h0000_000b);

    // wen low changes nothing, even with a live address and data.
    addr = 12'h340;
    wdata = 32'h0;
    ren = 1'b1;
    wen = 1'b0;
    @(posedge clk);
    #1;
    ren = 1'b0;
    check_read("wen low leaves mscratch alone", 12'h340, 32'hdead_beef);

    // mtvec: direct mode, 4-byte aligned base. The two low bits never stick.
    poke(12'h305, 32'h0000_0103);
    check_read("mtvec masks bits [1:0]", 12'h305, 32'h0000_0100);
    // mepc masks bit 0 only. Bit 1 is a legal value -- C makes 2-byte targets
    // legal -- so masking it too would be a bug rather than extra safety.
    poke(12'h341, 32'h0000_0103);
    check_read("mepc masks bit 0 only", 12'h341, 32'h0000_0102);
    poke(12'h300, 32'h0000_0000);
    check_read("mstatus MPP is hardwired to M", 12'h300, 32'h0000_1800);
    poke(12'h300, 32'hffff_ffff);
    check_read("mstatus keeps only MIE, MPIE and MPP", 12'h300, 32'h0000_1888);
    poke(12'h300, 32'h0000_0008);
    check_read("mstatus MIE alone", 12'h300, 32'h0000_1808);

    // Read-only CSRs ignore writes rather than trapping here: rtl/csrs.v never
    // decides an access is illegal, and the read-only test on the address lives
    // in rtl/decoder.v with every other trap cause. The real core therefore no
    // longer produces these writes, but the WARL fallback that swallows them is
    // what makes the RVFI report tell the truth about what landed.
    poke(12'h301, 32'hffff_ffff);
    check_read("misa ignores a write", 12'h301, 32'h4000_1104);
    poke(12'h304, 32'hffff_ffff);
    check_read("mie ignores a write", 12'h304, 32'h0);
    poke(12'h344, 32'hffff_ffff);
    check_read("mip ignores a write", 12'h344, 32'h0);
    poke(12'h343, 32'hffff_ffff);
    check_read("mtval ignores a write", 12'h343, 32'h0);
    poke(12'hF14, 32'hffff_ffff);
    check_read("mhartid ignores a write", 12'hF14, 32'h0);

    // mcycle counts cycles on its own; nothing has to ask it to.
    peek(12'hB00);
    before_lo = rdata;
    @(posedge clk);
    peek(12'hB00);
    check_hex("mcycle advances every cycle", rdata, before_lo + 32'd1);

    // minstret counts only the cycles decode says an instruction issued
    // (ADR-0027).
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

    // An explicit write beats that cycle's increment. Driven with instret high,
    // which is the case that would otherwise land value+1.
    instret = 1'b1;
    poke(12'hB02, 32'h0000_0100);
    instret = 1'b0;
    check_read("an explicit minstret write beats the increment", 12'hB02, 32'h0000_0100);

    // The halves are separate addresses of one 64-bit counter: writing the
    // low one leaves the high one alone.
    poke(12'hB82, 32'h0000_0007);
    check_read("minstreth round-trips", 12'hB82, 32'h0000_0007);
    poke(12'hB02, 32'h0000_0000);
    check_read("...and a low write leaves it alone", 12'hB82, 32'h0000_0007);

    peek(12'hB80);
    before_hi = rdata;
    poke(12'hB00, 32'h0000_0000);
    check_read("an explicit mcycle write beats the increment", 12'hB00, 32'h0000_0000);
    check_read("...and does not disturb mcycleh", 12'hB80, before_hi);

    // The same at the carry boundary, which is the only place that last claim
    // can be false. "Any CSR write takes precedence over the automatic
    // increment" (priv spec 20211203 §3.1.11) is about the 64-bit counter, not
    // the half the address names, so with mcycle == 32'hffff_ffff a write to the
    // low half must suppress a carry that would otherwise land in mcycleh.
    //
    // Nothing else in the tree can reach it: rvfi_csrw_check.sv reads only the
    // self-reported masks and never observes the register, every ladder check is
    // `mode bmc` from reset, and a `.S` program lands a write on that one cycle
    // only by calibrating instruction spacing. Do not drop these vectors as
    // duplicates of the two above (ADR-0048).
    poke(12'hB80, 32'h0000_0000);
    poke(12'hB00, 32'hffff_fffe);
    @(posedge clk);
    #1;
    check_read("mcycle free-runs up to the carry boundary", 12'hB00, 32'hffff_ffff);
    poke(12'hB00, 32'h0000_0000);
    check_read("a write at the boundary still beats the increment", 12'hB00, 32'h0000_0000);
    check_read("...and its discarded carry does not reach mcycleh", 12'hB80, 32'h0000_0000);

    // The same rule for minstret, where `instret` rather than the clock is
    // what drives the counter into the boundary.
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

    // Trap entry and mret: the privileged-spec sequence, driven directly.
    // Nothing on the riscv-formal ladder can see any of it -- rvfi_insn_check
    // drops every value assertion under `spec_trap`, and mtvec/mepc/mcause/
    // mstatus are off the [csrs] list because rvfi_csrw_check.sv has no WARL
    // model. This bench and test/asm/trap.S are the whole coverage.
    poke(12'h305, 32'h0000_0100);   // mtvec = 0x100
    poke(12'h300, 32'h0000_0008);   // mstatus.MIE = 1, MPIE = 0
    check_hex("mtvec_value echoes mtvec for the decoder", mtvec_value, 32'h0000_0100);

    take_trap(32'd4, 32'h0000_0080);
    check_read("a trap records the cause", 12'h342, 32'd4);
    check_read("...and the faulting pc in mepc", 12'h341, 32'h0000_0080);
    check_read("...pushes MIE into MPIE and clears MIE", 12'h300, 32'h0000_1880);
    check_hex("mepc_value echoes mepc for the decoder", mepc_value, 32'h0000_0080);

    // mret pops it back: MIE <- MPIE, MPIE <- 1. mepc is NOT touched by mret;
    // the handler's own `csrw mepc` is what moves it (test/asm/riscv_test.h).
    take_mret();
    check_read("mret restores MIE from MPIE and sets MPIE", 12'h300, 32'h0000_1888);
    check_read("...and leaves mepc alone", 12'h341, 32'h0000_0080);
    check_read("...and leaves mcause alone", 12'h342, 32'd4);

    // A trap taken with MIE already clear leaves MPIE clear too -- the value
    // pushed is MIE, not a constant.
    poke(12'h300, 32'h0000_0000);
    take_trap(32'd2, 32'h0000_0200);
    check_read("a trap with MIE clear pushes a clear MPIE", 12'h300, 32'h0000_1800);
    check_read("...and records the new cause", 12'h342, 32'd2);

    // mepc's WARL mask applies to trap entry too, and masks bit 0 only: a
    // compressed instruction faulting at pc % 4 == 2 must record an mepc with
    // bit 1 set, or the handler resumes two bytes early. test/asm/trap.S faults
    // a real `c.lw` at that alignment for the same reason.
    take_trap(32'd4, 32'h0000_0146);
    check_read("mepc preserves bit 1 on a 2-aligned faulting pc", 12'h341, 32'h0000_0146);
    take_trap(32'd4, 32'h0000_0147);
    check_read("...and still masks bit 0", 12'h341, 32'h0000_0146);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: CSR file (read mux, implemented set, WARL, suppression, counters, trap entry)");
      $finish;
    end
  end
endmodule
