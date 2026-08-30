`timescale 1 ns / 1 ps
`default_nettype none
// Runs CoreMark on littlecpu and Hazard3's iCE40 configuration, in one
// simulation, off one image, and counts each core's cycles on its own bus --
// the cycle factor docs/adr/0139-*.md left deferred.
//
// VexRiscv is NOT a third DUT here. soc/compare/bench_vexriscv.v's own header
// says why: that configuration is RV32IC, with no M extension at all -- not
// merely a slow one -- so a MUL or DIV encoding is not an instruction it can
// execute, and it has no trap machinery to survive one either. CoreMark needs
// real multiply/divide (its matrix and state-machine algorithms), so an
// RV32IMA image -- the ISA littlecpu and Hazard3's iCE40 build share -- is not
// one this harness can also hand VexRiscv. Dhrystone's own compare build works
// around the identical gap by dropping to the narrower ISA VexRiscv actually
// implements and letting libgcc's software routines carry the multiply; doing
// the same here would also drop littlecpu's and Hazard3's own hardware
// multipliers, which is exactly the difference this run exists to measure.
//
// The clock has to be counted out here for the same reason
// soc/compare/dhry_tb.v already counts Dhrystone's: Hazard3's iCE40
// configuration sets `CSR_COUNTER=0`, so it has no `mcycle` to self-time a run
// with. littlecpu is timed the same marker-counting way for consistency, the
// same choice dhry_tb.v makes for itself.
//
// THE GEOMETRY HERE IS NOT soc/compare/bench_hx8k.pcf'S EITHER. CoreMark's
// linked image is roughly four times Dhrystone's even at RV32IMA with no
// compressed encodings, so this is 16 KB of ROM and 16 KB of RAM against the
// placed harness's 4 KB and 2 KB -- soc/compare/coremark_fit.py prints the
// arithmetic on every run, the same shape soc/compare/dhry_fit.py already
// prints for Dhrystone.
//
// ---- what makes this comparison able to fail -------------------------------
//
// Two cores agreeing on CoreMark's own list/matrix/state CRCs against EEMBC's
// published values for the 2K performance run agree on far more than one bit.
// So the two data RAMs are compared word for word when the run ends: same
// image, same memories, no interrupt and no timer on either side, so the two
// RAMs hold the same 16 KB or one of the cores computed something else -- and
// the cycles counted for a core that computed something else are not a
// measurement of anything.
//
// Both RAMs are loaded with the same image and both register files zeroed
// before the run, for the reason test/testbench.v zeroes its ROM banks and
// dhry_tb.v zeroes both its cores': block RAM (and Hazard3's own register
// file, built with `RESET_REGFILE=0` -- see hazard3_regfile_1w2r.v -- so
// reset never touches it) comes up holding whatever the bitstream put there,
// and a simulated memory that is X where nothing was written is not a model
// of one.
//
// The RAM image carries the benchmark's initialised data AND its `.rodata`:
// soc/compare/bench_vexriscv.v's comment about giving its core no data path
// to the ROM does not apply to this pair, but CoreMark's own core_state.c
// reads its `intpat`/`floatpat`/`scipat`/`errpat` string tables
// algorithmically, not only to print them, and soc/compare/coremark.lds keeps
// all of it in the poked RAM region rather than splitting it -- see that file
// for the fuller reason. soc/compare/coremark_start.S is the startup with the
// copy loop removed.
module coremark_tb;
  // soc/compare/coremark.lds' ram ORIGIN, where its .coremarkctl section is
  // placed.
  localparam bit [31:0] CTL_MARK = 32'h0001_0000;
  localparam bit [31:0] CTL_DONE = 32'h0001_0004;
  // Held against soc/compare/coremark.lds by soc/compare/run_coremark_compare.sh,
  // which reads both regions out of that file and compares them with these.
  localparam int ROM_WORDS = 4096;
  localparam int RAM_WORDS = 4096;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic ours_led0_n, ours_led1_n, haz_led0_n, haz_led1_n;

  bench_littlecpu #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_EVEN("soc/compare/coremark_even.hex"),
    .INIT_ODD("soc/compare/coremark_odd.hex")
  ) dut_ours (
    .clk(clk), .led0_n(ours_led0_n), .led1_n(ours_led1_n)
  );

  bench_hazard3 #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_ROM("soc/compare/coremark_flat.hex")
  ) dut_haz (
    .clk(clk), .led0_n(haz_led0_n), .led1_n(haz_led1_n)
  );

  int unsigned cycle = 0;
  int unsigned cycle_limit;

  // Per core: the cycle each marker was seen, how many write cycles it spent,
  // and the self-check word the benchmark ended on.
  int unsigned ours_begin = 0, ours_end = 0, ours_marks = 0;
  int unsigned haz_begin = 0, haz_end = 0, haz_marks = 0;
  int unsigned ours_writes = 0, haz_writes = 0;
  int unsigned ours_verdict = 0, haz_verdict = 0;

  int i;
  initial begin
    $readmemh("soc/compare/coremark_ram.hex", dut_ours.dmem.ram);
    $readmemh("soc/compare/coremark_ram.hex", dut_haz.dmem.ram);
    for (i = 0; i < 32; i = i + 1) begin
      dut_ours.riscv.regfile.regs_a[i] = 32'b0;
      dut_ours.riscv.regfile.regs_b[i] = 32'b0;
      dut_haz.core.core.regs.real_dualport_noreset.mem[i] = 32'b0;
    end
  end

  always_ff @(posedge clk) begin
    cycle <= cycle + 1;

    if (|dut_ours.mem_wstrb) begin
      ours_writes <= ours_writes + 1;
      if (dut_ours.mem_addr == CTL_MARK) begin
        ours_marks <= ours_marks + 1;
        if (ours_marks == 0) ours_begin <= cycle;
        else if (ours_marks == 1) ours_end <= cycle;
      end
      if (dut_ours.mem_addr == CTL_DONE) ours_verdict <= dut_ours.mem_wdata;
    end

    // Hazard3 has no separate data bus -- see soc/compare/bench_hazard3.v and
    // soc/compare/bench_tb.v's own comment for why its publications are read
    // off `mem_addr_mux`/`mem_wstrb_mux`/`hwdata` rather than off `haddr` on
    // its own cycle.
    if (dut_haz.mem_wstrb_mux == 4'b1111) begin
      haz_writes <= haz_writes + 1;
      if (dut_haz.mem_addr_mux == CTL_MARK) begin
        haz_marks <= haz_marks + 1;
        if (haz_marks == 0) haz_begin <= cycle;
        else if (haz_marks == 1) haz_end <= cycle;
      end
      if (dut_haz.mem_addr_mux == CTL_DONE) haz_verdict <= dut_haz.hwdata;
    end
  end

  // Every fact this prints is raw. soc/compare/coremark_dmips.py grades them
  // and computes the figures, so the arithmetic is in a file
  // test/probe_gates.sh can drive against fixtures rather than in a
  // simulation nothing can force red.
  task automatic report(input string core, input int unsigned marks,
                        input int unsigned begin_cycle, input int unsigned end_cycle,
                        input int unsigned verdict, input int unsigned writes);
    $display("COREMARK core=%s marks=%0d cycles=%0d verdict=%0d writes=%0d",
             core, marks, end_cycle - begin_cycle, verdict, writes);
  endtask

  int unsigned differing = 0;
  initial begin
    if (!$value$plusargs("cycles=%d", cycle_limit)) cycle_limit = 200000000;

    while (cycle < cycle_limit && !(ours_verdict != 0 && haz_verdict != 0)) begin
      @(posedge clk);
    end
    // One more edge so the last write's registered effects are visible.
    @(posedge clk);

    for (i = 0; i < RAM_WORDS; i = i + 1) begin
      if (dut_ours.dmem.ram[i] !== dut_haz.dmem.ram[i]) differing = differing + 1;
    end

    $display("COREMARK ran %0d cycles of a %0d cycle limit", cycle, cycle_limit);
    report("littlecpu", ours_marks, ours_begin, ours_end, ours_verdict, ours_writes);
    report("hazard3", haz_marks, haz_begin, haz_end, haz_verdict, haz_writes);
    $display("COREMARK ramdiff=%0d of=%0d words", differing, RAM_WORDS);
    $finish;
  end
endmodule
