`timescale 1 ns / 1 ps
`default_nettype none
// Runs CoreMark on all three cores of this directory's harness, in one
// simulation, off one image, and counts each core's cycles on its own bus --
// the cycle factor docs/adr/0139-*.md left deferred, now widened to include
// VexRiscv once its generated build (soc/compare/vexriscv_pin.mk) carried a
// hardware multiplier and divider: the FormalSimple build this harness used
// to read had neither, so an RV32IM image was not one it could run.
//
// The clock has to be counted out here for the same reason
// soc/compare/dhry_tb.v already counts Dhrystone's: neither Hazard3's iCE40
// configuration (CSR_COUNTER=0) nor VexRiscv's generated CsrPluginConfig.small
// exposes the performance counters this port would otherwise read with
// `mcycle`. littlecpu is timed the same marker-counting way for consistency,
// the same choice dhry_tb.v makes for itself. soc/compare/dhry_monitor.v is
// the shared mechanism -- built for VexRiscv's CSR-free gap, reused rather
// than reinvented here, the way Hazard3's own CSR_COUNTER=0 already reuses it.
//
// Hazard3's D-port write buffer holds `d_hready` low for one cycle after
// every write's own address phase (soc/compare/bench_hazard3.v's
// `wr_pending_q`) -- see that comment for why -- and neither littlecpu's
// nor VexRiscv's harness pays an equivalent cost. Those cycles are counted
// directly, the same way dhry_tb.v counts them, rather than left folded
// into the cycle count with no way to size them back out.
//
// THE GEOMETRY HERE IS NOT soc/compare/bench_hx8k.pcf'S EITHER. CoreMark's
// linked image is roughly four times Dhrystone's even at RV32IM with no
// compressed or atomic encodings, so this is 16 KB of ROM and 16 KB of RAM
// against the placed harness's 4 KB and 2 KB -- soc/compare/coremark_fit.py
// prints the arithmetic on every run, the same shape soc/compare/dhry_fit.py
// already prints for Dhrystone.
//
// ---- what makes this comparison able to fail -------------------------------
//
// Three cores agreeing on CoreMark's own list/matrix/state CRCs against
// EEMBC's published values for the 2K performance run agree on far more than
// one bit. So this core's data RAM is compared word for word against each of
// the other two when the run ends: same image, same memories, no interrupt on
// any side, so all three RAMs hold the same 16 KB or one of the cores
// computed something else -- and the cycles counted for a core that computed
// something else are not a measurement of anything.
//
// All three RAMs are loaded with the same image and all three register files
// zeroed before the run, for the reason test/testbench.v zeroes its ROM banks
// and dhry_tb.v zeroes all three of its own: block RAM (and Hazard3's own
// register file, built with `RESET_REGFILE=0` -- see hazard3_regfile_1w2r.v --
// so reset never touches it) comes up holding whatever the bitstream put
// there, and a simulated memory that is X where nothing was written is not a
// model of one.
//
// The RAM image carries the benchmark's initialised data AND its `.rodata`:
// soc/compare/bench_vexriscv.v gives its core no data path to the ROM at all,
// and CoreMark's own core_state.c reads its `intpat`/`floatpat`/`scipat`/
// `errpat` string tables algorithmically, not only to print them, so
// soc/compare/coremark.lds keeps all of it in the poked RAM region rather
// than splitting it -- see that file for the fuller reason.
// soc/compare/coremark_start.S is the startup with the copy loop removed.
module coremark_tb;
  // soc/compare/coremark.lds' ram ORIGIN, where its .coremarkctl section is
  // placed -- the same window soc/compare/dhry.lds puts its own control
  // window at, which is what lets soc/compare/dhry_monitor.v watch it here
  // unmodified.
  // Held against soc/compare/coremark.lds by soc/compare/run_coremark_compare.sh,
  // which reads both regions out of that file and compares them with these.
  localparam int ROM_WORDS = 4096;
  localparam int RAM_WORDS = 4096;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic ours_led0_n, ours_led1_n, vex_led0_n, vex_led1_n, haz_led0_n, haz_led1_n;

  bench_littlecpu #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_EVEN("soc/compare/coremark_even.hex"),
    .INIT_ODD("soc/compare/coremark_odd.hex")
  ) dut_ours (
    .clk(clk), .led0_n(ours_led0_n), .led1_n(ours_led1_n)
  );

  bench_vexriscv #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_ROM("soc/compare/coremark_flat.hex")
  ) dut_vex (
    .clk(clk), .led0_n(vex_led0_n), .led1_n(vex_led1_n)
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
  // and the self-check word the benchmark ended on. Each is a
  // soc/compare/dhry_monitor.v output, which carries its own reset value.
  int unsigned ours_begin, ours_end, ours_marks;
  int unsigned vex_begin, vex_end, vex_marks;
  int unsigned haz_begin, haz_end, haz_marks;
  int unsigned ours_writes, vex_writes, haz_writes;
  int unsigned ours_verdict, vex_verdict, haz_verdict;
  // Cycles inside the measured window that Hazard3's D-port write buffer
  // spends holding `d_hready` low for a write's data phase -- see
  // soc/compare/bench_hazard3.v's `wr_pending_q` comment. littlecpu drives
  // `.bus_wait(1'b0)` and VexRiscv's bus here is always-ready, so neither of
  // the other two cores pays this; disclosing it beside Hazard3's cycle count
  // is what keeps that difference from hiding inside a single "cycles" number.
  int unsigned haz_wait_cycles = 0;

  // One monitor per core, instantiated on each DUT's own bus signal names --
  // soc/compare/dhry_monitor.v is the mechanism, shared rather than copied
  // three times.
  dhry_monitor mon_ours (
    .clk(clk), .cycle(cycle),
    .mem_addr(dut_ours.mem_addr), .mem_wdata(dut_ours.mem_wdata),
    .mem_wstrb(dut_ours.mem_wstrb),
    .marks(ours_marks), .begin_cycle(ours_begin), .end_cycle(ours_end),
    .writes(ours_writes), .verdict(ours_verdict)
  );
  dhry_monitor mon_vex (
    .clk(clk), .cycle(cycle),
    .mem_addr(dut_vex.dbus_cmd_address), .mem_wdata(dut_vex.dbus_cmd_data),
    .mem_wstrb(dut_vex.mem_wstrb),
    .marks(vex_marks), .begin_cycle(vex_begin), .end_cycle(vex_end),
    .writes(vex_writes), .verdict(vex_verdict)
  );
  // Hazard3's D-port write data trails its own address by a cycle:
  // dmem_wstrb_mux/dmem_addr_mux is the RAM port's captured write, with
  // d_hwdata (now valid) as its value -- the same signals
  // soc/compare/bench_tb.v's smoke check reads for the identical reason.
  dhry_monitor mon_haz (
    .clk(clk), .cycle(cycle),
    .mem_addr(dut_haz.dmem_addr_mux), .mem_wdata(dut_haz.d_hwdata),
    .mem_wstrb(dut_haz.dmem_wstrb_mux),
    .marks(haz_marks), .begin_cycle(haz_begin), .end_cycle(haz_end),
    .writes(haz_writes), .verdict(haz_verdict)
  );

  int i;
  initial begin
    $readmemh("soc/compare/coremark_ram.hex", dut_ours.dmem.ram);
    $readmemh("soc/compare/coremark_ram.hex", dut_vex.dmem.ram);
    $readmemh("soc/compare/coremark_ram.hex", dut_haz.dmem.ram);
    for (i = 0; i < 32; i = i + 1) begin
      dut_ours.riscv.regfile.regs_a[i] = 32'b0;
      dut_ours.riscv.regfile.regs_b[i] = 32'b0;
      dut_vex.riscv.RegFilePlugin_regFile[i] = 32'b0;
      dut_haz.core.core.regs.real_dualport_noreset.mem[i] = 32'b0;
    end
  end

  always_ff @(posedge clk) begin
    cycle <= cycle + 1;
    if (haz_marks == 1 && dut_haz.wr_pending_q) haz_wait_cycles <= haz_wait_cycles + 1;
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

  int unsigned differing_vex = 0, differing_haz = 0;
  initial begin
    if (!$value$plusargs("cycles=%d", cycle_limit)) cycle_limit = 200000000;

    while (cycle < cycle_limit &&
           !(ours_verdict != 0 && vex_verdict != 0 && haz_verdict != 0)) begin
      @(posedge clk);
    end
    // One more edge so the last write's registered effects are visible.
    @(posedge clk);

    for (i = 0; i < RAM_WORDS; i = i + 1) begin
      if (dut_ours.dmem.ram[i] !== dut_vex.dmem.ram[i]) differing_vex = differing_vex + 1;
      if (dut_ours.dmem.ram[i] !== dut_haz.dmem.ram[i]) differing_haz = differing_haz + 1;
    end

    $display("COREMARK ran %0d cycles of a %0d cycle limit", cycle, cycle_limit);
    report("littlecpu", ours_marks, ours_begin, ours_end, ours_verdict, ours_writes);
    report("vexriscv", vex_marks, vex_begin, vex_end, vex_verdict, vex_writes);
    report("hazard3", haz_marks, haz_begin, haz_end, haz_verdict, haz_writes);
    $display("COREMARK core=hazard3 wait_cycles=%0d", haz_wait_cycles);
    // Each non-reference core against littlecpu; agreement is transitive, so
    // this is the same claim as a three-way comparison for one fewer full-RAM
    // scan.
    $display("COREMARK ramdiff core=vexriscv diff=%0d of=%0d words", differing_vex, RAM_WORDS);
    $display("COREMARK ramdiff core=hazard3 diff=%0d of=%0d words", differing_haz, RAM_WORDS);
    $finish;
  end
endmodule
