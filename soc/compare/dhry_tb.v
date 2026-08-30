`timescale 1 ns / 1 ps
`default_nettype none
// Runs Dhrystone on all three cores of this directory's harness, in one
// simulation, off one image, and counts each core's cycles on its own data
// bus.
//
// The clock has to be counted out here because none of the three cores share a
// way to count it themselves: the VexRiscv configuration in the pinned
// riscv-formal clone has no CSR file, and Hazard3's iCE40 configuration sets
// CSR_COUNTER=0, so neither can execute `csrr mcycle` -- test/bench/dhry_port.c's
// timer. soc/compare/dhry_port.c stores a marker to the word at the base of RAM
// instead, immediately before and immediately after the measured loop, and what
// is counted below is the cycles between the two markers appearing on each
// core's own write bus.
//
// THE GEOMETRY HERE IS NOT THE ONE soc/compare/bench_hx8k.pcf PLACES. Dhrystone
// needs more memory than an hx8k has block RAM for, so this is 8 KB of ROM and
// 16 KB of RAM against the placed harness's 4 KB and 2 KB -- the same enlarged
// map on all three cores, which is what keeps the cycle counts comparable to
// each other. soc/compare/dhry_fit.py prints the arithmetic on every run.
//
// THE IMAGE IS RV32I, not the two-core comparison's RV32IC: Hazard3's iCE40
// build has EXTENSION_C=0, and the pinned VexRiscv has no M extension, so plain
// RV32I is the only subset every core here executes. Multiply and divide are
// libgcc calls on all three, even on the two cores whose netlists carry a
// hardware multiplier.
//
// ---- what makes this comparison able to fail ------------------------------
//
// Two cores agreeing on a benchmark's self-check agree on one bit. So this
// core's data RAM is compared word for word against each of the other two when
// the run ends: same image, same memories, no interrupt and no timer on any
// side, so all three RAMs hold the same 16 KB or one of the cores computed
// something else -- and the cycles counted for a core that computed something
// else are not a measurement of anything. The bus traffic itself is not what is
// compared: this core can re-present a store while it is stalled, VexRiscv
// replicates store data across all four lanes, and Hazard3's AHB5 adapter
// trails a write's data by a cycle behind its address, so the three write
// streams differ in shape while committing the same bytes.
//
// All three RAMs are loaded with the same image and all three register files
// zeroed before the run, for the reason test/testbench.v zeroes its ROM banks:
// block RAM comes up holding whatever the bitstream put there, and a simulated
// memory that is X where nothing was written is not a model of one. Left X, the
// cores' prologues store different undefined registers and the comparison above
// fails for a reason that is not about any of them. Hazard3's regfile needs the
// same treatment for the same reason the other two do: RESET_REGFILE defaults
// to 0 in soc/compare/bench_hazard3.v's configuration, matching
// fpga_icebreaker.v, so nothing but this testbench clears it.
//
// The RAM image carries the benchmark's initialised data, because
// soc/compare/bench_vexriscv.v gives its core no data path to the ROM: a load
// from a ROM address reads back zero there, so a startup that copied `.data`
// out of ROM would move zeros on one core and the real bytes on the others.
// soc/compare/dhry_ram.py builds the image and soc/compare/dhry_start.S is the
// startup with the copy loop removed.
module dhry_tb;
  // Held against soc/compare/dhry.lds by soc/compare/run_dhrystone.sh, which
  // reads both regions out of that file and compares them with these.
  localparam int ROM_WORDS = 2048;
  localparam int RAM_WORDS = 4096;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic ours_led0_n, ours_led1_n, vex_led0_n, vex_led1_n, haz_led0_n, haz_led1_n;

  bench_littlecpu #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_EVEN("soc/compare/dhry_even.hex"),
    .INIT_ODD("soc/compare/dhry_odd.hex")
  ) dut_ours (
    .clk(clk), .led0_n(ours_led0_n), .led1_n(ours_led1_n)
  );

  bench_vexriscv #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_ROM("soc/compare/dhry_flat.hex")
  ) dut_vex (
    .clk(clk), .led0_n(vex_led0_n), .led1_n(vex_led1_n)
  );

  bench_hazard3 #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_ROM("soc/compare/dhry_flat.hex")
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
  // Cycles inside the measured window that Hazard3's AHB5 adapter spends
  // holding `hready` low for a write's data phase -- see
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
  // Hazard3 has no separate data bus: mem_wstrb_mux/mem_addr_mux is the one
  // memory port's captured write, with hwdata (now valid) as its value -- the
  // same signals soc/compare/bench_tb.v's smoke check reads for the identical
  // reason.
  dhry_monitor mon_haz (
    .clk(clk), .cycle(cycle),
    .mem_addr(dut_haz.mem_addr_mux), .mem_wdata(dut_haz.hwdata),
    .mem_wstrb(dut_haz.mem_wstrb_mux),
    .marks(haz_marks), .begin_cycle(haz_begin), .end_cycle(haz_end),
    .writes(haz_writes), .verdict(haz_verdict)
  );

  int i;
  initial begin
    $readmemh("soc/compare/dhry_ram.hex", dut_ours.dmem.ram);
    $readmemh("soc/compare/dhry_ram.hex", dut_vex.dmem.ram);
    $readmemh("soc/compare/dhry_ram.hex", dut_haz.dmem.ram);
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

  // Every fact this prints is raw. soc/compare/dhry_dmips.py grades them and
  // computes the figures, so the arithmetic is in a file test/probe_gates.sh can
  // drive against fixtures rather than in a simulation nothing can force red.
  task automatic report(input string core, input int unsigned marks,
                        input int unsigned begin_cycle, input int unsigned end_cycle,
                        input int unsigned verdict, input int unsigned writes);
    $display("DHRY core=%s marks=%0d cycles=%0d verdict=%0d writes=%0d",
             core, marks, end_cycle - begin_cycle, verdict, writes);
  endtask

  int unsigned differing_vex = 0, differing_haz = 0;
  initial begin
    if (!$value$plusargs("cycles=%d", cycle_limit)) cycle_limit = 2000000;

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

    $display("DHRY ran %0d cycles of a %0d cycle limit", cycle, cycle_limit);
    report("littlecpu", ours_marks, ours_begin, ours_end, ours_verdict, ours_writes);
    report("vexriscv", vex_marks, vex_begin, vex_end, vex_verdict, vex_writes);
    report("hazard3", haz_marks, haz_begin, haz_end, haz_verdict, haz_writes);
    $display("DHRY core=hazard3 wait_cycles=%0d", haz_wait_cycles);
    // Each non-reference core against littlecpu; agreement is transitive, so
    // this is the same claim as a three-way comparison for two fewer full-RAM
    // scans.
    $display("DHRY ramdiff core=vexriscv diff=%0d of=%0d words", differing_vex, RAM_WORDS);
    $display("DHRY ramdiff core=hazard3 diff=%0d of=%0d words", differing_haz, RAM_WORDS);
    $finish;
  end
endmodule
