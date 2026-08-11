`timescale 1 ns / 1 ps
`default_nettype none
// Runs Dhrystone on both cores of this directory's harness, in one simulation,
// off one image, and counts each core's cycles on its own data bus.
//
// The clock has to be counted out here because the two cores do not share a way
// to count it themselves: the VexRiscv configuration in the pinned riscv-formal
// clone has no CSR file, so `csrr mcycle` -- test/bench/dhry_port.c's timer --
// is not an instruction it can execute. soc/compare/dhry_port.c stores a marker
// to the word at the base of RAM instead, immediately before and immediately
// after the measured loop, and what is counted below is the cycles between the
// two markers appearing on each core's own write bus.
//
// THE GEOMETRY HERE IS NOT THE ONE soc/compare/bench_hx8k.pcf PLACES. Dhrystone
// needs more memory than an hx8k has block RAM for, so this is 8 KB of ROM and
// 16 KB of RAM against the placed harness's 4 KB and 2 KB -- the same enlarged
// map on BOTH cores, which is what keeps the two cycle counts comparable to
// each other. soc/compare/dhry_fit.py prints the arithmetic on every run.
//
// ---- what makes this comparison able to fail ------------------------------
//
// Two cores agreeing on a benchmark's self-check agree on one bit. So the two
// data RAMs are compared word for word when the run ends: same image, same
// memories, no interrupt and no timer on either side, so the two RAMs hold the
// same 16 KB or one of the cores computed something else -- and the cycles
// counted for a core that computed something else are not a measurement of
// anything. The bus traffic itself is not what is compared: this core can
// re-present a store while it is stalled and VexRiscv replicates store data
// across all four lanes, so the two write streams differ in shape while
// committing the same bytes.
//
// Both RAMs are loaded with the same image and both register files zeroed
// before the run, for the reason test/testbench.v zeroes its ROM banks: block
// RAM comes up holding whatever the bitstream put there, and a simulated memory
// that is X where nothing was written is not a model of one. Left X, the two
// cores' prologues store different undefined registers and the comparison above
// fails for a reason that is not about either core.
//
// The RAM image carries the benchmark's initialised data, because
// soc/compare/bench_vexriscv.v gives its core no data path to the ROM: a load
// from a ROM address reads back zero there, so a startup that copied `.data`
// out of ROM would move zeros on one core and the real bytes on the other.
// soc/compare/dhry_ram.py builds the image and soc/compare/dhry_start.S is the
// startup with the copy loop removed.
module dhry_tb;
  // soc/compare/dhry.lds' ram ORIGIN, where its .dhryctl section is placed.
  localparam bit [31:0] CTL_MARK = 32'h0001_0000;
  localparam bit [31:0] CTL_DONE = 32'h0001_0004;
  // Held against soc/compare/dhry.lds by soc/compare/run_dhrystone.sh, which
  // reads both regions out of that file and compares them with these.
  localparam int ROM_WORDS = 2048;
  localparam int RAM_WORDS = 4096;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic ours_led0_n, ours_led1_n, vex_led0_n, vex_led1_n;

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

  int unsigned cycle = 0;
  int unsigned cycle_limit;

  // Per core: the cycle each marker was seen, how many write cycles it spent,
  // and the self-check word the benchmark ended on.
  int unsigned ours_begin = 0, ours_end = 0, ours_marks = 0;
  int unsigned vex_begin = 0, vex_end = 0, vex_marks = 0;
  int unsigned ours_writes = 0, vex_writes = 0;
  int unsigned ours_verdict = 0, vex_verdict = 0;

  int i;
  initial begin
    $readmemh("soc/compare/dhry_ram.hex", dut_ours.dmem.ram);
    $readmemh("soc/compare/dhry_ram.hex", dut_vex.dmem.ram);
    for (i = 0; i < 32; i = i + 1) begin
      dut_ours.riscv.regfile.regs_a[i] = 32'b0;
      dut_ours.riscv.regfile.regs_b[i] = 32'b0;
      dut_vex.riscv.RegFilePlugin_regFile[i] = 32'b0;
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

    if (|dut_vex.mem_wstrb) begin
      vex_writes <= vex_writes + 1;
      if (dut_vex.dbus_cmd_address == CTL_MARK) begin
        vex_marks <= vex_marks + 1;
        if (vex_marks == 0) vex_begin <= cycle;
        else if (vex_marks == 1) vex_end <= cycle;
      end
      if (dut_vex.dbus_cmd_address == CTL_DONE) vex_verdict <= dut_vex.dbus_cmd_data;
    end
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

  int unsigned differing = 0;
  initial begin
    if (!$value$plusargs("cycles=%d", cycle_limit)) cycle_limit = 2000000;

    while (cycle < cycle_limit && !(ours_verdict != 0 && vex_verdict != 0)) begin
      @(posedge clk);
    end
    // One more edge so the last write's registered effects are visible.
    @(posedge clk);

    for (i = 0; i < RAM_WORDS; i = i + 1) begin
      if (dut_ours.dmem.ram[i] !== dut_vex.dmem.ram[i]) differing = differing + 1;
    end

    $display("DHRY ran %0d cycles of a %0d cycle limit", cycle, cycle_limit);
    report("littlecpu", ours_marks, ours_begin, ours_end, ours_verdict, ours_writes);
    report("vexriscv", vex_marks, vex_begin, vex_end, vex_verdict, vex_writes);
    $display("DHRY ramdiff=%0d of=%0d words", differing, RAM_WORDS);
    $finish;
  end
endmodule
