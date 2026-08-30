`timescale 1 ns / 1 ps
`default_nettype none
// Runs Dhrystone on THIS core alone, at soc/compare/dhry_tb.v's own enlarged
// geometry (8 KB ROM, 16 KB RAM -- the same map, the same linker script, the
// same markers, the same soc/compare/dhry_monitor.v mechanism), so its cycle
// count is comparable to soc/compare/dhry_tb.v's three-way RV32I row with only
// the ISA differing.
//
// soc/compare/dhry_tb.v answers "what does the shared subset cost all three
// cores"; this answers "what does the shared subset cost THIS core, against
// its own native one" -- the second half of that question needs a core that
// executes an ISA the other two do not share, so it cannot be asked inside the
// three-way harness without also asking the RAM-comparison and the marker
// protocol to survive two different opcodes reaching two different vintages of
// hardware. One core, one image, one geometry keeps that isolated to the ISA
// the caller compiled the image with.
module dhry_solo_tb;
  // Held against soc/compare/dhry.lds by soc/compare/run_dhrystone.sh, the same
  // way soc/compare/dhry_tb.v is.
  localparam int ROM_WORDS = 2048;
  localparam int RAM_WORDS = 4096;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic led0_n, led1_n;

  bench_littlecpu #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_EVEN("soc/compare/dhry_even.hex"),
    .INIT_ODD("soc/compare/dhry_odd.hex")
  ) dut (
    .clk(clk), .led0_n(led0_n), .led1_n(led1_n)
  );

  int unsigned cycle = 0;
  int unsigned cycle_limit;
  int unsigned marks, begin_cycle, end_cycle, writes, verdict;

  dhry_monitor mon (
    .clk(clk), .cycle(cycle),
    .mem_addr(dut.mem_addr), .mem_wdata(dut.mem_wdata), .mem_wstrb(dut.mem_wstrb),
    .marks(marks), .begin_cycle(begin_cycle), .end_cycle(end_cycle),
    .writes(writes), .verdict(verdict)
  );

  int i;
  initial begin
    $readmemh("soc/compare/dhry_ram.hex", dut.dmem.ram);
    for (i = 0; i < 32; i = i + 1) begin
      dut.riscv.regfile.regs_a[i] = 32'b0;
      dut.riscv.regfile.regs_b[i] = 32'b0;
    end
  end

  always_ff @(posedge clk) cycle <= cycle + 1;

  initial begin
    if (!$value$plusargs("cycles=%d", cycle_limit)) cycle_limit = 2000000;

    while (cycle < cycle_limit && verdict == 0) begin
      @(posedge clk);
    end
    // One more edge so the last write's registered effects are visible.
    @(posedge clk);

    $display("DHRY ran %0d cycles of a %0d cycle limit", cycle, cycle_limit);
    $display("DHRY core=%s marks=%0d cycles=%0d verdict=%0d writes=%0d",
             "littlecpu", marks, end_cycle - begin_cycle, verdict, writes);
    $finish;
  end
endmodule
