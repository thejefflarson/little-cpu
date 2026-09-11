`timescale 1 ns / 1 ps
`default_nettype none
// CoreMark's own "ISA-cost row" (soc/compare/dhry_solo_tb.v's idea for Dhrystone), held
// against soc/compare/coremark.lds by soc/compare/run_coremark_compare.sh.
module coremark_solo_tb;
  localparam int ROM_WORDS = 4096;
  localparam int RAM_WORDS = 4096;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic led0_n, led1_n;

  bench_littlecpu #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_EVEN("soc/compare/coremark_even.hex"),
    .INIT_ODD("soc/compare/coremark_odd.hex")
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
    $readmemh("soc/compare/coremark_ram.hex", dut.dmem.ram);
    for (i = 0; i < 32; i = i + 1) begin
      dut.riscv.regfile.regs_a[i] = 32'b0;
      dut.riscv.regfile.regs_b[i] = 32'b0;
    end
  end

  always_ff @(posedge clk) cycle <= cycle + 1;

  initial begin
    if (!$value$plusargs("cycles=%d", cycle_limit)) cycle_limit = 200000000;

    while (cycle < cycle_limit && verdict == 0) begin
      @(posedge clk);
    end
    @(posedge clk);

    $display("COREMARK ran %0d cycles of a %0d cycle limit", cycle, cycle_limit);
    $display("COREMARK core=%s marks=%0d cycles=%0d verdict=%0d writes=%0d",
             "littlecpu", marks, end_cycle - begin_cycle, verdict, writes);
    $finish;
  end
endmodule
