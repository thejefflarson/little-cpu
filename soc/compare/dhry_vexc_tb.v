`timescale 1 ns / 1 ps
`default_nettype none
// Dhrystone on littlecpu and VexRiscv alone, both at rv32imc -- the widest ISA this pair
// shares, since VexRiscv's `compressedGen = true` gives it C but no AtomicPlugin. Held
// against soc/compare/dhry.lds by soc/compare/run_dhrystone.sh, the same way
// soc/compare/dhry_tb.v is.
module dhry_vexc_tb;
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

  int unsigned ours_begin, ours_end, ours_marks;
  int unsigned vex_begin, vex_end, vex_marks;
  int unsigned ours_writes, vex_writes;
  int unsigned ours_verdict, vex_verdict;

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

  always_ff @(posedge clk) cycle <= cycle + 1;

  task automatic report(input string core, input int unsigned marks,
                        input int unsigned begin_cycle, input int unsigned end_cycle,
                        input int unsigned verdict, input int unsigned writes);
    $display("DHRY core=%s marks=%0d cycles=%0d verdict=%0d writes=%0d",
             core, marks, end_cycle - begin_cycle, verdict, writes);
  endtask

  int unsigned differing_vex = 0;
  initial begin
    if (!$value$plusargs("cycles=%d", cycle_limit)) cycle_limit = 2000000;

    while (cycle < cycle_limit && !(ours_verdict != 0 && vex_verdict != 0)) begin
      @(posedge clk);
    end
    @(posedge clk);

    for (i = 0; i < RAM_WORDS; i = i + 1) begin
      if (dut_ours.dmem.ram[i] !== dut_vex.dmem.ram[i]) differing_vex = differing_vex + 1;
    end

    $display("DHRY ran %0d cycles of a %0d cycle limit", cycle, cycle_limit);
    report("littlecpu", ours_marks, ours_begin, ours_end, ours_verdict, ours_writes);
    report("vexriscv", vex_marks, vex_begin, vex_end, vex_verdict, vex_writes);
    $display("DHRY ramdiff core=vexriscv diff=%0d of=%0d words", differing_vex, RAM_WORDS);
    $finish;
  end
endmodule
