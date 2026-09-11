`timescale 1 ns / 1 ps
`default_nettype none
// Dhrystone on littlecpu and Hazard3 alone, both at RV32IMA -- the widest ISA this pair
// shares, since Hazard3's iCE40 build sets EXTENSION_A=1 but EXTENSION_C=0. Held against
// soc/compare/dhry.lds by soc/compare/run_dhrystone.sh, the same way soc/compare/dhry_tb.v
// is.
module dhry_haza_tb;
  localparam int ROM_WORDS = 2048;
  localparam int RAM_WORDS = 4096;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic ours_led0_n, ours_led1_n, haz_led0_n, haz_led1_n;

  bench_littlecpu #(
    .ROM_WORDS(ROM_WORDS),
    .RAM_WORDS(RAM_WORDS),
    .INIT_EVEN("soc/compare/dhry_even.hex"),
    .INIT_ODD("soc/compare/dhry_odd.hex")
  ) dut_ours (
    .clk(clk), .led0_n(ours_led0_n), .led1_n(ours_led1_n)
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

  int unsigned ours_begin, ours_end, ours_marks;
  int unsigned haz_begin, haz_end, haz_marks;
  int unsigned ours_writes, haz_writes;
  int unsigned ours_verdict, haz_verdict;
  int unsigned haz_wait_cycles = 0;

  dhry_monitor mon_ours (
    .clk(clk), .cycle(cycle),
    .mem_addr(dut_ours.mem_addr), .mem_wdata(dut_ours.mem_wdata),
    .mem_wstrb(dut_ours.mem_wstrb),
    .marks(ours_marks), .begin_cycle(ours_begin), .end_cycle(ours_end),
    .writes(ours_writes), .verdict(ours_verdict)
  );
  dhry_monitor mon_haz (
    .clk(clk), .cycle(cycle),
    .mem_addr(dut_haz.dmem_addr_mux), .mem_wdata(dut_haz.d_hwdata),
    .mem_wstrb(dut_haz.dmem_wstrb_mux),
    .marks(haz_marks), .begin_cycle(haz_begin), .end_cycle(haz_end),
    .writes(haz_writes), .verdict(haz_verdict)
  );

  int i;
  initial begin
    $readmemh("soc/compare/dhry_ram.hex", dut_ours.dmem.ram);
    $readmemh("soc/compare/dhry_ram.hex", dut_haz.dmem.ram);
    for (i = 0; i < 32; i = i + 1) begin
      dut_ours.riscv.regfile.regs_a[i] = 32'b0;
      dut_ours.riscv.regfile.regs_b[i] = 32'b0;
      dut_haz.core.core.regs.real_dualport_noreset.mem[i] = 32'b0;
    end
  end

  always_ff @(posedge clk) begin
    cycle <= cycle + 1;
    if (haz_marks == 1 && dut_haz.wr_pending_q) haz_wait_cycles <= haz_wait_cycles + 1;
  end

  task automatic report(input string core, input int unsigned marks,
                        input int unsigned begin_cycle, input int unsigned end_cycle,
                        input int unsigned verdict, input int unsigned writes);
    $display("DHRY core=%s marks=%0d cycles=%0d verdict=%0d writes=%0d",
             core, marks, end_cycle - begin_cycle, verdict, writes);
  endtask

  int unsigned differing_haz = 0;
  initial begin
    if (!$value$plusargs("cycles=%d", cycle_limit)) cycle_limit = 2000000;

    while (cycle < cycle_limit && !(ours_verdict != 0 && haz_verdict != 0)) begin
      @(posedge clk);
    end
    @(posedge clk);

    for (i = 0; i < RAM_WORDS; i = i + 1) begin
      if (dut_ours.dmem.ram[i] !== dut_haz.dmem.ram[i]) differing_haz = differing_haz + 1;
    end

    $display("DHRY ran %0d cycles of a %0d cycle limit", cycle, cycle_limit);
    report("littlecpu", ours_marks, ours_begin, ours_end, ours_verdict, ours_writes);
    report("hazard3", haz_marks, haz_begin, haz_end, haz_verdict, haz_writes);
    $display("DHRY core=hazard3 wait_cycles=%0d", haz_wait_cycles);
    $display("DHRY ramdiff core=hazard3 diff=%0d of=%0d words", differing_haz, RAM_WORDS);
    $finish;
  end
endmodule
