`timescale 1 ns / 1 ps
`default_nettype none
// Runs both harnesses on the same image and requires the two cores to write the
// SAME sequence of values to the same address.
//
// The placement flow's own gate, soc/compare/placed_vs_synth.py, says the core
// is still in the netlist. It does not say the netlist WORKS, and the part most
// likely not to is soc/compare/bench_vexriscv.v's bus adapter: a store lane got
// wrong, or a response presented a cycle early, leaves a core spinning in a
// fault loop with every logic cell present and every timing number describing a
// design that does not run. This is the check that could have failed.
//
// Word stores to soc/compare/bench.S's publication address are recorded from
// each harness's data bus and compared element by element. The three cores
// retire at different rates, so what is compared is the sequence, not the
// timing.
//
// soc/compare/bench_hazard3.v has no separate data bus to watch -- one AHB5
// port carries fetch and load/store both -- so its publications are read off
// rtl/memory.v's own ports the way soc/compare/bench_littlecpu.v's are: a word
// store is `mem_wstrb == 4'b1111` at the harness's `haddr`/`hwdata`, which is
// the same signal name coincidence as `mem_addr`/`mem_wdata` below rather than
// a shared bus between two different cores.
module bench_tb;
  localparam int      CYCLES  = 120000;
  localparam int      WANT    = 6;
  localparam bit [31:0] PUBLISH = 32'h0001_0208;  // ram base + 520

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic ours_led0_n, ours_led1_n, vex_led0_n, vex_led1_n, haz_led0_n, haz_led1_n;

  bench_littlecpu dut_ours (
    .clk(clk), .led0_n(ours_led0_n), .led1_n(ours_led1_n)
  );
  bench_vexriscv dut_vex (
    .clk(clk), .led0_n(vex_led0_n), .led1_n(vex_led1_n)
  );
  bench_hazard3 dut_haz (
    .clk(clk), .led0_n(haz_led0_n), .led1_n(haz_led1_n)
  );

  logic [31:0] ours_seen[0:63];
  logic [31:0] vex_seen[0:63];
  logic [31:0] haz_seen[0:63];
  int          ours_n = 0, vex_n = 0, haz_n = 0;

  always_ff @(posedge clk) begin
    if (dut_ours.mem_wstrb == 4'b1111 && dut_ours.mem_addr == PUBLISH
        && ours_n < 64) begin
      ours_seen[ours_n] <= dut_ours.mem_wdata;
      ours_n            <= ours_n + 1;
    end
    if (dut_vex.mem_wstrb == 4'b1111 && dut_vex.dbus_cmd_address == PUBLISH
        && vex_n < 64) begin
      vex_seen[vex_n] <= dut_vex.dbus_cmd_data;
      vex_n           <= vex_n + 1;
    end
    if (dut_haz.mem_wstrb == 4'b1111 && dut_haz.haddr == PUBLISH
        && haz_n < 64) begin
      haz_seen[haz_n] <= dut_haz.hwdata;
      haz_n           <= haz_n + 1;
    end
  end

  int errors = 0;
  int i;
  initial begin
    repeat (CYCLES) @(posedge clk);

    if (ours_n < WANT) begin
      $display("FAIL: littlecpu published %0d values in %0d cycles, wanted %0d.",
               ours_n, CYCLES, WANT);
      $display("      The core is not running soc/compare/bench.S.");
      errors = errors + 1;
    end
    if (vex_n < WANT) begin
      $display("FAIL: VexRiscv published %0d values in %0d cycles, wanted %0d.",
               vex_n, CYCLES, WANT);
      $display("      The bus adapter in soc/compare/bench_vexriscv.v is wrong,");
      $display("      or the core is stuck.");
      errors = errors + 1;
    end
    if (haz_n < WANT) begin
      $display("FAIL: Hazard3 published %0d values in %0d cycles, wanted %0d.",
               haz_n, CYCLES, WANT);
      $display("      The bus adapter in soc/compare/bench_hazard3.v is wrong,");
      $display("      or the core is stuck.");
      errors = errors + 1;
    end

    for (i = 0; i < WANT; i = i + 1) begin
      if (i < ours_n && i < vex_n && ours_seen[i] !== vex_seen[i]) begin
        $display("FAIL: publication %0d differs: littlecpu %08x, VexRiscv %08x.",
                 i, ours_seen[i], vex_seen[i]);
        $display("      One harness is not presenting the same machine to its core.");
        errors = errors + 1;
      end
      if (i < ours_n && i < haz_n && ours_seen[i] !== haz_seen[i]) begin
        $display("FAIL: publication %0d differs: littlecpu %08x, Hazard3 %08x.",
                 i, ours_seen[i], haz_seen[i]);
        $display("      One harness is not presenting the same machine to its core.");
        errors = errors + 1;
      end
    end

    // Agreement on a constant is agreement stuck cores could reach. The
    // program's mix was an `or` once, which saturates s0 to all-ones and made
    // every publication identical; this is what caught it.
    if (ours_n >= WANT && ours_seen[0] === ours_seen[WANT-1]) begin
      $display("FAIL: all %0d publications are %08x. The program's value does not",
               WANT, ours_seen[0]);
      $display("      change, so this comparison cannot tell a running core from");
      $display("      a stuck one.");
      errors = errors + 1;
    end

    if (errors == 0) begin
      $display("bench_tb: %0d published values, littlecpu, VexRiscv and Hazard3 agree; first %08x, last matched %08x",
               WANT, ours_seen[0], ours_seen[WANT-1]);
      $display("PASS");
    end else begin
      $display("bench_tb: %0d error(s)", errors);
      $fatal(1, "bench_tb failed");
    end
    $finish;
  end
endmodule
