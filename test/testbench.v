`timescale 1 ns / 1 ps
//`define RISCV_FORMAL
module testbench(
`ifndef ICARUS
	input clk,
	input reset
`endif
);
  // THE MEMORIES ARE THE REAL ONES (ADR-0054). This bench used to carry its
  // own inline ROM and RAM -- a combinational `rom` array and an `always_ff`
  // over `memory` -- and rtl/imemory.v / rtl/memory.v were reachable from no
  // simulation at all. They are the modules instantiated below now, so the
  // whole `.S` suite and the Sail co-simulation run against the memory system
  // rtl/littlesoc.v synthesises, not against a second description of it.
  //
  // That is not bookkeeping: rtl/imemory.v is SYNCHRONOUS, addressed off the
  // fetch address decode publishes one cycle early, and if that lockstep were
  // wrong every program here would execute the wrong instructions. Before this
  // change nothing in the tree would have noticed.
  //
  // Sized to test/asm/sections.lds (ADR-0008): rom holds >=16K of .text,
  // ram holds >=4K of .data/.rodata/.bss based at RAM_BASE, which matches the
  // ram region's ORIGIN there. RAM_BASE is non-zero so a wild store through an
  // uninitialized/zero pointer lands outside the mapped region instead of
  // silently aliasing real test data. The cxxrtl runner (test/cxxrtl.cc)
  // subtracts RAM_BASE back out of the `--ram` image's word addresses before
  // poking `dmem`'s array via debug_items, and de-interleaves the `--rom` image
  // across `imem`'s two banks. rom grew from 8K/2048 words to 16K/4096 when
  // test/asm/rvc.S landed (ADR-0003/ADR-0021) -- see sections.lds for why.
  //
  // ROM_WORDS HERE IS LARGER THAN THE SHIPPING SoC's, deliberately: simulation
  // has no block RAM to run out of, and rtl/littlesoc.v's 2048 words are what
  // the part's 30 EBRs allow (ADR-0054). rvc.S is the one program in the suite
  // that needs more.
  localparam logic [31:0] RAM_BASE  = 32'h0001_0000;
  localparam int          ROM_WORDS = 4096;
  localparam int          RAM_WORDS = 1024;
  logic [31:0] imem_addr;
  logic [31:0] imem_data;
  // ADR-0003: the second word of the dual-word combinational fetch window
  // (rtl/fetcher.v drives imem_addr2 = imem_addr + 4), read from the same
  // `rom` array at a second, independent index.
  logic [31:0] imem_addr2;
  logic [31:0] imem_data2;
  // ADR-0054: the word address the fetch window will read on the NEXT edge.
  // rtl/imemory.v is synchronous (every memory primitive on the target part
  // is, ADR-0044), so it latches this and answers `imem_addr` for the whole of
  // the cycle that names it.
  logic [31:0] imem_addr_next;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  logic        trap;
 `ifdef RISCV_FORMAL
  logic        rvfi_valid;
  logic [63:0] rvfi_order;
  logic [31:0] rvfi_insn;
  logic        rvfi_trap;
  logic        rvfi_halt;
  logic        rvfi_intr;
  logic [4:0]  rvfi_rs1_addr;
  logic [4:0]  rvfi_rs2_addr;
  logic [31:0] rvfi_rs1_rdata;
  logic [31:0] rvfi_rs2_rdata;
  logic [4:0]  rvfi_rd_addr;
  logic [31:0] rvfi_rd_wdata;
  logic [31:0] rvfi_pc_rdata;
  logic [31:0] rvfi_pc_wdata;
  logic [31:0] rvfi_mem_addr;
  logic [3:0]  rvfi_mem_rmask;
  logic [3:0]  rvfi_mem_wmask;
  logic [31:0] rvfi_mem_rdata;
  logic [31:0] rvfi_mem_wdata;
  // ADR-0006: the monitor's own per-retire error code (0 = no error this
  // cycle). test/cxxrtl.cc reads this by hierarchical debug-item name
  // ("monitor errcode") to turn a mismatch into a distinct process exit
  // code; under iverilog the monitor's own $display diagnostics (which
  // this triggers) are the loud failure.
  logic [15:0] rvfi_monitor_errcode;
 `endif //  `ifdef RISCV_FORMAL
 `ifdef ICARUS
  logic clk = 0;
  logic reset = 1;
  always #5 clk = ~clk;
 `endif
  memory #(.BASE(RAM_BASE), .RAM_WORDS(RAM_WORDS)) dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata)
  );

  // No init files: every consumer of this bench fills the banks itself. The
  // cxxrtl runners poke them through `debug_items` (test/cxxrtl.cc's
  // `load_rom_banks`), and `make waves` writes the program in below.
  imemory #(.ROM_WORDS(ROM_WORDS)) imem (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2)
  );

  littlecpu uut (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    // SPIKE: this bench keeps the Harvard split, so no data access can reach
    // text and the arbiter never steals a fetch.
    .mem_ren(),
    .fetch_stall(1'b0),
    .trap(trap)
   `ifdef RISCV_FORMAL
    , .rvfi_valid(rvfi_valid),
    .rvfi_order(rvfi_order),
    .rvfi_insn(rvfi_insn),
    .rvfi_trap(rvfi_trap),
    .rvfi_halt(rvfi_halt),
    .rvfi_intr(rvfi_intr),
    .rvfi_rs1_addr(rvfi_rs1_addr),
    .rvfi_rs2_addr(rvfi_rs2_addr),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_rd_addr(rvfi_rd_addr),
    .rvfi_rd_wdata(rvfi_rd_wdata),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_pc_wdata(rvfi_pc_wdata),
    .rvfi_mem_addr(rvfi_mem_addr),
    .rvfi_mem_rmask(rvfi_mem_rmask),
    .rvfi_mem_wmask(rvfi_mem_wmask),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .rvfi_mem_wdata(rvfi_mem_wdata)
   `endif
  );
 `ifdef RISCV_FORMAL
  // THE ONE WIRE EVERY RETIRE OBSERVER IN THIS BENCH READS. The monitor below,
  // the spec probe that feeds the observation counters, and the counters
  // themselves all take `rvfi_valid` through here rather than each reaching for
  // the DUT's port separately. That is not decoration: a counter that could
  // stay high while the monitor went blind would be telemetry about itself
  // rather than about the oracle, and the whole point of the counters is that
  // "the monitor observed nothing" cannot happen quietly. Wiring them together
  // makes the two go blind together, structurally, instead of by promise.
  //
  // It is also what makes this repo's liveness probe for the counters a single
  // line. To watch the gate go red on a blind oracle:
  //
  //     assign rvfi_valid_observed = 1'b0;   // instead of = rvfi_valid
  //
  // then `make test`. Every program still reaches `tohost` and still PASSes on
  // its own assertions; the gate reports 52 x MONITOR-SILENT and exits 1. The
  // same edit on the tree before the counters existed reports 52/52 and exits
  // 0, which is the defect this closes. Same idea as deleting the rs2
  // write-through bypass to check that reg_ch0 can still fail: reach for it
  // before believing a green `make test` means the oracle looked.
  logic rvfi_valid_observed;
  assign rvfi_valid_observed = rvfi_valid;

  monitor monitor (
    .clock(clk),
    .reset(reset),
    .rvfi_valid(rvfi_valid_observed),
    .rvfi_order(rvfi_order),
    .rvfi_insn(rvfi_insn),
    .rvfi_trap(rvfi_trap),
    .rvfi_halt(rvfi_halt),
    .rvfi_intr(rvfi_intr),
    .rvfi_rs1_addr(rvfi_rs1_addr),
    .rvfi_rs2_addr(rvfi_rs2_addr),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_rd_addr(rvfi_rd_addr),
    .rvfi_rd_wdata(rvfi_rd_wdata),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_pc_wdata(rvfi_pc_wdata),
    .rvfi_mem_addr(rvfi_mem_addr),
    .rvfi_mem_rmask(rvfi_mem_rmask),
    .rvfi_mem_wmask(rvfi_mem_wmask),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .rvfi_mem_wdata(rvfi_mem_wdata),
    .errcode(rvfi_monitor_errcode)
  );

 `ifdef ICARUS
  // Matches test/cxxrtl.cc's exit 4. errcode is a one-cycle pulse
  // (test/monitor.sim.v resets it every cycle), so this checks every
  // cycle rather than once at the end of the run.
  always @(posedge clk) begin
    if (rvfi_monitor_errcode != 16'b0) begin
      $display("RVFI MONITOR ERROR %0d -- see the diagnostic above", rvfi_monitor_errcode);
      $fatal(1);
    end
  end
 `endif

  // ---------------------------------------------------------------------------
  // DID THE ORACLE EVER LOOK?
  //
  // The monitor above is a per-retire oracle only on the cycles it actually
  // examines. It examines a cycle when `rvfi_valid` is high, and it compares
  // that retire's VALUES only when its spec model recognises the instruction
  // (`ch0_spec_valid`). Neither was measured. `test/cxxrtl.cc` sampled the
  // monitor's `errcode` and counted nothing, so a monitor that never saw a
  // single retire — an under-sensitivity defect of exactly the kind ADR-0037
  // found in the iverilog leg, an `ifdef` that dropped the shadow payload, a
  // `write_cxxrtl` that optimised the instance away — left every program in
  // test/asm reporting PASS off `tohost` alone. `test/EXPECTED_FAIL` is empty,
  // so there was no red entry whose disappearance would have said otherwise
  // either, and ADR-0032 already measured that end-state checking on its own
  // misses real architectural corruption.
  //
  // So observation is a GRADED QUANTITY here, not an inference. These two
  // counters are read by debug-item name in test/cxxrtl.cc, printed on every
  // run, and graded by test/run_tests.sh: zero of either is MONITOR-SILENT, a
  // failing status, and a count below test/OBSERVED_FLOOR is BELOW-FLOOR.
  //
  // WHY A SECOND `monitor_isa_spec` INSTEAD OF LOOKING INSIDE THE MONITOR.
  // `ch0_spec_valid` is internal to the monitor, and test/monitor.v is
  // generated-but-tracked (CLAUDE.md invariant 7) — it may not be hand-edited
  // to export it, and giving test/sanitize_monitor.py a fourth rule for
  // telemetry would perturb the most carefully built content assertions in the
  // repo for no correctness gain. A hierarchical reference is not an option
  // either, and fails in the worst possible way: yosys does not resolve one, it
  // IMPLICITLY DECLARES the name and carries on ("Warning: Identifier
  // `\monitor.ch0_spec_valid' is implicitly declared"), so the counter would
  // read a dangling constant while the build stayed clean. This instead
  // instantiates the same module the monitor instantiates, from the same wires
  // its own instance is wired from, so `probe_spec_valid` is `ch0_spec_valid`
  // by construction: one combinational function, one set of inputs. The one
  // input that decides whether a retire is observed at all is shared through
  // `rvfi_valid_observed` above, so the two cannot go blind independently.
  // KEEP THE REMAINING FIVE PORT CONNECTIONS IDENTICAL TO THE MONITOR'S —
  // rewiring one and not the other is the only way left for this to start
  // counting retires the monitor never checked.
  //
  // A LOW SPEC-CHECKED COUNT IS EXPECTED, AND IS NOT A DEFECT. riscv-formal
  // ships no spec model at all for `ecall`, `ebreak`, `mret` or the `csrr*`
  // family at the pinned SHA (formal/pin.mk), so `spec_valid` is 0 for every
  // one of those retires BY DESIGN and the monitor's whole semantic block is
  // skipped for them — the behaviour M3 added is checked by test/asm/trap.S,
  // test/csr_tb.v and test/decoder_tb.v instead, against assertions this repo
  // wrote. csr.S, minstret.S and trap.S therefore show the widest gap between
  // the two counts, and that gap is the pin's coverage boundary, not a bug.
  // Only ZERO is a failure.
  logic       probe_spec_valid;
  logic       probe_spec_trap;
  logic [4:0] probe_spec_rs1_addr;
  logic [4:0] probe_spec_rs2_addr;
  logic [4:0] probe_spec_rd_addr;
  logic [31:0] probe_spec_rd_wdata;
  logic [31:0] probe_spec_pc_wdata;
  logic [31:0] probe_spec_mem_addr;
  logic [3:0]  probe_spec_mem_rmask;
  logic [3:0]  probe_spec_mem_wmask;
  logic [31:0] probe_spec_mem_wdata;

  monitor_isa_spec spec_probe (
    .rvfi_valid(rvfi_valid_observed),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(probe_spec_valid),
    .spec_trap(probe_spec_trap),
    .spec_rs1_addr(probe_spec_rs1_addr),
    .spec_rs2_addr(probe_spec_rs2_addr),
    .spec_rd_addr(probe_spec_rd_addr),
    .spec_rd_wdata(probe_spec_rd_wdata),
    .spec_pc_wdata(probe_spec_pc_wdata),
    .spec_mem_addr(probe_spec_mem_addr),
    .spec_mem_rmask(probe_spec_mem_rmask),
    .spec_mem_wmask(probe_spec_mem_wmask),
    .spec_mem_wdata(probe_spec_mem_wdata)
  );

  // `(* keep *)` for the same reason `trap_to_zero` below carries one: these
  // are read by debug-item name from test/cxxrtl.cc and nothing inside the
  // design reads them, so without it yosys is free to optimise both away and
  // the runner would report a setup error it cannot distinguish from a real
  // one. 32 bits is far more than the 5000-cycle budget in test/run_tests.sh
  // can ever need, so no wrap is possible.
  //
  // The gating mirrors the monitor's own outer condition exactly
  // (`if (!reset && ch0_rvfi_valid) ... if (ch0_spec_valid)`), so these count
  // the retires it looked at and the subset whose values it compared — not an
  // approximation of them.
  //
  // Plain `always @(posedge clk)`, not `always_ff`, because the `initial`
  // below assigns the same variables; that is the pattern every other
  // bookkeeping block in this file uses and nothing here is synthesised.
  (* keep *) logic [31:0] rvfi_retires;
  (* keep *) logic [31:0] rvfi_spec_retires;
  initial begin
    rvfi_retires = 32'b0;
    rvfi_spec_retires = 32'b0;
  end
  always @(posedge clk) begin
    if (!reset && rvfi_valid_observed) begin
      rvfi_retires <= rvfi_retires + 32'd1;
      if (probe_spec_valid) begin
        rvfi_spec_retires <= rvfi_spec_retires + 32'd1;
      end
    end
  end

 `ifdef ICARUS
  // A counter independent of the retire count below: a stalled pipeline
  // can keep retiring instructions with no stores at all, which a
  // retire-only floor would miss.
  (* keep *) logic [31:0] mem_write_count;
  initial mem_write_count = 32'b0;
  always @(posedge clk) begin
    if (!reset && mem_wstrb != 4'b0000) begin
      mem_write_count <= mem_write_count + 32'd1;
    end
  end

  // Measured on this program at 238a066: 20 writes, 79 retires. These
  // floors sit a margin under both so a few extra stall cycles still
  // pass; reverting `live_rs1`/`live_rs2` to ADR-0037's
  // `live_producer(r)` function form gives 0 writes, 1 retire here.
  localparam int unsigned WRITE_FLOOR  = 15;
  localparam int unsigned RETIRE_FLOOR = 60;

  // Owns dumpfile setup, reset and the whole run: the floor check below
  // needs `rvfi_retires`/`mem_write_count`, declared earlier in this
  // `ifdef RISCV_FORMAL` region, and iverilog will not bind a forward
  // reference to a `logic` declared later in the same module.
  initial begin
    $dumpfile("testbench.vcd");
    $dumpvars(0, testbench);
    repeat (1) @(posedge clk);
    reset <= 0;
    repeat (200) @(posedge clk);
    #1;
    if (mem_write_count < WRITE_FLOOR || rvfi_retires < RETIRE_FLOOR) begin
      $display("FLOOR VIOLATION: writes=%0d (need >= %0d) retires=%0d (need >= %0d) spec-checked=%0d",
                mem_write_count, WRITE_FLOOR, rvfi_retires, RETIRE_FLOOR, rvfi_spec_retires);
      $fatal(1);
    end
    $display("RETIRES %0d SPEC-CHECKED %0d WRITES %0d", rvfi_retires, rvfi_spec_retires, mem_write_count);
    $finish;
  end
 `endif
 `endif
`ifdef ICARUS
  // `make waves`' program: the six-instruction increment loop this file used to
  // carry in an `initial rom[...]` block, written straight into rtl/imemory.v's
  // two banks (ADR-0054 -- word 2i is even, word 2i+1 is odd). It exercises
  // fetch, a load, a store and a backward jump, which is what the waveform is
  // for; the store address is outside ADR-0008's RAM region, so the loads read
  // zero, and that was already true before this change.
  //
  // A HIERARCHICAL REFERENCE, hence `ifdef ICARUS` -- the same guard the clock
  // and reset generation above carries. yosys does not RESOLVE one of these: it
  // implicitly declares the dotted name and carries on with an undriven wire,
  // which CLAUDE.md records as a real hazard. The cxxrtl legs never take this
  // path; they load their ROM through `debug_items`.
  initial begin
    imem.rom_even[0] = 32'h3fc00093; //       li      x1, 1020
    imem.rom_odd [0] = 32'h0000a023; //       sw      x0, 0(x1)
    imem.rom_even[1] = 32'h0000a103; // loop: lw      x2, 0(x1)
    imem.rom_odd [1] = 32'h00110113; //       addi    x2, x2, 1
    imem.rom_even[2] = 32'h0020a023; //       sw      x2, 0(x1)
    imem.rom_odd [2] = 32'hff5ff06f; //       j       loop
  end
`endif

  logic [31:0] past_addr;
  initial past_addr = 32'b0;
  always @(posedge clk) begin
    if (past_addr != imem_addr) begin
      $display("ifetch 0x%08x: 0x%08x", imem_addr, imem_data);
      past_addr <= imem_addr;
    end
  end

  always @(posedge clk) begin
    if (mem_wstrb != 4'b0000) begin
      $display("write  0x%08x: 0x%08x (wstrb=%b)", mem_addr, mem_wdata, mem_wstrb);
    end else begin
      $display("read   0x%08x: 0x%08x", mem_addr, mem_rdata);
    end
    if (trap) begin
      $display("trap!");
    end
  end

  // ADR-0029: `mtvec` resets to 0 and test/asm/sections.lds links `.text`
  // there, so a trap taken before a test installs its handler redirects to
  // `_start` and the program silently RESTARTS. That failure mode looks like a
  // livelock or a timeout, produces no error, and puts the actual cause -- a
  // fault several instructions earlier -- nowhere near the symptom.
  //
  // This is a HARNESS assertion, not an RTL one: a real program is entitled to
  // put a handler at 0, and it would need revisiting if sections.lds ever
  // moved `.text`. It also needs no hierarchical reference into the CSR file.
  // `trap` is ADR-0028's one-cycle "trap entry committed" pulse, and decode
  // registers `pc <= mtvec` on that same edge, so the fetch address one cycle
  // later IS mtvec (rtl/fetcher.v drives imem_addr straight off pc).
  //
  // `(* keep *)` because test/cxxrtl.cc reads `trap_to_zero` by debug-item
  // name to turn this into a distinct process exit code; without it yosys is
  // free to optimise away a register nothing in the design reads.
  //
  // Plain `always @(posedge clk)`, not `always_ff`, for the same reason every
  // other diagnostic block in this file is: iverilog warns that a system task
  // cannot be synthesised inside an `always_ff` process, and CLAUDE.md says
  // warnings are errors. Nothing in this bench is synthesised.
  logic trap_taken_d;
  (* keep *) logic trap_to_zero;
  initial begin
    trap_taken_d = 1'b0;
    trap_to_zero = 1'b0;
  end
  always @(posedge clk) begin
    trap_taken_d <= !reset && trap;
    if (trap_taken_d && imem_addr == 32'b0) begin
      trap_to_zero <= 1'b1;
      $display("TRAP TO ZERO: a trap was taken while mtvec == 0 (ADR-0029).");
      $display("No handler was installed, so the program has restarted at _start.");
     `ifdef ICARUS
      $fatal(1);
     `endif
    end
  end

  // ADR-0009: for every store, mem_wstrb is high for
  // exactly one cycle. Direct regression for the divide-replay defect, where
  // the accessor kept re-issuing the same store every cycle the executor sat
  // busy in `divide` because nothing upstream defaulted back to zero in
  // between. Two DIFFERENT consecutive real stores legitimately produce two
  // adjacent high cycles, so this checks for the same request repeating
  // (identical address, data, and strobe on back-to-back high cycles), not
  // merely back-to-back nonzero cycles.
  //
  // $fatal is Icarus-only below: yosys's read_verilog (the write_cxxrtl leg
  // this file also feeds, per the Makefile) doesn't implement it at all, so
  // this stays inside `ifdef ICARUS` the same way the clock/reset generation
  // above does. The $display fires unconditionally, so the violation is
  // still visible under cxxrtl if it were ever encountered there.
  logic [31:0] prev_wstrb_mem_addr, prev_wstrb_mem_wdata;
  logic [3:0]  prev_mem_wstrb;
  initial prev_mem_wstrb = 4'b0000;
  always @(posedge clk) begin
    if (mem_wstrb != 4'b0000 && prev_mem_wstrb != 4'b0000 &&
        mem_addr == prev_wstrb_mem_addr && mem_wdata == prev_wstrb_mem_wdata &&
        mem_wstrb == prev_mem_wstrb) begin
      $display("ASSERTION FAILED: mem_wstrb held high for >1 cycle on the same store (addr=0x%08x)",
                mem_addr);
     `ifdef ICARUS
      $fatal(1);
     `endif
    end
    prev_wstrb_mem_addr <= mem_addr;
    prev_wstrb_mem_wdata <= mem_wdata;
    prev_mem_wstrb <= mem_wstrb;
  end
endmodule
