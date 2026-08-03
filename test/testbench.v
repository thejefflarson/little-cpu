`timescale 1 ns / 1 ps
//`define RISCV_FORMAL
module testbench(
`ifndef ICARUS
	input clk,
	input reset
`endif
);
  // Sized to test/asm/sections.lds. RAM_BASE is non-zero so a wild store
  // through a zero pointer lands outside the mapped region rather than on real
  // test data, and ROM_WORDS exceeds the shipping SoC's 2048 because simulation
  // has no block RAM to run out of and rvc.S needs more than 30 EBRs allow.
  localparam logic [31:0] RAM_BASE  = 32'h0001_0000;
  localparam int          ROM_WORDS = 4096;
  localparam int          RAM_WORDS = 1024;
  logic [31:0] imem_addr;
  logic [31:0] imem_data;
  logic [31:0] imem_addr2;
  logic [31:0] imem_data2;
  logic [31:0] imem_addr_next;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren;
  logic [31:0] mem_rdata;
  logic        fetch_stall;
  // Both memories answer zero outside their own range, so the buses join with
  // an OR. Same wiring as rtl/littlesoc.v, so the legs cannot drift apart.
  logic [31:0] imem_mem_rdata, dmem_mem_rdata;
  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata;
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
  // test/cxxrtl.cc reads this by debug-item name ("monitor errcode").
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
    .mem_rdata(dmem_mem_rdata)
  );

  // No init files: the cxxrtl runners fill the banks through `debug_items`
  // (test/cxxrtl.cc's `load_rom_banks`); `make waves` writes its program below.
  imemory #(.ROM_WORDS(ROM_WORDS)) imem (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(imem_mem_rdata),
    .fetch_stall(fetch_stall)
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
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
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
  // The monitor, the spec probe and the retire counters all read `rvfi_valid`
  // through this one wire. Keep it that way: a counter that could stay high
  // while the monitor went blind would be telemetry about itself, and this
  // makes the two go blind together by construction.
  //
  // It is also the liveness probe. Replace the assign with `1'b0` and `make
  // test` reports every program MONITOR-SILENT; before the counters existed the
  // same edit passed the whole suite.
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
  // errcode is a one-cycle pulse (test/monitor.sim.v clears it every cycle), so
  // it must be checked every cycle -- a read at the end of the run sees
  // nothing. Matches test/cxxrtl.cc's exit 4.
  always @(posedge clk) begin
    if (rvfi_monitor_errcode != 16'b0) begin
      $display("RVFI MONITOR ERROR %0d -- see the diagnostic above", rvfi_monitor_errcode);
      $fatal(1);
    end
  end
 `endif

  // A second `monitor_isa_spec` rather than a read of `ch0_spec_valid` inside
  // the monitor, because test/monitor.v may not be hand-edited to export it
  // (CLAUDE.md invariant 7) and yosys does not resolve a hierarchical
  // reference -- it implicitly declares the name, so the counter would read a
  // dangling constant while the build stayed clean. Keep these port
  // connections identical to the monitor's above: rewiring one and not the
  // other is what would make this count retires the monitor never checked.
  //
  // A low spec-checked count is expected, since riscv-formal ships no spec
  // model for `ecall`, `ebreak`, `mret` or `csrr*` at the pin. Only zero fails.
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

  // `(* keep *)` because nothing in the design reads these and yosys would
  // otherwise optimise them away. The gating mirrors the monitor's own outer
  // condition, so they count what it looked at rather than an approximation.
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
  // Independent of the retire count: a pipeline can keep retiring with no
  // stores at all, which a retire-only floor would miss.
  (* keep *) logic [31:0] mem_write_count;
  initial mem_write_count = 32'b0;
  always @(posedge clk) begin
    if (!reset && mem_wstrb != 4'b0000) begin
      mem_write_count <= mem_write_count + 32'd1;
    end
  end

  // The baked-in program does 20 writes and 79 retires, so these sit a margin
  // under both. They are what makes a deadlocked pipeline loud: reverting
  // `live_rs1`/`live_rs2` to ADR-0037's `live_producer(r)` function form gives
  // 0 writes and 1 retire.
  localparam int unsigned WRITE_FLOOR  = 15;
  localparam int unsigned RETIRE_FLOOR = 60;

  // Owns reset and the whole run, because the floor check needs the counters
  // above and iverilog will not bind a forward reference.
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
  // `make waves`' program, written into rtl/imemory.v's two banks (word 2i is
  // even, 2i+1 odd). It counts in RAM because an address in the text range
  // would take the banks' write port and steal the fetch behind it.
  //
  // `ifdef ICARUS` because it is a hierarchical reference, which yosys does not
  // resolve (see the spec probe above).
  initial begin
    imem.rom_even[0] = 32'h000100b7; //       lui     x1, 0x10
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

  // `mtvec` resets to 0 and sections.lds links `.text` there, so a trap taken
  // before a test installs its handler restarts the program silently -- it
  // looks like a livelock, with the real cause several instructions earlier
  // (ADR-0029). A harness assertion, not an RTL one: a real program may
  // legitimately put a handler at 0.
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

  // Two different consecutive stores legitimately produce two adjacent high
  // cycles, so this looks for the same request repeating rather than for
  // back-to-back nonzero ones. `$fatal` is Icarus-only because yosys's
  // read_verilog does not implement it.
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
