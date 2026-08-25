`timescale 1 ns / 1 ps
module testbench(
`ifndef ICARUS
	input clk,
	input reset
`endif
);
  // ROM_WORDS is the ONLY thing this harness sizes differently from
  // rtl/littlesoc.v, and it is deliberate: simulation has no block RAM to run
  // out of, and rvc.S pads past what the part's 30 EBRs allow. The data RAM and
  // the timer take rtl/memory.v's and rtl/timer.v's own parameter defaults --
  // the same ones rtl/littlesoc.v takes -- so neither file states the map and
  // neither can drift from the other. This harness once modelled a RAM sixteen
  // times smaller than the SoC's, and every program fit, so nothing said so.
  // test/memmap_test.sh is what keeps an override from reappearing here.
  localparam int ROM_WORDS = 4096;
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
  logic        imem_fault;
  logic        mem_reservable;
  logic        atomic_supported;
  logic        mem_lock;
  logic        bus_request;
  logic [31:0] atomic_addr;
  logic        irq_timer;
  // All four memories answer zero outside their own range, so the buses join
  // with an OR, exactly as rtl/littlesoc.v joins them.
  logic [31:0] imem_mem_rdata, dmem_mem_rdata, timer_mem_rdata, uart_mem_rdata;
  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata | timer_mem_rdata | uart_mem_rdata;
  // Left unread on purpose: this harness grades programs through `tohost`, and
  // the serial line itself is decoded bit by bit in test/uart_tb.v instead.
  logic        uart_tx;
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
  `ifdef RISCV_FORMAL_MEM_FAULT
  logic        rvfi_mem_fault;
  logic [3:0]  rvfi_mem_fault_rmask;
  logic [3:0]  rvfi_mem_fault_wmask;
  `endif
  // test/cxxrtl.cc reads this by debug-item name ("monitor errcode").
  logic [15:0] rvfi_monitor_errcode;
 `endif //  `ifdef RISCV_FORMAL
 `ifdef ICARUS
  logic clk = 0;
  logic reset = 1;
  always #5 clk = ~clk;
 `endif
  memory dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dmem_mem_rdata),
    .reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported)
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
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault)
  );

  timer mtimer (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(timer_mem_rdata),
    .mtip(irq_timer)
  );

  uart tty (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(uart_mem_rdata),
    .tx(uart_tx)
  );

  // The same localparam the `imemory` above is given, so the core's copy of the
  // map describes THIS machine's text window rather than the part's. It is the
  // one parameter of the map an integrator states, for the same reason it is the
  // one memory sized here.
  littlecpu #(.LS_TEXT_WORDS(ROM_WORDS)) uut (
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
    .imem_fault(imem_fault),
    .mem_reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    // One bus initiator in this machine, so the bus is never withheld and nothing
    // but the core writes memory. `mem_lock` has no arbiter to tell.
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    .bus_request(bus_request),
    .irq_timer(irq_timer),
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
    .rvfi_mem_wdata(rvfi_mem_wdata),
    `ifdef RISCV_FORMAL_MEM_FAULT
    .rvfi_mem_fault(rvfi_mem_fault),
    .rvfi_mem_fault_rmask(rvfi_mem_fault_rmask),
    .rvfi_mem_fault_wmask(rvfi_mem_fault_wmask)
    `endif
   `endif
  );
 `ifdef RISCV_FORMAL
  // The monitor, the spec probe and the counters all read rvfi_valid through
  // this one wire. Keep it that way. A counter that could stay high while the
  // monitor stopped looking would only be counting itself.
  //
  // It is also how to test the counters: set this to 1'b0 and every program
  // should report MONITOR-SILENT. Do the same on a tree without the counters
  // and the whole suite still passes, because each program reaches tohost on
  // its own assertions with nothing checking a single instruction.
  //
  // What it is NOT is where a refused access is excused. The spec model has no
  // memory map, so a retire this platform refused disagrees with it -- but
  // hiding the retire here leaves a hole in `rvfi_order` that the monitor's
  // reorder buffer reads as a lost instruction, and every retire after it is
  // graded against the wrong shadow. `rvfi_mem_fault` goes to the monitor
  // instead and test/sanitize_monitor.py's rules 4 to 6 are what read it.
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
    .rvfi_mem_fault(rvfi_mem_fault),
    .errcode(rvfi_monitor_errcode)
  );

 `ifdef ICARUS
  // The error code is high for one cycle only; test/monitor.sim.v clears it
  // every cycle. Check it every cycle -- reading it once at the end misses it.
  // Same failure as test/cxxrtl.cc's exit 4.
  always @(posedge clk) begin
    if (rvfi_monitor_errcode != 16'b0) begin
      $display("RVFI MONITOR ERROR %0d -- see the diagnostic above", rvfi_monitor_errcode);
      $fatal(1);
    end
  end
 `endif

  // A second monitor_isa_spec, rather than reading ch0_spec_valid inside the
  // monitor. Two reasons. test/monitor.v is generated and gets regenerated, so
  // an edit exporting that signal would be lost. And yosys will not reach into
  // the instance: it makes a new undriven wire with that name, so the counter
  // would read a constant and the build would look fine.
  //
  // Keep these connections the same as the monitor's above. Rewire one and not
  // the other and this counts instructions the monitor never checked.
  //
  // A low spec-checked count is normal. riscv-formal has no model for ecall,
  // ebreak, mret or csrr*, so those retire unchecked. Only zero is wrong.
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

  // Nothing in the design reads these, so without (* keep *) yosys removes
  // them. The gating matches the monitor's own, so they count what it looked at.
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

  // The program below does 20 writes and 79 retires. These sit under both, so a
  // few extra stall cycles still pass but a pipeline that has stopped making
  // progress fails. Write rtl/decoder.v's live_rs1 and live_rs2 as a function
  // call instead of continuous assigns and this drops to 0 writes and 1 retire:
  // iverilog builds the sensitivity list from the call arguments, so the assign
  // never runs again when the signals inside the function change.
  localparam int unsigned WRITE_FLOOR  = 15;
  localparam int unsigned RETIRE_FLOOR = 60;

  // Drives reset and ends the run. It lives here because the check above needs
  // the counters, and iverilog will not look forward for them.
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
  //
  // The banks are zeroed first and that is not tidiness. Fetch reads two
  // adjacent words every cycle and decode reads the second one's register
  // fields, so the word after the LAST instruction of the program is read on
  // every pass through it. A bitstream defines every word of a block RAM; an
  // array poked at six addresses does not, and one X there reaches the register
  // file's address port and turns the whole pipeline X. cxxrtl is two-state and
  // cannot see it; this leg is the only place it shows. Do not add a program
  // here without the loop.
  initial begin
    for (int i = 0; i < ROM_WORDS / 2; i++) begin
      imem.rom_even[i] = 32'b0;
      imem.rom_odd [i] = 32'b0;
    end
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

  // mtvec resets to 0 and sections.lds puts .text there. So a trap before a
  // test installs its handler jumps to _start and the program starts over. It
  // looks like a hang, and the real fault was several instructions back.
  //
  // This belongs in the harness, not the RTL: a real program is allowed to put
  // a handler at address 0.
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

  // Two different stores back to back are fine, so this looks for the same
  // request repeating, not for two busy cycles in a row. $fatal is Icarus-only
  // because yosys does not implement it.
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
