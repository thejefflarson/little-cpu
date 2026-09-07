`timescale 1 ns / 1 ps
module testbench(
`ifndef ICARUS
	input clk,
	input reset
`endif
);
  // ROM_WORDS is the ONLY thing this harness sizes differently from rtl/littlesoc.v, and
  // it is deliberate: simulation has no block RAM to run out of, and rvc.S pads past what
  // the part's 30 EBRs allow.
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
  // All five memories answer zero outside their own range, so the buses join with an OR,
  // exactly as rtl/littlesoc.v joins them.
  logic [31:0] imem_mem_rdata, dmem_mem_rdata, timer_mem_rdata, uart_mem_rdata;
  logic [31:0] flash_mem_rdata;
  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata | timer_mem_rdata | uart_mem_rdata
                   | flash_mem_rdata;
  // Left unread on purpose: this harness grades programs through `tohost`, and the serial
  // line itself is decoded bit by bit in test/uart_tb.v instead.
  logic        uart_tx;
  // The flash's four wires, with test/spiflash_model.v on the other end of them.
  logic        spi_sck, spi_mosi, spi_miso, spi_cs_n;
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

  spiflash flash (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(flash_mem_rdata),
    .sck(spi_sck),
    .mosi(spi_mosi),
    .miso(spi_miso),
    .cs_n(spi_cs_n)
  );

  spiflash_model flash_part (
    .clk(clk),
    .sck(spi_sck),
    .cs_n(spi_cs_n),
    .mosi(spi_mosi),
    .miso(spi_miso)
  );

  // The same localparam the `imemory` above is given, so the core's copy of the map
  // describes THIS machine's text window rather than the part's.
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
    // One bus initiator in this machine, so the bus is never withheld and nothing but the
    // core writes memory.
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
  // The monitor, the spec probe and the counters all read rvfi_valid through this one
  // wire.
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
  // The error code is high for one cycle only; test/monitor.sim.v clears it every cycle.
  always @(posedge clk) begin
    if (rvfi_monitor_errcode != 16'b0) begin
      $display("RVFI MONITOR ERROR %0d -- see the diagnostic above", rvfi_monitor_errcode);
      $fatal(1);
    end
  end
 `endif

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
  (* keep *) logic [31:0] mem_write_count;
  initial mem_write_count = 32'b0;
  always @(posedge clk) begin
    if (!reset && mem_wstrb != 4'b0000) begin
      mem_write_count <= mem_write_count + 32'd1;
    end
  end

  localparam int unsigned WRITE_FLOOR  = 15;
  localparam int unsigned RETIRE_FLOOR = 60;

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
      $display("TRAP TO ZERO: a trap was taken while mtvec == 0.");
      $display("No handler was installed, so the program has restarted at _start.");
     `ifdef ICARUS
      $fatal(1);
     `endif
    end
  end

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
