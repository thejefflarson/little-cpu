`timescale 1 ns / 1 ps
// nano's sim harness top level: one flat memory on the picorv32 bus, the same per-retire
// RVFI monitor littlecpu's two sim legs read, and the cross-core Dhrystone/CoreMark marker.
module nano_testbench(
`ifndef ICARUS
  input clk,
  input reset
`endif
);
`ifndef NANO_WAIT_STATES
`define NANO_WAIT_STATES 0
`endif
`ifndef NANO_QSPI_PREFETCH_DEPTH
`define NANO_QSPI_PREFETCH_DEPTH 0
`endif
`ifndef NANO_QSPI_LOOP_KIND
`define NANO_QSPI_LOOP_KIND 0
`endif
`ifndef NANO_QSPI_LOOP_WINDOW
`define NANO_QSPI_LOOP_WINDOW 0
`endif
`ifndef NANO_QSPI_PREAMBLE_CYCLES
`define NANO_QSPI_PREAMBLE_CYCLES 24
`endif
`ifndef NANO_QSPI_PSRAM_LOAD_CYCLES
`define NANO_QSPI_PSRAM_LOAD_CYCLES 44
`endif
`ifndef NANO_QSPI_PSRAM_STORE_CYCLES
`define NANO_QSPI_PSRAM_STORE_CYCLES 33
`endif

  localparam int MEM_WORDS = 20480;
  localparam int unsigned TOHOST_INDEX = 32'h0001_0000 / 4;

  (* keep *) logic mem_valid;
  logic        mem_instr;
  logic        mem_ready;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [ 3:0] mem_wstrb;
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
  logic [15:0] rvfi_monitor_errcode;
`endif

`ifdef ICARUS
  logic clk = 0;
  logic reset = 1;
  always #5 clk = ~clk;
`endif

`ifdef NANO_QSPI_TIMING
  (* keep *) logic reason_parcel_wait;
  (* keep *) logic reason_redirect_preamble;
  (* keep *) logic reason_loop_hit;
  (* keep *) logic reason_handshake;
  (* keep *) logic reason_psram_wait;
  (* keep *) logic stream_fault;
  // Echoes this build's own parameters so nano_cxxrtl.cc can print a MODEL line from the
  // binary itself, rather than trusting the script that invoked its build.
  (* keep *) int unsigned model_prefetch_depth;
  (* keep *) int unsigned model_loop_kind;
  (* keep *) int unsigned model_loop_window;
  (* keep *) int unsigned model_preamble_cycles;
  (* keep *) int unsigned model_parcel_cycles;
  (* keep *) int unsigned model_psram_load_cycles;
  (* keep *) int unsigned model_psram_store_cycles;
  assign model_prefetch_depth = `NANO_QSPI_PREFETCH_DEPTH;
  assign model_loop_kind = `NANO_QSPI_LOOP_KIND;
  assign model_loop_window = `NANO_QSPI_LOOP_WINDOW;
  assign model_preamble_cycles = `NANO_QSPI_PREAMBLE_CYCLES;
  assign model_parcel_cycles = 8;
  assign model_psram_load_cycles = `NANO_QSPI_PSRAM_LOAD_CYCLES;
  assign model_psram_store_cycles = `NANO_QSPI_PSRAM_STORE_CYCLES;
  nano_qspi_memory #(
    .WORDS(MEM_WORDS),
    .PREFETCH_DEPTH(`NANO_QSPI_PREFETCH_DEPTH),
    .LOOP_KIND(`NANO_QSPI_LOOP_KIND),
    .LOOP_WINDOW(`NANO_QSPI_LOOP_WINDOW),
    .PREAMBLE_CYCLES(`NANO_QSPI_PREAMBLE_CYCLES),
    .PSRAM_LOAD_CYCLES(`NANO_QSPI_PSRAM_LOAD_CYCLES),
    .PSRAM_STORE_CYCLES(`NANO_QSPI_PSRAM_STORE_CYCLES)
  ) mem (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ready(mem_ready),
    .mem_rdata(mem_rdata),
    .reason_parcel_wait(reason_parcel_wait),
    .reason_redirect_preamble(reason_redirect_preamble),
    .reason_loop_hit(reason_loop_hit),
    .reason_handshake(reason_handshake),
    .reason_psram_wait(reason_psram_wait),
    .stream_fault(stream_fault)
  );
`elsif NANO_QSPI_PINS
  logic sck, flash_cs_n, psram_cs_n, spare_cs_n;
  logic [3:0] sio_ctrl_out, sio_flash_out, sio_psram_out;
  logic       sio_ctrl_oe, sio_flash_oe, sio_psram_oe;
  logic [3:0] sio_bus;
  // The shared pin: whichever device's own chip select grants it drives, else the controller.
  assign sio_bus = sio_flash_oe ? sio_flash_out : (sio_psram_oe ? sio_psram_out : sio_ctrl_out);

  nano_qspi_ctrl ctrl (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .sck(sck),
    .flash_cs_n(flash_cs_n),
    .psram_cs_n(psram_cs_n),
    .spare_cs_n(spare_cs_n),
    .sio_out(sio_ctrl_out),
    .sio_oe(sio_ctrl_oe),
    .sio_in(sio_bus)
  );

  nano_qspi_flash_model #(.WORDS(MEM_WORDS)) flash (
    .clk(clk),
    .reset(reset),
    .sck(sck),
    .cs_n(flash_cs_n),
    .sio_in(sio_bus),
    .sio_out(sio_flash_out),
    .sio_oe(sio_flash_oe)
  );

  nano_qspi_psram_model #(.WORDS(MEM_WORDS)) psram (
    .clk(clk),
    .reset(reset),
    .sck(sck),
    .cs_n(psram_cs_n),
    .sio_in(sio_bus),
    .sio_out(sio_psram_out),
    .sio_oe(sio_psram_oe)
  );
`else
  nano_memory #(.WORDS(MEM_WORDS), .WAIT_STATES(`NANO_WAIT_STATES)) mem (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ready(mem_ready),
    .mem_rdata(mem_rdata)
  );
`endif

  riscv uut (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
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
  logic rvfi_valid_observed;
  assign rvfi_valid_observed = rvfi_valid;

  // nano's bus carries no fault line (CLAUDE.md: no CSR, no memory map faults on this
  // bus), so the monitor's mem_fault gate -- built for a refused access the spec model
  // cannot see -- is tied low rather than never wired.
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
    .rvfi_mem_fault(1'b0),
    .errcode(rvfi_monitor_errcode)
  );

  (* keep *) logic [31:0] rvfi_retires;
  initial rvfi_retires = 32'b0;
  always @(posedge clk) begin
    if (!reset && rvfi_valid_observed) begin
      rvfi_retires <= rvfi_retires + 32'd1;
    end
  end
`endif

  // The cross-core harness's marker mechanism (soc/compare/dhry_monitor.v): it watches
  // this bus for two magic addresses and needs no mcycle on the DUT side, which is what
  // makes it reusable unmodified for a core with no CSR at all.
  int unsigned cycle;
  initial cycle = 0;
  always @(posedge clk) cycle <= cycle + 1;

  (* keep *) int unsigned bench_marks;
  (* keep *) int unsigned bench_begin_cycle;
  (* keep *) int unsigned bench_end_cycle;
  (* keep *) int unsigned bench_writes;
  (* keep *) int unsigned bench_verdict;

  dhry_monitor bench_mon (
    .clk(clk),
    .cycle(cycle),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_valid && mem_ready ? mem_wstrb : 4'b0000),
    .marks(bench_marks),
    .begin_cycle(bench_begin_cycle),
    .end_cycle(bench_end_cycle),
    .writes(bench_writes),
    .verdict(bench_verdict)
  );

  logic trap_d;
  (* keep *) logic trap_latched;
  initial begin
    trap_d = 1'b0;
    trap_latched = 1'b0;
  end
  always @(posedge clk) begin
    trap_d <= !reset && trap;
    if (trap_d) trap_latched <= 1'b1;
  end
`ifdef ICARUS
  string icarus_rom_path, icarus_ram_path;
  int unsigned icarus_cycle_limit, icarus_cycle;

  task automatic finish_run(string msg = "");
    if (msg.len() > 0) $display("%s", msg);
    $display("RETIRES %0d", rvfi_retires);
    $finish;
  endtask

  initial begin
`ifdef NANO_QSPI_PINS
    // Two physical devices, not one flat array, matching the ROM/RAM split nano.lds states.
    for (int unsigned i = 0; i < MEM_WORDS; i = i + 1) begin
      flash.mem[i] = 32'b0;
      psram.mem[i] = 32'b0;
    end
    if ($value$plusargs("ROM=%s", icarus_rom_path)) $readmemh(icarus_rom_path, flash.mem);
    if ($value$plusargs("RAM=%s", icarus_ram_path)) $readmemh(icarus_ram_path, psram.mem);
`else
    for (int unsigned i = 0; i < MEM_WORDS; i = i + 1) mem.mem[i] = 32'b0;
    if ($value$plusargs("ROM=%s", icarus_rom_path)) $readmemh(icarus_rom_path, mem.mem);
    if ($value$plusargs("RAM=%s", icarus_ram_path)) $readmemh(icarus_ram_path, mem.mem);
`endif
    if (!$value$plusargs("CYCLES=%d", icarus_cycle_limit)) icarus_cycle_limit = 5000;

    $dumpfile("nano_testbench.vcd");
    $dumpvars(0, nano_testbench);

    for (icarus_cycle = 0; icarus_cycle < icarus_cycle_limit; icarus_cycle = icarus_cycle + 1) begin
      @(posedge clk);
      #1;
      // Cycle 0 is the reset edge here too, matching nano_cxxrtl.cc's own timing.
      if (icarus_cycle == 0) reset <= 0;
`ifdef RISCV_FORMAL
      if (rvfi_monitor_errcode != 16'b0) begin
        finish_run($sformatf("RVFI monitor error %0d at cycle %0d", rvfi_monitor_errcode, icarus_cycle));
      end
      // iverilog is four-state and cxxrtl is not; validity itself may read X, not just 1.
      if ((rvfi_valid_observed === 1'b1 || rvfi_valid_observed === 1'bx) &&
          (^rvfi_insn === 1'bx || ^rvfi_pc_rdata === 1'bx || ^rvfi_pc_wdata === 1'bx ||
           ^rvfi_rs1_rdata === 1'bx || ^rvfi_rs2_rdata === 1'bx || ^rvfi_rd_wdata === 1'bx ||
           ^rvfi_mem_addr === 1'bx ||
           (rvfi_mem_wmask[0] && ^rvfi_mem_wdata[7:0] === 1'bx) ||
           (rvfi_mem_wmask[1] && ^rvfi_mem_wdata[15:8] === 1'bx) ||
           (rvfi_mem_wmask[2] && ^rvfi_mem_wdata[23:16] === 1'bx) ||
           (rvfi_mem_wmask[3] && ^rvfi_mem_wdata[31:24] === 1'bx))) begin
        finish_run($sformatf("X reached a retiring instruction's RVFI fields at cycle %0d", icarus_cycle));
      end
`endif
      if (trap_latched) begin
        finish_run($sformatf("trap taken at cycle %0d", icarus_cycle));
      end
`ifdef NANO_QSPI_PINS
      if (psram.mem[TOHOST_INDEX] != 32'b0) begin
        if (psram.mem[TOHOST_INDEX] == 32'b1) begin
          finish_run("PASS");
        end else begin
          finish_run($sformatf("FAIL %0d", psram.mem[TOHOST_INDEX] >> 1));
        end
      end
`else
      if (mem.mem[TOHOST_INDEX] != 32'b0) begin
        if (mem.mem[TOHOST_INDEX] == 32'b1) begin
          finish_run("PASS");
        end else begin
          finish_run($sformatf("FAIL %0d", mem.mem[TOHOST_INDEX] >> 1));
        end
      end
`endif
    end
    finish_run("TIMEOUT");
  end
`endif
endmodule
