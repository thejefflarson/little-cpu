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

  localparam int MEM_WORDS = 20480;

  logic        mem_valid;
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

`ifdef ICARUS
  always @(posedge clk) begin
    if (rvfi_monitor_errcode != 16'b0) begin
      $display("RVFI MONITOR ERROR %0d -- see the diagnostic above", rvfi_monitor_errcode);
      $fatal(1);
    end
  end
`endif

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
  initial begin
    $dumpfile("nano_testbench.vcd");
    $dumpvars(0, nano_testbench);
  end
`endif
endmodule
