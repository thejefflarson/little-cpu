`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_ALIGNED_MEM
`include "rvfi_macros.vh"
`include "rvfi_channel.sv"
`include "rvfi_dmem_check.sv"

module testbench (
  input clk
);
  logic reset = 1;
  logic trap;

  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES

  logic [31:0] dmem_addr;

  rvfi_dmem_check checker_inst (
    .clock(clk),
    .reset(reset),
    .enable(1'b1),
    .dmem_addr(dmem_addr),
    `RVFI_CONN
  );

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

  // Tied low rather than left free: this task is about the data bus, so a steal
  // would add stall cycles and no coverage. formal/wrapper.v models the arbiter.
  logic        fetch_stall = 1'b0;

  // Constrains the environment rather than duplicating the checker, which
  // builds its own load-after-store shadow from rvfi_mem_* alone: mem_rdata is
  // otherwise free every cycle and no design could satisfy that assertion. It
  // reads the raw bus because rvfi_dmem_check cannot see cycles that retire no
  // memory op. Gating on `mem_wstrb` alone is exact, since rtl/accessor.v
  // defaults the strobe to 0 on every cycle that is not a real store.
  logic [31:0] dmem_data;
  always_ff @(posedge clk) begin
    if (!reset && mem_addr == dmem_addr) begin
      if (mem_wstrb[0]) dmem_data[ 7: 0] <= mem_wdata[ 7: 0];
      if (mem_wstrb[1]) dmem_data[15: 8] <= mem_wdata[15: 8];
      if (mem_wstrb[2]) dmem_data[23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) dmem_data[31:24] <= mem_wdata[31:24];
    end
  end

  // Assumed: the bus returns, one cycle after the request, whatever was last
  // written to that address. Discharged only inside the mapped region, by
  // test/mem_tb.v -- outside it the real memory drops the write and reads zero
  // while this model retains, and `dmem_addr` is free, so the assume is
  // stronger than rtl/memory.v over most of the address space (ADR-0049 F4).
  // Its scope is the one rvfi_dmem_check assertion here, and it constrains a
  // DUT input, so it narrows the environment and can never excuse the core.
  //
  // Against LAST cycle's mem_addr because ADR-0015's load turnaround registers
  // mem_rdata the cycle after the address is presented, and rtl/accessor.v
  // reverts mem_addr to 0 on that response cycle.
  always_ff @(posedge clk) begin
    if (!reset && $past(mem_addr) == dmem_addr && !$past(mem_wstrb))
      assume(dmem_data == mem_rdata);
  end

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
    .trap(trap),
    `RVFI_CONN
  );
endmodule
