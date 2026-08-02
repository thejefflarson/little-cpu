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
  // ADR-0003: dmemcheck only cares about the data bus, so the dual-word
  // fetch window's second port is left free just like imem_addr/imem_data
  // above -- unconstrained, no assumes.
  logic [31:0] imem_addr2;
  logic [31:0] imem_data2;
  // ADR-0054: the fetch address one cycle early, for a synchronous memory.
  // Unread here for the same reason as the pair above: this task is about the
  // data bus. Connected rather than left dangling so every instantiation of the
  // core names every port.
  logic [31:0] imem_addr_next;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;

  // rvfi_dmem_check (riscv-formal, unmodified) already builds its own
  // load-after-store shadow purely from rvfi_mem_*, so it needs no
  // connection to the raw bus at all -- the block below exists only to
  // constrain the *environment*: without it mem_rdata is a free signal
  // every cycle (see wrapper.v), and no design, buggy or not, could ever
  // satisfy that RVFI-level shadow's assertion. It has to be built from
  // the raw bus because rvfi_dmem_check has no visibility into cycles that
  // don't retire a memory op.
  //
  // Write side: rtl/accessor.v (ADR-0015) drives a store's address, data,
  // and strobe together, combinationally, in the one cycle the request
  // fires ("mem_wdata must be combinational, in lockstep with
  // mem_addr/mem_wstrb" -- accessor.v's own comment on the bug this fixed).
  // Gating on `mem_wstrb` alone is exact: rtl/accessor.v's always_comb
  // defaults `mem_wstrb = 0` every cycle that isn't a real store, so no
  // separate valid signal is needed here the way the wave-0
  // mem_valid/mem_ready handshake needed one.
  logic [31:0] dmem_data;
  always_ff @(posedge clk) begin
    if (!reset && mem_addr == dmem_addr) begin
      if (mem_wstrb[0]) dmem_data[ 7: 0] <= mem_wdata[ 7: 0];
      if (mem_wstrb[1]) dmem_data[15: 8] <= mem_wdata[15: 8];
      if (mem_wstrb[2]) dmem_data[23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) dmem_data[31:24] <= mem_wdata[31:24];
    end
  end

  // FACT       the data bus returns, one cycle after the request, whatever was
  //            last written to that address -- an ordinary memory, modelled at
  //            one address because that is all rvfi_dmem_check pins.
  // DISCHARGED PARTIALLY, and only for part of the address space, which is
  //            what separates it from the imem assumes in wrapper.v and
  //            imemcheck.sv (those model rtl/imemory.v, a ROM that really does
  //            behave this way at every address). test/mem_tb.v checks
  //            rtl/memory.v's read-after-write behaviour directly and is on
  //            `make test-units` -- ADR-0010's "in no current test path" is
  //            stale, and mem_tb.v's own header repeats it. But rtl/memory.v
  //            answers an address at or past 4*RAM with `mem_rdata <=
  //            mem_wdata`, not with stored data, and mem_tb.v only asserts
  //            that such a read does not ALIAS ram[0] -- it never says what it
  //            returns. `dmem_addr` here is a free 32-bit value, so the model
  //            this proof assumes and the only writable memory this repo has
  //            disagree over most of the address space. ADR-0044 rules the
  //            placeholder out as a starting point and does not replace it, so
  //            when the real memory system is built there is no check anywhere
  //            that will hold it to what this proof assumed (ADR-0049 F4).
  // SCOPE      the one rvfi_dmem_check assertion this task contains -- what it
  //            was written for, and nothing more. Like imemcheck.sv's four,
  //            this constrains a DUT INPUT (mem_rdata), so it narrows the
  //            environment and can never excuse the core.
  //
  // Read side is genuinely one cycle behind the request, unlike the
  // wave-0 handshake harness this replaces: ADR-0015's load turnaround
  // registers mem_rdata the cycle *after* the address is presented, and
  // rtl/accessor.v's always_comb reverts mem_addr to its 0 default on that
  // response cycle (no new request is in flight), so this cycle's
  // mem_rdata must be checked against *last* cycle's mem_addr, not this
  // cycle's. $past(mem_addr) == dmem_addr can also fire for an ordinary
  // idle cycle when the solver happens to pick dmem_addr == 0 (RAM base
  // per ADR-0008, so 0 is a legal address and, absent a valid signal,
  // genuinely indistinguishable from idle by address alone) -- that
  // coincidence is harmless: rtl/accessor.v only ever reads mem_rdata on
  // the cycle following a *real* load request (its internal
  // `pending_valid`, invisible from here), so constraining mem_rdata on a
  // cycle the core doesn't consume it changes nothing observable.
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
    .mem_rdata(mem_rdata),
    .trap(trap),
    `RVFI_CONN
  );
endmodule
