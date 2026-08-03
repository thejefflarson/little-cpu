`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_ALIGNED_MEM
`include "rvfi_macros.vh"
`include "rvfi_channel.sv"

module testbench (
  input clk
);
  logic reset = 1;
  logic trap;

  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES

  logic [31:0] uut_imem_addr;
  logic [31:0] uut_imem_data;
  logic [31:0] uut_imem_addr2;
  logic [31:0] uut_imem_data2;
  // The fetch address one cycle early. Unread: the assume below pins the data
  // ports against the addresses the core presents in the same cycle, which is
  // the combinational fetch bus this task was written for.
  logic [31:0] uut_imem_addr_next;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren;
  logic [31:0] mem_rdata;
  logic        fetch_stall;
  logic        text_write;

  imem_arbiter arbiter (
    .clock(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .fetch_stall(fetch_stall),
    .text_write(text_write)
  );

  // The one halfword this task watches. Its address is fixed for the trace; what
  // memory holds there starts free and then follows any store that covers it.
  //
  // This is written here rather than taken from riscv-formal's rvfi_imem_check,
  // which fixes the contents for the whole trace as well as the address. That
  // model has no write port, and the checker is what drives the data, so there
  // is no way to tell it a store happened.
  `rvformal_rand_const_reg [31:0] shadow_addr;
  logic [15:0] shadow_data;
  logic        shadow_stored = 1'b0;

  logic        shadow_hit;
  logic [1:0]  shadow_wstrb;
  logic [15:0] shadow_wdata;
  assign shadow_hit   = text_write && mem_addr[31:2] == shadow_addr[31:2];
  assign shadow_wstrb = shadow_addr[1] ? mem_wstrb[3:2]   : mem_wstrb[1:0];
  assign shadow_wdata = shadow_addr[1] ? mem_wdata[31:16] : mem_wdata[15:0];

  always_ff @(posedge clk) begin
    if (shadow_hit) begin
      if (shadow_wstrb[0]) shadow_data[ 7:0] <= shadow_wdata[ 7:0];
      if (shadow_wstrb[1]) shadow_data[15:8] <= shadow_wdata[15:8];
      if (|shadow_wstrb)   shadow_stored     <= 1'b1;
    end
  end

  // Assumed: whenever either fetch port covers the watched halfword, the core is
  // handed what the shadow holds.
  //
  // Nothing discharges this. The backing is that rtl/imemory.v answers both
  // fetch ports from one array and writes it from the same strobes read here.
  // Believed, not proved. It reaches only the assertion below, and it constrains
  // inputs to the core rather than outputs, so it can shrink the set of traces
  // but cannot excuse a wrong answer on one that survives.
  //
  // Dropped on a stolen window, where the banks answered the data access and the
  // fetch ports carry that word instead. That is also the one cycle on which the
  // shadow can be ahead of the array: a store lands on the same edge it steals,
  // and the first fetch that can see it is the cycle after.
  //
  // Compared in the same cycle, with no handshake to wait on, because
  // rtl/fetcher.v drives both addresses and the windowed instruction
  // combinationally on every non-reset cycle.
  always_comb begin
    if (!reset && !fetch_stall) begin
      if (uut_imem_addr      == shadow_addr) assume(uut_imem_data [15: 0] == shadow_data);
      if (uut_imem_addr + 2  == shadow_addr) assume(uut_imem_data [31:16] == shadow_data);
      if (uut_imem_addr2     == shadow_addr) assume(uut_imem_data2[15: 0] == shadow_data);
      if (uut_imem_addr2 + 2 == shadow_addr) assume(uut_imem_data2[31:16] == shadow_data);
    end
  end

  // Every retire at the watched halfword reports it. Pinning both fetch ports to
  // one halfword model is what makes this the consistency check for the dual-word
  // fetch window: if imem_addr2 ever disagreed with imem_addr about what sits at
  // an address, this catches it.
  //
  // It stops at the first store to that halfword and does not resume. An
  // instruction fetched before the store can still retire after it, correctly
  // reporting the old encoding -- that is the window `fence.i` closes. Asserting
  // against the new value there would fail correct hardware.
  always_ff @(posedge clk) begin
    if (!reset && rvfi_valid && !shadow_stored) begin
      if (rvfi_pc_rdata == shadow_addr)
        assert(rvfi_insn[15:0] == shadow_data);
      if (rvfi_insn[1:0] == 2'b11 && rvfi_pc_rdata + 2 == shadow_addr)
        assert(rvfi_insn[31:16] == shadow_data);
    end
  end

  littlecpu uut (
    .clk(clk),
    .reset(reset),
    .imem_addr(uut_imem_addr),
    .imem_data(uut_imem_data),
    .imem_addr2(uut_imem_addr2),
    .imem_data2(uut_imem_data2),
    .imem_addr_next(uut_imem_addr_next),
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
