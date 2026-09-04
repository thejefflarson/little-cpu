`timescale 1 ns / 1 ps
`default_nettype none
// One core's Dhrystone marker/verdict monitor, instantiated once per DUT by
// soc/compare/dhry_tb.v and soc/compare/dhry_solo_tb.v rather than open-coded
// per core: the mechanism watching for soc/compare/dhry.lds' two control-window
// addresses is the same for every core in this harness, and only the bus
// signal names at the call site differ per DUT (soc/compare/bench_littlecpu.v's
// mem_addr/mem_wdata/mem_wstrb, soc/compare/bench_vexriscv.v's
// dbus_cmd_address/dbus_cmd_data/mem_wstrb, soc/compare/bench_hazard3.v's
// mem_addr_mux/hwdata/mem_wstrb_mux).
//
// `cycle` is the caller's shared testbench clock counter, not a counter of its
// own, so every core's marks are read against one clock rather than N
// independently-drifting ones.
module dhry_monitor (
  input  logic        clk,
  input  int unsigned cycle,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [ 3:0] mem_wstrb,

  output int unsigned marks       = 0,
  output int unsigned begin_cycle = 0,
  output int unsigned end_cycle   = 0,
  output int unsigned writes      = 0,
  output int unsigned verdict     = 0
);
  // soc/compare/dhry.lds' ram ORIGIN, where its .dhryctl section is placed.
  localparam bit [31:0] CTL_MARK = 32'h0001_0000;
  localparam bit [31:0] CTL_DONE = 32'h0001_0004;

  always_ff @(posedge clk) begin
    if (|mem_wstrb) begin
      writes <= writes + 1;
      if (mem_addr == CTL_MARK) begin
        marks <= marks + 1;
        if (marks == 0) begin_cycle <= cycle;
        else if (marks == 1) end_cycle <= cycle;
      end
      if (mem_addr == CTL_DONE) verdict <= mem_wdata;
    end
  end
endmodule
