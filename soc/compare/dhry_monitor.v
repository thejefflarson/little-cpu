`timescale 1 ns / 1 ps
`default_nettype none
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
