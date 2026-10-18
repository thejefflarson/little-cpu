`timescale 1 ns / 1 ps
`default_nettype none
// The interval must span several core cycles of accumulated jitter and stay under rtl/trng.v's
// 8,192-cycle edge timeout; test/entropy_geometry_test.py holds the nominal period between the two.
module icesugar_pro_entropy #(
  parameter integer DIV      = 128,
  parameter integer DIV_LOG2 = 9
) (
  output logic raw
);
  logic osc;
  OSCG #(.DIV(DIV)) oscg (.OSC(osc));

  logic [DIV_LOG2-1:0] count = '0;
  always_ff @(posedge osc) count <= count + 1'b1;
  assign raw = count[DIV_LOG2-1];
endmodule
`default_nettype wire
