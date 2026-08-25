`timescale 1 ns / 1 ps
`default_nettype none
// Grants `want` only once `release_in` has been read HIGH while nothing here
// was driving it, and holds that grant until `want` drops.
//
// Built for exactly one shape of problem: `want` also controls an output
// enable on the same physical pin `release_in` reads back, on a board where
// the far side of that pin is a second driver this design does not control.
// `grant = want && release_in` closes that into a loop through the pad --
// granting drives the pin low, which reads back as released, which grants --
// and yosys cannot see it as a loop because the pin is off-chip; it would
// synthesise and oscillate. The two-flop synchroniser below free-runs only
// while `grant` is low and freezes the instant it goes high, so the read
// happens before the pin is driven and the decision holds for the whole
// request rather than being re-read through it.
//
// `release_in` crosses from a source this design has no clock relationship
// with -- a second chip on the far side of the pin -- so it is only ever
// read through this synchroniser, never combinationally.
module pin_lockout (
  input  logic clk,
  input  logic want,
  input  logic release_in,
  output logic grant
);
  logic sync0, released;

  assign grant = want && released;

  always_ff @(posedge clk) begin
    if (!grant) begin
      sync0    <= release_in;
      released <= sync0;
    end
  end
endmodule
