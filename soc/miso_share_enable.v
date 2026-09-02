`timescale 1 ns / 1 ps
`default_nettype none
// Whether pin 14 -- shared between the UART and the flash's own MISO -- is
// safe for this design to drive right now.
//
// TURN-ON IS SYNCHRONISED. `released` is a genuine two-cycle-old sample of
// `now`, because `now` crosses from a source this design has no clock
// relationship with and a fresh transition needs those two cycles to settle
// before it is trusted. Getting turn-on wrong costs a beat of contention
// that starts a cycle early, so it waits.
//
// TURN-OFF IS COMBINATIONAL, ON `now` DIRECTLY, AND THAT IS THE POINT. The
// flash's own output driver goes live the instant its chip select reads low
// -- this module exists to model that -- so an enable gated only on
// `released` would keep this design's driver on pin 14 for up to two clock
// periods after the flash's is already live: two push-pull drivers on one
// pin, on every chip-select assertion a host makes, not a corner case of
// one. `enable = released && now` kills the enable the same cycle `now`
// goes low, which is the same edge the flash sees, while `released` still
// gates how fast it may come back on.
//
// A glitch on `now` can at worst truncate this design's own drive by a cycle,
// which costs a UART bit and not a fault.
module miso_share_enable (
  input  logic clk,
  input  logic now,
  output logic enable
);
  logic sync0, released;

  // Every `SB_DFF` on this part's FPGA fabric powers up at 0 out of
  // configuration, an SRAM-based part's usual guarantee, so the enable
  // stays off until two real samples of `now` have come back high rather
  // than racing a power-on value onto a pin the flash chip might still be
  // driving.
  always_ff @(posedge clk) begin
    sync0    <= now;
    released <= sync0;
  end

  assign enable = released && now;
endmodule
