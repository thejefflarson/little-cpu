`timescale 1 ns / 1 ps
`default_nettype none
// Turn-on waits two flops because `now` crosses from an unrelated clock;
// turn-off is combinational on `now` because the flash's data-out drives the
// instant its chip select reads low, and a delayed enable would fight it.
module miso_share_enable (
  input  logic clk,
  input  logic now,
  output logic enable
);
  logic sync0, released;

  // The fabric's flops power up at 0, so the enable is off until two real
  // samples of `now` have read high.
  always_ff @(posedge clk) begin
    sync0    <= now;
    released <= sync0;
  end

  assign enable = released && now;
endmodule
