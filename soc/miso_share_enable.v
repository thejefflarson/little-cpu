`timescale 1 ns / 1 ps
`default_nettype none
module miso_share_enable (
  input  logic clk,
  input  logic now,
  output logic enable
);
  logic sync0, released;

  always_ff @(posedge clk) begin
    sync0    <= now;
    released <= sync0;
  end

  assign enable = released && now;
endmodule
