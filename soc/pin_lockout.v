`timescale 1 ns / 1 ps
`default_nettype none
// `want` also drives the pin `release_in` reads back, so a synchroniser that
// kept sampling would loop through the pad and oscillate: it free-runs only
// while `grant` is low, and the read it froze on holds for the whole request.
module pin_lockout (
  input  logic clk,
  input  logic want,
  input  logic busy,
  // From a chip on the far side of the pin, with no clock relationship.
  input  logic release_in,
  output logic grant
);
  // rtl/spiflash.v's byte time. A grant held this long with `busy` never true
  // is presumed hung and both flops cleared for a fresh two-cycle resample; a
  // caller idle longer between bytes would see the grant blink mid-transfer.
  localparam int IDLE_LIMIT = 16;

  logic sync0, released;
  logic [3:0] idle_count;

  assign grant = want && released;

  always_ff @(posedge clk) begin
    if (!grant) begin
      sync0      <= release_in;
      released   <= sync0;
      idle_count <= '0;
    end else if (busy) begin
      idle_count <= '0;
    end else if (idle_count == IDLE_LIMIT - 1) begin
      idle_count <= '0;
      sync0      <= 1'b0;
      released   <= 1'b0;
    end else begin
      idle_count <= idle_count + 1'b1;
    end
  end
endmodule
