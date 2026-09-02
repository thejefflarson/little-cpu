`timescale 1 ns / 1 ps
`default_nettype none
// Grants `want` only once `release_in` has been read HIGH while nothing here
// was driving it, holds that grant for as long as `busy` keeps pulsing, and
// forces a fresh sample if it does not.
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
//
// THE GRANT IS BOUNDED, NOT INDEFINITE. Nothing else here stops `want` staying
// high forever -- it is a level, not a request pulse, and firmware that traps,
// hangs or forgets to lower it would otherwise hold three pins driven for the
// life of the chip. `busy` is the requester's own "a transfer is actually
// moving" signal (rtl/spiflash.v's, once a caller wires the two together); a
// grant held for IDLE_LIMIT cycles with `busy` never once true is presumed
// idle rather than mid-transfer, and both synchroniser flops are forced back
// to their cold-start value, which drops the grant for at least two cycles --
// long enough for a genuine resample of `release_in` rather than a reuse of
// whatever it read before. If nothing else wants the pins, the resample finds
// them released and the grant comes back; if something does, it does not,
// which is the same contract `release_in` reading low has always had. Sixteen
// cycles is rtl/spiflash.v's own byte time, so a caller pacing one byte after
// another never sees it. A CALLER WHOSE OWN OVERHEAD BETWEEN BYTES EXCEEDS
// SIXTEEN CYCLES WOULD SEE THE GRANT BLINK MID-TRANSFER, which this module
// cannot tell apart from a hang -- that is unmeasured against real compiled
// firmware and is owed to whichever change first wires a requester to this
// input.
//
// THIS BOUND ANSWERS A DIFFERENT QUESTION THAN "IS THE HOST THERE": it stops
// this design from holding the pins past its own need for them, which is a
// property of the requester alone. It says nothing about whether `released`
// itself can be trusted -- a pin driven high and a pin merely pulled high read
// identically, and no bound on how long a grant lasts changes that.
module pin_lockout (
  input  logic clk,
  input  logic want,
  input  logic busy,
  input  logic release_in,
  output logic grant
);
  // A byte's worth of idle cycles with the grant held and nothing moving.
  localparam int IDLE_LIMIT = 16;

  logic sync0, released;
  logic [3:0] idle_count;

  assign grant = want && released;

  // Only two of the four arms touch `sync0`/`released`, and each is stated once
  // rather than split across guards that would have to be proven mutually
  // exclusive by tracing them.
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
