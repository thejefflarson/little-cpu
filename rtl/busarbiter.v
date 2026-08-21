`timescale 1 ns / 1 ps
`default_nettype none
// At most one hart drives the shared data bus per cycle. `request` is a
// decode-level flag -- this hart launches a memory access -- and a hart that is
// not granted publishes nothing and asks again next cycle, so nothing issued
// and nothing has to be un-committed. It reads no core state: two harts, two
// request bits, two lock bits, and the only state is the grant itself.
//
// THE GRANT IS THE ROUND-ROBIN POINTER. A cycle both harts ask for goes to
// whichever of them did not hold the bus last cycle, and that is `grant`, so
// there is no second register to keep in step with it. Fixed priority would
// starve the loser; this alternates under sustained contention, and the wait is
// bounded rather than merely believed to be -- formal/busarbiter.sv proves the
// bound and states the constant. An idle bus reads 2'b00 and offers itself to
// hart 0, so hart 1's worst wait is one cycle longer than hart 0's, three
// against two. Both are bounded, which is the property.
//
// `mem_lock` IS READ ON A CYCLE THE HART IS GRANTED, and means "the access I am
// launching this cycle needs the next cycle too". An AMO reads its word on the
// cycle it issues and writes the result back on the cycle after, and the grant
// has to span both. It is the issue cycle that raises it and not the write
// cycle that follows: `grant` is registered, so this module has to be told
// before the cycle it must cover. A lock raised two cycles running holds the
// bus for two extra cycles, which is why the core owes this one a lock that
// lasts a single cycle -- the bound proved over this module assumes exactly
// that, and formal/busarbiter.sv says so where it assumes it.
module busarbiter (
    input  logic       clk,
    input  logic       reset,
    input  logic [1:0] request,
    input  logic [1:0] mem_lock,
    output logic [1:0] grant
);
  // Both asking is the only cycle the pointer decides anything: one asking
  // takes the bus and neither asking leaves it idle.
  logic [1:0] winner;
  assign winner = (request == 2'b11) ? (grant[0] ? 2'b10 : 2'b01) : request;

  // Only the hart holding the bus can lock it.
  logic held;
  assign held = |(grant & mem_lock);

  always_ff @(posedge clk) begin
    if (reset) grant <= 2'b00;
    else grant <= held ? grant : winner;
  end
endmodule
