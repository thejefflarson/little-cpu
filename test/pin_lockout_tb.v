`timescale 1 ns / 1 ps
`default_nettype none

// soc/pin_lockout.v driven directly, standing in for the pad it will sit
// behind on soc/board_upduino.v: `release_in` here is a testbench-driven
// level, the way a second chip driving the real pin would be, and `grant`
// is checked against it rather than against a pin this bench cannot loop
// back through -- the loop `grant`'s own comment warns against is a property
// of wiring `release_in` to a pin `grant` also drives, which this bench does
// not do and does not need to, to grade the module's own contract.
module pin_lockout_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic want, busy, release_in, grant;

  pin_lockout dut (
    .clk(clk),
    .want(want),
    .busy(busy),
    .release_in(release_in),
    .grant(grant)
  );

  int errors = 0;

  task automatic check_bit(input string what, input logic got, input logic expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%b expected=%b", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic step();
    begin
      @(posedge clk);
      #1;
    end
  endtask

  initial begin
    want       = 1'b0;
    busy       = 1'b0;
    release_in = 1'b1;

    //-------------------------------------------------------------------
    // A few idle cycles with the pin released settle the synchroniser from
    // its power-on state before anything is asked of it -- the same head
    // start the real pull-up gives the board before firmware's first access.
    //-------------------------------------------------------------------
    repeat (4) step();
    check_bit("idle, released, nothing wants it: no grant", grant, 1'b0);

    //-------------------------------------------------------------------
    // The uncontended case: released the whole time, a request is granted
    // within the synchroniser's own depth and stays granted for as long as
    // it is held.
    //-------------------------------------------------------------------
    want = 1'b1;
    step();
    step();
    check_bit("released throughout: granted inside two cycles", grant, 1'b1);
    repeat (5) begin
      step();
      check_bit("held want, held release: grant stays up", grant, 1'b1);
    end
    want = 1'b0;
    step();
    check_bit("want dropped: grant drops with it", grant, 1'b0);

    //-------------------------------------------------------------------
    // The contended case: the pin reads low -- something else holds it --
    // for a while before a request arrives. The request must not be granted
    // until AFTER release_in has been read high again, and the grant that
    // does land must never coincide with release_in reading low.
    //-------------------------------------------------------------------
    // Two cycles first, want still low, to flush the synchroniser's stale
    // high sample out before anything is asked of it -- a request arriving in
    // the same cycle release_in changes is a race no finite synchroniser
    // resolves, and is not the property under test here.
    release_in = 1'b0;
    step();
    step();
    want = 1'b1;
    repeat (6) begin
      step();
      check_bit("held low: never granted while release_in reads low", grant, 1'b0);
    end
    release_in = 1'b1;
    repeat (8) begin
      step();
      if (grant) check_bit("grant landed only once release_in reads high", release_in, 1'b1);
    end
    check_bit("release_in high long enough: eventually granted", grant, 1'b1);
    want       = 1'b0;
    release_in = 1'b1;
    step();

    //-------------------------------------------------------------------
    // No oscillation once granted: release_in glitching low WHILE granted --
    // which is what this module's own drive of the real pin would read back
    // as, on the board -- must not drop the grant mid-request. That is the
    // property the frozen synchroniser exists for.
    //-------------------------------------------------------------------
    want = 1'b1;
    repeat (4) step();
    check_bit("granted before the glitch", grant, 1'b1);
    release_in = 1'b0;
    repeat (6) begin
      step();
      check_bit("release_in glitching low mid-request: grant does not drop", grant, 1'b1);
    end
    release_in = 1'b1;
    want       = 1'b0;
    step();
    check_bit("released at last: grant follows want down, not release_in's glitch", grant, 1'b0);

    //-------------------------------------------------------------------
    // The bound protects a real transfer: `busy` shaped like
    // rtl/spiflash.v's own duty cycle -- sixteen cycles high for a byte
    // shifting, then a handful low for the software overhead between bytes
    // -- resets the idle counter every time, so a request held for several
    // such bytes, far longer than one IDLE_LIMIT window, never sees a
    // forced drop. 16 below mirrors soc/pin_lockout.v's own IDLE_LIMIT.
    //-------------------------------------------------------------------
    want       = 1'b1;
    release_in = 1'b1;
    repeat (2) step();
    check_bit("granted for the transfer", grant, 1'b1);
    repeat (4) begin
      busy = 1'b1;
      repeat (16) begin
        step();
        check_bit("busy high for a byte: grant holds", grant, 1'b1);
      end
      busy = 1'b0;
      repeat (10) begin
        step();
        check_bit("busy low between bytes, still under the bound: grant holds", grant, 1'b1);
      end
    end
    want = 1'b0;
    step();
    check_bit("transfer done, want dropped: grant follows it down", grant, 1'b0);

    //-------------------------------------------------------------------
    // No bound protects an idle hold: `want` stays up with `busy` never once
    // true, standing in for firmware that asserted the request and then
    // trapped, hung or forgot to lower it. `released` is already settled
    // from the idle cycle above, so the count is exact: IDLE_LIMIT (16)
    // granted cycles, then two more while the synchroniser gets a genuine
    // fresh sample, then -- uncontested -- the grant returns on its own.
    // One bounded window, not a permanent loss of the bus.
    //-------------------------------------------------------------------
    want       = 1'b1;
    busy       = 1'b0;
    release_in = 1'b1;
    repeat (15) step();
    check_bit("granted, one cycle short of the bound", grant, 1'b1);
    step();
    check_bit("bound hit: grant drops", grant, 1'b0);
    step();
    check_bit("still resyncing", grant, 1'b0);
    step();
    check_bit("uncontested: the grant comes back on its own", grant, 1'b1);
    want = 1'b0;
    step();

    //-------------------------------------------------------------------
    // Contested past the timeout: a host takes the pin in the window a
    // forced drop opens, and the grant must stay withheld exactly as if
    // this had been the very first request -- the bound does not weaken the
    // guard `release_in` reading low has always had.
    //-------------------------------------------------------------------
    want       = 1'b1;
    busy       = 1'b0;
    release_in = 1'b1;
    repeat (16) step();
    check_bit("bound hit again: grant drops", grant, 1'b0);
    release_in = 1'b0;
    repeat (10) begin
      step();
      check_bit("host took the window: grant stays withheld", grant, 1'b0);
    end
    release_in = 1'b1;
    want       = 1'b0;
    step();

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: pin_lockout (samples before granting, holds for the request, never re-reads while driving, and bounds an idle grant to one byte time)");
      $finish;
    end
  end
endmodule
