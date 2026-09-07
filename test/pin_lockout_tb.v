`timescale 1 ns / 1 ps
`default_nettype none

// soc/pin_lockout.v driven directly, standing in for the pad it will sit behind on
// soc/board_upduino.v: `release_in` here is a testbench-driven level, the way a second
// chip driving the real pin would be, and `grant` is checked against it rather than
// against a pin this bench cannot loop back through -- the loop `grant`'s own comment
// warns against is a property of wiring `release_in` to a pin `grant` also drives, which
// this bench does not do and does not need to, to grade the module's own contract.
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

    repeat (4) step();
    check_bit("idle, released, nothing wants it: no grant", grant, 1'b0);

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
