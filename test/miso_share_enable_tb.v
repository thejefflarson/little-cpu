`timescale 1 ns / 1 ps
`default_nettype none

// soc/miso_share_enable.v driven directly, standing in for the pad it will sit behind on
// soc/board_upduino.v: `now` here is a testbench-driven level, the way pin 16's own
// read-back would be.
module miso_share_enable_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic now, enable;

  miso_share_enable dut (
    .clk(clk),
    .now(now),
    .enable(enable)
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
    now = 1'b0;

    repeat (2) step();
    check_bit("power-on: enable is off before anything settles", enable, 1'b0);

    now = 1'b1;
    step();
    check_bit("one cycle of now high: not yet trusted", enable, 1'b0);
    step();
    check_bit("two cycles of now high: enable follows", enable, 1'b1);
    repeat (4) begin
      step();
      check_bit("now held high: enable stays", enable, 1'b1);
    end

    now = 1'b0;
    step();
    check_bit("now drops: enable drops the SAME cycle, not two later", enable, 1'b0);

    now = 1'b1;
    step();
    check_bit("now back high for one cycle: released has not caught up, no enable", enable, 1'b0);
    now = 1'b0;
    step();
    check_bit("now dropped again before released caught up: still no enable", enable, 1'b0);

    now = 1'b1;
    step();
    check_bit("first real sample of the re-acquisition", enable, 1'b0);
    step();
    check_bit("second real sample: enable follows", enable, 1'b1);
    repeat (3) begin
      step();
      check_bit("steady again: enable holds", enable, 1'b1);
    end

    now = 1'b0;
    step();
    check_bit("second turn-off: still same-cycle, not synchronised-late", enable, 1'b0);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: miso_share_enable (synchronised turn-on, combinational same-cycle turn-off)");
      $finish;
    end
  end
endmodule
