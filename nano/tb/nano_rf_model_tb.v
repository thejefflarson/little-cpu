`timescale 1ns / 1ps
// Grades nano.v's behavioural model of the register-file macro against the contract nano is built on: registered reads, a held address re-reading, and a read of the word being written on the same edge returning neither answer.
module nano_rf_model_tb;
  reg         clk = 0;
  reg  [31:0] w_data = 0;
  reg  [4:0]  w_addr = 0;
  reg         w_ena = 0;
  reg  [4:0]  ra_addr = 0, rb_addr = 0;
  wire [31:0] ra_data, rb_data;
  integer     failures = 0;

  rf_top dut (.clk(clk), .w_data(w_data), .w_addr(w_addr), .w_ena(w_ena),
              .ra_addr(ra_addr), .rb_addr(rb_addr), .ra_data(ra_data), .rb_data(rb_data));

  always #5 clk = ~clk;

  task check(input [1023:0] what, input [31:0] got, input [31:0] want);
    if (got !== want) begin
      $display("FAIL %0s: got %h, wanted %h", what, got, want);
      failures = failures + 1;
    end
  endtask

  task edge_then_settle;
    begin
      @(posedge clk);
      #1;
    end
  endtask

  initial begin
    @(negedge clk);
    w_ena = 1; w_addr = 3; w_data = 32'hA5A5_0003;
    edge_then_settle;
    w_addr = 7; w_data = 32'h5A5A_0007;
    edge_then_settle;
    w_ena = 0;

    ra_addr = 3; rb_addr = 7;
    check("a read address does not answer before the edge, port a", ra_data, 32'bx);
    edge_then_settle;
    check("a read returns its word one edge after its address, port a", ra_data, 32'hA5A5_0003);
    check("a read returns its word one edge after its address, port b", rb_data, 32'h5A5A_0007);

    ra_addr = 7;
    check("changing the address does not move the answer before the edge", ra_data, 32'hA5A5_0003);
    edge_then_settle;
    check("the new address answers on the next edge", ra_data, 32'h5A5A_0007);
    edge_then_settle;
    check("a held address keeps answering", ra_data, 32'h5A5A_0007);

    w_ena = 1; w_addr = 7; w_data = 32'hDEAD_BEEF;
    edge_then_settle;
    w_ena = 0;
    check("a read of the word written on the same edge returns the old word inverted",
          ra_data, ~32'h5A5A_0007);
    edge_then_settle;
    check("a held address returns the written word once the edge has passed", ra_data, 32'hDEAD_BEEF);

    w_ena = 1; w_addr = 3; w_data = 32'h1111_1111; rb_addr = 7;
    edge_then_settle;
    w_ena = 0;
    check("a write to one word leaves another word's read alone", rb_data, 32'hDEAD_BEEF);

    if (failures == 0) $display("PASS");
    else $display("FAIL %0d checks", failures);
    $finish;
  end
endmodule
