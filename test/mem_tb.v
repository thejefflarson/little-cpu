`timescale 1 ns / 1 ps
`default_nettype none

// Standalone bench for rtl/memory.v (see `eb18320`). memory.v is not
// exercised by any other test path: the cxxrtl top (`testbench`) has its own
// inline memory, and the synthesis path (littlesoc) is currently broken (see
// ADR-0010's technical notes), so this is the only place its read/write
// behaviour gets checked.
module mem_tb;
  localparam int RAM_WORDS = 16;

  logic clk = 0;
  always #5 clk = ~clk;

  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;

  memory #(.RAM(RAM_WORDS)) dut (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata)
  );

  int errors = 0;

  task automatic check(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_ne(input string what, input logic [31:0] got, input logic [31:0] unwanted);
    begin
      if (got === unwanted) begin
        $display("MISMATCH %s: got=%08x, which must not alias %08x", what, got, unwanted);
        errors++;
      end
    end
  endtask

  task automatic do_write(input logic [31:0] addr, input logic [31:0] data);
    begin
      mem_addr = addr;
      mem_wdata = data;
      mem_wstrb = 4'b1111;
      @(posedge clk);
      #1;
      mem_wstrb = 4'b0000;
    end
  endtask

  task automatic do_read(input logic [31:0] addr, output logic [31:0] data);
    begin
      mem_addr = addr;
      // A decoy write-data value, distinct from anything ever written, so a read
      // that accidentally echoes mem_wdata (rtl/memory.v's deleted bug) is caught
      // instead of coincidentally matching.
      mem_wdata = 32'hdeadbeef;
      mem_wstrb = 4'b0000;
      @(posedge clk);
      #1;
      data = mem_rdata;
    end
  endtask

  logic [31:0] got;

  initial begin
    mem_addr = 0;
    mem_wdata = 0;
    mem_wstrb = 0;
    @(posedge clk);
    #1;

    // Criterion 5a: a wstrb == 0 read following a write to the same address
    // returns ram[addr>>2], not a stale echo of the bus's mem_wdata.
    do_write(32'h00000004, 32'hcafef00d);
    do_read(32'h00000004, got);
    check("write-then-read same address", got, 32'hcafef00d);

    // A second pair at a different address, to rule out a fluke.
    do_write(32'h0000000c, 32'h01234567);
    do_read(32'h0000000c, got);
    check("write-then-read second address", got, 32'h01234567);

    // Criterion 5b: an out-of-range read must not alias an in-range ram word.
    do_write(32'h00000000, 32'ha5a5a5a5);
    do_read(32'h00000000, got);
    check("in-range sanity read", got, 32'ha5a5a5a5);

    do_read(4 * RAM_WORDS, got); // first address past the end of ram
    check_ne("out-of-range read does not alias ram[0] (boundary)", got, 32'ha5a5a5a5);

    do_read(32'hfffffffc, got); // far out of range
    check_ne("out-of-range read does not alias ram[0] (far)", got, 32'ha5a5a5a5);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: memory.v read-after-write and out-of-range checks");
      $finish;
    end
  end
endmodule
