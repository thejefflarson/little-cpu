`timescale 1 ns / 1 ps
`default_nettype none

// Standalone bench for rtl/memory.v. The `.S` suite and the Sail co-simulation
// also run against this module (test/testbench.v instantiates it), but only this
// bench drives the corners: an out-of-range access, and the no-change read.
module mem_tb;
  localparam int RAM_WORDS = 16;

  logic clk = 0;
  always #5 clk = ~clk;

  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;

  // BASE = 0 so the vectors below can address the array directly; the shipping
  // instances use a non-zero RAM base. The property the out-of-range
  // vectors check holds either way: an unmapped address must not alias a mapped
  // word.
  memory #(.BASE(32'h0), .RAM_WORDS(RAM_WORDS)) dut (
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
      // A decoy distinct from anything ever written, so a read that echoes
      // mem_wdata instead of the array is caught rather than coinciding.
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

    do_write(32'h00000004, 32'hcafef00d);
    do_read(32'h00000004, got);
    check("write-then-read same address", got, 32'hcafef00d);

    do_write(32'h0000000c, 32'h01234567);
    do_read(32'h0000000c, got);
    check("write-then-read second address", got, 32'h01234567);

    do_write(32'h00000000, 32'ha5a5a5a5);
    do_read(32'h00000000, got);
    check("in-range sanity read", got, 32'ha5a5a5a5);

    do_read(4 * RAM_WORDS, got); // first address past the end of ram
    check_ne("out-of-range read does not alias ram[0] (boundary)", got, 32'ha5a5a5a5);

    do_read(32'hfffffffc, got); // far out of range
    check_ne("out-of-range read does not alias ram[0] (far)", got, 32'ha5a5a5a5);

    // The read port holds on a write cycle. yosys infers `SB_SPRAM256KA` only
    // from a no-change read port, and the read-first spelling maps the same
    // array to 128 `SB_RAM40_4K` -- four times the part's block RAM -- with no
    // diagnostic. Nothing in the pipeline observes the difference (rtl/accessor.v
    // reads mem_rdata only on a load's response cycle), so nothing but this
    // vector would notice it being "fixed" back.
    do_read(32'h00000004, got);
    check("read-port setup for the hold check", got, 32'hcafef00d);
    do_write(32'h00000008, 32'h0f0f0f0f);
    check("read port holds across a write cycle", mem_rdata, 32'hcafef00d);
    do_read(32'h00000008, got);
    check("...and the write still landed", got, 32'h0f0f0f0f);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: memory.v read-after-write and out-of-range checks");
      $finish;
    end
  end
endmodule
