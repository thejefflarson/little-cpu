`timescale 1 ns / 1 ps
`default_nettype none

// Standalone bench for rtl/memory.v (see `eb18320`). Since ADR-0054 the module
// is also the data RAM `test/testbench.v` instantiates, so the whole `.S` suite
// and the Sail co-simulation run against it too -- this bench is no longer the
// only thing that touches it. What it still does, and they cannot, is drive the
// corners: an out-of-range access, and the no-change read behaviour below.
module mem_tb;
  localparam int RAM_WORDS = 16;

  logic clk = 0;
  always #5 clk = ~clk;

  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;

  // BASE = 0 so the vectors below can address the array directly. The shipping
  // instances use ADR-0008's non-zero RAM base (rtl/littlesoc.v,
  // test/testbench.v), and the out-of-range vectors here cover the decode
  // either way -- an unmapped address must read zero, not alias a mapped word.
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

    // ADR-0054: the read port HOLDS on a write cycle. This is not a taste
    // question and it is not free to change: yosys infers `SB_SPRAM256KA` only
    // from a no-change read port (`ice40/spram.txt` declares `rdwr no_change`),
    // and the read-first spelling silently maps the same array to 128
    // `SB_RAM40_4K` -- four times the part's entire block RAM, reported as a
    // normal synthesis run. Nothing in the pipeline observes the difference
    // (rtl/accessor.v reads mem_rdata only on a load's response cycle, when
    // decode is bubbled), so nothing but this vector would notice it being
    // "fixed" back.
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
