`timescale 1 ns / 1 ps
`default_nettype none

// rtl/timer.v's bus port, driven directly.
module timer_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  logic [31:0] mem_addr, mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  logic        mtip;

  localparam logic [31:0] BASE = 32'h0002_0000;
  localparam logic [31:0] MTIME_LO    = BASE + 32'd0;
  localparam logic [31:0] MTIME_HI    = BASE + 32'd4;
  localparam logic [31:0] MTIMECMP_LO = BASE + 32'd8;
  localparam logic [31:0] MTIMECMP_HI = BASE + 32'd12;

  timer #(.BASE(BASE)) dut (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .mtip(mtip)
  );

  // A second instance with two harts, on the same bus.
  localparam logic [31:0] CMP1_LO = BASE + 32'd16;
  localparam logic [31:0] CMP1_HI = BASE + 32'd20;

  logic [31:0] d_mem_rdata;
  logic [1:0]  d_mtip;

  timer #(.BASE(BASE), .NHARTS(2)) dut2 (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(d_mem_rdata),
    .mtip(d_mtip)
  );

  int errors = 0;

  task automatic check_hex(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_bit(input string what, input logic got, input logic expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%b expected=%b", what, got, expected);
        errors++;
      end
    end
  endtask

  // The bus is idle unless a task is driving it, so nothing here accidentally holds a
  // write strobe over an edge it did not mean to.
  task automatic idle();
    begin
      mem_addr  = 32'h0;
      mem_wstrb = 4'b0000;
      mem_wdata = 32'h0;
      @(posedge clk);
      #1;
    end
  endtask

  task automatic store(input logic [31:0] a, input logic [31:0] d, input logic [3:0] strb);
    begin
      mem_addr  = a;
      mem_wdata = d;
      mem_wstrb = strb;
      @(posedge clk);
      #1;
      mem_wstrb = 4'b0000;
    end
  endtask

  // The read port is registered, so the answer belongs to the address that was presented
  // across the previous edge -- the same one-cycle turnaround rtl/accessor.v gives every
  // load.
  task automatic load(input logic [31:0] a);
    begin
      mem_addr  = a;
      mem_wstrb = 4'b0000;
      @(posedge clk);
      #1;
    end
  endtask

  logic [31:0] first_read;

  // The level the privileged spec defines, read off the two architectural registers
  // rather than off whatever mtip was built from.
  logic level;
  assign level = dut.mtime >= dut.mtimecmp;

  // A store lands on the edge that ends the cycle driving it and both sides of the
  // comparison come out of flip-flops, so the cycle after one is a cycle the spec lets
  // mtip be stale for -- a change in the comparison is reflected eventually, not
  // immediately.
  logic level_prev, wrote_prev;
  int high_checks = 0, low_checks = 0;
  // A plain `always`: iverilog warns about the $display below in an `always_ff`.
  always @(posedge clk) begin
    if (reset) begin
      level_prev <= 1'b0;
      wrote_prev <= 1'b0;
    end else begin
      if (!wrote_prev && level === level_prev) begin
        if (level) high_checks++; else low_checks++;
        if (mtip !== level) begin
          $display("MISMATCH mtip against the level, mtime=%016x mtimecmp=%016x: got=%b expected=%b",
                   dut.mtime, dut.mtimecmp, mtip, level);
          errors++;
        end
      end
      level_prev <= level;
      wrote_prev <= dut.writing;
    end
  end

  initial begin
    reset     = 1'b1;
    mem_addr  = 32'h0;
    mem_wdata = 32'h0;
    mem_wstrb = 4'b0000;
    repeat (2) @(posedge clk);
    #1;
    reset = 1'b0;

    load(MTIMECMP_LO);
    check_hex("mtimecmp resets to zero (low)", mem_rdata, 32'h0);
    load(MTIMECMP_HI);
    check_hex("...and high", mem_rdata, 32'h0);
    check_bit("...so mtip is asserted out of reset, which the enables make harmless",
              mtip, 1'b1);

    load(MTIME_HI);
    check_hex("mtimeh starts at zero", mem_rdata, 32'h0);

    // Disarming is a store, and it is what a boot path does before enabling anything.
    store(MTIMECMP_HI, 32'hffff_ffff, 4'b1111);
    store(MTIMECMP_LO, 32'hffff_ffff, 4'b1111);
    idle();
    check_bit("moving mtimecmp out of reach disarms it", mtip, 1'b0);

    load(MTIME_LO);
    first_read = mem_rdata;
    load(MTIME_LO);
    check_hex("mtime advances one per cycle", mem_rdata, first_read + 32'd1);

    load(BASE - 32'd4);
    check_hex("below the range reads zero", mem_rdata, 32'h0);
    load(BASE + 32'd16);
    check_hex("just past the range reads zero, not mtime", mem_rdata, 32'h0);
    load(32'h0001_0000);
    check_hex("the data RAM's base reads zero here", mem_rdata, 32'h0);

    store(BASE + 32'd16, 32'hdead_beef, 4'b1111);
    load(MTIME_HI);
    check_hex("an out-of-range store lands nowhere", mem_rdata, 32'h0);

    store(MTIMECMP_HI, 32'h0000_0000, 4'b1111);
    store(MTIMECMP_LO, 32'h0000_0200, 4'b1111);
    store(MTIME_HI, 32'h0000_0000, 4'b1111);
    store(MTIME_LO, 32'h0000_01ff, 4'b1111);
    check_bit("mtime below mtimecmp raises nothing", mtip, 1'b0);
    idle();
    check_bit("mtime EQUAL to mtimecmp is pending -- the compare is >=, not >",
              mtip, 1'b1);

    idle();
    check_bit("...and it stays high, because it is a level and not a pulse",
              mtip, 1'b1);
    idle();
    check_bit("...still", mtip, 1'b1);

    store(MTIMECMP_LO, 32'hffff_ffff, 4'b1111);
    idle();
    check_bit("moving mtimecmp forward is what clears it", mtip, 1'b0);

    store(MTIME_HI, 32'h0000_0001, 4'b1111);
    store(MTIME_LO, 32'h0000_0000, 4'b1111);
    idle();
    check_bit("the compare is over all 64 bits, not the low half", mtip, 1'b1);

    store(MTIMECMP_HI, 32'h0000_0000, 4'b1111);
    store(MTIMECMP_LO, 32'h0000_0204, 4'b1111);
    store(MTIME_HI, 32'h0000_0000, 4'b1111);
    store(MTIME_LO, 32'h0000_0200, 4'b1111);
    check_bit("armed four ticks short of mtimecmp", mtip, 1'b0);
    idle();
    check_bit("...three ticks short is still nothing", mtip, 1'b0);
    idle();
    check_bit("...two", mtip, 1'b0);
    idle();
    check_bit("...one, and this is the tick an early compare would fire on",
              mtip, 1'b0);
    idle();
    check_bit("...and the tick that reaches mtimecmp raises it", mtip, 1'b1);

    store(MTIMECMP_HI, 32'h0000_0002, 4'b1111);
    store(MTIMECMP_LO, 32'h0000_0010, 4'b1111);
    store(MTIME_HI, 32'h0000_0001, 4'b1111);
    store(MTIME_LO, 32'h0000_0050, 4'b1111);
    idle();
    check_bit("mtime under mtimecmp over 64 bits raises nothing", mtip, 1'b0);

    store(MTIMECMP_HI, 32'h0000_0001, 4'b1111);
    idle();
    check_bit("high half first passes through a reachable pair, and it FIRES",
              mtip, 1'b1);
    store(MTIMECMP_LO, 32'hffff_fff0, 4'b1111);
    idle();
    check_bit("...even though the end state it reaches is out of reach again",
              mtip, 1'b0);

    store(MTIMECMP_HI, 32'h0000_0002, 4'b1111);
    store(MTIMECMP_LO, 32'h0000_0010, 4'b1111);
    store(MTIME_HI, 32'h0000_0001, 4'b1111);
    store(MTIME_LO, 32'h0000_0050, 4'b1111);
    idle();
    check_bit("back to the starting point", mtip, 1'b0);

    store(MTIMECMP_LO, 32'hffff_ffff, 4'b1111);
    idle();
    check_bit("step 1: the low half all ones, no smaller than the old value",
              mtip, 1'b0);
    store(MTIMECMP_HI, 32'h0000_0001, 4'b1111);
    idle();
    check_bit("step 2: the new high half, no smaller than the new value",
              mtip, 1'b0);
    store(MTIMECMP_LO, 32'hffff_fff0, 4'b1111);
    idle();
    check_bit("step 3: the new low half, and nothing fired on the way",
              mtip, 1'b0);

    store(MTIME_HI, 32'hffff_ffff, 4'b1111);
    store(MTIME_LO, 32'hffff_ffff, 4'b1111);
    idle();
    load(MTIME_LO);
    check_hex("mtime wraps past all ones rather than saturating there",
              mem_rdata, 32'h0000_0000);
    load(MTIME_HI);
    check_hex("...both halves, so it is one 64-bit counter", mem_rdata, 32'h0000_0000);

    store(MTIMECMP_LO, 32'h1122_3344, 4'b1111);
    store(MTIMECMP_LO, 32'h0000_00ff, 4'b0001);
    load(MTIMECMP_LO);
    check_hex("a byte store writes one byte", mem_rdata, 32'h1122_33ff);
    store(MTIMECMP_LO, 32'hee00_0000, 4'b1000);
    load(MTIMECMP_LO);
    check_hex("...at the top too", mem_rdata, 32'hee22_33ff);
    store(MTIMECMP_HI, 32'h0000_5566, 4'b0011);
    load(MTIMECMP_HI);
    check_hex("...and into the high word", mem_rdata, 32'h0000_5566);

    store(MTIME_HI, 32'h0000_0000, 4'b1111);
    store(MTIME_LO, 32'h0000_0040, 4'b1111);
    load(MTIME_LO);
    check_hex("a write to mtime beats that cycle's increment", mem_rdata, 32'h0000_0040);

    store(MTIME_LO, 32'hffff_ffff, 4'b1111);
    idle();
    load(MTIME_LO);
    check_hex("mtime wraps its low half", mem_rdata, 32'h0000_0000);
    load(MTIME_HI);
    check_hex("...and the carry reaches mtimeh", mem_rdata, 32'h0000_0001);

    store(MTIME_HI, 32'h0000_0000, 4'b1111);
    store(MTIME_LO, 32'hffff_ffff, 4'b1111);
    store(MTIME_LO, 32'h0000_0000, 4'b1111);
    load(MTIME_HI);
    check_hex("a write at the carry boundary discards the carry too", mem_rdata, 32'h0000_0000);
    load(MTIME_LO);
    check_hex("...and the low half restarts from what was written",
              mem_rdata, 32'h0000_0001);

    store(MTIMECMP_LO, 32'hffff_ffff, 4'b1111);
    store(MTIMECMP_HI, 32'hffff_ffff, 4'b1111);
    store(CMP1_LO,     32'hffff_ffff, 4'b1111);
    store(CMP1_HI,     32'hffff_ffff, 4'b1111);
    idle();
    check_hex("two harts: neither mtip is posted with both disarmed",
              {30'b0, d_mtip}, 32'h0);

    store(CMP1_LO, 32'h1234_5678, 4'b1111);
    store(CMP1_HI, 32'h9abc_def0, 4'b1111);
    load(CMP1_LO);
    check_hex("hart 1's mtimecmp reads back", d_mem_rdata, 32'h1234_5678);
    load(CMP1_HI);
    check_hex("...and its high half", d_mem_rdata, 32'h9abc_def0);
    load(MTIMECMP_LO);
    check_hex("hart 0's mtimecmp is untouched by it", d_mem_rdata, 32'hffff_ffff);
    load(MTIMECMP_HI);
    check_hex("...and so is its high half", d_mem_rdata, 32'hffff_ffff);

    load(MTIME_LO);
    first_read = d_mem_rdata;
    load(MTIME_LO);
    check_hex("two harts: mtime is shared and still advances one per cycle",
              d_mem_rdata, first_read + 32'd1);

    store(CMP1_LO, 32'h0000_0000, 4'b1111);
    store(CMP1_HI, 32'h0000_0000, 4'b1111);
    idle();
    check_hex("hart 1 armed posts hart 1's mtip alone", {30'b0, d_mtip}, 32'h2);

    store(MTIMECMP_LO, 32'h0000_0000, 4'b1111);
    store(MTIMECMP_HI, 32'h0000_0000, 4'b1111);
    idle();
    check_hex("...and arming hart 0 posts both", {30'b0, d_mtip}, 32'h3);

    store(CMP1_LO, 32'hffff_ffff, 4'b1111);
    store(CMP1_HI, 32'hffff_ffff, 4'b1111);
    idle();
    check_hex("...and disarming hart 1 lowers only its line",
              {30'b0, d_mtip}, 32'h1);

    store(BASE + 32'd24, 32'hdead_beef, 4'b1111);
    store(BASE + 32'd28, 32'hdead_beef, 4'b1111);
    load(BASE + 32'd24);
    check_hex("the reserved word reads zero", d_mem_rdata, 32'h0);
    load(BASE + 32'd28);
    check_hex("...and so does the one above it", d_mem_rdata, 32'h0);
    load(CMP1_LO);
    check_hex("...and neither store aliased hart 1's mtimecmp",
              d_mem_rdata, 32'hffff_ffff);

    load(BASE + 32'd32);
    check_hex("just past the eight-word window reads zero, not mtime",
              d_mem_rdata, 32'h0);
    load(CMP1_LO);
    check_hex("the one-hart instance does not answer hart 1's address",
              mem_rdata, 32'h0);

    if (high_checks == 0 || low_checks == 0) begin
      $display("MISMATCH the level check never ran both ways: %0d high, %0d low",
               high_checks, low_checks);
      errors++;
    end

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: machine timer (map, level compare, crossing, torn write, byte strobes, counter)");
      $finish;
    end
  end
endmodule
