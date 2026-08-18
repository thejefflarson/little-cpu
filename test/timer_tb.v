`timescale 1 ns / 1 ps
`default_nettype none

// rtl/timer.v's bus port, driven directly.
//
// Nothing else checks this module. It is outside the core, so `make fit` does
// not see it and no riscv-formal check instantiates it -- the generated checks
// tie the core's `irq_timer` off. The `.S` programs exercise it through real
// loads and stores, but only along the paths a working program takes; the
// awkward cases are here: an out-of-range access, a byte-granular write, a
// write racing the free-running increment, and the torn 64-bit `mtimecmp`
// update, which is the one sequence software has to get right and the one this
// module has to make safe.
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

  // The bus is idle unless a task is driving it, so nothing here accidentally
  // holds a write strobe over an edge it did not mean to.
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

  // The read port is registered, so the answer belongs to the address that was
  // presented across the previous edge -- the same one-cycle turnaround
  // rtl/accessor.v gives every load.
  task automatic load(input logic [31:0] a);
    begin
      mem_addr  = a;
      mem_wstrb = 4'b0000;
      @(posedge clk);
      #1;
    end
  endtask

  logic [31:0] first_read;

  // The level the privileged spec defines, read off the two architectural
  // registers rather than off whatever mtip was built from. mtip is a function
  // of these and of nothing else, and the bus vectors above are what say the
  // registers themselves hold what software wrote.
  logic level;
  assign level = dut.mtime >= dut.mtimecmp;

  // A store lands on the edge that ends the cycle driving it and both sides of
  // the comparison come out of flip-flops, so the cycle after one is a cycle
  // the spec lets mtip be stale for -- a change in the comparison is reflected
  // eventually, not immediately. Skipping that one cycle, and cycles the level
  // itself moved across, leaves cycles where mtip is owed the level exactly, in
  // BOTH directions: high with the level absent is a spurious interrupt, low
  // with the level present is one that never arrives. A scheme that produced
  // mtip a cycle late would still pass here, which is what makes this a check
  // on earliness rather than on one particular spelling of the compare.
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

    // Every register here resets to zero, so mtime >= mtimecmp holds from the
    // first cycle and mtip is asserted out of reset. Nothing is taken, because
    // mstatus.MIE and mie.MTIE reset to zero too; software sets mtimecmp before
    // it enables either. Asserted rather than left implicit, because it is the
    // one thing about this module a reader would guess wrong.
    load(MTIMECMP_LO);
    check_hex("mtimecmp resets to zero (low)", mem_rdata, 32'h0);
    load(MTIMECMP_HI);
    check_hex("...and high", mem_rdata, 32'h0);
    check_bit("...so mtip is asserted out of reset, which the enables make harmless",
              mtip, 1'b1);

    load(MTIME_HI);
    check_hex("mtimeh starts at zero", mem_rdata, 32'h0);

    // Disarming is a store, and it is what a boot path does before enabling
    // anything.
    store(MTIMECMP_HI, 32'hffff_ffff, 4'b1111);
    store(MTIMECMP_LO, 32'hffff_ffff, 4'b1111);
    idle();
    check_bit("moving mtimecmp out of reach disarms it", mtip, 1'b0);

    load(MTIME_LO);
    first_read = mem_rdata;
    load(MTIME_LO);
    check_hex("mtime advances one per cycle", mem_rdata, first_read + 32'd1);

    // Out of range in both directions, and the word just past the end, which is
    // the one an index built from truncated address bits would alias onto
    // mtime.
    load(BASE - 32'd4);
    check_hex("below the range reads zero", mem_rdata, 32'h0);
    load(BASE + 32'd16);
    check_hex("just past the range reads zero, not mtime", mem_rdata, 32'h0);
    load(32'h0001_0000);
    check_hex("the data RAM's base reads zero here", mem_rdata, 32'h0);

    // A store outside the range must not land either.
    store(BASE + 32'd16, 32'hdead_beef, 4'b1111);
    load(MTIME_HI);
    check_hex("an out-of-range store lands nowhere", mem_rdata, 32'h0);

    //-----------------------------------------------------------------------
    // mtip is `mtime >= mtimecmp`, a level rather than a pulse. Both sides come
    // out of flip-flops, so every check below spends one idle cycle first --
    // the write lands on one edge and the comparison of what landed on the
    // next.
    //-----------------------------------------------------------------------

    store(MTIMECMP_HI, 32'h0000_0000, 4'b1111);
    store(MTIMECMP_LO, 32'h0000_0200, 4'b1111);
    store(MTIME_HI, 32'h0000_0000, 4'b1111);
    // One short, with the increment suppressed by this very write, so the next
    // cycle lands mtime exactly ON mtimecmp.
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

    // The only way software lowers it.
    store(MTIMECMP_LO, 32'hffff_ffff, 4'b1111);
    idle();
    check_bit("moving mtimecmp forward is what clears it", mtip, 1'b0);

    // mtime = 0x1_0000_0000 against mtimecmp = 0x0_ffff_ffff. Pending over 64
    // bits; a comparison that looked at the low halves alone would read
    // 0x0000_0000 >= 0xffff_ffff and say no.
    store(MTIME_HI, 32'h0000_0001, 4'b1111);
    store(MTIME_LO, 32'h0000_0000, 4'b1111);
    idle();
    check_bit("the compare is over all 64 bits, not the low half", mtip, 1'b1);

    // The crossing software actually waits on: mtimecmp parked four ticks ahead
    // and no store anywhere near it. The count is what makes this a check --
    // mtip low on each of the three ticks before the one that reaches
    // mtimecmp, and high on that one. A comparison that fired a tick early
    // would be an interrupt taken before the deadline it was armed for.
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

    //-----------------------------------------------------------------------
    // The torn 64-bit write, and the red direction that makes it a check.
    //
    // A 32-bit store touches one half, so an update to mtimecmp is a sequence.
    // The privileged spec's own sample code for RV32 writes the LOW half all
    // ones, then the high half, then the low half -- every intermediate is then
    // at least as large as the smaller of the old and new values, so nothing
    // fires on the way through.
    //
    // The vectors below use a case where the naive order is genuinely unsafe:
    // mtime = 0x1_0000_0050, mtimecmp = {2, 0x10}, target {1, 0xffff_fff0}.
    // Writing the high half first passes through {1, 0x10}, which mtime is
    // past.
    //-----------------------------------------------------------------------

    store(MTIMECMP_HI, 32'h0000_0002, 4'b1111);
    store(MTIMECMP_LO, 32'h0000_0010, 4'b1111);
    store(MTIME_HI, 32'h0000_0001, 4'b1111);
    store(MTIME_LO, 32'h0000_0050, 4'b1111);
    idle();
    check_bit("mtime under mtimecmp over 64 bits raises nothing", mtip, 1'b0);

    // The wrong way, on purpose. A sequence whose failure path has never run is
    // not a check, and this is that failure path.
    store(MTIMECMP_HI, 32'h0000_0001, 4'b1111);
    idle();
    check_bit("high half first passes through a reachable pair, and it FIRES",
              mtip, 1'b1);
    store(MTIMECMP_LO, 32'hffff_fff0, 4'b1111);
    idle();
    // The end state is safe, which is what makes the transient the only thing
    // that was wrong.
    check_bit("...even though the end state it reaches is out of reach again",
              mtip, 1'b0);

    // The spec's way, over the same values.
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

    //-----------------------------------------------------------------------
    // mtime wraps on overflow, which the spec states normatively rather than
    // leaving undefined.
    //-----------------------------------------------------------------------

    store(MTIME_HI, 32'hffff_ffff, 4'b1111);
    store(MTIME_LO, 32'hffff_ffff, 4'b1111);
    idle();
    load(MTIME_LO);
    check_hex("mtime wraps past all ones rather than saturating there",
              mem_rdata, 32'h0000_0000);
    load(MTIME_HI);
    check_hex("...both halves, so it is one 64-bit counter", mem_rdata, 32'h0000_0000);

    //-----------------------------------------------------------------------
    // Byte strobes. `sb` to one byte of mtimecmp must not disturb the other
    // three, or the sequence above stops being safe.
    //-----------------------------------------------------------------------

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

    //-----------------------------------------------------------------------
    // A write to either half of mtime beats that cycle's increment, for the
    // whole 64-bit register. Suppressing per half instead differs only at the
    // carry boundary, where the carry would land in the high half while the
    // write replaced the low one -- so a `sw` of zero to mtime would advance
    // mtimeh. Nothing else in the tree reaches that cycle.
    //-----------------------------------------------------------------------

    store(MTIME_HI, 32'h0000_0000, 4'b1111);
    store(MTIME_LO, 32'h0000_0040, 4'b1111);
    load(MTIME_LO);
    check_hex("a write to mtime beats that cycle's increment", mem_rdata, 32'h0000_0040);

    // Every `load` costs a cycle, and mtime is running, so the expectations
    // below count them: the read port answers with the value the counter held
    // during the cycle the address was presented.
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
    // Plus the one cycle the mtimeh read above took.
    check_hex("...and the low half restarts from what was written",
              mem_rdata, 32'h0000_0001);

    // The window check above grades every quiet cycle in this file, so it is
    // worth nothing if the vectors above stopped reaching one of its two arms.
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
