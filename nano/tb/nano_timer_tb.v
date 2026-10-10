`timescale 1ns/1ps
// Drives nano_timer's bus port beside a stand-in for the core's mcycle (the one counter mtime aliases) and grades `mtip` against a model: early is an error, late by more than a cycle is.
// Run with iverilog; a failed check prints FAIL, a clean run prints PASS.
module nano_timer_tb;
  localparam logic [31:0] BASE = 32'h1080_0010;

  logic clk = 0;
  always #5 clk = ~clk;

  logic        reset = 1;
  logic [31:0] mem_addr = 32'b0, mem_wdata = 32'b0;
  logic [3:0]  mem_wstrb = 4'b0;
  logic [31:0] mem_rdata;
  logic        mtip, mtime_wr;
  logic [63:0] mc = 64'b0;
  logic        csr_we = 1'b0;
  logic [63:0] csr_val = 64'b0;

  nano_timer #(.BASE(BASE)) dut (
    .clk(clk), .reset(reset),
    .mtime(mc), .mem_addr(mem_addr), .mem_wdata(mem_wdata), .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata), .mtime_wr(mtime_wr), .mtip(mtip)
  );

  int errors = 0;
  task automatic fail(input string what);
    errors = errors + 1;
    $display("FAIL %s at %0t", what, $time);
  endtask

  logic [63:0] m_time = 64'b0, m_cmp = 64'b0;
  logic        cond_q = 1'b0, cond_qq = 1'b0, checking = 1'b0;
  int          rises = 0, falls = 0;
  logic        mtip_seen = 1'b0;

  function automatic logic [63:0] put(input logic [63:0] old, input logic hi,
                                      input logic [31:0] data, input logic [3:0] strb);
    logic [63:0] v;
    v = old;
    for (int b = 0; b < 4; b++) begin
      if (strb[b]) v[(hi ? 32 : 0) + 8*b +: 8] = data[8*b +: 8];
    end
    return v;
  endfunction

  // The core's side of the alias: a CSR write wins, a bus store merges its bytes into the half mem_addr[2] names, otherwise it ticks.
  logic [63:0] lane_mask;
  assign lane_mask = {{8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}}} << (mem_addr[2] ? 32 : 0);
  always @(posedge clk) begin
    if (reset)            mc <= 64'b0;
    else if (csr_we)      mc <= csr_val;
    else if (mtime_wr)    mc <= (({mem_wdata, mem_wdata} & lane_mask) | (mc & ~lane_mask));
    else                  mc <= mc + 64'd1;
  end

  always @(negedge clk) begin
    if (checking && mc !== m_time) fail("mcycle diverged from the mtime model");
  end

  logic        w_hit;
  logic [1:0]  w_word;
  assign w_hit  = mem_addr[31:4] == BASE[31:4] && |mem_wstrb;
  assign w_word = mem_addr[3:2];

  always @(posedge clk) begin
    if (checking) begin
      if (mtip && !cond_q) fail("mtip posted early: mtime was below mtimecmp the cycle before");
      if (!mtip && cond_q && cond_qq) fail("mtip late: mtime >= mtimecmp held two cycles and mtip is low");
      if (mtip && !mtip_seen) rises = rises + 1;
      if (!mtip && mtip_seen) falls = falls + 1;
      mtip_seen = mtip;
    end
    cond_qq <= cond_q;
    cond_q  <= !reset && m_time >= m_cmp;
    if (reset) begin
      m_time <= 64'b0;
      m_cmp  <= 64'b0;
    end else begin
      m_time <= csr_we ? csr_val :
                (w_hit && w_word == 2'd0) ? put(m_time, 1'b0, mem_wdata, mem_wstrb) :
                (w_hit && w_word == 2'd1) ? put(m_time, 1'b1, mem_wdata, mem_wstrb) :
                m_time + 64'd1;
      if (w_hit && w_word == 2'd2) m_cmp <= put(m_cmp, 1'b0, mem_wdata, mem_wstrb);
      if (w_hit && w_word == 2'd3) m_cmp <= put(m_cmp, 1'b1, mem_wdata, mem_wstrb);
    end
  end

  task automatic bus_write(input logic [31:0] addr, input logic [31:0] data, input logic [3:0] strb);
    @(negedge clk);
    mem_addr = addr; mem_wdata = data; mem_wstrb = strb;
    @(negedge clk);
    mem_addr = 32'b0; mem_wdata = 32'b0; mem_wstrb = 4'b0;
  endtask

  task automatic csr_write(input logic [63:0] v);
    @(negedge clk);
    csr_we = 1'b1; csr_val = v;
    @(negedge clk);
    csr_we = 1'b0;
  endtask

  task automatic wr(input logic [1:0] word, input logic [31:0] data);
    bus_write(BASE + {28'b0, word, 2'b00}, data, 4'hf);
  endtask

  task automatic rd_check(input logic [1:0] word, input string what);
    logic [63:0] model;
    logic [31:0] want;
    @(negedge clk);
    mem_addr = BASE + {28'b0, word, 2'b00};
    #1;
    model = word[1] ? m_cmp : m_time;
    want = word[0] ? model[63:32] : model[31:0];
    if (mem_rdata !== want) fail($sformatf("%s: read %08h, wanted %08h", what, mem_rdata, want));
    mem_addr = 32'b0;
  endtask

  task automatic idle(input int cycles);
    repeat (cycles) @(negedge clk);
  endtask

  // The spec's three stores, in the spec's order.
  task automatic set_cmp(input logic [63:0] v);
    wr(2'd2, 32'hffff_ffff);
    wr(2'd3, v[63:32]);
    wr(2'd2, v[31:0]);
  endtask

  task automatic set_time(input logic [63:0] v);
    wr(2'd1, v[63:32]);
    wr(2'd0, v[31:0]);
  endtask

  task automatic expect_mtip(input logic level, input string what);
    @(negedge clk);
    if (mtip !== level) fail($sformatf("%s: mtip is %b, wanted %b", what, mtip, level));
  endtask

  logic [63:0] deadline;
  int          before_rises;

  initial begin
    if (put(64'h0000_0000_ffff_ffff, 1'b0, 32'h0000_0001, 4'b0001) !== 64'h0000_0000_ffff_ff01) begin
      $display("ORACLE BROKEN: put() merges a byte wrongly");
      $finish;
    end
    if (put(64'h0, 1'b1, 32'h8000_0000, 4'b1000) !== 64'h8000_0000_0000_0000) begin
      $display("ORACLE BROKEN: put() addresses the high word wrongly");
      $finish;
    end

    repeat (3) @(negedge clk);
    reset = 0;
    checking = 1;

    // Out of reset mtimecmp is zero, so the level posts on its own.
    idle(4);
    expect_mtip(1'b1, "after reset");

    set_cmp(64'hffff_ffff_ffff_ffff);
    idle(3);
    expect_mtip(1'b0, "after mtimecmp moved out of reach");

    // Near future: nothing before the deadline, the level by two cycles after it.
    deadline = m_time + 64'd40;
    set_cmp(deadline);
    before_rises = rises;
    while (m_time < deadline - 64'd1) begin
      @(negedge clk);
      if (mtip) fail("mtip high before the deadline");
    end
    idle(3);
    expect_mtip(1'b1, "after the deadline");
    if (rises != before_rises + 1) fail("the level did not post exactly once");

    idle(8);
    expect_mtip(1'b1, "still posted");
    set_cmp(64'hffff_ffff_ffff_ffff);
    idle(3);
    expect_mtip(1'b0, "after mtimecmp moved on");

    // The carry into the high word: mtime {0, ffff_fff0} reaches {1, 8} 24 cycles on.
    set_cmp(64'h0000_0001_0000_0008);
    set_time(64'h0000_0000_ffff_fff0);
    before_rises = rises;
    idle(6);
    expect_mtip(1'b0, "mtime still in the low word");
    idle(40);
    expect_mtip(1'b1, "mtime carried into the high word");
    if (rises != before_rises + 1) fail("the carry case posted other than once");
    rd_check(2'd1, "mtime high after the carry");

    // The wrong order leaves a value at or below mtime, and the level posts on it.
    set_cmp(64'hffff_ffff_ffff_ffff);
    set_time(64'h0000_0001_0000_0050);
    set_cmp(64'h0000_0002_0000_0010);
    idle(3);
    expect_mtip(1'b0, "before the wrong-order update");
    wr(2'd3, 32'h0000_0001);
    idle(3);
    expect_mtip(1'b1, "after the high word written first");
    wr(2'd2, 32'hffff_fff0);
    idle(3);
    expect_mtip(1'b0, "after the second store repaired it");

    // The right order never passes through it.
    set_cmp(64'h0000_0002_0000_0010);
    set_time(64'h0000_0001_0000_0050);
    before_rises = rises;
    wr(2'd2, 32'hffff_ffff);
    wr(2'd3, 32'h0000_0001);
    idle(4);
    wr(2'd2, 32'hffff_fff0);
    idle(30);
    expect_mtip(1'b0, "after the update in spec order");
    if (rises != before_rises) fail("the spec's order posted a spurious level");

    // Byte strobes land only on their own lanes, in mtime and in mtimecmp.
    set_cmp(64'h1122_3344_5566_7788);
    bus_write(BASE + 32'd8, 32'h0000_00aa, 4'b0001);
    bus_write(BASE + 32'd12, 32'hbb00_0000, 4'b1000);
    rd_check(2'd2, "mtimecmp low after a byte store");
    rd_check(2'd3, "mtimecmp high after a byte store");
    bus_write(BASE + 32'd0, 32'h0000_cc00, 4'b0010);
    rd_check(2'd0, "mtime low after a byte store");

    // A write to either half suspends that cycle's tick, so no carry crosses it.
    set_cmp(64'hffff_ffff_ffff_ffff);
    wr(2'd1, 32'h0000_0000);
    @(negedge clk);
    mem_addr = BASE; mem_wdata = 32'hffff_ffff; mem_wstrb = 4'hf;
    @(negedge clk);
    mem_wdata = 32'hffff_fff0;
    @(negedge clk);
    mem_addr = 32'b0; mem_wdata = 32'b0; mem_wstrb = 4'b0;
    if (m_time[63:32] !== 32'h0) fail("the model carried across a write");
    rd_check(2'd1, "mtime high after a low write on the carry edge");
    rd_check(2'd0, "mtime low after the same write");

    // mtime is mcycle: a CSR write moves what the timer window reads and what mtip compares, and a store moves mcycle.
    csr_write(64'h0000_0007_ffff_ff00);
    rd_check(2'd1, "mtime high after a CSR write to mcycle");
    rd_check(2'd0, "mtime low after a CSR write to mcycle");
    if (mc[63:32] < 32'd7) fail("mcycle did not take the CSR write");
    set_cmp(64'h0000_0008_0000_0010);
    idle(3);
    expect_mtip(1'b0, "mcycle below mtimecmp");
    csr_write(64'h0000_0009_0000_0000);
    idle(3);
    expect_mtip(1'b1, "after a CSR write put mcycle past mtimecmp");
    set_time(64'h0000_0001_0000_0000);
    if (mc[63:32] !== 32'd1) fail("a store to mtime did not move mcycle");
    idle(3);
    expect_mtip(1'b0, "after a store to mtime pulled mcycle back");

    bus_write(BASE + 32'd16, 32'hdead_beef, 4'hf);
    bus_write(BASE - 32'd4, 32'hdead_beef, 4'hf);
    rd_check(2'd2, "mtimecmp low after stray stores");
    rd_check(2'd3, "mtimecmp high after stray stores");
    @(negedge clk);
    mem_addr = BASE + 32'd16;
    #1;
    if (mem_rdata !== 32'b0) fail("a read past the window returned data");
    mem_addr = 32'b0;

    if (rises < 3 || falls < 3) fail($sformatf("only %0d rises and %0d falls were exercised", rises, falls));

    if (errors == 0) begin
      $display("PASS");
    end else begin
      $display("FAIL %0d checks", errors);
    end
    $finish;
  end

  initial begin
    #2_000_000;
    $display("FAIL the bench did not finish");
    $finish;
  end
endmodule
