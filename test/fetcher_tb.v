`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// rtl/fetcher.v over a ROM that answers a cycle after `imem_addr_next`, with a scripted decode
// that holds, redirects and refetches at random: whenever it issues, the instruction at `pc`
// must be the ROM's, whatever stole, stalled, flushed or straddled on the way there.
module fetcher_tb;
  localparam int ROM_WORDS = 64;
  localparam int FAULT_WORD = 48;

  logic clk = 0;
  always #5 clk = ~clk;

  logic        reset = 1'b1;
  logic [31:0] pc = 32'b0;
  logic        issuing, refetch, redirect = 1'b0;
  logic [31:0] redirect_target = 32'b0, refetch_target = 32'b0;
  logic        hold = 1'b0, want_refetch = 1'b0, steal = 1'b0, restart = 1'b0;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next, imem_data;
  logic        imem_stall, imem_fault, fetch_stall, fault;
  fetcher_output out;

  fetcher dut (
    .clk(clk),
    .reset(reset),
    .pc(pc),
    .issuing(issuing),
    .refetch(refetch),
    .redirect(redirect),
    .redirect_target(redirect_target),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(32'b0),
    .imem_addr_next(imem_addr_next),
    .imem_stall(imem_stall),
    .imem_fault(imem_fault),
    .fetch_stall(fetch_stall),
    .fault(fault),
    .out(out)
  );

  logic [31:0] rom[0:ROM_WORDS-1];
  function automatic logic [31:0] word(input logic [31:0] w);
    word = w < FAULT_WORD ? rom[w[5:0]] : 32'b0;
  endfunction

  logic [31:0] addr_word;
  assign addr_word = imem_addr_next[31:2];
  always_ff @(posedge clk) begin
    imem_data  <= steal ? 32'hdead_beef : word(addr_word);
    imem_fault <= addr_word >= FAULT_WORD;
    imem_stall <= steal;
  end

  assign issuing = !reset && !fetch_stall && !hold;
  assign refetch = issuing && want_refetch;

  int steal_pct = 0, hold_pct = 0, redirect_pct = 0, refetch_pct = 0;
  int errors = 0, issues = 0, straddles = 0, retries = 0, x_flushes = 0, d_flushes = 0;
  int faults = 0, rewrites = 0;

  task automatic check(input string what, input logic [31:0] got, input logic [31:0] expected);
    if (got !== expected) begin
      $display("MISMATCH %s: pc=%08x got=%08x expected=%08x", what, pc, got, expected);
      errors++;
    end
  endtask

  function automatic logic chance(input int pct);
    chance = ($urandom % 100) < pct;
  endfunction

  function automatic logic [31:0] any_pc();
    any_pc = ($urandom % (ROM_WORDS * 2)) << 1;
  endfunction

  // What the ROM holds at `pc`, read procedurally: iverilog derives a continuous assign's
  // sensitivity from a function's arguments, and the ROM is not one of them.
  logic        unc, checking = 1'b1;
  logic [31:0] wpc, want, len;
  logic [63:0] pair;

  always @(posedge clk) begin
    if (!reset) begin
      wpc  = pc[31:2];
      pair = {word(wpc + 32'd1), word(wpc)} >> (pc[1] ? 16 : 0);
      want = pair[31:0];
      unc  = want[1:0] == 2'b11;
      len  = unc ? 32'd4 : 32'd2;
      if (dut.retry) retries++;
      if (dut.redirect) x_flushes++;
      if (dut.refetch_q) d_flushes++;
      if (dut.flush) check("a flush cycle offers no instruction", {31'b0, !fetch_stall}, 32'b0);
      if (issuing && checking) begin
        issues++;
        if (pc[1] && unc) straddles++;
        if (fault) faults++;
        check("pc", out.pc, pc);
        check("instr", unc ? out.instr : {16'b0, out.instr[15:0]},
                       unc ? want : {16'b0, want[15:0]});
        check("fault", {31'b0, fault},
              {31'b0, wpc >= FAULT_WORD || (pc[1] && unc && wpc + 1 >= FAULT_WORD)});
      end
      // A refetch is how `fence.i` and a taken guess reach the fetcher: what it lands on is
      // read after this edge, so a rewrite here must show through and a stale word must not.
      if (refetch && chance(50)) begin
        rom[refetch_target[7:2]] = $urandom;
        rewrites++;
      end
      pc <= redirect ? redirect_target : issuing ? (refetch ? refetch_target : pc + len) : pc;
      redirect        <= restart || chance(redirect_pct);
      redirect_target <= restart ? 32'b0 : any_pc();
      refetch_target  <= any_pc();
      want_refetch    <= chance(refetch_pct);
      hold            <= chance(hold_pct);
      steal           <= chance(steal_pct);
    end
  end

  task automatic phase(input string name, input int cycles, input int steal_p, input int hold_p,
                       input int redirect_p, input int refetch_p);
    int base_issues;
    begin
      steal_pct = steal_p; hold_pct = hold_p; redirect_pct = redirect_p; refetch_pct = refetch_p;
      restart = 1'b1;
      @(posedge clk);
      restart = 1'b0;
      repeat (4) @(posedge clk);
      base_issues = issues;
      repeat (cycles) @(posedge clk);
      $display("phase %-28s %5d cycles, %5d issues", name, cycles, issues - base_issues);
    end
  endtask

  task automatic throughput(input string name, input int cycles, input int min_issues);
    int base_issues;
    begin
      steal_pct = 0; hold_pct = 0; redirect_pct = 0; refetch_pct = 0;
      restart = 1'b1;
      @(posedge clk);
      restart = 1'b0;
      repeat (10) @(posedge clk);
      base_issues = issues;
      repeat (cycles) @(posedge clk);
      if (issues - base_issues < min_issues) begin
        $display("MISMATCH %s: %0d issues in %0d cycles, wanted at least %0d", name,
                 issues - base_issues, cycles, min_issues);
        errors++;
      end
    end
  endtask

  // New contents while words are queued would fail for the right reason, so look away and resync.
  task automatic fill(input int compressed_pct);
    checking = 1'b0;
    for (int i = 0; i < ROM_WORDS; i++) begin
      logic [15:0] lo, hi;
      lo = $urandom; hi = $urandom;
      lo[1:0] = chance(compressed_pct) ? 2'b01 : 2'b11;
      hi[1:0] = chance(compressed_pct) ? 2'b10 : 2'b11;
      rom[i] = {hi, lo};
    end
    restart = 1'b1;
    @(posedge clk);
    restart = 1'b0;
    repeat (4) @(posedge clk);
    checking = 1'b1;
  endtask

  initial begin
    fill(50);
    repeat (3) @(posedge clk);
    #1 reset = 1'b0;
    fill(0);   throughput("all 32-bit, straight line", 60, 58);
    fill(100); throughput("all 16-bit, straight line", 60, 58);
    fill(50);  throughput("mixed lengths, straight line", 60, 40);
    phase("hold only", 400, 0, 40, 0, 0);
    phase("steal only", 400, 30, 0, 0, 0);
    phase("steal and hold", 400, 30, 30, 0, 0);
    phase("x redirects", 600, 0, 0, 10, 0);
    phase("refetches and rom rewrites", 600, 0, 0, 0, 15);
    phase("everything", 4000, 25, 25, 8, 12);
    fill(100); phase("everything, all 16-bit", 1500, 25, 25, 8, 12);
    fill(0);   phase("everything, all 32-bit", 1500, 25, 25, 8, 12);

    if (straddles < 50) begin $display("MISMATCH too few straddles: %0d", straddles); errors++; end
    if (retries < 50) begin $display("MISMATCH too few retries: %0d", retries); errors++; end
    if (x_flushes < 50) begin $display("MISMATCH too few x flushes: %0d", x_flushes); errors++; end
    if (d_flushes < 50) begin $display("MISMATCH too few d flushes: %0d", d_flushes); errors++; end
    if (faults < 10) begin $display("MISMATCH too few faults: %0d", faults); errors++; end
    if (rewrites < 20) begin $display("MISMATCH too few rewrites: %0d", rewrites); errors++; end
    $display("issues=%0d straddles=%0d retries=%0d x_flushes=%0d d_flushes=%0d faults=%0d rewrites=%0d",
             issues, straddles, retries, x_flushes, d_flushes, faults, rewrites);
    if (errors == 0) $display("PASSED: fetcher");
    else $display("FAILED: fetcher, %0d errors", errors);
    $finish;
  end
endmodule
