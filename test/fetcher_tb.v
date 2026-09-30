`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// rtl/fetcher.v over a ROM that answers a cycle after `imem_addr_next`, with a scripted decode
// that moves `pc` wherever it likes each issue: the window at `pc` must always be there.
module fetcher_tb;
  localparam int ROM_WORDS = 64;
  localparam int FAULT_WORD = 32;

  logic clk = 0;
  always #5 clk = ~clk;

  logic        reset = 1'b1;
  logic [31:0] pc = 32'b0, next_pc, target = 32'b0;
  logic        hold_now = 1'b0, steal = 1'b0, issuing;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next, imem_data, imem_data2;
  logic        imem_stall, imem_fault, fetch_stall, fault;
  fetcher_output out;

  fetcher dut (
    .clk(clk),
    .reset(reset),
    .pc(pc),
    .next_pc(next_pc),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .imem_stall(imem_stall),
    .imem_fault(imem_fault),
    .fetch_stall(fetch_stall),
    .fault(fault),
    .out(out)
  );

  logic [31:0] rom[0:ROM_WORDS-1];
  logic [5:0]  rom_word;
  assign rom_word = imem_addr_next[7:2];
  always_ff @(posedge clk) begin
    imem_data  <= steal ? 32'hdead_beef : rom[rom_word];
    imem_data2 <= steal ? 32'hfeed_face : rom[rom_word + 6'd1];
    imem_fault <= rom_word >= FAULT_WORD;
    imem_stall <= steal;
  end

  assign issuing = !reset && !fetch_stall && !hold_now;
  assign next_pc = reset ? 32'b0 : issuing ? target : pc;
  always_ff @(posedge clk) pc <= next_pc;

  logic [31:0] past_addr_next;
  always_ff @(posedge clk) past_addr_next <= imem_addr_next;

  int errors = 0;

  task automatic check(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_window(input string what);
    logic [63:0] pair;
    logic [31:0] want_instr;
    begin
      pair = {rom[pc[7:2] + 6'd1], rom[pc[7:2]]} >> (pc[1] ? 16 : 0);
      want_instr = pair[31:0];
      check({what, ": instr at pc"}, out.instr, want_instr);
      check({what, ": out.pc"}, out.pc, pc);
      check({what, ": fault"}, {31'b0, fault}, {31'b0, pc[7:2] >= FAULT_WORD});
    end
  endtask

  // One instruction at `at`: wait out any miss, hold `holds` cycles, then issue toward `to`.
  // `steal_issue` takes the ROM's port on the issuing edge (the next window is lost);
  // `steal_hold` takes it on the first held edge (the held cycles re-read it, so it needs two).
  task automatic step(input logic [31:0] at, input logic [31:0] to, input int holds,
                      input int want_stalls, input logic steal_issue, input logic steal_hold);
    int stalls, h;
    string what;
    begin
      #1;
      stalls = 0;
      h = holds;
      what = $sformatf("pc %02x -> %02x", at, to);
      check({what, ": decode is where the script says"}, pc, at);
      while (fetch_stall) begin
        stalls++;
        hold_now = 1'b0;
        @(posedge clk);
        #1;
      end
      check({what, ": miss cycles before it"}, stalls, want_stalls);
      while (h > 0) begin
        if (!fetch_stall) check_window({what, " (held)"});
        hold_now = 1'b1;
        steal = steal_hold && h == holds;
        @(posedge clk);
        #1;
        steal = 1'b0;
        h--;
      end
      check({what, ": a stolen held window is re-read before it issues"}, {31'b0, fetch_stall}, 32'b0);
      check_window(what);
      hold_now = 1'b0;
      target = to;
      steal = steal_issue;
      @(posedge clk);
      #1;
      steal = 1'b0;
    end
  endtask

  always @(posedge clk) begin
    #1;
    if (!reset) check("the ROM's address is last cycle's fetch address", imem_addr, past_addr_next);
  end

  initial begin
    for (int i = 0; i < ROM_WORDS; i++) rom[i] = 32'b0;
    rom[0]  = 32'h0010_0093;  // addi x1, x0, 1
    rom[1]  = 32'h0085_0001;  // c.nop; c.addi x1, 1
    rom[2]  = 32'h0070_0193;  // addi x3, x0, 7
    rom[3]  = 32'h0213_0001;  // c.nop; the low half of addi x4, x0, 9
    rom[4]  = 32'h0001_0090;  // its high half; c.nop
    rom[5]  = 32'h0012_8293;  // addi x5, x5, 1
    rom[6]  = 32'hfe62_9ee3;  // bne x5, x6, -4  (backward)
    rom[7]  = 32'h0000_0463;  // beq x0, x0, +8  (forward)
    rom[8]  = 32'h0038_0393;  // addi x7, x0, 3
    rom[9]  = 32'h3fdd_0001;  // c.nop; c.j -10
    rom[10] = 32'h0000_0013;
    rom[11] = 32'h0000_0013;
    rom[12] = 32'h0010_0093;
    rom[13] = 32'h0000_0013;
    rom[14] = 32'h0020_0113;
    rom[15] = 32'h0030_0193;
    rom[16] = 32'h0040_0213;
    rom[17] = 32'h0050_0293;
    rom[18] = 32'h0060_0313;

    repeat (2) @(posedge clk);
    #1;
    reset = 1'b0;

    step(32'h00, 32'h04, 0, 0, 0, 0);
    step(32'h04, 32'h06, 0, 0, 0, 0);
    step(32'h06, 32'h08, 0, 0, 0, 0);
    step(32'h08, 32'h0c, 0, 0, 0, 0);
    step(32'h0c, 32'h0e, 0, 0, 0, 0);
    step(32'h0e, 32'h12, 0, 0, 0, 0);   // straddles words 3 and 4
    step(32'h12, 32'h14, 0, 0, 0, 0);
    step(32'h14, 32'h18, 0, 0, 0, 0);

    // A taken redirect is answered by the address alone: no miss for the target.
    step(32'h18, 32'h00, 0, 0, 0, 0);
    step(32'h00, 32'h0e, 0, 0, 0, 0);   // a far jump onto a straddle
    step(32'h0e, 32'h04, 0, 0, 0, 0);   // and straight off it, into the second word
    step(32'h04, 32'h0e, 0, 0, 0, 0);   // back onto the straddle, one word earlier
    step(32'h0e, 32'h12, 3, 0, 0, 0);   // decode holds three cycles on the straddle
    step(32'h12, 32'h14, 0, 0, 0, 0);
    step(32'h14, 32'h0a, 0, 0, 0, 0);   // onto a halfword-aligned word 2 upper half
    step(32'h0a, 32'h0e, 0, 0, 0, 0);

    // A stolen read costs one miss and is re-read; stolen while held, the hold covers the re-read.
    step(32'h0e, 32'h12, 0, 0, 1, 0);   // the window at 12 is stolen on the issuing edge
    step(32'h12, 32'h14, 0, 1, 0, 0);
    step(32'h14, 32'h18, 2, 0, 0, 1);   // stolen while decode holds: the second held cycle re-reads it
    step(32'h18, 32'h0e, 0, 0, 1, 0);   // a stolen redirect target, on a straddle
    step(32'h0e, 32'h12, 0, 1, 0, 0);   // one miss, and the straddle still reads whole

    // A faulting word arrives with its flag at no cost.
    step(32'h12, 32'h80, 0, 0, 0, 0);
    step(32'h80, 32'h00, 0, 0, 0, 0);
    step(32'h00, 32'h04, 0, 0, 0, 0);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: fetcher.v window at pc for sequential, straddling, redirected, stolen and faulting fetches");
      $finish;
    end
  end
endmodule
