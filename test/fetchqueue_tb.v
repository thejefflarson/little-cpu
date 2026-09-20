`timescale 1 ns / 1 ps
`default_nettype none

// rtl/fetchqueue.v's contract: occupancy, per-word fault, flush discard, retry, and room.
module fetchqueue_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic         reset, flush, req_valid, req_half, imem_fault, pop;
  logic [31:0]  imem_data, imem_data2;
  logic [31:0]  q0, q1;
  logic         q0_fault, q1_fault, q_valid, room;
  logic [2:0]   count;

  fetchqueue dut (
    .clk(clk),
    .reset(reset),
    .flush(flush),
    .req_valid(req_valid),
    .req_half(req_half),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .imem_fault(imem_fault),
    .pop(pop),
    .q0(q0),
    .q0_fault(q0_fault),
    .q1(q1),
    .q1_fault(q1_fault),
    .q_valid(q_valid),
    .count(count),
    .room(room)
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

  task automatic check_hex(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_count(input string what, input logic [2:0] got, input logic [2:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%0d expected=%0d", what, got, expected);
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

  task automatic cycle(input logic [31:0] lo, input logic [31:0] hi, input logic flt,
                       input logic rv, input logic fl, input logic pp);
    begin
      imem_data  = lo;
      imem_data2 = hi;
      imem_fault = flt;
      req_valid  = rv;
      flush      = fl;
      pop        = pp;
      step();
    end
  endtask

  // Changes req_valid without crossing a clock edge, so `room` still reflects this
  // cycle's own request rather than the settled state a `cycle()` call leaves behind.
  task automatic set_req_valid(input logic rv);
    begin
      req_valid = rv;
      #1;
    end
  endtask

  initial begin
    reset      = 1'b1;
    flush      = 1'b0;
    req_valid  = 1'b0;
    req_half   = 1'b0;
    imem_data  = 32'b0;
    imem_data2 = 32'b0;
    imem_fault = 1'b0;
    pop        = 1'b0;
    step();
    reset = 1'b0;

    // --- Push and pop across every occupancy, including full and empty. ---
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    check_count("empty right after reset", count, 3'd0);
    check_bit("q_valid empty right after reset", q_valid, 1'b0);

    cycle(32'hAAAA_0000, 32'hAAAA_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    check_count("the first request's pair landed", count, 3'd2);
    check_bit("q_valid with one pair queued", q_valid, 1'b1);
    check_hex("q0 is the low word of the first pair", q0, 32'hAAAA_0000);
    check_hex("q1 is the high word of the first pair", q1, 32'hAAAA_0004);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    check_count("the second request's pair not due yet", count, 3'd2);

    cycle(32'hBBBB_0000, 32'hBBBB_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    check_count("full: two pairs queued", count, 3'd4);
    check_bit("q_valid while full", q_valid, 1'b1);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("one word popped off a full queue", count, 3'd3);
    check_hex("q0 advanced to the first pair's high word", q0, 32'hAAAA_0004);
    check_hex("q1 advanced to the second pair's low word", q1, 32'hBBBB_0000);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("popped down to one word", count, 3'd1);
    check_bit("q_valid drops below a full pair", q_valid, 1'b0);
    check_hex("q0 is the second pair's high word, the last one left", q0, 32'hBBBB_0004);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("popped to empty", count, 3'd0);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("popping an empty queue is a no-op", count, 3'd0);

    // --- The fault bit travels with its own word, not as one register for the queue. ---
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    cycle(32'hFA55_0000, 32'hFA55_0004, 1'b1, 1'b1, 1'b0, 1'b0);
    check_bit("q0_fault set on the faulting pair's low word", q0_fault, 1'b1);
    check_bit("q1_fault set on the faulting pair's high word", q1_fault, 1'b1);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    cycle(32'hCAFE_0000, 32'hCAFE_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    check_count("both pairs queued for the fault-travel scenario", count, 3'd4);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_bit("q0_fault clear once the faulting pair is fully popped away", q0_fault, 1'b0);
    check_bit("q1_fault stays clear on the clean pair's high word", q1_fault, 1'b0);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("drained before the retry scenario", count, 3'd0);

    // --- Same request re-issued after a stolen cycle arrives exactly once. ---
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    check_count("launched: no response is due yet", count, 3'd0);

    cycle(32'hDEAD_BEEF, 32'hDEAD_BEEF, 1'b0, 1'b0, 1'b0, 1'b0);
    check_count("stolen cycle: the garbage on the bus must not be pushed", count, 3'd0);

    cycle(32'hFEED_0000, 32'hFEED_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    check_count("the retried request's real response lands exactly once", count, 3'd2);
    check_hex("q0 is the retried request's own word", q0, 32'hFEED_0000);
    check_hex("q1 is its high word", q1, 32'hFEED_0004);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    check_count("no duplicate push follows the retried response", count, 3'd2);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("drained before the flush scenario", count, 3'd0);

    // --- A flush discards the response of the request already in flight. ---
    cycle(32'hdead_dead, 32'hdead_dead, 1'b0, 1'b1, 1'b1, 1'b0);
    check_count("flush wins over a response due the same cycle: nothing lands", count, 3'd0);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    check_count("the settling cycle: the overtaken request's reply never landed", count, 3'd0);

    cycle(32'hFEED_0000, 32'hFEED_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    check_count("the redirect target's own request lands instead", count, 3'd2);
    check_hex("q0 is the redirect target's own word, never the stale one", q0, 32'hFEED_0000);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("drained before the back-to-back scenario", count, 3'd0);

    // --- Two flushes one settling cycle apart. ---
    cycle(32'hbeef_beef, 32'hbeef_beef, 1'b0, 1'b1, 1'b1, 1'b0);   // flush #1
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);                   // the one settling cycle
    check_count("settling on the first target: nothing queued yet", count, 3'd0);

    cycle(32'hfeed_face, 32'hfeed_face, 1'b0, 1'b1, 1'b1, 1'b0);   // flush #2
    check_count("flush #2 discards the first target's own reply too", count, 3'd0);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    check_count("settling on the second target: still nothing stale queued", count, 3'd0);

    // --- A half push queues the low word alone; the next pair lands right behind it. ---
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b0);
    check_count("drained before the half-push scenario", count, 3'd0);
    req_half = 1'b1;
    cycle(32'h1111_0000, 32'h1111_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    req_half = 1'b0;
    check_count("a half push lands one word", count, 3'd1);
    check_bit("q_valid stays low on a lone word", q_valid, 1'b0);
    cycle(32'h2222_0000, 32'h2222_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    check_count("the target's pair lands behind the half word", count, 3'd3);
    check_hex("q0 is the half-pushed word", q0, 32'h1111_0000);
    check_hex("q1 is the target pair's low word, not the dropped high word", q1, 32'h2222_0000);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_hex("after one pop q0 is the target pair's low word", q0, 32'h2222_0000);
    check_hex("and q1 its high word", q1, 32'h2222_0004);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("drained after the half-push scenario", count, 3'd0);

    cycle(32'hC0FF_0000, 32'hC0FF_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    check_count("the second target's own request lands", count, 3'd2);
    check_hex("q0 is the second target's word, never the first's or the overtaken one's",
              q0, 32'hC0FF_0000);

    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    cycle(32'h0, 32'h0, 1'b0, 1'b0, 1'b0, 1'b1);
    check_count("drained before the room-check scenario", count, 3'd0);

    // --- room reserves for the response landing this cycle. ---
    cycle(32'h5678_0000, 32'h5678_0004, 1'b0, 1'b1, 1'b0, 1'b0);
    set_req_valid(1'b0);
    check_count("one pair queued for the room check", count, 3'd2);
    check_bit("no response due: room stays open", room, 1'b1);

    set_req_valid(1'b1);
    check_bit("a response due this cycle, on top of one queued pair: no room left",
              room, 1'b0);
    set_req_valid(1'b0);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: fetchqueue push/pop across occupancy, per-word fault, retry, flush discard, back-to-back flush, room check");
      $finish;
    end
  end
endmodule
