`timescale 1 ns / 1 ps
`default_nettype none

// rtl/pairtable.v's contract, which is entirely about WHEN each answer is
// available and never about whether it is right. A wrong pair costs the
// operand-fetch cycle decode was already paying, so nothing here can be graded
// against "the guess was correct".
//
// What can be wrong, and what these vectors are for:
//
//   - The entry has to arrive with the instruction it describes. It is read off
//     `next_pc` at one edge and consumed on the cycle `pc` reaches that value,
//     so a read addressed off `pc` instead would be a cycle late and would
//     answer for the previous instruction.
//   - The pair stored has to go against the PREVIOUS issue's address, not this
//     one's. Store it against this one and the table answers "what did I read",
//     which decode already knows.
//   - The tag has to turn a collision into a miss. Without it a second address
//     sharing the low bits reads the first one's pair, and the miss it should
//     have been becomes a wrong guess against a real pair.
//   - An unwritten entry must read as a defined value. On the part an EBR with
//     no INIT is zeros; a simulation whose array started undefined would push
//     an X into the register file's address port, and only the two-state runner
//     would stay green.
module pairtable_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic        reset;
  logic [31:0] next_pc, pc;
  logic        issuing;
  logic [4:0]  rs1, rs2;
  logic        hit;
  logic [4:0]  guess_rs1, guess_rs2;

  pairtable dut (
    .clk(clk),
    .reset(reset),
    .next_pc(next_pc),
    .issuing(issuing),
    .pc(pc),
    .rs1(rs1),
    .rs2(rs2),
    .hit(hit),
    .guess_rs1(guess_rs1),
    .guess_rs2(guess_rs2)
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

  task automatic check_num(input string what, input logic [4:0] got, input logic [4:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=x%0d expected=x%0d", what, got, expected);
        errors++;
      end
    end
  endtask

  // One issuing cycle at `at`, reading `a1`/`a2`, with the fetch about to move
  // to `to`. Settles just after the edge the way the decoder's combinational
  // outputs do.
  task automatic issue(input logic [31:0] at, input logic [31:0] to,
                       input logic [4:0] a1, input logic [4:0] a2);
    begin
      @(posedge clk);
      #1;
      pc      = at;
      next_pc = to;
      rs1     = a1;
      rs2     = a2;
      issuing = 1'b1;
      #1;
    end
  endtask

  // A cycle that consumes nothing: the pc holds and no entry is written.
  task automatic stalled(input logic [31:0] at);
    begin
      @(posedge clk);
      #1;
      pc      = at;
      next_pc = at;
      issuing = 1'b0;
      #1;
    end
  endtask

  initial begin
    reset   = 1;
    pc      = 32'h0000_0000;
    next_pc = 32'h0000_0000;
    rs1     = 5'd0;
    rs2     = 5'd0;
    issuing = 1'b0;
    repeat (2) @(posedge clk);
    #1;
    reset = 0;
    #1;

    // An entry nobody has written. The value is what the bitstream puts in an
    // EBR, and the point is that it is a value at all.
    check_num("an unwritten entry reads a defined rs1", guess_rs1, 5'd0);
    check_num("...and a defined rs2", guess_rs2, 5'd0);

    // Walk a straight line: 0x100 reading x1/x2, then 0x104 reading x3/x4. The
    // entry for 0x100 must end up holding x3/x4 -- the pair of what followed it
    // -- and it is written on the cycle 0x104 issues, not before.
    issue(32'h0000_0100, 32'h0000_0104, 5'd1, 5'd2);
    issue(32'h0000_0104, 32'h0000_0108, 5'd3, 5'd4);
    issue(32'h0000_0108, 32'h0000_0100, 5'd5, 5'd6);

    // Back at 0x100. The read was addressed off next_pc on the previous edge,
    // so the entry is here now.
    issue(32'h0000_0100, 32'h0000_0104, 5'd1, 5'd2);
    check_bit("the entry for a revisited address hits", hit, 1'b1);
    check_num("...and holds the successor's rs1", guess_rs1, 5'd3);
    check_num("...and its rs2", guess_rs2, 5'd4);

    // Not the pair the instruction at 0x100 itself reads. Storing against this
    // cycle's address instead of the previous one is the easy way to build
    // this, and it answers a question decode did not ask.
    if (guess_rs1 === 5'd1 && guess_rs2 === 5'd2) begin
      $display("MISMATCH the entry holds the address's OWN pair, not its successor's");
      errors++;
    end

    // A stalled cycle re-presents the same address and writes nothing, so the
    // entry survives it unchanged.
    stalled(32'h0000_0104);
    stalled(32'h0000_0104);
    next_pc = 32'h0000_0100;
    @(posedge clk);
    #1;
    issuing = 1'b0;
    #1;
    check_bit("a stalled cycle leaves the table alone", hit, 1'b1);
    check_num("...with the entry intact", guess_rs1, 5'd3);

    // An entry written this cycle is not visible to a read addressed this
    // cycle. Both ports of an EBR are synchronous and the read is read-first,
    // so a two-instruction loop -- where the address about to be fetched is the
    // one being written -- misses on its second pass and hits on its third.
    // Stated here because the alternative is to discover it as a vector that
    // fails for a reason nobody wrote down; it costs a guess, never a value.
    issue(32'h0000_0700, 32'h0000_0704, 5'd17, 5'd18);
    issue(32'h0000_0704, 32'h0000_0700, 5'd19, 5'd20);
    issue(32'h0000_0700, 32'h0000_0704, 5'd17, 5'd18);
    check_bit("an entry written on the cycle it is read is not seen yet", hit, 1'b0);
    issue(32'h0000_0704, 32'h0000_0700, 5'd19, 5'd20);
    issue(32'h0000_0700, 32'h0000_0704, 5'd17, 5'd18);
    check_bit("...and is seen on the pass after that", hit, 1'b1);
    check_num("...with the successor's pair", guess_rs1, 5'd19);

    // Same index, different tag: 0x100 and 0x100 + 512. The index is eight bits
    // starting at bit 1, so these two collide there and are separated only by
    // the tag. A tagless table would answer x3/x4 here.
    issue(32'h0000_0300, 32'h0000_0304, 5'd7, 5'd8);
    issue(32'h0000_0304, 32'h0000_0308, 5'd9, 5'd10);
    issue(32'h0000_0308, 32'h0000_0300, 5'd21, 5'd22);
    issue(32'h0000_0300, 32'h0000_0304, 5'd7, 5'd8);
    check_bit("the aliasing address hits its own entry", hit, 1'b1);
    check_num("...with its own successor's pair", guess_rs1, 5'd9);

    // ...and the tag makes the other one a miss rather than a wrong answer,
    // because 0x300's entry has overwritten 0x100's at the shared index. The
    // branch back to 0x100 needs its own issuing cycle: the read is addressed
    // off `next_pc`, so an address nothing was about to fetch is never read.
    issue(32'h0000_0304, 32'h0000_0100, 5'd9, 5'd10);
    issue(32'h0000_0100, 32'h0000_0104, 5'd1, 5'd2);
    check_bit("the evicted address misses instead of reading its neighbour's pair",
              hit, 1'b0);

    // A two-byte-aligned address is a different entry from the word-aligned one
    // beside it. Indexing from bit 2 would alias every compressed instruction
    // with the one before it.
    issue(32'h0000_0500, 32'h0000_0502, 5'd11, 5'd12);
    issue(32'h0000_0502, 32'h0000_0504, 5'd13, 5'd14);
    issue(32'h0000_0504, 32'h0000_0500, 5'd15, 5'd16);
    issue(32'h0000_0500, 32'h0000_0502, 5'd11, 5'd12);
    check_bit("a compressed pair does not share one entry (hit)", hit, 1'b1);
    check_num("...and the halfword address keeps its own successor",
              guess_rs1, 5'd13);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: pairtable entry timing, successor storage, tag and halfword index");
      $finish;
    end
  end
endmodule
