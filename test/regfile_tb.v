`timescale 1 ns / 1 ps
`default_nettype none

// rtl/regfile.v's contract. The read
// is registered, so an operand takes two cycles to obtain and the two are not
// interchangeable:
//
//   FETCH cycle  -- rs1/rs2 are presented. The posedge that ends it captures
//                   the array contents at those addresses, write-first, so a
//                   write presented in THIS cycle is captured too.
//   USE cycle    -- rs1/rs2 are held unchanged. reg_rs1/reg_rs2 are valid, and
//                   the write-through bypass covers a write presented now.
//
// Those two forwarding points together make the operand the cycle-N
// architectural value including a cycle-N writeback. rtl/decoder.v's
// `operand_stall` supplies the fetch cycle and holds the PC so the addresses
// cannot move; every vector below samples where a real consumer does.
//
// The read mux keys off a registered copy of the address pair, so it answers
// the fetch cycle's addresses in both cycles. Holding the pair is what makes
// those the same addresses.
module regfile_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic [4:0]  rs1, rs2;
  logic [31:0] reg_rs1, reg_rs2;
  logic        wen;
  logic [4:0]  waddr;
  logic [31:0] wdata;

  regfile dut (
    .clk(clk),
    .rs1(rs1),
    .rs2(rs2),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
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

  task automatic check_ne(input string what, input logic [31:0] got, input logic [31:0] unwanted);
    begin
      if (got === unwanted) begin
        $display("MISMATCH %s: got=%08x, which must not have landed", what, got);
        errors++;
      end
    end
  endtask

  // Nothing downstream would notice the two arrays drifting apart: test/cosim.cc
  // reads `regs_a` alone, and `reg_rs2` is the only consumer of `regs_b`. So
  // assert the duplication directly rather than trusting the write block by
  // inspection.
  task automatic check_mirrors(input string what);
    begin
      for (int i = 0; i < 32; i++) begin
        if (dut.regs_a[i] !== dut.regs_b[i]) begin
          $display("MISMATCH %s: regs_a[%0d]=%08x regs_b[%0d]=%08x", what, i,
                   dut.regs_a[i], i, dut.regs_b[i]);
          errors++;
        end
      end
    end
  endtask

  // Open a cycle and drive it, settling just after the posedge the way decode's
  // combinational outputs do.
  task automatic drive(input logic [4:0] a1, input logic [4:0] a2,
                       input logic we, input logic [4:0] wa, input logic [31:0] wd);
    begin
      @(posedge clk);
      #1;
      rs1   = a1;
      rs2   = a2;
      wen   = we;
      waddr = wa;
      wdata = wd;
      // Let the read mux settle before the caller samples. Without this the
      // check races the `always_comb` and reads the previous cycle's operand,
      // which looks exactly like a broken bypass.
      #1;
    end
  endtask

  // A plain write cycle. The addresses park on x0, which reads 0
  // unconditionally and can never be written, so no read is in flight.
  task automatic write_cycle(input logic [4:0] wa, input logic [31:0] wd);
    begin
      drive(5'd0, 5'd0, 1'b1, wa, wd);
    end
  endtask

  initial begin
    rs1   = 5'd0;
    rs2   = 5'd0;
    wen   = 1'b0;
    waddr = 5'd0;
    wdata = 32'b0;

    // Seed three registers, one write per cycle, so the reads below have known
    // array contents that did not arrive through either forwarding path.
    write_cycle(5'd5, 32'hcafef00d);
    write_cycle(5'd6, 32'h22222222);
    write_cycle(5'd7, 32'h33333333);

    // The plain case: fetch, then use, nothing being written.
    drive(5'd5, 5'd6, 1'b0, 5'd0, 32'h0);   // fetch
    drive(5'd5, 5'd6, 1'b0, 5'd0, 32'h0);   // use, addresses held
    check_hex("registered read of the array (rs1)", reg_rs1, 32'hcafef00d);
    check_hex("registered read of the array (rs2)", reg_rs2, 32'h22222222);
    check_mirrors("arrays agree after the seed writes");

    // Write-first: the write lands in the FETCH cycle, on the same posedge that
    // captures the read. Without the write-first term in rtl/regfile.v the
    // captured word is the pre-write contents and the operand is exactly one
    // writeback stale -- a cycle earlier than the scoreboard can see, so the
    // stall-only interlock never fires on it.
    drive(5'd5, 5'd5, 1'b1, 5'd5, 32'h44444444);   // fetch, writing x5
    drive(5'd5, 5'd5, 1'b0, 5'd0, 32'h0);          // use, no write
    check_hex("write-first capture in the fetch cycle (rs1)", reg_rs1, 32'h44444444);
    check_hex("write-first capture in the fetch cycle (rs2)", reg_rs2, 32'h44444444);

    // Write-through bypass: the write lands in the USE cycle. Nothing has
    // reached the arrays yet, so the bypass mux is the only path that can
    // produce this value.
    drive(5'd6, 5'd6, 1'b0, 5'd0, 32'h0);          // fetch, no write
    drive(5'd6, 5'd6, 1'b1, 5'd6, 32'h55555555);   // use, writing x6
    check_hex("write-through bypass in the use cycle (rs1)", reg_rs1, 32'h55555555);
    check_hex("write-through bypass in the use cycle (rs2)", reg_rs2, 32'h55555555);

    // And it really did land, one cycle later, through the array rather than
    // through a forwarding path.
    drive(5'd6, 5'd6, 1'b0, 5'd0, 32'h0);
    drive(5'd6, 5'd6, 1'b0, 5'd0, 32'h0);
    check_hex("the bypassed write reached the array (rs1)", reg_rs1, 32'h55555555);
    check_mirrors("arrays agree after the forwarded writes");

    // The read register is keyed to the address presented in the FETCH cycle,
    // not to whatever rs1 holds during the use cycle. This is not a
    // behaviour to rely on -- it is why rtl/decoder.v must hold the PC across
    // the pair, and it is what breaks loudly if the read is made combinational
    // again without removing `operand_stall`.
    drive(5'd5, 5'd0, 1'b0, 5'd0, 32'h0);          // fetch x5
    drive(5'd6, 5'd0, 1'b0, 5'd0, 32'h0);          // use, but rs1 now points at x6
    check_hex("the read is keyed to the fetched address, not the current one",
              reg_rs1, 32'h44444444);
    check_ne("...and specifically is NOT x6's value", reg_rs1, 32'h55555555);

    // The bypass select is on the held address too. Point the port at another
    // register in the use cycle and write to the one that was fetched: the
    // written word still comes back. Select on the presented address and the
    // match is missed, so the array answers instead. Every other vector here
    // holds the pair and cannot tell the two spellings apart.
    drive(5'd0, 5'd5, 1'b0, 5'd0, 32'h0);          // fetch x5 on rs2
    drive(5'd0, 5'd6, 1'b1, 5'd5, 32'haaaaaaaa);   // use, rs2 now x6, writing x5
    check_hex("bypass keyed to the held address (rs2)", reg_rs2, 32'haaaaaaaa);

    drive(5'd5, 5'd0, 1'b0, 5'd0, 32'h0);          // fetch x5 on rs1
    drive(5'd6, 5'd0, 1'b1, 5'd5, 32'h99999999);   // use, rs1 now x6, writing x5
    check_hex("bypass keyed to the held address (rs1)", reg_rs1, 32'h99999999);

    // x0 reads 0 on both ports, even with waddr == 0 and wen == 1 aimed straight
    // at it. That is forced off the held address like every other read, so the
    // cycle that first presents x0 still answers the address held from the
    // vector above -- x6 -- and the zero arrives a cycle later. Nothing in the
    // core reads an operand in a fetch cycle, so this bench is the only place
    // that shows.
    drive(5'd0, 5'd0, 1'b1, 5'd0, 32'hdeadbeef);
    check_hex("presenting x0 still answers the held address (rs1)", reg_rs1, 32'h55555555);
    drive(5'd0, 5'd0, 1'b1, 5'd0, 32'hdeadbeef);
    check_hex("x0 reads 0 in the use cycle (rs1)", reg_rs1, 32'h00000000);
    check_hex("x0 reads 0 in the use cycle (rs2)", reg_rs2, 32'h00000000);

    // White box, because the read mux would hide a missing write guard
    // completely: the suppressed write must have reached neither array.
    drive(5'd0, 5'd0, 1'b0, 5'd0, 32'h0);
    check_ne("x0 write never reaches regs_a", dut.regs_a[0], 32'hdeadbeef);
    check_ne("x0 write never reaches regs_b", dut.regs_b[0], 32'hdeadbeef);

    // The bypass fires only on an address match.
    drive(5'd7, 5'd7, 1'b0, 5'd0, 32'h0);          // fetch x7
    drive(5'd7, 5'd7, 1'b1, 5'd5, 32'h66666666);   // use, writing an unrelated x5
    check_hex("bypass does not leak into an unrelated address (rs1)", reg_rs1, 32'h33333333);
    check_hex("bypass does not leak into an unrelated address (rs2)", reg_rs2, 32'h33333333);

    // Different addresses, one answered from its array and one from the bypass,
    // then the same with the roles swapped -- so a bypass wired to the wrong
    // array cannot pass both directions.
    drive(5'd7, 5'd6, 1'b0, 5'd0, 32'h0);
    drive(5'd7, 5'd6, 1'b1, 5'd6, 32'h77777777);
    check_hex("independent ports, array side (rs1)", reg_rs1, 32'h33333333);
    check_hex("independent ports, bypass side (rs2)", reg_rs2, 32'h77777777);

    drive(5'd6, 5'd7, 1'b0, 5'd0, 32'h0);
    drive(5'd6, 5'd7, 1'b1, 5'd6, 32'h88888888);
    check_hex("independent ports, bypass side (rs1)", reg_rs1, 32'h88888888);
    check_hex("independent ports, array side (rs2)", reg_rs2, 32'h33333333);

    drive(5'd5, 5'd6, 1'b0, 5'd0, 32'h0);
    drive(5'd5, 5'd6, 1'b0, 5'd0, 32'h0);
    check_hex("x5 holds its last written value", reg_rs1, 32'h66666666);
    check_hex("x6 holds its last written value", reg_rs2, 32'h88888888);
    check_mirrors("arrays agree at the end of the run");

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: regfile registered read / write-first / write-through bypass / x0 / regs_a == regs_b");
      $finish;
    end
  end
endmodule
