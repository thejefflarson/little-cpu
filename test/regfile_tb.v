`timescale 1 ns / 1 ps
`default_nettype none

// rtl/regfile.v's contract.
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

  // Nothing downstream would notice the two arrays drifting apart: test/cosim.cc reads
  // `regs_a` alone, and `reg_rs2` is the only consumer of `regs_b`.
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
      // Let the read mux settle before the caller samples.
      #1;
    end
  endtask

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

    write_cycle(5'd5, 32'hcafef00d);
    write_cycle(5'd6, 32'h22222222);
    write_cycle(5'd7, 32'h33333333);

    // The plain case: fetch, then use, nothing being written.
    drive(5'd5, 5'd6, 1'b0, 5'd0, 32'h0);   // fetch
    drive(5'd5, 5'd6, 1'b0, 5'd0, 32'h0);   // use, addresses held
    check_hex("registered read of the array (rs1)", reg_rs1, 32'hcafef00d);
    check_hex("registered read of the array (rs2)", reg_rs2, 32'h22222222);
    check_mirrors("arrays agree after the seed writes");

    drive(5'd5, 5'd5, 1'b1, 5'd5, 32'h44444444);   // fetch, writing x5
    drive(5'd5, 5'd5, 1'b0, 5'd0, 32'h0);          // use, no write
    check_hex("write-first capture in the fetch cycle (rs1)", reg_rs1, 32'h44444444);
    check_hex("write-first capture in the fetch cycle (rs2)", reg_rs2, 32'h44444444);

    drive(5'd6, 5'd6, 1'b0, 5'd0, 32'h0);          // fetch, no write
    drive(5'd6, 5'd6, 1'b1, 5'd6, 32'h55555555);   // use, writing x6
    check_hex("write-through bypass in the use cycle (rs1)", reg_rs1, 32'h55555555);
    check_hex("write-through bypass in the use cycle (rs2)", reg_rs2, 32'h55555555);

    drive(5'd6, 5'd6, 1'b0, 5'd0, 32'h0);
    drive(5'd6, 5'd6, 1'b0, 5'd0, 32'h0);
    check_hex("the bypassed write reached the array (rs1)", reg_rs1, 32'h55555555);
    check_mirrors("arrays agree after the forwarded writes");

    drive(5'd5, 5'd0, 1'b0, 5'd0, 32'h0);          // fetch x5
    drive(5'd6, 5'd0, 1'b0, 5'd0, 32'h0);          // use, but rs1 now points at x6
    check_hex("the read is keyed to the fetched address, not the current one",
              reg_rs1, 32'h44444444);
    check_ne("...and specifically is NOT x6's value", reg_rs1, 32'h55555555);

    drive(5'd0, 5'd5, 1'b0, 5'd0, 32'h0);          // fetch x5 on rs2
    drive(5'd0, 5'd6, 1'b1, 5'd5, 32'haaaaaaaa);   // use, rs2 now x6, writing x5
    check_hex("bypass keyed to the held address (rs2)", reg_rs2, 32'haaaaaaaa);

    drive(5'd5, 5'd0, 1'b0, 5'd0, 32'h0);          // fetch x5 on rs1
    drive(5'd6, 5'd0, 1'b1, 5'd5, 32'h99999999);   // use, rs1 now x6, writing x5
    check_hex("bypass keyed to the held address (rs1)", reg_rs1, 32'h99999999);

    drive(5'd0, 5'd0, 1'b1, 5'd0, 32'hdeadbeef);
    check_hex("presenting x0 still answers the held address (rs1)", reg_rs1, 32'h55555555);
    drive(5'd0, 5'd0, 1'b1, 5'd0, 32'hdeadbeef);
    check_hex("x0 reads 0 in the use cycle (rs1)", reg_rs1, 32'h00000000);
    check_hex("x0 reads 0 in the use cycle (rs2)", reg_rs2, 32'h00000000);

    drive(5'd0, 5'd0, 1'b0, 5'd0, 32'h0);
    check_ne("x0 write never reaches regs_a", dut.regs_a[0], 32'hdeadbeef);
    check_ne("x0 write never reaches regs_b", dut.regs_b[0], 32'hdeadbeef);

    drive(5'd7, 5'd7, 1'b0, 5'd0, 32'h0);          // fetch x7
    drive(5'd7, 5'd7, 1'b1, 5'd5, 32'h66666666);   // use, writing an unrelated x5
    check_hex("bypass does not leak into an unrelated address (rs1)", reg_rs1, 32'h33333333);
    check_hex("bypass does not leak into an unrelated address (rs2)", reg_rs2, 32'h33333333);

    drive(5'd7, 5'd6, 1'b0, 5'd0, 32'h0);
    drive(5'd7, 5'd6, 1'b1, 5'd6, 32'h77777777);
    check_hex("independent ports, array side (rs1)", reg_rs1, 32'h33333333);
    check_hex("independent ports, bypass side (rs2)", reg_rs2, 32'h77777777);

    drive(5'd6, 5'd7, 1'b0, 5'd0, 32'h0);
    drive(5'd6, 5'd7, 1'b1, 5'd6, 32'h88888888);
    check_hex("independent ports, bypass side (rs1)", reg_rs1, 32'h88888888);
    check_hex("independent ports, array side (rs2)", reg_rs2, 32'h33333333);

    drive(5'd7, 5'd7, 1'b0, 5'd7, 32'hdeadbeef);   // fetch x7, nothing written
    drive(5'd7, 5'd7, 1'b0, 5'd7, 32'hdeadbeef);   // use, still nothing written
    check_hex("a wen-low write is not captured write-first (rs1)", reg_rs1, 32'h33333333);
    check_hex("...and does not reach the bypass (rs2)", reg_rs2, 32'h33333333);
    drive(5'd0, 5'd0, 1'b0, 5'd0, 32'h0);
    check_ne("...nor either array", dut.regs_a[7], 32'hdeadbeef);
    check_ne("...on both copies", dut.regs_b[7], 32'hdeadbeef);

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
