`timescale 1 ns / 1 ps
`default_nettype none

// ADR-0004: rtl/regfile.v is combinational-read with write-through
// bypass. A write in cycle N must be visible on reg_rs1/reg_rs2 in cycle N
// for the same address (not one cycle later, which was the original defect —
// see CLAUDE.md and ADR-0004), and rs == 0 must always read 0, even while
// waddr == 0 and wen == 1.
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

  initial begin
    rs1 = 0;
    rs2 = 0;
    wen = 0;
    waddr = 0;
    wdata = 0;
    @(posedge clk);
    #1;

    // Write-through: a write to x5 in this same cycle is visible on reg_rs1
    // this same cycle, not the next one — the original defect was a
    // registered read port, one cycle stale.
    rs1 = 5'd5;
    waddr = 5'd5;
    wdata = 32'hcafef00d;
    wen = 1'b1;
    #1;
    check_hex("write-through same-cycle read (rs1)", reg_rs1, 32'hcafef00d);

    // Same check on the rs2 port, since the operand skew observed while
    // landing the cxxrtl runner (`9cd0c67`) was NOT uniform across ports — rs1 and rs2 recovered
    // at different depths, so both ports need their own direct check.
    rs2 = 5'd5;
    #1;
    check_hex("write-through same-cycle read (rs2)", reg_rs2, 32'hcafef00d);

    // Let the write actually land, then confirm the value is still readable
    // (via the array, not the bypass) once wen drops.
    @(posedge clk);
    #1;
    wen = 1'b0;
    #1;
    check_hex("registered write persists after wen drops (rs1)", reg_rs1, 32'hcafef00d);
    check_hex("registered write persists after wen drops (rs2)", reg_rs2, 32'hcafef00d);

    // x0 always reads 0, even with waddr == 0 and wen == 1 pointed straight
    // at it (the write itself must also be a no-op, checked next).
    rs1 = 5'd0;
    rs2 = 5'd0;
    waddr = 5'd0;
    wdata = 32'hdeadbeef;
    wen = 1'b1;
    #1;
    check_hex("x0 reads 0 while waddr == 0, wen == 1 (rs1)", reg_rs1, 32'h00000000);
    check_hex("x0 reads 0 while waddr == 0, wen == 1 (rs2)", reg_rs2, 32'h00000000);

    @(posedge clk);
    #1;
    wen = 1'b0;
    // White-box: confirm the attempted write never reached the backing
    // array, not just that x0 still reads 0 (which the read mux guarantees
    // unconditionally regardless of what's stored at index 0).
    check_ne("x0 write never reaches the backing array", dut.regs[0], 32'hdeadbeef);

    // A different register, unrelated to the write in progress, must not be
    // disturbed by the bypass (bypass only fires when waddr matches the read
    // address being asked for). Give x6 a known value first, then confirm a
    // concurrent bypass-write to x5 doesn't leak into reading x6.
    waddr = 5'd6;
    wdata = 32'h22222222;
    wen = 1'b1;
    @(posedge clk);
    #1;
    waddr = 5'd5;
    wdata = 32'h11111111;
    wen = 1'b1;
    rs1 = 5'd6;
    #1;
    check_hex("bypass does not leak into an unrelated read address", reg_rs1, 32'h22222222);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: regfile combinational read / write-through bypass / x0 semantics");
      $finish;
    end
  end
endmodule
