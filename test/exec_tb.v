`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// Standalone randomized differential bench for the executor's multiply/divide
// datapath (ADR-0010): this is the *primary* guarantee for real mul/div
// arithmetic, because riscv-formal runs under RISCV_FORMAL_ALTOPS and never
// checks it. Drives `executor` directly (no decoder/pipeline involved) and
// compares against SystemVerilog `*`, `/`, `%` with RISC-V divide-by-zero and
// INT_MIN / -1 semantics. Exits non-zero (via $fatal) on any mismatch.
module exec_tb;
  localparam int RANDOM_VECTORS = 10000;
  localparam logic [1:0] DIVIDE_STATE = 2'b10; // must match executor.v's `divide` state

  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  decoder_output in;
  executor_output out;

  // No accessor in this standalone bench, so no reason for it to ever
  // freeze — tie the input low.
  executor dut (
    .clk(clk),
    .reset(reset),
    .in(in),
    .accessor_stall(1'b0),
    .out(out)
  );

  int errors = 0;

  // ---- reference model: SystemVerilog *, /, % with RISC-V semantics ----
  function automatic logic [31:0] ref_mul(input logic [31:0] a, input logic [31:0] b);
    logic [63:0] p;
    begin
      p = a * b;
      ref_mul = p[31:0];
    end
  endfunction

  function automatic logic [31:0] ref_mulh(input logic [31:0] a, input logic [31:0] b);
    logic signed [63:0] p;
    begin
      p = $signed(a) * $signed(b);
      ref_mulh = p[63:32];
    end
  endfunction

  function automatic logic [31:0] ref_mulhu(input logic [31:0] a, input logic [31:0] b);
    logic [63:0] p;
    begin
      p = a * b;
      ref_mulhu = p[63:32];
    end
  endfunction

  function automatic logic [31:0] ref_mulhsu(input logic [31:0] a, input logic [31:0] b);
    logic signed [63:0] p;
    begin
      // rs1 is signed, rs2 is unsigned (zero-extended one bit so it stays non-negative
      // once cast signed).
      p = $signed(a) * $signed({1'b0, b});
      ref_mulhsu = p[63:32];
    end
  endfunction

  function automatic logic [31:0] ref_div(input logic [31:0] a, input logic [31:0] b);
    begin
      if (b == 0) ref_div = 32'hffffffff;
      else if (a == 32'h80000000 && b == 32'hffffffff) ref_div = 32'h80000000;
      else ref_div = $signed(a) / $signed(b);
    end
  endfunction

  function automatic logic [31:0] ref_divu(input logic [31:0] a, input logic [31:0] b);
    begin
      ref_divu = (b == 0) ? 32'hffffffff : (a / b);
    end
  endfunction

  function automatic logic [31:0] ref_rem(input logic [31:0] a, input logic [31:0] b);
    begin
      if (b == 0) ref_rem = a;
      else if (a == 32'h80000000 && b == 32'hffffffff) ref_rem = 32'b0;
      else ref_rem = $signed(a) % $signed(b);
    end
  endfunction

  function automatic logic [31:0] ref_remu(input logic [31:0] a, input logic [31:0] b);
    begin
      ref_remu = (b == 0) ? a : (a % b);
    end
  endfunction

  task automatic clear_in;
    begin
      in = '0;
      in.rd = 5'd1;
    end
  endtask

  task automatic check(input string op_name, input logic [31:0] rs1_v, input logic [31:0] rs2_v,
                        input logic [31:0] expected);
    begin
      if (out.rd_data !== expected) begin
        $display("MISMATCH %s rs1=%08x rs2=%08x got=%08x expected=%08x",
                  op_name, rs1_v, rs2_v, out.rd_data, expected);
        errors++;
      end
    end
  endtask

  // Multiply-family ops settle one cycle after issue (the executor never leaves
  // `init` for them). Divide-family ops may enter the multi-cycle restoring
  // divider (see criterion 4 below); the special-case (div-by-zero, INT_MIN/-1)
  // paths finish within `init` like the multiply-family ops do. Either way, the
  // executor is done exactly when it has returned to `init` after the issue
  // edge, so poll for that instead of hardcoding a cycle count — that keeps this
  // bench correct regardless of exactly how long the divider takes, and leaves
  // criterion 4's specific cycle count to the dedicated timing monitor below.
  task automatic run_op(input string op_name, input logic [31:0] rs1_v, input logic [31:0] rs2_v,
                         input logic [31:0] expected);
    int guard;
    begin
      clear_in();
      in.rs1 = rs1_v;
      in.rs2 = rs2_v;
      if (op_name == "mul") in.is_mul = 1'b1;
      else if (op_name == "mulh") in.is_mulh = 1'b1;
      else if (op_name == "mulhu") in.is_mulhu = 1'b1;
      else if (op_name == "mulhsu") in.is_mulhsu = 1'b1;
      else if (op_name == "div") in.is_div = 1'b1;
      else if (op_name == "divu") in.is_divu = 1'b1;
      else if (op_name == "rem") in.is_rem = 1'b1;
      else if (op_name == "remu") in.is_remu = 1'b1;
      @(posedge clk); // issue edge
      #1;
      guard = 0;
      while (dut.state == DIVIDE_STATE) begin
        @(posedge clk);
        #1;
        guard++;
        if (guard > 64) begin
          $display("HANG: %s never left the divide state (rs1=%08x rs2=%08x)",
                    op_name, rs1_v, rs2_v);
          $fatal(1);
        end
      end
      check(op_name, rs1_v, rs2_v, expected);
    end
  endtask

  logic [31:0] a, b;
  int i;

  initial begin
    reset = 1;
    clear_in();
    repeat (2) @(posedge clk);
    #1;
    reset = 0;

    // Required directed vectors (ADR-0010): exactly the cases the swapped sign
    // enables get wrong.
    run_op("mulh",   32'hffffffff, 32'hffffffff, 32'h00000000);
    run_op("mulhsu", 32'hffffffff, 32'h00000001, 32'hffffffff);
    run_op("mulhu",  32'hffffffff, 32'hffffffff, 32'hfffffffe);

    // Directed divide-by-zero / INT_MIN-over-negative-one vectors.
    run_op("div",  32'h00000064, 32'h00000000, 32'hffffffff);
    run_op("divu", 32'h00000064, 32'h00000000, 32'hffffffff);
    run_op("rem",  32'h00000064, 32'h00000000, 32'h00000064);
    run_op("remu", 32'h00000064, 32'h00000000, 32'h00000064);
    run_op("div",  32'h80000000, 32'hffffffff, 32'h80000000);
    run_op("rem",  32'h80000000, 32'hffffffff, 32'h00000000);

    // Randomized differential coverage: >=10,000 vectors per operation.
    for (i = 0; i < RANDOM_VECTORS; i++) begin
      a = $random;
      b = $random;
      run_op("mul",    a, b, ref_mul(a, b));
      run_op("mulh",   a, b, ref_mulh(a, b));
      run_op("mulhu",  a, b, ref_mulhu(a, b));
      run_op("mulhsu", a, b, ref_mulhsu(a, b));
      run_op("div",    a, b, ref_div(a, b));
      run_op("divu",   a, b, ref_divu(a, b));
      run_op("rem",    a, b, ref_rem(a, b));
      run_op("remu",   a, b, ref_remu(a, b));
    end

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: %0d randomized vectors per op (mul/mulh/mulhu/mulhsu/div/divu/rem/remu), 0 mismatches", RANDOM_VECTORS);
      $finish;
    end
  end

  // Criterion 4: the divider produces its result exactly 33 cycles after issue
  // (32 restoring-division iterations plus one capture cycle), not 66. Checked
  // against every real division performed during the run above, not just once.
  int cycle_count;
  logic prev_divide;

  initial begin
    cycle_count = 0;
    prev_divide = 0;
    forever begin
      @(posedge clk);
      #1;
      if (dut.state == DIVIDE_STATE) begin
        cycle_count++;
        prev_divide = 1;
      end else if (prev_divide) begin
        if (cycle_count !== 33) begin
          $display("TIMING MISMATCH: divider completed in %0d cycles, expected 33", cycle_count);
          errors++;
          $fatal(1);
        end
        prev_divide = 0;
        cycle_count = 0;
      end
    end
  end
endmodule
