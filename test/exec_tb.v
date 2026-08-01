`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// Standalone randomized differential bench for the executor's multiply/divide
// datapath (ADR-0010): this is the *primary* guarantee for real mul/div
// arithmetic, because riscv-formal runs under RISCV_FORMAL_ALTOPS and never
// checks it. Drives `executor` directly (no decoder/pipeline involved) and
// compares against SystemVerilog `*`, `/`, `%` with RISC-V divide-by-zero and
// INT_MIN / -1 semantics. Exits non-zero (via $fatal) on any mismatch.
//
// It also covers the three shift arms -- SLL / SRL / SRA -- on the same
// randomized differential footing. Those had no coverage here at all until
// this bench grew it, despite SRA being the one arm in the executor whose
// correctness depends on a signedness that IEEE 1800 will silently take away
// from you; read the comment above the shift reference model below before
// touching it.
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

  // ---- reference model: shifts ----------------------------------------
  //
  // READ THIS BEFORE EDITING THE THREE FUNCTIONS BELOW.
  //
  // Every shift here is computed in a STATEMENT OF ITS OWN, into a local whose
  // signedness is declared, and is never an arm of a `?:` or any other
  // conditional expression. That is not a style preference, it is the whole
  // reason these functions are shaped the way they are.
  //
  // IEEE 1800 sign-context propagation makes a conditional expression unsigned
  // if any of its operands is unsigned, and then pushes that unsignedness DOWN
  // into the context-determined operands. A `$signed(a) >>> sh` sitting in one
  // arm of such a conditional therefore evaluates as a LOGICAL shift -- with no
  // warning, and only for negative operands, so every non-negative vector still
  // agrees. The assignment target cannot rescue it: an expression's type does
  // not depend on its left-hand side.
  //
  // This is not hypothetical, and it is not new here. ADR-0019 is the same
  // defect at two sites in the generated riscv-formal monitor -- its DIV and
  // REM spec models, where `-7 / 2` scored as `0x7ffffffc` and failed `div.S`
  // and `rem.S` against a correct core, until the sanitizer made the
  // arithmetic self-determined. It then happened AGAIN, here, while these very
  // vectors were being written: the natural one-line `ref_sra` written as a
  // `?:` arm reported six mismatches, and the thing that was wrong was the
  // oracle. A bench whose oracle is broken is worse than no bench -- it blames
  // the hardware and teaches the reader to distrust the bench. Keep each of
  // these a standalone statement, and see `ref_selftest` below, which pins
  // them against hand-computed literals that cannot degrade along with the
  // expression.
  //
  // The `b[4:0]` masks are this reference's own model of the RV32I rule that
  // the shift amount is rs2[4:0] and the upper 27 bits are ignored (spec Vol I
  // 2.4.2, and rtl/executor.v's comment on the same). The vectors drive rs2
  // values far above 31 so the RTL has to agree rather than merely coincide.
  function automatic logic [31:0] ref_sll(input logic [31:0] a, input logic [31:0] b);
    logic [31:0] v;
    begin
      v = a << b[4:0];
      ref_sll = v;
    end
  endfunction

  function automatic logic [31:0] ref_srl(input logic [31:0] a, input logic [31:0] b);
    logic [31:0] v;
    begin
      v = a >> b[4:0];
      ref_srl = v;
    end
  endfunction

  function automatic logic [31:0] ref_sra(input logic [31:0] a, input logic [31:0] b);
    logic signed [31:0] sa;
    logic signed [31:0] v;
    begin
      sa = $signed(a);
      v  = sa >>> b[4:0];
      ref_sra = v;
    end
  endfunction

  task automatic clear_in;
    begin
      in = '0;
      in.rd = 5'd1;
    end
  endtask

  // Checks the shift ORACLE, not the core -- no RTL is involved. An SRA
  // reference that had silently degraded to a logical shift (see the comment
  // above the shift functions) agrees with a correct one on every non-negative
  // operand, so it would pass thousands of randomized vectors and then report
  // mismatches against correct hardware. These expected values are computed by
  // hand and written as literals, so they cannot degrade along with it.
  task automatic ref_selftest(input string what, input logic [31:0] got,
                               input logic [31:0] want);
    begin
      if (got !== want) begin
        $display("ORACLE BROKEN: %s got=%08x expected=%08x", what, got, want);
        errors++;
      end
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
      else if (op_name == "sll") in.is_sll = 1'b1;
      else if (op_name == "srl") in.is_srl = 1'b1;
      else if (op_name == "sra") in.is_sra = 1'b1;
      else begin
        // A typo in an op name would otherwise issue an all-zero decoder_output
        // and silently compare against whatever the executor left on rd_data.
        $display("BENCH BUG: unknown op name '%s'", op_name);
        $fatal(1);
      end
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

  // Shift patterns chosen so that a lost sign bit, a dropped mask or an
  // off-by-one fill is visible: INT_MIN, all-ones, INT_MAX, a negative
  // non-trivial value, a positive alternating one, and 1 (which walks a single
  // bit off each end).
  localparam int SHIFT_PATTERNS = 6;
  logic [31:0] pattern [0:SHIFT_PATTERNS-1];
  int p, sh;
  logic [31:0] rs2_masked, rs2_dirty;

  initial begin
    reset = 1;
    clear_in();
    repeat (2) @(posedge clk);
    #1;
    reset = 0;

    // The oracle is checked before it is trusted to judge anything.
    ref_selftest("sra(80000000,4)",  ref_sra(32'h80000000, 32'd4),  32'hf8000000);
    ref_selftest("sra(80000000,31)", ref_sra(32'h80000000, 32'd31), 32'hffffffff);
    ref_selftest("sra(deadbeef,8)",  ref_sra(32'hdeadbeef, 32'd8),  32'hffdeadbe);
    ref_selftest("sra(ffffffff,1)",  ref_sra(32'hffffffff, 32'd1),  32'hffffffff);
    ref_selftest("sra(7fffffff,1)",  ref_sra(32'h7fffffff, 32'd1),  32'h3fffffff);
    ref_selftest("sra(80000000,0)",  ref_sra(32'h80000000, 32'd0),  32'h80000000);
    ref_selftest("srl(80000000,4)",  ref_srl(32'h80000000, 32'd4),  32'h08000000);
    ref_selftest("sll(00000001,31)", ref_sll(32'h00000001, 32'd31), 32'h80000000);
    // ...including the reference's own rs2[4:0] masking, on both sides of 32.
    ref_selftest("sra(80000000,36)", ref_sra(32'h80000000, 32'd36), 32'hf8000000);
    ref_selftest("sll(00000001,32)", ref_sll(32'h00000001, 32'd32), 32'h00000001);
    if (errors != 0) begin
      $display("FAILED: the shift reference model is broken; no core result below means anything");
      $fatal(1);
    end

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

    // Directed shift sweep: every shift amount 0..31 against each pattern,
    // driven TWICE -- once with a clean rs2 and once with all 27 upper bits
    // set. $random alone reaches every amount with overwhelming probability but
    // does not guarantee it, and the two forms together are what turns "the
    // masking happens to work on random inputs" into "the same amount with and
    // without garbage above bit 4 gives the same answer", which is the actual
    // rs2[4:0] rule (rtl/executor.v).
    pattern[0] = 32'h80000000;
    pattern[1] = 32'hffffffff;
    pattern[2] = 32'h7fffffff;
    pattern[3] = 32'hdeadbeef;
    pattern[4] = 32'h55555555;
    pattern[5] = 32'h00000001;
    for (p = 0; p < SHIFT_PATTERNS; p++) begin
      for (sh = 0; sh < 32; sh++) begin
        rs2_masked = {27'b0, sh[4:0]};
        rs2_dirty  = {27'h7ffffff, sh[4:0]};
        run_op("sll", pattern[p], rs2_masked, ref_sll(pattern[p], rs2_masked));
        run_op("srl", pattern[p], rs2_masked, ref_srl(pattern[p], rs2_masked));
        run_op("sra", pattern[p], rs2_masked, ref_sra(pattern[p], rs2_masked));
        run_op("sll", pattern[p], rs2_dirty,  ref_sll(pattern[p], rs2_masked));
        run_op("srl", pattern[p], rs2_dirty,  ref_srl(pattern[p], rs2_masked));
        run_op("sra", pattern[p], rs2_dirty,  ref_sra(pattern[p], rs2_masked));
      end
    end

    // Randomized differential coverage: >=10,000 vectors per operation.
    // $random's rs2 is a full 32-bit word, so the great majority of shift
    // vectors here also carry a shift amount above 31.
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
      run_op("sll",    a, b, ref_sll(a, b));
      run_op("srl",    a, b, ref_srl(a, b));
      run_op("sra",    a, b, ref_sra(a, b));
    end

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: %0d randomized vectors per op (mul/mulh/mulhu/mulhsu/div/divu/rem/remu/sll/srl/sra),",
                RANDOM_VECTORS);
      $display("        plus %0d directed shift vectors per shift op (amounts 0-31, clean and dirty rs2),",
                SHIFT_PATTERNS * 32 * 2);
      $display("        0 mismatches");
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
