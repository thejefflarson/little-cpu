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
//
// ---- THIS BENCH ASSERTS ITS OWN SHAPE BEFORE IT ASSERTS ANYTHING ABOUT THE
//      CORE, AND THAT IS DELIBERATE ------------------------------------------
//
// ADR-0045 closes M2's mul/div term by NAMING this bench as the oracle. This
// repo's rule for a named gate (ADR-0033, ADR-0035) is that it must be unable
// to stop checking without going red -- a check that quietly stopped running is
// worse than no check, because it still reads as coverage. Three mechanisms
// enforce that here, all of them running before or alongside the RTL vectors:
//
//   * `ref_selftest` pins every reference function -- mul, div AND shift --
//     against literals computed BY HAND, never copied from a run of this bench.
//     A literal derived from the thing it checks proves nothing. A degraded
//     reference says ORACLE BROKEN and stops, instead of blaming the core.
//   * `check_vector_counts` pins the per-operation vector count against
//     ADR-0010's contract written as its own literal, so a loop bound that
//     drifted -- to 100 during a debugging session, or to zero -- fails here.
//     Before this existed, RANDOM_VECTORS = 0 printed PASSED and exited 0.
//   * `check_directed_manifest` pins that each REQUIRED directed corner vector
//     actually ran, from a manifest that lives apart from the call sites. So
//     deleting a call site fails; it does not merely shrink the run.
module exec_tb;
  localparam int RANDOM_VECTORS = 10000;
  localparam logic [1:0] DIVIDE_STATE = 2'b10; // must match executor.v's `divide` state

  // ADR-0010's contract is ">= 10,000 randomized operand pairs per operation".
  // These two are that CONTRACT, written as literals of their own -- they are
  // not a second copy of the loop bounds, and the whole point of them is that
  // they do not move when a loop bound does. `MIN_DIRECTED_SHIFT_PER_OP` is
  // 6 patterns x 32 shift amounts x {clean rs2, dirty rs2}.
  //
  // If a count check below goes red, the fix is to restore the loop, never to
  // edit the number down to match it. Same rule as `formal/EXPECTED_FAIL` and
  // `test/EXPECTED_FAIL`: re-derive it, never silence it.
  localparam int MIN_RANDOM_PER_OP        = 10000;
  localparam int MIN_DIRECTED_SHIFT_PER_OP = 384;

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

  // ---- the operation table -------------------------------------------------
  //
  // One table, one source of truth for the eleven op names. `run_op` resolves
  // its `op_name` argument through it, which is also what makes a typo'd name
  // fatal rather than silently comparing against whatever the executor left on
  // rd_data.
  localparam int OP_MUL    = 0;
  localparam int OP_MULH   = 1;
  localparam int OP_MULHU  = 2;
  localparam int OP_MULHSU = 3;
  localparam int OP_DIV    = 4;
  localparam int OP_DIVU   = 5;
  localparam int OP_REM    = 6;
  localparam int OP_REMU   = 7;
  localparam int OP_SLL    = 8;
  localparam int OP_SRL    = 9;
  localparam int OP_SRA    = 10;
  localparam int NUM_OPS   = 11;

  string op_names  [0:NUM_OPS-1];
  int    vec_count [0:NUM_OPS-1];

  // ---- the directed-vector manifest ---------------------------------------
  //
  // These are the corner vectors this bench is REQUIRED to run. The first three
  // are ADR-0010's, verbatim: MULH(-1,-1) = 0, MULHSU(-1,1) = -1,
  // MULHU(-1,-1) = 0xFFFFFFFE -- exactly the cases a swapped sign enable gets
  // wrong, and exactly the cases every randomized vector can pass around.
  // The remaining six are the RISC-V divide-by-zero and INT_MIN / -1 results,
  // which are architecturally specified values rather than arithmetic ones and
  // so are reachable by no random operand pair at all.
  //
  // The manifest is deliberately NOT the thing that issues them. `run_op`
  // witnesses each (op, rs1, rs2) it actually drove into the DUT against this
  // list, and `check_directed_manifest` fails on any entry never witnessed. So
  // deleting a directed call site below is a red bench, not a shorter run --
  // which is the whole difference between a gate and a claim.
  //
  // It also carries each vector's expected value and checks the CALL SITE
  // against it, so a directed vector weakened in place (kept, but with its
  // expectation edited to whatever the core happens to produce) is caught too.
  localparam int DIRECTED_N = 9;
  int          dir_op  [0:DIRECTED_N-1];
  logic [31:0] dir_rs1 [0:DIRECTED_N-1];
  logic [31:0] dir_rs2 [0:DIRECTED_N-1];
  logic [31:0] dir_exp [0:DIRECTED_N-1];
  logic        dir_ran [0:DIRECTED_N-1];
  string       dir_why [0:DIRECTED_N-1];
  // Witnessing is a linear scan per vector driven, and there are ~111,000 of
  // them. This counts down to zero on the ninth directed call site -- all of
  // which run before the sweeps -- after which `run_op` skips the scan. Purely
  // a cost thing; `check_directed_manifest` still reads `dir_ran` itself, so a
  // deleted call site leaves this above zero and is caught either way.
  int          dir_pending;

  task automatic add_directed(input int slot, input int op, input logic [31:0] rs1_v,
                               input logic [31:0] rs2_v, input logic [31:0] exp_v,
                               input string why);
    begin
      dir_op[slot]  = op;
      dir_rs1[slot] = rs1_v;
      dir_rs2[slot] = rs2_v;
      dir_exp[slot] = exp_v;
      dir_ran[slot] = 1'b0;
      dir_why[slot] = why;
    end
  endtask

  task automatic init_tables;
    int k;
    begin
      op_names[OP_MUL]    = "mul";
      op_names[OP_MULH]   = "mulh";
      op_names[OP_MULHU]  = "mulhu";
      op_names[OP_MULHSU] = "mulhsu";
      op_names[OP_DIV]    = "div";
      op_names[OP_DIVU]   = "divu";
      op_names[OP_REM]    = "rem";
      op_names[OP_REMU]   = "remu";
      op_names[OP_SLL]    = "sll";
      op_names[OP_SRL]    = "srl";
      op_names[OP_SRA]    = "sra";
      for (k = 0; k < NUM_OPS; k++) vec_count[k] = 0;
      dir_pending = DIRECTED_N;

      // ADR-0010's three required sign-enable vectors.
      add_directed(0, OP_MULH,   32'hffffffff, 32'hffffffff, 32'h00000000, "ADR-0010 MULH(-1,-1)=0");
      add_directed(1, OP_MULHSU, 32'hffffffff, 32'h00000001, 32'hffffffff, "ADR-0010 MULHSU(-1,1)=-1");
      add_directed(2, OP_MULHU,  32'hffffffff, 32'hffffffff, 32'hfffffffe, "ADR-0010 MULHU(-1,-1)=0xFFFFFFFE");
      // Divide-by-zero and INT_MIN / -1: specified values, not computed ones.
      add_directed(3, OP_DIV,    32'h00000064, 32'h00000000, 32'hffffffff, "div by zero");
      add_directed(4, OP_DIVU,   32'h00000064, 32'h00000000, 32'hffffffff, "divu by zero");
      add_directed(5, OP_REM,    32'h00000064, 32'h00000000, 32'h00000064, "rem by zero");
      add_directed(6, OP_REMU,   32'h00000064, 32'h00000000, 32'h00000064, "remu by zero");
      add_directed(7, OP_DIV,    32'h80000000, 32'hffffffff, 32'h80000000, "div INT_MIN/-1");
      add_directed(8, OP_REM,    32'h80000000, 32'hffffffff, 32'h00000000, "rem INT_MIN/-1");
    end
  endtask

  function automatic int op_index(input string name);
    int k;
    begin
      op_index = -1;
      for (k = 0; k < NUM_OPS; k++)
        if (op_names[k] == name) op_index = k;
    end
  endfunction

  // ---- reference model: SystemVerilog *, /, % with RISC-V semantics ----
  //
  // The signed-reference rule stated at length above the shift references below
  // governs these too, and `ref_div` / `ref_rem` are where it bites hardest:
  // their RISC-V special cases are written as an if / else-if / else over
  // separate STATEMENTS rather than as one `?:` chain, because a `?:` chain
  // whose other arms are unsigned literals evaluates `$signed(a) / $signed(b)`
  // unsigned. That is not hypothetical -- it is exactly ADR-0019's defect in
  // the generated riscv-formal monitor, at exactly these two functions, where
  // -7 / 2 scored 0x7ffffffc. `ref_selftest` pins both against hand-computed
  // literals before any of them is trusted to judge the core.
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

  // Checks the ORACLE, not the core -- no RTL is involved in any call to this.
  // An SRA reference that had silently degraded to a logical shift (see the
  // comment above the shift functions) agrees with a correct one on every
  // non-negative operand, so it would pass thousands of randomized vectors and
  // then report mismatches against correct hardware. The same is true of the
  // signed divide and the signed multiply high halves.
  //
  // EVERY expected value passed to this task is computed BY HAND and written as
  // a literal. That is the load-bearing property and it is easy to destroy: a
  // literal captured from a run of this bench is derived from the very function
  // it is supposed to check, which is the same defect one level down. If you add
  // a case here, do the arithmetic on paper.
  task automatic ref_selftest(input string what, input logic [31:0] got,
                               input logic [31:0] want);
    begin
      if (got !== want) begin
        $display("ORACLE BROKEN: %s got=%08x expected=%08x", what, got, want);
        errors++;
      end
    end
  endtask

  // Checks that the bench RAN what it says it runs. Before this existed a loop
  // bound that had drifted to zero -- RANDOM_VECTORS = 0, an inner `for` edited
  // during a debugging session and never put back -- printed PASSED and exited
  // 0, and ADR-0045 names this bench as M2's mul/div oracle. The comparison is
  // against MIN_RANDOM_PER_OP / MIN_DIRECTED_SHIFT_PER_OP, which are ADR-0010's
  // contract written as literals rather than second copies of the loop bounds.
  task automatic check_vector_counts;
    int k, want;
    begin
      for (k = 0; k < NUM_OPS; k++) begin
        want = MIN_RANDOM_PER_OP;
        if (k == OP_SLL || k == OP_SRL || k == OP_SRA)
          want = want + MIN_DIRECTED_SHIFT_PER_OP;
        if (vec_count[k] < want) begin
          $display("COVERAGE SHORTFALL: %s ran %0d vectors, the contract requires at least %0d",
                    op_names[k], vec_count[k], want);
          errors++;
        end
      end
    end
  endtask

  // Checks that every REQUIRED directed corner vector reached the DUT. The
  // manifest lives apart from the call sites precisely so that deleting a call
  // site is red rather than merely shorter; a randomized vector cannot cover
  // for a deleted one, because these are the cases random operands do not
  // reach (a specified divide-by-zero result) or reach only by accident.
  task automatic check_directed_manifest;
    int d;
    begin
      for (d = 0; d < DIRECTED_N; d++) begin
        if (!dir_ran[d]) begin
          $display("DIRECTED VECTOR MISSING: %s (%s rs1=%08x rs2=%08x) never ran",
                    dir_why[d], op_names[dir_op[d]], dir_rs1[d], dir_rs2[d]);
          errors++;
        end
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
    int id;
    int d;
    begin
      id = op_index(op_name);
      if (id < 0) begin
        // A typo in an op name would otherwise issue an all-zero decoder_output
        // and silently compare against whatever the executor left on rd_data.
        $display("BENCH BUG: unknown op name '%s'", op_name);
        $fatal(1);
      end
      clear_in();
      in.rs1 = rs1_v;
      in.rs2 = rs2_v;
      case (id)
        OP_MUL:    in.is_mul    = 1'b1;
        OP_MULH:   in.is_mulh   = 1'b1;
        OP_MULHU:  in.is_mulhu  = 1'b1;
        OP_MULHSU: in.is_mulhsu = 1'b1;
        OP_DIV:    in.is_div    = 1'b1;
        OP_DIVU:   in.is_divu   = 1'b1;
        OP_REM:    in.is_rem    = 1'b1;
        OP_REMU:   in.is_remu   = 1'b1;
        OP_SLL:    in.is_sll    = 1'b1;
        OP_SRL:    in.is_srl    = 1'b1;
        OP_SRA:    in.is_sra    = 1'b1;
      endcase

      // Everything this bench claims about its own coverage is measured here,
      // on the vectors it actually drove into the DUT -- never inferred from a
      // loop bound or a localparam.
      vec_count[id]++;
      if (dir_pending > 0) begin
        for (d = 0; d < DIRECTED_N; d++) begin
          if (!dir_ran[d] && dir_op[d] == id && dir_rs1[d] === rs1_v && dir_rs2[d] === rs2_v) begin
            if (dir_exp[d] !== expected) begin
              $display("MANIFEST MISMATCH: required vector %0d (%s) was driven with expected=%08x, manifest says %08x",
                        d, dir_why[d], expected, dir_exp[d]);
              errors++;
            end
            dir_ran[d] = 1'b1;
            dir_pending--;
          end
        end
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
    init_tables();
    clear_in();
    repeat (2) @(posedge clk);
    #1;
    reset = 0;

    // The oracle is checked before it is trusted to judge anything.
    //
    // ---- multiply references ------------------------------------------------
    // MUL: low 32 bits, signedness-independent. (-1)*(-1) = 1; 2^31 * 2 = 2^32,
    // whose low half is zero; 65535^2 = 0xfffe0001.
    ref_selftest("mul(ffffffff,ffffffff)",  ref_mul(32'hffffffff, 32'hffffffff), 32'h00000001);
    ref_selftest("mul(80000000,00000002)",  ref_mul(32'h80000000, 32'h00000002), 32'h00000000);
    ref_selftest("mul(0000ffff,0000ffff)",  ref_mul(32'h0000ffff, 32'h0000ffff), 32'hfffe0001);
    // MULH: signed x signed, high 32. (-1)*(-1) = +1 -> high half 0.
    // (-2^31)*(-2^31) = 2^62 = 0x4000_0000_0000_0000 -> high half 0x40000000.
    // (-2^31)*2 = -2^32 = 0xffff_ffff_0000_0000 -> high half 0xffffffff.
    // 2*(-2^31) = -2^32 as well: this one is the pair for the MULHSU/MULHU
    // cases just below, which take the same operands and differ.
    ref_selftest("mulh(ffffffff,ffffffff)", ref_mulh(32'hffffffff, 32'hffffffff), 32'h00000000);
    ref_selftest("mulh(80000000,80000000)", ref_mulh(32'h80000000, 32'h80000000), 32'h40000000);
    ref_selftest("mulh(80000000,00000002)", ref_mulh(32'h80000000, 32'h00000002), 32'hffffffff);
    ref_selftest("mulh(00000002,80000000)", ref_mulh(32'h00000002, 32'h80000000), 32'hffffffff);
    // MULHU: unsigned x unsigned, high 32. (2^32-1)^2 = 2^64 - 2^33 + 1, whose
    // high half is 0xfffffffe. (2^32-1)*2^31 = 2^63 - 2^31 = 0x7fff_ffff_8000_0000.
    ref_selftest("mulhu(ffffffff,ffffffff)", ref_mulhu(32'hffffffff, 32'hffffffff), 32'hfffffffe);
    ref_selftest("mulhu(00000002,80000000)", ref_mulhu(32'h00000002, 32'h80000000), 32'h00000001);
    ref_selftest("mulhu(ffffffff,80000000)", ref_mulhu(32'hffffffff, 32'h80000000), 32'h7fffffff);
    // MULHSU: rs1 SIGNED, rs2 UNSIGNED, high 32. Each of these three differs
    // from at least one of the two above on the same operands, which is what
    // makes them a check of the signedness rather than of the multiply:
    //   (-1)*1        = -1                     -> 0xffff_ffff_ffff_ffff, high 0xffffffff
    //   2 * 2^31      = 2^32                   -> 0x0000_0001_0000_0000, high 0x00000001
    //                                             (mulh of the same pair is 0xffffffff)
    //   (-1) * 2^31   = -2^31                  -> 0xffff_ffff_8000_0000, high 0xffffffff
    //                                             (mulhu of the same pair is 0x7fffffff)
    ref_selftest("mulhsu(ffffffff,00000001)", ref_mulhsu(32'hffffffff, 32'h00000001), 32'hffffffff);
    ref_selftest("mulhsu(00000002,80000000)", ref_mulhsu(32'h00000002, 32'h80000000), 32'h00000001);
    ref_selftest("mulhsu(ffffffff,80000000)", ref_mulhsu(32'hffffffff, 32'h80000000), 32'hffffffff);
    // ---- divide references --------------------------------------------------
    // These are ADR-0019's own case. -7 / 2 truncates toward zero to -3
    // (0xfffffffd); a reference that had degraded to an unsigned divide scores
    // 0x7ffffffc -- which is what `divu(fffffff9,00000002)` legitimately is, and
    // it is written out just below so the two are visibly different numbers.
    ref_selftest("div(fffffff9,00000002)",  ref_div(32'hfffffff9, 32'h00000002), 32'hfffffffd);
    ref_selftest("div(00000007,fffffffe)",  ref_div(32'h00000007, 32'hfffffffe), 32'hfffffffd);
    ref_selftest("div(fffffff9,fffffffe)",  ref_div(32'hfffffff9, 32'hfffffffe), 32'h00000003);
    ref_selftest("div(00000064,00000000)",  ref_div(32'h00000064, 32'h00000000), 32'hffffffff);
    ref_selftest("div(80000000,ffffffff)",  ref_div(32'h80000000, 32'hffffffff), 32'h80000000);
    ref_selftest("divu(fffffff9,00000002)", ref_divu(32'hfffffff9, 32'h00000002), 32'h7ffffffc);
    ref_selftest("divu(00000064,00000000)", ref_divu(32'h00000064, 32'h00000000), 32'hffffffff);
    // REM takes the sign of the DIVIDEND: -7 % 2 = -1, 7 % -2 = +1. An unsigned
    // degradation scores 0x00000001 for the first, which is the second's answer.
    ref_selftest("rem(fffffff9,00000002)",  ref_rem(32'hfffffff9, 32'h00000002), 32'hffffffff);
    ref_selftest("rem(00000007,fffffffe)",  ref_rem(32'h00000007, 32'hfffffffe), 32'h00000001);
    ref_selftest("rem(00000064,00000000)",  ref_rem(32'h00000064, 32'h00000000), 32'h00000064);
    ref_selftest("rem(80000000,ffffffff)",  ref_rem(32'h80000000, 32'hffffffff), 32'h00000000);
    ref_selftest("remu(fffffff9,00000002)", ref_remu(32'hfffffff9, 32'h00000002), 32'h00000001);
    ref_selftest("remu(00000064,00000000)", ref_remu(32'h00000064, 32'h00000000), 32'h00000064);
    // ---- shift references ---------------------------------------------------
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
      $display("FAILED: the reference model is broken; no core result below means anything");
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

    // The bench grades its own shape LAST and reports it FIRST: everything
    // above is a claim about the core, and these two are the claim that the
    // above happened at all.
    check_vector_counts();
    check_directed_manifest();

    if (errors != 0) begin
      $display("FAILED: %0d errors", errors);
      $fatal(1);
    end else begin
      $display("PASSED: 0 mismatches. Vectors actually driven into the DUT, counted:");
      for (i = 0; i < NUM_OPS; i++)
        $display("        %-7s %0d", op_names[i], vec_count[i]);
      $display("        (>= %0d randomized per op; shift ops carry %0d directed on top;",
                MIN_RANDOM_PER_OP, MIN_DIRECTED_SHIFT_PER_OP);
      $display("         all %0d required directed corner vectors ran)", DIRECTED_N);
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
