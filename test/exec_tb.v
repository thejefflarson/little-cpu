`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// Randomized differential bench for the executor's arithmetic, driving
// `executor` directly with no decoder or pipeline.
//
// For mul and div it is the primary guarantee, because riscv-formal runs under
// RISCV_FORMAL_ALTOPS and never checks that arithmetic at all. The shifts and
// ADD/SUB do have generated checks, but those are `mode bmc`, and an ADD
// off-by-one was measured reaching `tohost` through the `.S` suite and the Sail
// co-simulation while every unit bench stayed silent.
//
// So it asserts its own shape before it asserts anything about the core: the
// three check_* tasks below each exist because this bench could once stop
// checking and still print PASSED.
module exec_tb;
  localparam int RANDOM_VECTORS = 10000;
  localparam logic [1:0] DIVIDE_STATE = 2'b10; // must match executor.v's `divide` state

  // The required coverage, deliberately not a second copy of the loop bounds:
  // these must not move when a loop bound does. If a count check below goes red,
  // restore the loop; never edit the number down to match it.
  localparam int MIN_RANDOM_PER_OP        = 10000;
  localparam int MIN_DIRECTED_SHIFT_PER_OP = 384;

  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  decoder_output in;
  executor_output out;

  executor dut (
    .clk(clk),
    .reset(reset),
    .in(in),
    .out(out)
  );

  int errors = 0;

  // `run_op` resolves its `op_name` argument through this table, which is what
  // makes a typo fatal rather than silent.
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
  localparam int OP_ADD    = 11;
  localparam int OP_SUB    = 12;
  localparam int NUM_OPS   = 13;

  string op_names  [0:NUM_OPS-1];
  int    vec_count [0:NUM_OPS-1];

  // The corner vectors this bench is required to run. None is meaningfully
  // reachable by the randomized sweep, since each names both operands exactly --
  // ten thousand `$random` pairs hit one with probability around 10^4 / 2^64 --
  // so deleting one is a real loss of coverage rather than a rounding error on a
  // big number.
  //
  // The manifest deliberately does not issue them. `run_op` witnesses each
  // (op, rs1, rs2) it drove into the DUT against this list, so a deleted call
  // site is red, and each entry's expected value is checked against the call
  // site, which catches one kept but weakened in place.
  localparam int DIRECTED_N = 9;
  int          dir_op  [0:DIRECTED_N-1];
  logic [31:0] dir_rs1 [0:DIRECTED_N-1];
  logic [31:0] dir_rs2 [0:DIRECTED_N-1];
  logic [31:0] dir_exp [0:DIRECTED_N-1];
  logic        dir_ran [0:DIRECTED_N-1];
  string       dir_why [0:DIRECTED_N-1];
  // Witnessing is a linear scan per vector driven, and there are ~111,000 of
  // them; this counts down so `run_op` can skip the scan once the last directed
  // call site has run. Purely a cost thing.
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
      op_names[OP_ADD]    = "add";
      op_names[OP_SUB]    = "sub";
      for (k = 0; k < NUM_OPS; k++) vec_count[k] = 0;
      dir_pending = DIRECTED_N;

      add_directed(0, OP_MULH,   32'hffffffff, 32'hffffffff, 32'h00000000, "swapped sign enable: MULH(-1,-1)=0");
      add_directed(1, OP_MULHSU, 32'hffffffff, 32'h00000001, 32'hffffffff, "swapped sign enable: MULHSU(-1,1)=-1");
      add_directed(2, OP_MULHU,  32'hffffffff, 32'hffffffff, 32'hfffffffe, "swapped sign enable: MULHU(-1,-1)=0xFFFFFFFE");
      // Specified values, not computed ones.
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

  // The sign-context rule further down, above the shift references, governs
  // these too, and `ref_div`/`ref_rem` are where it bites: their cases are an
  // if / else-if / else over separate statements rather than one `?:` chain,
  // because a chain whose other arms are unsigned literals evaluates
  // `$signed(a) / $signed(b)` unsigned. That has happened at exactly these two
  // functions, where -7 / 2 scored 0x7ffffffc.
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

  // Two's-complement add and sub are signedness-independent bit for bit, so the
  // sign-context hazard below does not apply to these two.
  function automatic logic [31:0] ref_add(input logic [31:0] a, input logic [31:0] b);
    ref_add = a + b;
  endfunction

  function automatic logic [31:0] ref_sub(input logic [31:0] a, input logic [31:0] b);
    ref_sub = a - b;
  endfunction

  // Each shift below is computed in a statement of its own, into a local whose
  // signedness is declared, and is never an arm of a `?:`. Keep them that way. A
  // conditional expression is unsigned if any operand is unsigned, and that
  // pushes down into the context-determined operands, so a `$signed(a) >>> sh`
  // in one arm evaluates as a logical shift -- with no warning, and only for
  // negative operands, so every non-negative vector still agrees. The assignment
  // target cannot rescue it. This repo has been bitten twice, the second time
  // here, where the natural one-line `ref_sra` written as a `?:` arm reported
  // six mismatches against correct hardware.
  //
  // The `b[4:0]` masks are this reference's own model of the rs2[4:0] rule. The
  // vectors drive rs2 far above 31 so the RTL has to agree rather than coincide.
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

  // Checks the oracle, not the core -- no RTL is involved in any call to this. A
  // reference that degraded to a logical shift, or an unsigned divide, agrees
  // with a correct one on every non-negative operand, so it would pass thousands
  // of randomized vectors and then report mismatches against correct hardware.
  //
  // Every expected value here is computed by hand. A literal captured from a run
  // of this bench is derived from the function it is supposed to check, which is
  // the same defect one level down; do the arithmetic on paper.
  task automatic ref_selftest(input string what, input logic [31:0] got,
                               input logic [31:0] want);
    begin
      if (got !== want) begin
        $display("ORACLE BROKEN: %s got=%08x expected=%08x", what, got, want);
        errors++;
      end
    end
  endtask

  // Checks that the bench ran what it says it runs. A loop bound edited to zero
  // during a debugging session and never put back used to print PASSED.
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

  // A randomized vector cannot cover for a deleted directed one: these are the
  // cases random operands do not reach, or reach only by accident.
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

  // When set, run_op zeroes `in` on the cycle after issue and leaves it zeroed.
  // That is what the pipeline does to a running divide -- the divider stall is
  // low on the issue cycle, so decode issues normally and then publishes an
  // operand-fetch bubble that the stall holds -- and it is the one input
  // sequence a divide really sees. A divider that read `in` after issue instead
  // of its own latches would divide by zero here and nowhere else.
  logic bubble_after_issue;

  // The executor is done when it has returned to `init` after the issue edge --
  // immediately for the multiply family and the divide short-circuits, 33 cycles
  // later for a real division. Polling rather than hardcoding a count keeps this
  // correct however long the divider takes; the count is checked at the bottom
  // of this file.
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
        OP_ADD:    in.is_add    = 1'b1;
        OP_SUB:    in.is_sub    = 1'b1;
      endcase

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
      if (bubble_after_issue) in = '0;
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

  // Chosen so a lost sign bit, a dropped mask or an off-by-one fill is visible:
  // INT_MIN, all-ones, INT_MAX, a negative non-trivial value, a positive
  // alternating one, and 1, which walks a single bit off each end.
  localparam int SHIFT_PATTERNS = 6;
  logic [31:0] pattern [0:SHIFT_PATTERNS-1];
  int p, sh;
  logic [31:0] rs2_masked, rs2_dirty;

  initial begin
    reset = 1;
    bubble_after_issue = 1'b0;
    init_tables();
    clear_in();
    repeat (2) @(posedge clk);
    #1;
    reset = 0;

    // The arithmetic behind each literal is written out so a reader can verify
    // it without running anything.
    //
    // MUL: low 32 bits, signedness-independent. (-1)*(-1) = 1; 2^31 * 2 = 2^32,
    // whose low half is zero; 65535^2 = 0xfffe0001.
    ref_selftest("mul(ffffffff,ffffffff)",  ref_mul(32'hffffffff, 32'hffffffff), 32'h00000001);
    ref_selftest("mul(80000000,00000002)",  ref_mul(32'h80000000, 32'h00000002), 32'h00000000);
    ref_selftest("mul(0000ffff,0000ffff)",  ref_mul(32'h0000ffff, 32'h0000ffff), 32'hfffe0001);
    // MULH: signed x signed, high 32. (-1)*(-1) = +1 -> 0.
    // (-2^31)*(-2^31) = 2^62 -> 0x40000000. (-2^31)*2 = 2*(-2^31) = -2^32 ->
    // 0xffffffff, and that last pair recurs in the MULHU/MULHSU cases below.
    ref_selftest("mulh(ffffffff,ffffffff)", ref_mulh(32'hffffffff, 32'hffffffff), 32'h00000000);
    ref_selftest("mulh(80000000,80000000)", ref_mulh(32'h80000000, 32'h80000000), 32'h40000000);
    ref_selftest("mulh(80000000,00000002)", ref_mulh(32'h80000000, 32'h00000002), 32'hffffffff);
    ref_selftest("mulh(00000002,80000000)", ref_mulh(32'h00000002, 32'h80000000), 32'hffffffff);
    // MULHU: unsigned x unsigned, high 32. (2^32-1)^2 = 2^64 - 2^33 + 1, whose
    // high half is 0xfffffffe. (2^32-1)*2^31 = 2^63 - 2^31 = 0x7fff_ffff_8000_0000.
    ref_selftest("mulhu(ffffffff,ffffffff)", ref_mulhu(32'hffffffff, 32'hffffffff), 32'hfffffffe);
    ref_selftest("mulhu(00000002,80000000)", ref_mulhu(32'h00000002, 32'h80000000), 32'h00000001);
    ref_selftest("mulhu(ffffffff,80000000)", ref_mulhu(32'hffffffff, 32'h80000000), 32'h7fffffff);
    // MULHSU: rs1 signed, rs2 unsigned, high 32. Each of these three differs
    // from at least one of the two above on the same operands, which is what
    // makes them a check of the signedness rather than of the multiply:
    //   (-1)*1      = -1    -> 0xffffffff
    //   2 * 2^31    = 2^32  -> 0x00000001 (mulh of the same pair is 0xffffffff)
    //   (-1) * 2^31 = -2^31 -> 0xffffffff (mulhu of the same pair is 0x7fffffff)
    ref_selftest("mulhsu(ffffffff,00000001)", ref_mulhsu(32'hffffffff, 32'h00000001), 32'hffffffff);
    ref_selftest("mulhsu(00000002,80000000)", ref_mulhsu(32'h00000002, 32'h80000000), 32'h00000001);
    ref_selftest("mulhsu(ffffffff,80000000)", ref_mulhsu(32'hffffffff, 32'h80000000), 32'hffffffff);
    // -7 / 2 truncates toward zero to -3 (0xfffffffd). A reference degraded to
    // an unsigned divide scores 0x7ffffffc, which is what
    // `divu(fffffff9,00000002)` legitimately is -- written out just below so the
    // two are visibly different numbers.
    ref_selftest("div(fffffff9,00000002)",  ref_div(32'hfffffff9, 32'h00000002), 32'hfffffffd);
    ref_selftest("div(00000007,fffffffe)",  ref_div(32'h00000007, 32'hfffffffe), 32'hfffffffd);
    ref_selftest("div(fffffff9,fffffffe)",  ref_div(32'hfffffff9, 32'hfffffffe), 32'h00000003);
    ref_selftest("div(00000064,00000000)",  ref_div(32'h00000064, 32'h00000000), 32'hffffffff);
    ref_selftest("div(80000000,ffffffff)",  ref_div(32'h80000000, 32'hffffffff), 32'h80000000);
    ref_selftest("divu(fffffff9,00000002)", ref_divu(32'hfffffff9, 32'h00000002), 32'h7ffffffc);
    ref_selftest("divu(00000064,00000000)", ref_divu(32'h00000064, 32'h00000000), 32'hffffffff);
    // REM takes the sign of the dividend: -7 % 2 = -1, 7 % -2 = +1. An unsigned
    // degradation scores 0x00000001 for the first, which is the second's answer.
    ref_selftest("rem(fffffff9,00000002)",  ref_rem(32'hfffffff9, 32'h00000002), 32'hffffffff);
    ref_selftest("rem(00000007,fffffffe)",  ref_rem(32'h00000007, 32'hfffffffe), 32'h00000001);
    ref_selftest("rem(00000064,00000000)",  ref_rem(32'h00000064, 32'h00000000), 32'h00000064);
    ref_selftest("rem(80000000,ffffffff)",  ref_rem(32'h80000000, 32'hffffffff), 32'h00000000);
    ref_selftest("remu(fffffff9,00000002)", ref_remu(32'hfffffff9, 32'h00000002), 32'h00000001);
    ref_selftest("remu(00000064,00000000)", ref_remu(32'h00000064, 32'h00000000), 32'h00000064);
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
    // Two's complement wraps: -1 + 1 = 0; INT_MAX + 1 overflows into the sign
    // bit; 0 - 1 wraps to all-ones; INT_MIN - 1 wraps to INT_MAX.
    ref_selftest("add(ffffffff,00000001)", ref_add(32'hffffffff, 32'h00000001), 32'h00000000);
    ref_selftest("add(7fffffff,00000001)", ref_add(32'h7fffffff, 32'h00000001), 32'h80000000);
    ref_selftest("sub(00000000,00000001)", ref_sub(32'h00000000, 32'h00000001), 32'hffffffff);
    ref_selftest("sub(80000000,00000001)", ref_sub(32'h80000000, 32'h00000001), 32'h7fffffff);
    if (errors != 0) begin
      $display("FAILED: the reference model is broken; no core result below means anything");
      $fatal(1);
    end

    // The required directed vectors: the cases a swapped sign enable gets wrong.
    run_op("mulh",   32'hffffffff, 32'hffffffff, 32'h00000000);
    run_op("mulhsu", 32'hffffffff, 32'h00000001, 32'hffffffff);
    run_op("mulhu",  32'hffffffff, 32'hffffffff, 32'hfffffffe);

    run_op("div",  32'h00000064, 32'h00000000, 32'hffffffff);
    run_op("divu", 32'h00000064, 32'h00000000, 32'hffffffff);
    run_op("rem",  32'h00000064, 32'h00000000, 32'h00000064);
    run_op("remu", 32'h00000064, 32'h00000000, 32'h00000064);
    run_op("div",  32'h80000000, 32'hffffffff, 32'h80000000);
    run_op("rem",  32'h80000000, 32'hffffffff, 32'h00000000);

    // Every shift amount 0..31 against each pattern, driven twice: once with a
    // clean rs2 and once with all 27 upper bits set. `$random` reaches every
    // amount with overwhelming probability but does not guarantee it, and the
    // pair is what states the rs2[4:0] rule.
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

    // `$random`'s rs2 is a full 32-bit word, so most shift vectors here also
    // carry a shift amount above 31.
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
      run_op("add",    a, b, ref_add(a, b));
      run_op("sub",    a, b, ref_sub(a, b));
    end

    // The same divide family again, this time with `in` bubbled the cycle after
    // issue. Both signs of both operands, so the magnitude conversion and the
    // sign restoration are exercised from latched state alone; a full-width
    // divisor, which no capped proof reaches; and the two short-circuits, which
    // must still answer from the issue cycle.
    bubble_after_issue = 1'b1;
    run_op("div",  32'h00000064, 32'h00000007, ref_div(32'h00000064, 32'h00000007));
    run_op("div",  32'hffffff9c, 32'h00000007, ref_div(32'hffffff9c, 32'h00000007));
    run_op("div",  32'h00000064, 32'hfffffff9, ref_div(32'h00000064, 32'hfffffff9));
    run_op("div",  32'hffffff9c, 32'hfffffff9, ref_div(32'hffffff9c, 32'hfffffff9));
    run_op("rem",  32'hffffff9c, 32'h00000007, ref_rem(32'hffffff9c, 32'h00000007));
    run_op("rem",  32'h00000064, 32'hfffffff9, ref_rem(32'h00000064, 32'hfffffff9));
    run_op("divu", 32'hdeadbeef, 32'hfffffffe, ref_divu(32'hdeadbeef, 32'hfffffffe));
    run_op("remu", 32'hdeadbeef, 32'hfffffffe, ref_remu(32'hdeadbeef, 32'hfffffffe));
    run_op("divu", 32'hffffffff, 32'h00000001, ref_divu(32'hffffffff, 32'h00000001));
    run_op("remu", 32'hffffffff, 32'hfffffffe, ref_remu(32'hffffffff, 32'hfffffffe));
    run_op("div",  32'h80000000, 32'hffffffff, 32'h80000000);
    run_op("divu", 32'h00000064, 32'h00000000, 32'hffffffff);
    bubble_after_issue = 1'b0;

    // Everything above is a claim about the core; these two are the claim that
    // it happened at all.
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

  // 32 restoring-division iterations plus one capture cycle, checked against
  // every real division performed in the run above.
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
