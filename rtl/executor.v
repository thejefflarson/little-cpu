`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module executor(
  input  logic clk,
  input  logic reset,

  // inputs
  input  decoder_output in,
  // ADR-0009-style stall broadcast from the accessor (ADR-0015): high for the
  // one cycle a load's response is still in flight there. Upstream of the
  // stalling stage freezes — the executor is upstream of the accessor, so it
  // must freeze (emit a bubble) rather than advance, or a fresh instruction
  // would collide with the completing load over the accessor's single output
  // register and the single-port memory bus.
  input  logic accessor_stall,
  // outputs
  output executor_output out,
  // ADR-0009: broadcast to littlecpu.v (and from there back to the decoder)
  // whenever a multi-cycle divide is in flight, so decode freezes upstream
  // and bubbles downstream until it drains.
  output logic stalled
);
  logic [31:0] rs1, rs2;
  assign rs1 = in.rs1;
  assign rs2 = in.rs2;

  // Signed div/rem operate on the two's-complement magnitude of the operands; the
  // restoring divider below only performs unsigned division. Quotient/remainder signs
  // are restored from the original operand signs once the loop completes.
  logic [31:0] div_x, div_y;
  assign div_x = (in.is_div || in.is_rem) && rs1[31] ? -rs1 : rs1;
  assign div_y = (in.is_div || in.is_rem) && rs2[31] ? -rs2 : rs2;

  logic [1:0]  state;
  localparam init = 2'b00;
  localparam divide = 2'b10;
  logic [6:0]  mul_div_counter;
  logic [63:0] mul_div_x, mul_div_y;
  logic [63:0] mul_div_store;
  always_comb
    stalled = state != init;

  // ADR-0009: once decode is stalled for a divide, it bubbles (rather than
  // literally holding `in` unchanged) so the executor doesn't misread a
  // stale in-flight instruction the moment it returns to `init` — see
  // rtl/decoder.v. That means `in` cannot be relied on at completion time,
  // so the op-select and operand signs the last iteration needs are latched
  // here at issue, once, instead of read live off `in` 32 iterations later.
  logic op_is_div, op_is_divu, op_is_rem, op_is_remu, op_sign_x, op_sign_y;

 `ifdef RISCV_FORMAL_ALTOPS
  // The operands as latched by the ALTOPS issue arm below, named so the
  // completion arm reads them as operands rather than as slices of the
  // divider's working registers. Raw rs1/rs2 -- ALTOPS does no magnitude
  // conversion, so no sign wrapper applies here (contrast ADR-0012).
  logic [31:0] div_alt_rs1, div_alt_rs2;
  assign div_alt_rs1 = mul_div_x[31:0];
  assign div_alt_rs2 = mul_div_y[62:31];
 `endif

  // multiply: continuous assignments, so the product tracks the operands every cycle
  // instead of being latched once at time 0. sign_x covers is_mulh (signed rs1) and
  // is_mulhsu (rs1 is signed, rs2 is unsigned); sign_y covers only is_mulh. Extension
  // bits come from bit 31 (the true sign bit), not bit 0.
  logic mul_sign_x, mul_sign_y;
  assign mul_sign_x = in.rs1[31] & (in.is_mulh | in.is_mulhsu);
  assign mul_sign_y = in.rs2[31] & in.is_mulh;
  logic signed [63:0] multiply;
  assign multiply = $signed({mul_sign_x, in.rs1}) * $signed({mul_sign_y, in.rs2});

  // state machine
  always_ff @(posedge clk) begin
    if (reset) begin
      state <= init;
      out <= 0;
      mul_div_counter <= 0;
      mul_div_store <= 0;
      mul_div_x <= 0;
      mul_div_y <= 0;
      op_is_div <= 0;
      op_is_divu <= 0;
      op_is_rem <= 0;
      op_is_remu <= 0;
      op_sign_x <= 0;
      op_sign_y <= 0;
    end else if (accessor_stall) begin
      // Freeze: emit a bubble and hold every other register (state,
      // mul_div_*, op_*) unchanged. Never overlaps with a real divide in
      // progress — decode already freezes everything upstream of the
      // executor while dividing, so a load can't reach the accessor (and
      // assert accessor_stall) until the divide has long since drained.
      out <= 0;
    end else begin
      (* parallel_case, full_case *)
      case (state)
        init: begin
          // ADR-0009: a bubble (in.valid == 0) propagates straight through as
          // a bubble here too, except when it's the one real cycle a divide
          // is issued (out.valid held low below until the divide completes).
          out.valid <= in.valid;
         `ifdef RISCV_FORMAL
          // Latched here, once, at issue -- same reasoning as op_is_div/
          // op_sign_x below: this is the only cycle `in` is trustworthy for
          // a divide, and out.rd/out.rd_data follow the same rule (set here,
          // held unchanged for the whole divide, read back at completion).
          out.rvfi <= in.rvfi;
         `endif
          out.rd <= in.rd;
          out.rd_data <= 0;
          out.mem_addr <= in.mem_addr;
          out.mem_data <= in.rs2;
          out.is_lb <= in.is_lb;
          out.is_lbu <= in.is_lbu;
          out.is_lh <= in.is_lh;
          out.is_lhu <= in.is_lhu;
          out.is_lw <= in.is_lw;
          out.is_sb <= in.is_sb;
          out.is_sh <= in.is_sh;
          out.is_sw <= in.is_sw;
          (* parallel_case, full_case *)
          case (1'b1)
            in.is_add: out.rd_data <= rs1 + rs2;
            in.is_lui: out.rd_data <= rs1;
            in.is_sub: out.rd_data <= rs1 - rs2;
            // RV32I shift amount is rs2[4:0] only (RISC-V spec Vol I §2.4.2);
            // the upper 27 bits are ignored, not folded into the shift as a
            // wider count.
            in.is_sll: out.rd_data <= rs1 << rs2[4:0];
            in.is_slt: out.rd_data <= {31'b0, $signed(rs1) < $signed(rs2)};
            in.is_sltu: out.rd_data <= {31'b0, rs1 < rs2};
            in.is_xor: out.rd_data <= rs1 ^ rs2;
            in.is_srl: out.rd_data <= rs1 >> rs2[4:0];
            in.is_sra: out.rd_data <= $signed(rs1) >>> rs2[4:0];
            in.is_or: out.rd_data <= rs1 | rs2;
            in.is_and: out.rd_data <= rs1 & rs2;
            in.is_mul || in.is_mulh || in.is_mulhu || in.is_mulhsu: begin
             `ifndef RISCV_FORMAL_ALTOPS
              if (in.is_mul) begin
                out.rd_data <= multiply[31:0];
              end else begin
                out.rd_data <= multiply[63:32];
              end
             `else
              (* parallel_case, full_case *)
              case (1'b1)
                in.is_mul: out.rd_data <= (in.rs1 + in.rs2) ^ 32'h5876063e;
                in.is_mulh: out.rd_data <= (in.rs1 + in.rs2) ^ 32'hf6583fb7;
                in.is_mulhu: out.rd_data <= (in.rs1 + in.rs2) ^ 32'h949ce5e8;
                in.is_mulhsu: out.rd_data <= (in.rs1 - in.rs2) ^ 32'hecfbe137;
              endcase
             `endif
            end

            in.is_div || in.is_divu || in.is_rem || in.is_remu: begin
              // Latched regardless of which sub-path fires below: harmless
              // for the div-by-zero/overflow short-circuits (state stays
              // init, so completion never reads them back), and required for
              // the real divide (see the comment on their declaration).
              op_is_div <= in.is_div;
              op_is_divu <= in.is_divu;
              op_is_rem <= in.is_rem;
              op_is_remu <= in.is_remu;
              op_sign_x <= in.rs1[31];
              op_sign_y <= in.rs2[31];
             `ifndef RISCV_FORMAL_ALTOPS
              if (rs2 == 0) begin
                // Division by zero per RISC-V spec Vol I §7.2
                if (in.is_rem || in.is_remu) out.rd_data <= rs1; // remainder = dividend
                else out.rd_data <= 32'hffffffff; // quotient = -1 (div) or MAX_UINT (divu)
                // state stays init; no division needed
              end else if ((in.is_div || in.is_rem) &&
                           rs1 == 32'h80000000 && rs2 == 32'hffffffff) begin
                // Signed overflow: INT_MIN / -1 per RISC-V spec Vol I §7.2
                if (in.is_div) out.rd_data <= 32'h80000000; // quotient = INT_MIN
                else out.rd_data <= 32'b0; // remainder = 0
                // state stays init; no division needed
              end else begin
                mul_div_counter <= 32;
                state <= divide;
                mul_div_store <= 0;
                mul_div_x <= {32'b0, div_x};
                mul_div_y <= {1'b0, div_y, 31'b0};
                // The result isn't ready this cycle — ADR-0009's replay fix:
                // downstream drains a bubble, not a repeat of whatever the
                // executor last held, for the entire multi-cycle divide.
                out.valid <= 1'b0;
              end
             `else
              mul_div_counter <= 32;
              state <= divide;
              mul_div_store <= 0;
              mul_div_x <= {32'b0, rs1};
              mul_div_y <= {1'b0, rs2, 31'b0};
              out.valid <= 1'b0;
             `endif
            end
            // A Zicsr access does not appear here at all: rtl/csrs.v reads
            // and commits it in decode (ADR-0005) and the read result
            // arrives as `is_add` with rs2 zeroed, so by the time it reaches
            // this stage it IS an add. That is why decoder_output no longer
            // carries is_csrrw/is_csrrs/is_csrrc -- see rtl/structs.v.
            in.is_ecall || in.is_ebreak: ;
            in.is_valid_instr: ;
          endcase // case (1'b1)
        end // case: init

        divide: begin
         `ifndef RISCV_FORMAL_ALTOPS
          if (mul_div_counter > 0) begin
            if (mul_div_x >= mul_div_y) begin
              mul_div_store <= (mul_div_store << 1) | 1;
              mul_div_x <= mul_div_x - mul_div_y;
            end else begin
              mul_div_store <= mul_div_store << 1;
            end
            mul_div_y <= mul_div_y >> 1;
            mul_div_counter <= mul_div_counter - 1;
          end else begin
            // Uses the op-select and sign bits latched at issue, not `in`
            // (which decode has long since bubbled) — see the comment on
            // their declaration above.
            (* parallel_case, full_case *)
            case (1'b1)
              op_is_div: out.rd_data <= op_sign_x != op_sign_y ? -mul_div_store[31:0] : mul_div_store[31:0];
              op_is_divu: out.rd_data <= mul_div_store[31:0];
              op_is_rem: out.rd_data <= op_sign_x ? -mul_div_x[31:0] : mul_div_x[31:0];
              op_is_remu: out.rd_data <= mul_div_x[31:0];
            endcase
            out.valid <= 1'b1;
            state <= init;
          end
         `else
          // Read the operands LATCHED AT ISSUE, not `in`. ALTOPS collapses the
          // divide to a single cycle, so by the time this arm runs `in` already
          // holds the next decoded instruction and `in.rs1`/`in.rs2` are that
          // instruction's operands -- the placeholder would be computed from
          // the wrong values. Same reasoning as the op_is_*/op_sign_* latches
          // above: this is not the cycle `in` is trustworthy.
          //
          // The ALTOPS issue arm latches raw rs1/rs2 (unlike the real divider's
          // arm, which latches magnitudes per ADR-0012), so these are exactly
          // the values riscv-formal's placeholder model expects.
          (* parallel_case, full_case *)
          case (1'b1)
            op_is_div: out.rd_data <= (div_alt_rs1 - div_alt_rs2) ^ 32'h7f8529ec;
            op_is_divu: out.rd_data <= (div_alt_rs1 - div_alt_rs2) ^ 32'h10e8fd70;
            op_is_rem: out.rd_data <= (div_alt_rs1 - div_alt_rs2) ^ 32'h8da68fa5;
            op_is_remu: out.rd_data <= (div_alt_rs1 - div_alt_rs2) ^ 32'h3138d0e1;
          endcase
          out.valid <= 1'b1;
          state <= init;
         `endif
        end // case: divide
        default: ;
      endcase
    end
  end

  // state machine
 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // FACT       reset is asserted before the first clock edge.
  // DISCHARGED NOWHERE. Structural: every harness that instantiates this core
  //            (test/testbench.v, formal/*.sv, the generated ladder's
  //            RISCV_FORMAL_RESET_CYCLES) drives reset high initially. No
  //            check asserts it, and "nowhere" is the honest answer (ADR-0049).
  // SCOPE      the whole task. Unguarded, so in force over every assertion in
  //            this block -- which is what it was written for: without it no
  //            assertion here is meaningful, since `state` and the mul_div
  //            registers have no defined value before the first edge.
  // assume we've reset at clk 0
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // `state` otherwise has no defined value before the first clock edge applies
  // `reset`, which lets the solver "start" mid-divide for free at step 0 —
  // pin it down explicitly so the arithmetic assertions below only fire on
  // real, freshly-issued operations.
  initial state = init;
  // The divide-family assertions below span many cycles; without this, the
  // solver can re-assert `reset` mid-divide to jump straight from `divide` to
  // `init` (out.rd_data zeroed by the reset branch, not computed by the
  // divider) and "complete" a division in a handful of steps. Reset is a
  // once-at-the-start pulse everywhere else in this design, so assume it
  // stays low for the remainder of the trace.
  //
  // FACT       reset never returns after the first cycle.
  // DISCHARGED NOWHERE. True of every harness in the tree; asserted by none.
  // SCOPE      the whole task -- LARGER than what it was written for. The
  //            paragraph above is about the divide-family assertions, but this
  //            is an unguarded `always_comb assume`, so it equally excuses the
  //            four multiply assertions and the ADR-0015 freeze block from
  //            ever seeing a mid-trace reset. That is harmless here (a
  //            re-asserted reset is not a real environment) and is written
  //            down because ADR-0049 clause 3 requires the reader to be told
  //            the scope rather than left to infer it from proximity.
  always_comb if (clocked) assume(!reset);

  // This component proof stands alone (no accessor instantiated), so
  // accessor_stall is a free input. It used to be assumed away entirely
  // (`assume(!accessor_stall)`), which scoped the freeze branch at the top
  // of the state machine out of this proof altogether. It is now left
  // completely free — not even ADR-0015's at-most-one-consecutive-cycle
  // bound is assumed, which makes this stronger than the real environment:
  // a held stall just keeps the freeze obligations below in force. The
  // price is a `!$past(accessor_stall)` guard on each multiply assertion
  // further down: a frozen edge computes nothing (out is bubbled), so an
  // ungated assertion there would be asserting arithmetic about the bubble.
  // The divide-state invariants need no such guard (a freeze holds every
  // register they mention), and the divide completion assertions' guard
  // ($past(state) == divide && state == init) already implies the
  // transition edge was not frozen — a frozen edge cannot change state.

  // ADR-0015 freeze coverage: the cycle after accessor_stall is asserted,
  // the executor emitted a bubble and held everything else — state, the
  // divider datapath, and the op/sign latches — unchanged. This is the
  // `else if (accessor_stall)` branch of the state machine, asserted rather
  // than assumed out of existence. Guarded on !$past(reset) because reset
  // wins over the freeze in the RTL (and the pre-reset initial values of
  // the mul_div registers are free).
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(accessor_stall)) begin
      assert(out == '0);
      assert(state == $past(state));
      assert(mul_div_counter == $past(mul_div_counter));
      assert(mul_div_store == $past(mul_div_store));
      assert(mul_div_x == $past(mul_div_x));
      assert(mul_div_y == $past(mul_div_y));
      assert(op_is_div == $past(op_is_div));
      assert(op_is_divu == $past(op_is_divu));
      assert(op_is_rem == $past(op_is_rem));
      assert(op_is_remu == $past(op_is_remu));
      assert(op_sign_x == $past(op_sign_x));
      assert(op_sign_y == $past(op_sign_y));
    end

  // ---- Executor arithmetic BMC (ADR-0006 component proof #2 / ADR-0010) ----
  // The *primary* guarantee for real mul/div arithmetic is the randomized
  // differential bench, test/exec_tb.v (>=10,000 vectors per op) — riscv-formal
  // runs under RISCV_FORMAL_ALTOPS and never checks this arithmetic at all. This
  // BMC is secondary and bounded-effort per ADR-0010. It was previously vacuous
  // (PASS 0 0): these are its first real assertions.
  //
  // The op-select flags (is_add, is_mul, ...) are guaranteed mutually exclusive
  // by the decoder in the real pipeline; this component proof stands alone, so
  // that has to be an explicit environmental assumption or the solver can pick
  // nonsensical combinations no real instruction produces.
  //
  // FACT       the decoder emits at most one op-select flag per instruction.
  // DISCHARGED rtl/decoder.v's `one_of` assertion -- `assert($onehot(...))`
  //            under `instr_valid`, in components_decoder, which runs in CI
  //            (.github/workflows/ci.yml's `components` job). This is the ONE
  //            assume in this repo with a real, running discharge; every other
  //            one is believed on structural grounds (ADR-0049's census).
  // SCOPE      the whole task, and deliberately so. It was written for the
  //            arithmetic assertions but is equally correct over the ADR-0015
  //            freeze block and every divide invariant, because the decoder
  //            really does emit at most one flag on every cycle of all of
  //            them. Checked against rtl/structs.v while auditing: all 29
  //            `is_*` fields of `decoder_output` are listed, and
  //            `is_valid_instr` is correctly not among them. Adding a flag to
  //            the struct without adding it here silently widens the
  //            environment for every assertion below.
  always_comb assume($onehot0({in.is_add, in.is_sub, in.is_xor, in.is_or, in.is_and,
    in.is_sll, in.is_slt, in.is_sltu, in.is_srl, in.is_sra, in.is_lui,
    in.is_mul, in.is_mulh, in.is_mulhu, in.is_mulhsu,
    in.is_div, in.is_divu, in.is_rem, in.is_remu,
    in.is_lb, in.is_lbu, in.is_lh, in.is_lhu, in.is_lw, in.is_sb, in.is_sh, in.is_sw,
    in.is_ecall, in.is_ebreak}));

  // THESE FOUR ASSERTIONS DO NOT SEE 32-BIT OPERANDS TODAY, DESPITE THE
  // PARAGRAPH BELOW (ADR-0049 finding F1). The divide invariant further down
  // caps operands with two UNGUARDED `always_comb assume(in.rs1 <= 32'h0f)`
  // statements, and an unguarded assume is proof-global: it constrains every
  // assertion in this module, including these. With operands in 0..15 every
  // product's high half is zero and every operand's bit 31 is zero, so
  // MULH/MULHU/MULHSU here assert `out.rd_data == 0` and nothing more.
  //
  // Measured by mutation (scratch copy, `sby -f components.sby executor`):
  // forcing `multiply[63:32]` to zero PASSES; `mul_sign_x = 1'b0` PASSES;
  // `mul_sign_y = 1'b0` PASSES; making MULHSU treat rs2 as signed PASSES.
  // Three of the four multiplier defects ADR-0010 names by hand survive the
  // proof written to catch them. A low-half off-by-one and taking the sign
  // from bit 0 are still caught in 0.5s, which is what makes it insidious:
  // these assertions are narrowed, not dead, and the task reads green either
  // way. test/exec_tb.v -- ADR-0010's PRIMARY guarantee -- is what actually
  // covers this today, over 10,000 randomized vectors per op with the
  // directed MULH(-1,-1)/MULHSU(-1,1)/MULHU(-1,-1) cases.
  //
  // The fix is to guard the cap down to the divide assertions, which is
  // exactly what the paragraph below says these do not need. Not done here:
  // those lines are owned by in-flight work on the executor's real arithmetic,
  // and narrowing them changes what this proof proves.
  //
  // THE SAME CAP ALSO BLANKS THE SIGNED DIVIDE PATH (ADR-0049 finding F5), and
  // that one is not fixed by narrowing the cap to "the divide assertions" --
  // it needs new assertions. With operands in 0..15, `op_sign_x` and
  // `op_sign_y` are constant zero, so the two `assert(op_sign_* == in.rs*[31])`
  // tie-backs below read `assert(0 == 0)`, and the only completion assertions
  // in this proof are the UNSIGNED pair (`divu_ref` / `remu_ref`). There is no
  // assertion anywhere that the signed DIV/REM sign restore happens at all.
  // Measured the same way: deleting `op_sign_x != op_sign_y ? -...` from the
  // `op_is_div` capture PASSES, and deleting `op_sign_x ? -...` from the
  // `op_is_rem` capture PASSES. ADR-0012's sign wrapper -- the whole reason
  // this divider can be unsigned -- has no formal coverage in this task.
  // test/exec_tb.v covers it; nothing here does.
  //
  // Multiply: a single combinational stage, so full 32-bit-operand correctness
  // is provable directly (no restriction needed). Independently re-derives
  // MUL/MULH/MULHU/MULHSU against plain SystemVerilog signed/unsigned
  // multiplication — this catches exactly the swapped-sign-enable / wrong-
  // sign-bit class of bug this ticket fixes, not just re-states the RTL.
  logic [63:0] mul_ref_uu;
  logic signed [63:0] mul_ref_ss, mul_ref_su;
  assign mul_ref_uu = in.rs1 * in.rs2;                            // mul, mulhu
  assign mul_ref_ss = $signed(in.rs1) * $signed(in.rs2);          // mulh
  assign mul_ref_su = $signed(in.rs1) * $signed({1'b0, in.rs2});  // mulhsu
  // Sliced into plain 32-bit halves so $past() below applies to a simple
  // signal rather than a part-select of a $past() result.
  logic [31:0] mul_ref_lo, mul_ref_uu_hi, mul_ref_ss_hi, mul_ref_su_hi;
  assign mul_ref_lo = mul_ref_uu[31:0];
  assign mul_ref_uu_hi = mul_ref_uu[63:32];
  assign mul_ref_ss_hi = mul_ref_ss[63:32];
  assign mul_ref_su_hi = mul_ref_su[63:32];

  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && !$past(accessor_stall) && $past(state) == init && $past(in.is_mul))
      assert(out.rd_data == $past(mul_ref_lo));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && !$past(accessor_stall) && $past(state) == init && $past(in.is_mulh))
      assert(out.rd_data == $past(mul_ref_ss_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && !$past(accessor_stall) && $past(state) == init && $past(in.is_mulhu))
      assert(out.rd_data == $past(mul_ref_uu_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && !$past(accessor_stall) && $past(state) == init && $past(in.is_mulhsu))
      assert(out.rd_data == $past(mul_ref_su_hi));

  // Divide: an end-to-end assertion unrolling the restoring divider's full
  // 33-cycle latency (out.rd_data correct N cycles after issue) is not provable
  // by k-induction as such a property — sby's default engine depth here is 20
  // steps, well short of 33, and induction can't "remember" that far back
  // without restating the whole trajectory as an invariant anyway. So this
  // proves a loop invariant instead (below), which needs only one induction
  // step regardless of how many iterations the divider takes, and separately
  // restricts operands to 4 bits (0-15) so the invariant's intermediate
  // arithmetic stays comfortably inside 64 bits without wraparound (ADR-0010:
  // "restrict it... record the restriction as a comment in the sby task" —
  // recorded here rather than in formal/components.sby because the restriction
  // is a property of this proof's invariant, so it belongs next to the
  // invariant it constrains).
  //
  // The invariant, proved by ordinary one-step induction instead of unrolling:
  // at every point while state == divide,
  //   dividend == remainder-so-far + quotient-so-far * (divisor << iterations-remaining)
  // and remainder-so-far < (divisor << iterations-remaining). Both are
  // preserved by each iteration of the shift/subtract loop regardless of how
  // many iterations have run, which is exactly what makes them provable
  // without unrolling the whole operation. At the last iteration (0 remaining)
  // this collapses to the ordinary division identity dividend == remainder +
  // quotient * divisor with 0 <= remainder < divisor, tying the invariant to
  // real division and — combined with the assertions below — to out.rd_data.
  always_comb assume(in.rs1 <= 32'h0000000f);
  always_comb assume(in.rs2 <= 32'h0000000f);
  // The pipeline (once ADR-0004's stall-only interlock lands) holds `in` steady
  // for the full duration of a multi-cycle divide; assume that environment here
  // so the opcode selection at capture time can't be corrupted by a free-
  // running formal input mid-divide (the real bug this would otherwise hide:
  // executor.v reads in.is_div/is_divu/is_rem/is_remu fresh at capture time,
  // not latched at issue).
  always_ff @(posedge clk)
    // Covers both the first divide-state cycle (state == divide; otherwise
    // that cycle's `in` is a free input that can disagree with what was just
    // latched into mul_div_x/mul_div_y at issue) and the capture cycle
    // ($past(state) == divide, state == init; otherwise the capture edge's
    // `in.is_divu`/`is_remu` used below can disagree with the operands the
    // divider actually ran on).
    if (clocked && (state == divide || $past(state) == divide)) begin
      assume(in.rs1 == $past(in.rs1));
      assume(in.rs2 == $past(in.rs2));
      assume(in.is_div == $past(in.is_div));
      assume(in.is_divu == $past(in.is_divu));
      assume(in.is_rem == $past(in.is_rem));
      assume(in.is_remu == $past(in.is_remu));
    end

  // Ties the op_is_*/op_sign_* latches (see their declaration
  // above) back to `in` while dividing, mirroring what the environmental
  // assumption above already establishes about `in` staying constant. Without
  // this as its own stated (one-step-inductive) invariant, k-induction has no
  // intermediate fact linking a latch taken once at issue to `in.is_divu`
  // read many cycles later at the completion assertions below, and induction
  // (not just the base case) fails to close.
  always_comb if (state == divide) assert(op_is_div == in.is_div);
  always_comb if (state == divide) assert(op_is_divu == in.is_divu);
  always_comb if (state == divide) assert(op_is_rem == in.is_rem);
  always_comb if (state == divide) assert(op_is_remu == in.is_remu);
  always_comb if (state == divide) assert(op_sign_x == in.rs1[31]);
  always_comb if (state == divide) assert(op_sign_y == in.rs2[31]);

  // k-induction otherwise has no reason to rule out an (unreachable) starting
  // state with a wild mul_div_counter value — bound it to the range the RTL
  // actually drives it through, so the counter can serve as an exact iteration
  // count ("how many halvings of the divisor remain") in the invariant below.
  always_comb if (state == divide) assert(mul_div_counter <= 32);

  // Ties the actual mul_div_y register to the same "divisor scaled by
  // iterations remaining" quantity the invariant below reasons about — without
  // this, mul_div_y is a free register as far as the solver's concerned, and
  // the comparison the RTL actually branches on (mul_div_x >= mul_div_y) has
  // no connection to what the invariant expects it to be. Scoped to
  // mul_div_counter > 0: at counter == 0, mul_div_y has taken its last
  // (possibly LSB-dropping) right-shift and is no longer read by anything.
  always_comb
    if (state == divide && mul_div_counter > 0)
      assert(mul_div_y == ({32'b0, div_y} << (mul_div_counter - 1)));

  // Uses div_x/div_y (what the divider actually loaded), not in.rs1/in.rs2
  // directly, so this covers is_div/is_rem's abs-value conversion too, not just
  // the unsigned ops.
  logic [63:0] div_scaled_divisor;
  assign div_scaled_divisor = ({32'b0, div_y}) << mul_div_counter;
  always_comb
    if (state == divide)
      assert(mul_div_x + mul_div_store * div_scaled_divisor == {32'b0, div_x});
  always_comb
    if (state == divide)
      assert(mul_div_x < div_scaled_divisor);
  // Without this, the equality invariant above is only an equation over
  // 64-bit *modular* arithmetic, and the solver can satisfy it with a
  // mul_div_store far outside any value the real 32-iteration algorithm would
  // ever produce (a wraparound "solution" mod 2^64) as an induction starting
  // point. True quotients never exceed the dividend for a divisor >= 1, so
  // this is also a real fact about the algorithm, not just a proof crutch.
  always_comb
    if (state == divide)
      assert(mul_div_store <= {32'b0, div_x});

  logic [31:0] divu_ref, remu_ref;
  assign divu_ref = (in.rs2 == 0) ? 32'hffffffff : (in.rs1 / in.rs2);
  assign remu_ref = (in.rs2 == 0) ? in.rs1 : (in.rs1 % in.rs2);

  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(in.is_divu))
      assert(out.rd_data == divu_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(in.is_remu))
      assert(out.rd_data == remu_ref);
 `endif
endmodule
