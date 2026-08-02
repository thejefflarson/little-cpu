`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module executor(
  input  logic clk,
  input  logic reset,

  // inputs
  input  decoder_output in,
  // High for the one cycle a load's response is still in flight in the accessor
  // (ADR-0015). The executor is upstream of it and must freeze rather than
  // advance, or a fresh instruction collides with the completing load over the
  // accessor's single output register and the single-port memory bus.
  input  logic accessor_stall,
  // outputs
  output executor_output out,
  // Broadcast to littlecpu.v, and from there back to the decoder, whenever a
  // multi-cycle divide is in flight.
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

  // Once decode is stalled for a divide it bubbles, rather than holding `in`
  // unchanged, so that the executor cannot misread a stale in-flight instruction
  // the moment it returns to `init` (ADR-0009). `in` is therefore not to be
  // trusted at completion time, and the op-select and operand signs the last
  // iteration needs are latched here at issue instead.
  logic op_is_div, op_is_divu, op_is_rem, op_is_remu, op_sign_x, op_sign_y;

 `ifdef RISCV_FORMAL_ALTOPS
  // The operands as latched by the ALTOPS issue arm below, named so the
  // completion arm reads them as operands rather than as slices of the divider's
  // working registers. Raw rs1/rs2: ALTOPS does no magnitude conversion, so the
  // sign wrapper above does not apply.
  logic [31:0] div_alt_rs1, div_alt_rs2;
  assign div_alt_rs1 = mul_div_x[31:0];
  assign div_alt_rs2 = mul_div_y[62:31];
 `endif

  // Continuous assignments, so the product tracks the operands every cycle
  // instead of being latched once at time 0. sign_x covers is_mulh (signed rs1)
  // and is_mulhsu (rs1 signed, rs2 unsigned); sign_y covers only is_mulh.
  logic mul_sign_x, mul_sign_y;
  assign mul_sign_x = in.rs1[31] & (in.is_mulh | in.is_mulhsu);
  assign mul_sign_y = in.rs2[31] & in.is_mulh;
  logic signed [63:0] multiply;
  assign multiply = $signed({mul_sign_x, in.rs1}) * $signed({mul_sign_y, in.rs2});

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
      // Freeze: emit a bubble and hold every other register unchanged. This
      // never overlaps a divide in progress, because decode freezes everything
      // upstream while dividing, so no load can reach the accessor until the
      // divide has drained.
      out <= 0;
    end else begin
      (* parallel_case, full_case *)
      case (state)
        init: begin
          // A bubble propagates straight through, except on the cycle a divide
          // issues, where out.valid is held low below until it completes.
          out.valid <= in.valid;
         `ifdef RISCV_FORMAL
          // Latched at issue for the same reason as op_is_div/op_sign_x below,
          // as are out.rd and out.rd_data: this is the only cycle `in` is
          // trustworthy for a divide.
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
            // The RV32I shift amount is rs2[4:0] only (spec Vol I §2.4.2): the
            // upper 27 bits are ignored, not folded in as a wider count.
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
              // Latched regardless of which sub-path fires below: harmless for
              // the short-circuits, which never leave `init` and so never read
              // them back, and required for the real divide.
              op_is_div <= in.is_div;
              op_is_divu <= in.is_divu;
              op_is_rem <= in.is_rem;
              op_is_remu <= in.is_remu;
              op_sign_x <= in.rs1[31];
              op_sign_y <= in.rs2[31];
             `ifndef RISCV_FORMAL_ALTOPS
              // The two specified results (spec Vol I §7.2). Both stay in
              // `init`, so neither enters the divider.
              if (rs2 == 0) begin
                if (in.is_rem || in.is_remu) out.rd_data <= rs1; // remainder = dividend
                else out.rd_data <= 32'hffffffff; // quotient = -1 (div) or MAX_UINT (divu)
              end else if ((in.is_div || in.is_rem) &&
                           rs1 == 32'h80000000 && rs2 == 32'hffffffff) begin
                // Signed overflow: INT_MIN / -1.
                if (in.is_div) out.rd_data <= 32'h80000000; // quotient = INT_MIN
                else out.rd_data <= 32'b0; // remainder = 0
              end else begin
                mul_div_counter <= 32;
                state <= divide;
                mul_div_store <= 0;
                mul_div_x <= {32'b0, div_x};
                mul_div_y <= {1'b0, div_y, 31'b0};
                // The result is not ready this cycle, so downstream drains a
                // bubble rather than a repeat of whatever the executor last held
                // (ADR-0009).
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
            // A Zicsr access never appears here: rtl/csrs.v reads and commits it
            // in decode (ADR-0005), and the read result arrives as `is_add` with
            // rs2 zeroed, so by this stage it is an add.
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
            // The op-select and sign bits latched at issue, not `in`, which
            // decode has long since bubbled.
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
          // The operands latched at issue, not `in`. ALTOPS collapses the divide
          // to a single cycle, so by the time this arm runs `in` already holds
          // the next decoded instruction and the placeholder would be computed
          // from its operands. The ALTOPS issue arm latches raw rs1/rs2 rather
          // than ADR-0012's magnitudes, which is what riscv-formal's placeholder
          // model expects.
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

 `ifdef FORMAL
  // The executor component proof. The multiplication operator itself is checked
  // differentially rather than exhaustively -- there is no miter here between
  // two 32x32 products, because such a miter returns no verdict in two minutes
  // even standing alone (no divider, no pipeline state, no cap, `mode bmc depth
  // 1`): the obstacle is two structurally distinct `bvmul` terms, and neither
  // bit width nor engine moves it. What is proved instead is everything around
  // the product -- the 33-bit operands the multiplier is handed, the slice of it
  // each variant retires, and three lemmas over the term itself that multiply by
  // a CONSTANT, which a solver sees as shifts and adds. The residual, a `*`
  // wrong for some operand pair no lemma names, is covered by test/exec_tb.v's
  // >= 10,000 randomized vectors per op, which ADR-0010 makes the primary
  // guarantee for this arithmetic (ADR-0051; ADR-0012 made the same move for
  // the divider).
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // Assumed: reset is asserted before the first clock edge. Every harness that
  // instantiates this core drives it high initially, but no check asserts that,
  // so this is discharged nowhere. It is unguarded and therefore in force over
  // every assertion here, which is what it was written for -- `state` and the
  // mul_div registers have no defined value before the first edge (ADR-0049).
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // Otherwise the solver may start mid-divide, for free, at step 0.
  initial state = init;
  // Assumed: reset never returns after the first cycle. True of every harness in
  // the tree and asserted by none, so again discharged nowhere. It was written
  // for the divide-family assertions, which span many cycles and where a
  // re-asserted reset would jump from `divide` to `init` with out.rd_data zeroed
  // by the reset branch rather than computed by the divider. Being unguarded its
  // scope is larger than that: it equally excuses the multiply section and the
  // freeze block from ever seeing a mid-trace reset, which is harmless because a
  // re-asserted reset is not a real environment.
  always_comb if (clocked) assume(!reset);

  // This proof stands alone with no accessor, so accessor_stall is left a free
  // input -- not even ADR-0015's at-most-one-consecutive-cycle bound is assumed,
  // which makes it stronger than the real environment. The price is the
  // `!$past(accessor_stall)` guard on each of the four multiply slice assertions
  // below: a frozen edge computes nothing, so an ungated assertion there would
  // be asserting arithmetic about a bubble. Nothing else here needs it. The
  // operand and lemma assertions are combinational over `in` and mention `out`
  // nowhere, a freeze holds every register the divide invariants read, and the
  // completion guard ($past(state) == divide && state == init) already implies
  // an unfrozen edge, since a frozen one cannot change state.

  // The freeze branch itself: the cycle after accessor_stall, the executor
  // emitted a bubble and held state, the divider datapath and the op/sign
  // latches unchanged. Guarded on !$past(reset) because reset wins over the
  // freeze in the RTL, and the pre-reset values of the mul_div registers are
  // free.
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

  // Assumed: the decoder emits at most one op-select flag per instruction. This
  // proof stands alone, so without it the solver picks combinations no real
  // instruction produces. It is discharged by rtl/decoder.v's own `$onehot`
  // assertion under `instr_valid`, in components_decoder, which CI runs -- the
  // one assume in this repo with a running discharge rather than a structural
  // argument (ADR-0049). Unguarded, so it is in force over the freeze block and
  // the divide invariants too, and correctly so.
  //
  // All 29 `is_*` fields of `decoder_output` are listed here and
  // `is_valid_instr` is deliberately not among them. Adding a flag to the struct
  // without adding it here silently widens the environment for everything below.
  always_comb assume($onehot0({in.is_add, in.is_sub, in.is_xor, in.is_or, in.is_and,
    in.is_sll, in.is_slt, in.is_sltu, in.is_srl, in.is_sra, in.is_lui,
    in.is_mul, in.is_mulh, in.is_mulhu, in.is_mulhsu,
    in.is_div, in.is_divu, in.is_rem, in.is_remu,
    in.is_lb, in.is_lbu, in.is_lh, in.is_lhu, in.is_lw, in.is_sb, in.is_sh, in.is_sw,
    in.is_ecall, in.is_ebreak}));

  // Multiply, against free 32-bit operands: the divide invariant's cap below is
  // guarded to the divide family, so nothing here is bounded. It was unguarded
  // until ADR-0051, and an unguarded assume is proof-global, so these assertions
  // ran against operands in 0..15 where every high half and every sign bit is
  // zero -- MULH/MULHU/MULHSU asserted `out.rd_data == 0` and three of the four
  // multiplier defects ADR-0010 names passed the proof written to catch them.
  //
  // Three obligations replace the miter, each returning in seconds:
  //
  //   (a) the 33-bit operands handed to `multiply` are the correct extensions
  //       of rs1/rs2 for the variant being issued;
  //   (b) the correct SLICE of that same `multiply` term reaches out.rd_data;
  //   (c) three constant-multiplication lemmas over `multiply` itself.
  //
  // (a) and (b) read the same product term the RTL does, so neither can see a
  // product that is simply wrong. That is why (c) is not optional: it is the
  // only part of this section that constrains the term's own value.

  // (a) MUL and MULHU take both operands unsigned, MULH takes both signed, and
  // MULHSU takes rs1 signed and rs2 unsigned -- an asymmetry ADR-0010 names
  // getting backwards as a defect. The extensions are derived by width-extending
  // assignment from a self-determined right-hand side rather than by restating
  // the RTL's `in.rs1[31] & (...)`, so a sign taken from the wrong bit disagrees
  // here.
  logic [32:0] rs1_sext33, rs2_sext33, rs1_zext33, rs2_zext33;
  assign rs1_sext33 = $signed(in.rs1);
  assign rs2_sext33 = $signed(in.rs2);
  assign rs1_zext33 = {1'b0, in.rs1};
  assign rs2_zext33 = {1'b0, in.rs2};
  logic [32:0] mul_op_x_ref, mul_op_y_ref;
  // The `?:` selects between two already-computed 33-bit nets, with no
  // arithmetic in an arm of it: signed arithmetic in a reference model must be a
  // self-determined statement of its own, or IEEE 1800's sign-context rules
  // silently evaluate it unsigned (ADR-0019).
  assign mul_op_x_ref = (in.is_mulh || in.is_mulhsu) ? rs1_sext33 : rs1_zext33;
  assign mul_op_y_ref = in.is_mulh ? rs2_sext33 : rs2_zext33;
  always_comb assert({mul_sign_x, in.rs1} == mul_op_x_ref);
  always_comb assert({mul_sign_y, in.rs2} == mul_op_y_ref);

  // (b) Sliced into plain 32-bit signals so the $past() below applies to a
  // simple signal rather than to a part-select of a $past() result.
  logic [31:0] mul_lo, mul_hi;
  assign mul_lo = multiply[31:0];
  assign mul_hi = multiply[63:32];

  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && !$past(accessor_stall) && $past(state) == init && $past(in.is_mul))
      assert(out.rd_data == $past(mul_lo));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && !$past(accessor_stall) && $past(state) == init && $past(in.is_mulh))
      assert(out.rd_data == $past(mul_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && !$past(accessor_stall) && $past(state) == init && $past(in.is_mulhu))
      assert(out.rd_data == $past(mul_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && !$past(accessor_stall) && $past(state) == init && $past(in.is_mulhsu))
      assert(out.rd_data == $past(mul_hi));

  // (c) Each of these multiplies by a constant -- zero, zero and one -- so the
  // solver sees shifts and adds rather than a second `bvmul` term. They catch
  // what (a) and (b) structurally cannot: a masked high half, a product forced
  // to a constant, a product that has stopped tracking its operands. The third
  // is the load-bearing one, pinning all 64 bits of the product for a family of
  // operand pairs, which is why deleting the high half fails here while (a) and
  // (b) stay green.
  //
  // An `in.rs2 == 32'hffffffff` lemma (multiply by -1) is the obvious fourth and
  // is measured not to work -- no verdict in two minutes. Do not add it back
  // without re-measuring (ADR-0051).
  always_comb if (in.rs1 == 32'b0) assert(multiply == 64'b0);
  always_comb if (in.rs2 == 32'b0) assert(multiply == 64'b0);
  always_comb if (in.rs2 == 32'h1 && !mul_sign_y)
    assert(multiply == {{32{mul_sign_x}}, in.rs1});

  // The divider is proved through a loop invariant rather than by unrolling its
  // 33-cycle latency, which sby's 20-step default depth cannot reach anyway. At
  // every point while state == divide,
  //   dividend == remainder-so-far + quotient-so-far * (divisor << iterations-remaining)
  // and remainder-so-far < (divisor << iterations-remaining). Each iteration of
  // the shift/subtract loop preserves both regardless of how many have run,
  // which is what makes them provable in one induction step; at 0 iterations
  // remaining they collapse to the division identity, tying the invariant to
  // real division and, with the completion assertions below, to out.rd_data.
  //
  // Assumed: the values the divider loads (`div_x`/`div_y`, the ADR-0012
  // magnitudes) are at most 15, which keeps the invariant's intermediate
  // arithmetic inside 64 bits without wraparound. This is ADR-0010's recorded
  // restriction on the proof rather than a claim about the design, so nothing
  // discharges it and nothing should; the divider at full width is covered by
  // test/exec_tb.v. It is scoped to the divide family and `state == divide`, so
  // every multiply assertion above runs against free 32-bit operands.
  //
  // Two things about the shape of this cap are non-obvious and both were
  // measured (ADR-0051):
  //
  // 1. The `|| state == divide` term is required and is not redundant with the
  //    op flags. k-induction may start in `divide` with every `is_div*` low, and
  //    without that term the cap lapses in exactly those states, whereupon the
  //    `mul_div_store <= div_x` invariant fails induction.
  // 2. It caps `div_x`/`div_y` rather than `in.rs1`/`in.rs2`, which is what
  //    makes it a MAGNITUDE cap and leaves the sign free: `div_x <= 15` admits
  //    `in.rs1` anywhere in -15..15, so `in.rs1[31]` is a free bit and the
  //    completion assertions below mean something. The `in.rs1 <= 32'h0f` form
  //    it replaces held bit 31 at constant zero, so ADR-0012's magnitude wrapper
  //    had no coverage here at all and deleting either sign restore passed.
  //    Capping `$signed(in.rs1)` to -15..15 instead was built and does not work:
  //    it leaves `div_y` free up to ~2^32 for DIVU/REMU, where no magnitude
  //    conversion happens, so the product wraps mod 2^64 and induction fails on
  //    the very bound that exists to rule wraparound out. Do not simplify this
  //    cap back onto in.rs1/in.rs2.
  logic div_family;
  assign div_family = in.is_div || in.is_divu || in.is_rem || in.is_remu ||
                      state == divide;
  always_comb if (div_family) assume(div_x <= 32'h0000000f);
  always_comb if (div_family) assume(div_y <= 32'h0000000f);

  // Assumed: ADR-0009's stall protocol holds `in` steady for the duration of a
  // multi-cycle divide, so a free-running formal input cannot corrupt the opcode
  // selection at capture time. Discharged nowhere -- rtl/decoder.v's own task
  // asserts that the pc holds on a stalled cycle, not that `decoder_out` does.
  // The guard scopes it to the divide assertions and nothing else.
  always_ff @(posedge clk)
    // It covers the first divide-state cycle, where `in` would otherwise be free
    // to disagree with what was just latched into mul_div_x/mul_div_y, and the
    // capture cycle, where `in.is_divu`/`is_remu` read below would otherwise be
    // free to disagree with the operands the divider actually ran on.
    if (clocked && (state == divide || $past(state) == divide)) begin
      assume(in.rs1 == $past(in.rs1));
      assume(in.rs2 == $past(in.rs2));
      assume(in.is_div == $past(in.is_div));
      assume(in.is_divu == $past(in.is_divu));
      assume(in.is_rem == $past(in.is_rem));
      assume(in.is_remu == $past(in.is_remu));
    end

  // Ties the op_is_*/op_sign_* latches back to `in` while dividing. Without this
  // as a stated one-step-inductive invariant, k-induction has no intermediate
  // fact linking a latch taken once at issue to the `in.is_divu` read many
  // cycles later at the completion assertions, and induction fails to close.
  always_comb if (state == divide) assert(op_is_div == in.is_div);
  always_comb if (state == divide) assert(op_is_divu == in.is_divu);
  always_comb if (state == divide) assert(op_is_rem == in.is_rem);
  always_comb if (state == divide) assert(op_is_remu == in.is_remu);
  always_comb if (state == divide) assert(op_sign_x == in.rs1[31]);
  always_comb if (state == divide) assert(op_sign_y == in.rs2[31]);

  // k-induction otherwise has no reason to rule out an unreachable starting
  // state with a wild counter, and the invariant below reads it as an exact
  // count of the divisor halvings remaining.
  always_comb if (state == divide) assert(mul_div_counter <= 32);

  // Ties mul_div_y to the same "divisor scaled by iterations remaining" quantity
  // the invariant below reasons about. Without it mul_div_y is a free register
  // to the solver, and the comparison the RTL branches on (mul_div_x >=
  // mul_div_y) has no connection to what the invariant expects. Scoped to
  // counter > 0, since at 0 mul_div_y has taken its last, possibly
  // LSB-dropping, right-shift and nothing reads it again.
  always_comb
    if (state == divide && mul_div_counter > 0)
      assert(mul_div_y == ({32'b0, div_y} << (mul_div_counter - 1)));

  // div_x/div_y, what the divider actually loaded, rather than in.rs1/in.rs2 --
  // so this covers is_div/is_rem's magnitude conversion and not just the
  // unsigned ops.
  logic [63:0] div_scaled_divisor;
  assign div_scaled_divisor = ({32'b0, div_y}) << mul_div_counter;
  always_comb
    if (state == divide)
      assert(mul_div_x + mul_div_store * div_scaled_divisor == {32'b0, div_x});
  always_comb
    if (state == divide)
      assert(mul_div_x < div_scaled_divisor);
  // Without this the equality invariant above is only an equation over 64-bit
  // modular arithmetic, and the solver can satisfy it from an induction start
  // whose mul_div_store is a wraparound solution mod 2^64. True quotients never
  // exceed the dividend for a divisor >= 1, so this is a real fact about the
  // algorithm rather than only a proof crutch.
  always_comb
    if (state == divide)
      assert(mul_div_store <= {32'b0, div_x});

  // Completion: the four divide-family results. The signed pair
  // (`div_ref`/`rem_ref`) is the only thing in this proof that exercises
  // ADR-0012's magnitude wrapper -- the divider loop is unsigned, `div_x`/`div_y`
  // are absolute values, and the sign is restored at capture. Under the old
  // value cap those two capture arms could be deleted outright and the task
  // still passed.
  //
  // `div_q` and `div_r` are computed first and only the already-computed 32-bit
  // results are selected by the divide-by-zero mux, so no sign context is taken
  // away from the division itself the way ADR-0019's monitor models had it taken
  // away from theirs.
  logic signed [31:0] div_srs1, div_srs2;
  assign div_srs1 = $signed(in.rs1);
  assign div_srs2 = $signed(in.rs2);
  logic signed [31:0] div_q, div_r;
  assign div_q = div_srs1 / div_srs2;
  assign div_r = div_srs1 % div_srs2;

  logic [31:0] divu_ref, remu_ref, div_ref, rem_ref;
  assign divu_ref = (in.rs2 == 0) ? 32'hffffffff : (in.rs1 / in.rs2);
  assign remu_ref = (in.rs2 == 0) ? in.rs1 : (in.rs1 % in.rs2);
  assign div_ref  = (in.rs2 == 0) ? 32'hffffffff : div_q;
  assign rem_ref  = (in.rs2 == 0) ? in.rs1 : div_r;

  // The guard `$past(state) == divide && state == init` is unreachable in the
  // basecase: components.sby sets no depth, so `mode prove`'s basecase runs 20
  // steps and the real divider needs 33 cycles from issue. A mutation that
  // breaks one of these therefore reports `UNKNOWN (rc=4)` -- basecase pass,
  // induction FAIL -- rather than `FAIL`, which is this assertion class's normal
  // detection signal. Raising the basecase past 33 is a depth change needing its
  // own evidence (ADR-0025, ADR-0049 F3).
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(in.is_divu))
      assert(out.rd_data == divu_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(in.is_remu))
      assert(out.rd_data == remu_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(in.is_div))
      assert(out.rd_data == div_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(in.is_rem))
      assert(out.rd_data == rem_ref);
 `endif
endmodule
