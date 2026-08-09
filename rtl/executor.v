`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module executor(
  input  logic clk,
  input  logic reset,

  input  decoder_output in,
  // High for the one cycle a load's response is still in flight in the
  // accessor. Freeze rather than advance, or a fresh instruction collides with
  // the completing load over the accessor's single output register.
  input  logic accessor_stall,
  output executor_output out,
  // High while a multi-cycle divide runs. littlecpu.v routes it back to decode.
  output logic stalled
);
  logic [31:0] rs1, rs2;
  assign rs1 = in.rs1;
  assign rs2 = in.rs2;

  // One subtraction serves sub, slt and sltu. Unsigned less-than is the borrow
  // out; signed less-than is the same except when the sign bits disagree, and
  // then the negative operand is the smaller. Nothing here is a signed
  // expression, so it cannot lose its signedness to a neighbouring arm.
  logic [32:0] alu_sub;
  logic        alu_ltu, alu_lt;
  assign alu_sub = {1'b0, rs1} - {1'b0, rs2};
  assign alu_ltu = alu_sub[32];
  assign alu_lt  = (rs1[31] ^ rs2[31]) ? rs1[31] : alu_sub[32];

  // One right shifter serves all three shifts. A left shift is a right shift
  // with both ends reversed, and a reversal is wiring rather than logic; an
  // arithmetic shift is the same shifter with the sign bit filling in behind it.
  // Written as three operators this is three barrel shifters. The RV32I shift
  // amount is rs2[4:0]; the upper 27 bits are ignored.
  logic [31:0] rs1_rev, shift_src, shift_res, shift_rev;
  logic        shift_fill;
  logic signed [32:0] shift_wide;
  for (genvar i = 0; i < 32; i++) begin : l_shift_rev
    assign rs1_rev[i]   = rs1[31-i];
    assign shift_rev[i] = shift_res[31-i];
  end
  assign shift_src  = in.is_sll ? rs1_rev : rs1;
  assign shift_fill = in.is_sra ? rs1[31] : 1'b0;
  assign shift_wide = $signed({shift_fill, shift_src}) >>> rs2[4:0];
  assign shift_res  = shift_wide[31:0];

  // The restoring divider below only divides unsigned, so signed div/rem hand it
  // the magnitudes. The result signs are restored from the original operand
  // signs once the loop finishes.
  logic [31:0] div_x, div_y;
  assign div_x = (in.is_div || in.is_rem) && rs1[31] ? -rs1 : rs1;
  assign div_y = (in.is_div || in.is_rem) && rs2[31] ? -rs2 : rs2;

  logic [1:0]  state;
  localparam init = 2'b00;
  localparam divide = 2'b10;
  logic [6:0]  mul_div_counter;
  // The divisor stays still and the remainder shifts through the dividend
  // register, which is also where the quotient is built: a quotient bit enters
  // at the bottom on the same edge the dividend's top bit leaves. A 32-bit
  // division therefore needs three 32-bit registers, not three 64-bit ones --
  // the top halves were provably dead, and nothing in the expression said so.
  logic [31:0] div_rem, div_quot, div_divisor;

  // One subtraction, not a comparison and then a subtraction. `rem >= divisor`
  // is exactly "this subtract did not borrow", so the borrow out replaces a
  // comparator that computed the same fact a second time, and its inverse is
  // the quotient bit. `div_rem < div_divisor <= 2**32-1` holds every iteration,
  // which is what makes 33 bits enough here and the write back to 32 lossless:
  // a borrow can only fire when the shifted remainder is below the divisor, and
  // so below 2**32.
  logic [32:0] rem_shifted, rem_sub;
  assign rem_shifted = {div_rem, div_quot[31]};
  assign rem_sub     = rem_shifted - {1'b0, div_divisor};

  always_comb
    stalled = state != init;

  // Decode bubbles rather than holding `in` while a divide runs, so `in` cannot
  // be read when the divide completes. The op-select and operand signs the last
  // iteration needs are latched here at issue instead.
  logic op_is_div, op_is_divu, op_is_rem, op_is_remu, op_sign_x, op_sign_y;

  // Pick the magnitude first, then negate it once. Spelled as four arms that
  // each carry their own sign restoration this is two independent 32-bit
  // negators inside a one-hot mux, and the mux does not collapse either.
  // DIV's result is negative when the operands' signs disagree; REM's takes the
  // dividend's sign; the unsigned pair never negates.
  logic [31:0] div_result_mag;
  logic        div_negate;
  assign div_result_mag = (op_is_div || op_is_divu) ? div_quot : div_rem;
  assign div_negate     = (op_is_div && (op_sign_x != op_sign_y)) || (op_is_rem && op_sign_x);

 `ifdef RISCV_FORMAL_ALTOPS
  // Named so the completion arm reads these as operands rather than as slices of
  // the divider's working registers. ALTOPS does no magnitude conversion.
  logic [31:0] div_alt_rs1, div_alt_rs2;
  assign div_alt_rs1 = div_quot;
  assign div_alt_rs2 = div_divisor;
 `endif

  // sign_x covers is_mulh (rs1 signed) and is_mulhsu (rs1 signed, rs2
  // unsigned); sign_y covers only is_mulh.
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
      div_rem <= 0;
      div_quot <= 0;
      div_divisor <= 0;
      op_is_div <= 0;
      op_is_divu <= 0;
      op_is_rem <= 0;
      op_is_remu <= 0;
      op_sign_x <= 0;
      op_sign_y <= 0;
    end else if (accessor_stall) begin
      // This never overlaps a divide: decode freezes everything upstream while
      // dividing, so no load can reach the accessor until the divide finishes.
      out <= 0;
    end else begin
      (* parallel_case, full_case *)
      case (state)
        init: begin
          out.valid <= in.valid;
         `ifdef RISCV_FORMAL
          // Latched at issue for the same reason op_is_div is: this is the only
          // cycle `in` can be read for a divide.
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
            in.is_sub: out.rd_data <= alu_sub[31:0];
            in.is_sll: out.rd_data <= shift_rev;
            in.is_slt: out.rd_data <= {31'b0, alu_lt};
            in.is_sltu: out.rd_data <= {31'b0, alu_ltu};
            in.is_xor: out.rd_data <= rs1 ^ rs2;
            // Both right shifts read the one shifter; `shift_fill` is what
            // separates them.
            in.is_srl || in.is_sra: out.rd_data <= shift_res;
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
              // Latched whichever sub-path fires below. The short-circuits never
              // leave `init` and never read them back; the real divide does.
              op_is_div <= in.is_div;
              op_is_divu <= in.is_divu;
              op_is_rem <= in.is_rem;
              op_is_remu <= in.is_remu;
              op_sign_x <= in.rs1[31];
              op_sign_y <= in.rs2[31];
             `ifndef RISCV_FORMAL_ALTOPS
              // The two architecturally specified results. Both stay in `init`,
              // so neither enters the divider.
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
                div_rem <= 0;
                div_quot <= div_x;
                div_divisor <= div_y;
                // The result is not ready this cycle, so send a bubble rather
                // than a repeat of whatever the executor last held.
                out.valid <= 1'b0;
              end
             `else
              mul_div_counter <= 32;
              state <= divide;
              div_rem <= 0;
              div_quot <= rs1;
              div_divisor <= rs2;
              out.valid <= 1'b0;
             `endif
            end
            // A Zicsr access never appears here. rtl/csrs.v reads and commits it
            // in decode, and the read result arrives as `is_add` with rs2
            // zeroed, so by this stage it is an add.
            in.is_ecall || in.is_ebreak: ;
            in.is_valid_instr: ;
          endcase // case (1'b1)
        end // case: init

        divide: begin
         `ifndef RISCV_FORMAL_ALTOPS
          if (|mul_div_counter) begin
            div_quot <= {div_quot[30:0], ~rem_sub[32]};
            div_rem  <= rem_sub[32] ? rem_shifted[31:0] : rem_sub[31:0];
            mul_div_counter <= mul_div_counter - 1;
          end else begin
            out.rd_data <= div_negate ? -div_result_mag : div_result_mag;
            out.valid <= 1'b1;
            state <= init;
          end
         `else
          // ALTOPS collapses the divide to one cycle, so by the time this arm
          // runs `in` already holds the next instruction.
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
  // There is no miter between two 32x32 products: one returns no verdict in two
  // minutes even standing alone, because the obstacle is two structurally
  // distinct `bvmul` terms and neither bit width nor engine moves it. What is
  // proved instead is everything around the product, plus three lemmas that
  // multiply by a constant. A `*` wrong for some operand pair no lemma names is
  // covered by test/exec_tb.v's randomized vectors.
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // Assumed: reset is asserted before the first clock edge. True of every
  // harness in the tree and asserted by none, so nothing discharges it, and
  // `state` and the mul_div registers have no defined value before that edge.
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // Otherwise the solver may start mid-divide, for free, at step 0.
  initial state = init;
  // Assumed: reset never returns after the first cycle. Discharged nowhere
  // either. Without it a re-asserted reset jumps to `init` mid-divide with
  // out.rd_data zeroed by the reset branch rather than computed.
  always_comb if (clocked) assume(!reset);

  // No accessor here, so accessor_stall is a free input -- not even the
  // at-most-one-consecutive-cycle bound is assumed. The price is the
  // `!$past(accessor_stall)` guard on the four multiply slice assertions below:
  // a frozen edge computes nothing, so an ungated one would be asserting
  // arithmetic about a bubble.

  // The cycle after accessor_stall, the executor emitted a bubble and held
  // state, the divider datapath and the op/sign latches. Guarded on
  // !$past(reset) because reset wins over the freeze in the RTL.
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(accessor_stall)) begin
      assert(out == '0);
      assert(state == $past(state));
      assert(mul_div_counter == $past(mul_div_counter));
      assert(div_rem == $past(div_rem));
      assert(div_quot == $past(div_quot));
      assert(div_divisor == $past(div_divisor));
      assert(op_is_div == $past(op_is_div));
      assert(op_is_divu == $past(op_is_divu));
      assert(op_is_rem == $past(op_is_rem));
      assert(op_is_remu == $past(op_is_remu));
      assert(op_sign_x == $past(op_sign_x));
      assert(op_sign_y == $past(op_sign_y));
    end

  // Assumed: the decoder emits at most one op-select flag per instruction.
  // Without it the solver picks combinations no real instruction produces.
  // Discharged by rtl/decoder.v's own `$onehot` assertion under `instr_valid`.
  //
  // All 29 `is_*` fields of `decoder_output` are listed here and
  // `is_valid_instr` deliberately is not. Adding a flag to the struct without
  // adding it here silently widens the environment for everything below.
  always_comb assume($onehot0({in.is_add, in.is_sub, in.is_xor, in.is_or, in.is_and,
    in.is_sll, in.is_slt, in.is_sltu, in.is_srl, in.is_sra, in.is_lui,
    in.is_mul, in.is_mulh, in.is_mulhu, in.is_mulhsu,
    in.is_div, in.is_divu, in.is_rem, in.is_remu,
    in.is_lb, in.is_lbu, in.is_lh, in.is_lhu, in.is_lw, in.is_sb, in.is_sh, in.is_sw,
    in.is_ecall, in.is_ebreak}));

  // The shared subtractor and the shared shifter, against the operators they
  // replaced. Each reference is its own self-determined statement over signed
  // nets rather than an arm of a conditional, so nothing here can lose its
  // signedness to a neighbour. These are what say the borrow bit really is
  // unsigned less-than, that the sign-bit correction really is signed
  // less-than, and that reversing both ends of a right shifter really is a left
  // shift -- three facts the RTL now depends on and no operator states.
  logic signed [31:0] alu_ref_x, alu_ref_y;
  assign alu_ref_x = rs1;
  assign alu_ref_y = rs2;
  always_comb assert(alu_sub[31:0] == rs1 - rs2);
  always_comb assert(alu_ltu == (rs1 < rs2));
  always_comb assert(alu_lt == (alu_ref_x < alu_ref_y));

  logic [31:0] shift_sll_ref, shift_srl_ref;
  logic signed [31:0] shift_sra_ref;
  assign shift_sll_ref = rs1 << rs2[4:0];
  assign shift_srl_ref = rs1 >> rs2[4:0];
  assign shift_sra_ref = alu_ref_x >>> rs2[4:0];
  always_comb if (in.is_sll) assert(shift_rev == shift_sll_ref);
  always_comb if (in.is_srl) assert(shift_res == shift_srl_ref);
  always_comb if (in.is_sra) assert(shift_res == shift_sra_ref);

  // The divider's compare-and-subtract is one subtraction read twice, and 33
  // bits is enough for it. Both statements need the loop's own bound as a
  // hypothesis rather than as a neighbouring assertion, because k-induction
  // checks every assertion at the same step and cannot use one to discharge
  // another: above the bound a shifted remainder can carry into bit 32 and the
  // bit stops being a borrow.
  logic [32:0] div_divisor_wide;
  assign div_divisor_wide = {1'b0, div_divisor};
  always_comb
    if (div_rem < div_divisor) assert(rem_sub[32] == (rem_shifted < div_divisor_wide));
  always_comb
    if (div_rem < div_divisor && rem_sub[32]) assert(rem_shifted[32] == 1'b0);

  // Multiply, against free 32-bit operands: the divide cap below is guarded to
  // the divide family. It was unguarded once, and an unguarded assume is
  // proof-global, so these assertions ran against operands in 0..15 where every
  // high half and sign bit is zero -- MULH/MULHU/MULHSU asserted
  // `out.rd_data == 0` and three known multiplier defects passed the proof
  // written to catch them.

  // MUL and MULHU take both operands unsigned, MULH both signed, MULHSU rs1
  // signed and rs2 unsigned. The extensions are derived by width-extending
  // assignment rather than by restating the RTL's own sign expression, so a sign
  // taken from the wrong bit disagrees here.
  logic [32:0] rs1_sext33, rs2_sext33, rs1_zext33, rs2_zext33;
  assign rs1_sext33 = $signed(in.rs1);
  assign rs2_sext33 = $signed(in.rs2);
  assign rs1_zext33 = {1'b0, in.rs1};
  assign rs2_zext33 = {1'b0, in.rs2};
  logic [32:0] mul_op_x_ref, mul_op_y_ref;
  // The `?:` selects between two already-computed 33-bit nets, with no
  // arithmetic in an arm of it: signed arithmetic in a reference model must be a
  // self-determined statement of its own, or the sign-context rules silently
  // evaluate it unsigned.
  assign mul_op_x_ref = (in.is_mulh || in.is_mulhsu) ? rs1_sext33 : rs1_zext33;
  assign mul_op_y_ref = in.is_mulh ? rs2_sext33 : rs2_zext33;
  always_comb assert({mul_sign_x, in.rs1} == mul_op_x_ref);
  always_comb assert({mul_sign_y, in.rs2} == mul_op_y_ref);

  // Sliced into plain 32-bit signals so the $past() below applies to a simple
  // signal rather than to a part-select of a $past() result.
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

  // Each multiplies by a constant -- zero, zero and one -- so the solver sees
  // shifts and adds rather than a second `bvmul` term. These are the only
  // assertions here that constrain the product's own value; the two above read
  // the same term the RTL does. The third pins all 64 bits for a family of
  // operand pairs, which is why deleting the high half fails here.
  //
  // An `in.rs2 == 32'hffffffff` lemma is the obvious fourth and is measured not
  // to work: no verdict in two minutes. Do not add it back without re-measuring.
  always_comb if (in.rs1 == 32'b0) assert(multiply == 64'b0);
  always_comb if (in.rs2 == 32'b0) assert(multiply == 64'b0);
  always_comb if (in.rs2 == 32'h1 && !mul_sign_y)
    assert(multiply == {{32{mul_sign_x}}, in.rs1});

  // Decode does not hold `in` steady for a multi-cycle divide -- it bubbles it.
  // The divider stall is low on the cycle a divide issues, so decode issues
  // normally that cycle, the operand-fetch cycle publishes a bubble, and the
  // stall then holds that bubble. `in` is a zeroed bubble for every cycle of a
  // divide except the one that loaded it, so a proof that assumed the hold was
  // checking an input sequence the pipeline never produces, with references
  // built from operands that are zero by the time they are read.
  //
  // The operands are kept in proof-only copies instead, taken on every `init`
  // edge -- which is the edge the divider loads on. Nothing in the RTL reads
  // `in` outside `init`, so `in` is left completely free while dividing: that
  // covers the bubble the pipeline really presents and every other sequence too,
  // and is weaker than either assumption it replaces.
  logic [31:0] div_ghost_rs1, div_ghost_rs2;
  always_ff @(posedge clk)
    if (!reset && !accessor_stall && state == init) begin
      div_ghost_rs1 <= in.rs1;
      div_ghost_rs2 <= in.rs2;
    end

  // The magnitudes the divider loaded, rebuilt from those copies the way div_x
  // and div_y build them from `in`.
  logic [31:0] div_mag_x, div_mag_y;
  assign div_mag_x = (op_is_div || op_is_rem) && div_ghost_rs1[31] ? -div_ghost_rs1 : div_ghost_rs1;
  assign div_mag_y = (op_is_div || op_is_rem) && div_ghost_rs2[31] ? -div_ghost_rs2 : div_ghost_rs2;

  // Exactly one op flag is latched, because the arm that latches them fires only
  // when one of the four is set and the $onehot0 assume above bounds the other
  // side. The completion arm's magnitude select and its negate term both read
  // the flags as disjoint, and this is what says they are.
  always_comb
    if (state == divide) assert($onehot({op_is_div, op_is_divu, op_is_rem, op_is_remu}));
  always_comb if (state == divide) assert(op_sign_x == div_ghost_rs1[31]);
  always_comb if (state == divide) assert(op_sign_y == div_ghost_rs2[31]);
  always_comb if (state == divide) assert(div_divisor == div_mag_y);

  // k-induction otherwise has no reason to rule out a starting state with a wild
  // counter, and the invariant below reads it as an exact count of the
  // iterations left.
  always_comb if (state == divide) assert(mul_div_counter <= 32);

  // The divider is proved through a loop invariant rather than by unrolling its
  // 33-cycle latency, which sby's 20-step default depth cannot reach. Let n be
  // the counter and k = 32 - n the iterations already run. The dividend's top k
  // bits have been divided and their quotient sits in the low k bits of
  // div_quot, with the running remainder beside it:
  //   (div_quot & (2**k - 1)) * div_divisor + div_rem == div_mag_x >> n
  // and div_rem < div_divisor. The dividend's remaining n bits are still in the
  // top of div_quot:
  //   div_quot >> k == div_mag_x & (2**n - 1)
  // Each iteration preserves all three however many have run, and at n == 0 they
  // collapse to the division identity over the whole dividend.
  //
  // The product is 32 bits by 32 into 64, which cannot wrap: (2**32-1)**2 is
  // below 2**64. So the equation over 64-bit modular arithmetic is the equation
  // over the integers, and no bound is needed to rule wraparound solutions out
  // -- the old invariant needed one for exactly that, because its scaled divisor
  // reached 2**64.
  //
  // Assumed: the magnitudes the divider loaded are at most this cap. That is a
  // recorded restriction on the proof, not a claim about the design; full width
  // is covered by test/exec_tb.v's randomized vectors. What it buys now is
  // solver time on one symbolic 32x32 product, and the width was measured rather
  // than guessed -- see the ADR for the sweep.
  localparam [31:0] div_proof_cap = 32'h000000ff;
  always_comb if (state == divide) assume(div_mag_x <= div_proof_cap);
  always_comb if (state == divide) assume(div_mag_y <= div_proof_cap);

  logic [5:0]  div_done;
  logic [63:0] div_quot_done, div_quot_left, div_mag_x_done, div_mag_x_left;
  assign div_done       = 6'd32 - mul_div_counter[5:0];
  assign div_quot_done  = {32'b0, div_quot} & ((64'b1 << div_done) - 64'b1);
  assign div_quot_left  = {32'b0, div_quot} >> div_done;
  assign div_mag_x_done = {32'b0, div_mag_x} >> mul_div_counter;
  assign div_mag_x_left = {32'b0, div_mag_x} & ((64'b1 << mul_div_counter) - 64'b1);
  always_comb
    if (state == divide)
      assert(div_quot_done * {32'b0, div_divisor} + {32'b0, div_rem} == div_mag_x_done);
  always_comb if (state == divide) assert(div_rem < div_divisor);
  always_comb if (state == divide) assert(div_quot_left == div_mag_x_left);

  // Completion: the four divide-family results, against the operands the divide
  // was issued with. `div_ref`/`rem_ref` are the only things here that exercise
  // the magnitude wrapper and the sign restoration, since the divider loop is
  // unsigned. `div_q` and `div_r` are computed first, so the divide-by-zero mux
  // selects between already-computed results and takes no sign context away from
  // the division itself. A zero divisor never reaches the loop, and
  // `div_rem < div_divisor` is what says so from inside the proof.
  logic signed [31:0] div_srs1, div_srs2;
  assign div_srs1 = $signed(div_ghost_rs1);
  assign div_srs2 = $signed(div_ghost_rs2);
  logic signed [31:0] div_q, div_r;
  assign div_q = div_srs1 / div_srs2;
  assign div_r = div_srs1 % div_srs2;

  logic [31:0] divu_ref, remu_ref, div_ref, rem_ref;
  assign divu_ref = (div_ghost_rs2 == 0) ? 32'hffffffff : (div_ghost_rs1 / div_ghost_rs2);
  assign remu_ref = (div_ghost_rs2 == 0) ? div_ghost_rs1 : (div_ghost_rs1 % div_ghost_rs2);
  assign div_ref  = (div_ghost_rs2 == 0) ? 32'hffffffff : div_q;
  assign rem_ref  = (div_ghost_rs2 == 0) ? div_ghost_rs1 : div_r;

  // The guard below is unreachable in the basecase: components.sby sets no
  // depth, so `mode prove` runs 20 basecase steps and the real divider needs 33
  // cycles from issue. A mutation that breaks one of these reports
  // `UNKNOWN (rc=4)` -- basecase pass, induction FAIL -- rather than `FAIL`.
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(op_is_divu))
      assert(out.rd_data == divu_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(op_is_remu))
      assert(out.rd_data == remu_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(op_is_div))
      assert(out.rd_data == div_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(op_is_rem))
      assert(out.rd_data == rem_ref);
 `endif
endmodule
