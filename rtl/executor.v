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
  logic [63:0] mul_div_x, mul_div_y;
  logic [63:0] mul_div_store;
  always_comb
    stalled = state != init;

  // Decode bubbles rather than holding `in` while a divide runs, so `in` cannot
  // be read when the divide completes. The op-select and operand signs the last
  // iteration needs are latched here at issue instead.
  logic op_is_div, op_is_divu, op_is_rem, op_is_remu, op_sign_x, op_sign_y;

 `ifdef RISCV_FORMAL_ALTOPS
  // Named so the completion arm reads these as operands rather than as slices of
  // the divider's working registers. ALTOPS does no magnitude conversion.
  logic [31:0] div_alt_rs1, div_alt_rs2;
  assign div_alt_rs1 = mul_div_x[31:0];
  assign div_alt_rs2 = mul_div_y[62:31];
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
            in.is_sub: out.rd_data <= rs1 - rs2;
            // The RV32I shift amount is rs2[4:0]; the upper 27 bits are ignored.
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
                mul_div_store <= 0;
                mul_div_x <= {32'b0, div_x};
                mul_div_y <= {1'b0, div_y, 31'b0};
                // The result is not ready this cycle, so send a bubble rather
                // than a repeat of whatever the executor last held.
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
            // A Zicsr access never appears here. rtl/csrs.v reads and commits it
            // in decode, and the read result arrives as `is_add` with rs2
            // zeroed, so by this stage it is an add.
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

  // The divider is proved through a loop invariant rather than by unrolling its
  // 33-cycle latency, which sby's 20-step default depth cannot reach. At every
  // point while state == divide,
  //   dividend == remainder-so-far + quotient-so-far * (divisor << iterations-left)
  // and remainder-so-far < (divisor << iterations-left). Each iteration
  // preserves both however many have run, and at zero iterations left they
  // collapse to the division identity.
  //
  // Assumed: the values the divider loads are at most 15, which keeps the
  // invariant's arithmetic inside 64 bits. That is a recorded restriction on the
  // proof rather than a claim about the design; full width is covered by
  // test/exec_tb.v.
  //
  // Two things about the cap's shape were measured and must not be simplified
  // away. The `|| state == divide` term is not redundant with the op flags:
  // k-induction may start in `divide` with every `is_div*` low, and without it
  // the `mul_div_store <= div_x` invariant fails induction. And it caps
  // `div_x`/`div_y` rather than `in.rs1`/`in.rs2`, which leaves the sign free --
  // `div_x <= 15` admits `in.rs1` anywhere in -15..15, so the completion
  // assertions below mean something. Capping `$signed(in.rs1)` instead leaves
  // `div_y` free up to ~2^32 for DIVU/REMU, where the product wraps mod 2^64 and
  // induction fails on the very bound that rules wraparound out.
  logic div_family;
  assign div_family = in.is_div || in.is_divu || in.is_rem || in.is_remu ||
                      state == divide;
  always_comb if (div_family) assume(div_x <= 32'h0000000f);
  always_comb if (div_family) assume(div_y <= 32'h0000000f);

  // Assumed: the stall protocol holds `in` steady for the whole of a multi-cycle
  // divide. Discharged nowhere -- rtl/decoder.v's own task asserts that the pc
  // holds on a stalled cycle, not that `decoder_out` does. Without it `in` is
  // free to disagree with what was latched into mul_div_x/mul_div_y, and the
  // `in.is_divu`/`is_remu` read below free to disagree with the operands the
  // divider actually ran on.
  always_ff @(posedge clk)
    if (clocked && (state == divide || $past(state) == divide)) begin
      assume(in.rs1 == $past(in.rs1));
      assume(in.rs2 == $past(in.rs2));
      assume(in.is_div == $past(in.is_div));
      assume(in.is_divu == $past(in.is_divu));
      assume(in.is_rem == $past(in.is_rem));
      assume(in.is_remu == $past(in.is_remu));
    end

  // Ties the op_is_*/op_sign_* latches back to `in` while dividing. Without it
  // k-induction has no fact linking a latch taken once at issue to the
  // `in.is_divu` read many cycles later, and induction fails to close.
  always_comb if (state == divide) assert(op_is_div == in.is_div);
  always_comb if (state == divide) assert(op_is_divu == in.is_divu);
  always_comb if (state == divide) assert(op_is_rem == in.is_rem);
  always_comb if (state == divide) assert(op_is_remu == in.is_remu);
  always_comb if (state == divide) assert(op_sign_x == in.rs1[31]);
  always_comb if (state == divide) assert(op_sign_y == in.rs2[31]);

  // k-induction otherwise has no reason to rule out a starting state with a wild
  // counter, and the invariant below reads it as an exact count of the divisor
  // halvings remaining.
  always_comb if (state == divide) assert(mul_div_counter <= 32);

  // Ties mul_div_y to the same "divisor scaled by iterations left" quantity the
  // invariant below reasons about. Without it the comparison the RTL branches on
  // has no connection to what the invariant expects. Scoped to counter > 0,
  // since at 0 mul_div_y has taken its last right-shift and nothing reads it.
  always_comb
    if (state == divide && mul_div_counter > 0)
      assert(mul_div_y == ({32'b0, div_y} << (mul_div_counter - 1)));

  // div_x/div_y, what the divider actually loaded, rather than in.rs1/in.rs2 --
  // so this covers is_div/is_rem's magnitude conversion too.
  logic [63:0] div_scaled_divisor;
  assign div_scaled_divisor = ({32'b0, div_y}) << mul_div_counter;
  always_comb
    if (state == divide)
      assert(mul_div_x + mul_div_store * div_scaled_divisor == {32'b0, div_x});
  always_comb
    if (state == divide)
      assert(mul_div_x < div_scaled_divisor);
  // Without this the equality above is only an equation over 64-bit modular
  // arithmetic, and the solver can satisfy it from an induction start whose
  // mul_div_store is a wraparound solution mod 2^64. True quotients never exceed
  // the dividend for a divisor >= 1, so it is a real fact about the algorithm.
  always_comb
    if (state == divide)
      assert(mul_div_store <= {32'b0, div_x});

  // Completion: the four divide-family results. `div_ref`/`rem_ref` are the only
  // things here that exercise the magnitude wrapper, since the divider loop is
  // unsigned and the sign is restored at capture -- under the old operand cap
  // those two capture arms could be deleted outright and the task still passed.
  // `div_q` and `div_r` are computed first, so the divide-by-zero mux selects
  // between already-computed results and takes no sign context away from the
  // division itself.
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

  // The guard below is unreachable in the basecase: components.sby sets no
  // depth, so `mode prove` runs 20 basecase steps and the real divider needs 33
  // cycles from issue. A mutation that breaks one of these reports
  // `UNKNOWN (rc=4)` -- basecase pass, induction FAIL -- rather than `FAIL`.
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
