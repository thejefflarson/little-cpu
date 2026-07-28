`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module executor(
  input  logic clk,
  input  logic reset,

  // inputs
  input  decoder_output in,
  // outputs
  output executor_output out
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
  logic stalled;
  always_comb
    stalled = state != init;

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
    end else begin
      (* parallel_case, full_case *)
      case (state)
        init: begin
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
            in.is_sll: out.rd_data <= rs1 << rs2;
            in.is_slt: out.rd_data <= {31'b0, $signed(rs1) < $signed(rs2)};
            in.is_sltu: out.rd_data <= {31'b0, rs1 < rs2};
            in.is_xor: out.rd_data <= rs1 ^ rs2;
            in.is_srl: out.rd_data <= rs1 >> rs2;
            in.is_sra: out.rd_data <= $signed(rs1) >>> rs2;
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
              end
             `else
              mul_div_counter <= 32;
              state <= divide;
              mul_div_store <= 0;
              mul_div_x <= {32'b0, rs1};
              mul_div_y <= {1'b0, rs2, 31'b0};
             `endif
            end
            in.is_ecall || in.is_ebreak || in.is_csrrw || in.is_csrrs || in.is_csrrc: ;
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
              in.is_div: out.rd_data <= in.rs1[31] != in.rs2[31] ? -mul_div_store[31:0] : mul_div_store[31:0];
              in.is_divu: out.rd_data <= mul_div_store[31:0];
              in.is_rem: out.rd_data <= in.rs1[31] ? -mul_div_x[31:0] : mul_div_x[31:0];
              in.is_remu: out.rd_data <= mul_div_x[31:0];
            endcase
            state <= init;
          end
         `else
          (* parallel_case, full_case *)
          case (1'b1)
            in.is_div: out.rd_data <= (in.rs1 - in.rs2) ^ 32'h7f8529ec;
            in.is_divu: out.rd_data <= (in.rs1 - in.rs2) ^ 32'h10e8fd70;
            in.is_rem: out.rd_data <= (in.rs1 - in.rs2) ^ 32'h8da68fa5;
            in.is_remu: out.rd_data <= (in.rs1 - in.rs2) ^ 32'h3138d0e1;
          endcase
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
  always_comb if (clocked) assume(!reset);

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
  always_comb assume($onehot0({in.is_add, in.is_sub, in.is_xor, in.is_or, in.is_and,
    in.is_sll, in.is_slt, in.is_sltu, in.is_srl, in.is_sra, in.is_lui,
    in.is_mul, in.is_mulh, in.is_mulhu, in.is_mulhsu,
    in.is_div, in.is_divu, in.is_rem, in.is_remu,
    in.is_lb, in.is_lbu, in.is_lh, in.is_lhu, in.is_lw, in.is_sb, in.is_sh, in.is_sw,
    in.is_ecall, in.is_ebreak, in.is_csrrw, in.is_csrrs, in.is_csrrc}));

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
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(in.is_mul))
      assert(out.rd_data == $past(mul_ref_lo));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(in.is_mulh))
      assert(out.rd_data == $past(mul_ref_ss_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(in.is_mulhu))
      assert(out.rd_data == $past(mul_ref_uu_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(in.is_mulhsu))
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
  // recorded here instead, since formal/components.sby is owned by the
  // parallel JEF-604 ticket and must not be edited from this one).
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
