`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module executor(
  input  logic clk,
  input  logic reset,

  input  decoder_output in,
  output executor_output out,
  output logic stalled
);
  logic [31:0] rs1, rs2;
  assign rs1 = in.rs1;
  assign rs2 = in.rs2;

  logic [32:0] alu_sub;
  logic        alu_ltu, alu_lt;
  assign alu_sub = {1'b0, rs1} - {1'b0, rs2};
  assign alu_ltu = alu_sub[32];
  assign alu_lt  = (rs1[31] ^ rs2[31]) ? rs1[31] : alu_sub[32];

  // A left shift is a right shift with both ends reversed, so one shifter serves
  // all three.
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

  // The divider is unsigned; signed div and rem hand it magnitudes and restore
  // the sign on completion.
  logic [31:0] div_x, div_y;
  assign div_x = (in.is_div || in.is_rem) && rs1[31] ? -rs1 : rs1;
  assign div_y = (in.is_div || in.is_rem) && rs2[31] ? -rs2 : rs2;

  logic [1:0]  state;
  localparam init = 2'b00;
  localparam divide = 2'b10;
  logic [6:0]  mul_div_counter;
  // div_quot holds the dividend: a quotient bit shifts in at the bottom on the
  // edge each dividend bit leaves the top.
  logic [31:0] div_rem, div_quot, div_divisor;

  // The borrow out of rem_sub is the quotient bit's inverse, and div_rem <
  // div_divisor every iteration is what makes 33 bits enough.
  logic [32:0] rem_shifted, rem_sub;
  assign rem_shifted = {div_rem, div_quot[31]};
  assign rem_sub     = rem_shifted - {1'b0, div_divisor};

  always_comb
    stalled = state != init;

  // `in` does not hold the divide while it runs -- decode has issued the next
  // instruction and the stall holds that one -- so completion reads these.
  logic op_is_div, op_is_divu, op_is_rem, op_is_remu, op_sign_x, op_sign_y;

  logic [31:0] div_result_mag;
  logic        div_negate;
  assign div_result_mag = (op_is_div || op_is_divu) ? div_quot : div_rem;
  assign div_negate     = (op_is_div && (op_sign_x != op_sign_y)) || (op_is_rem && op_sign_x);

 `ifdef RISCV_FORMAL_ALTOPS
  logic [31:0] div_alt_rs1, div_alt_rs2;
  assign div_alt_rs1 = div_quot;
  assign div_alt_rs2 = div_divisor;
 `endif

  logic mul_sign_x, mul_sign_y;
  assign mul_sign_x = in.rs1[31] & (in.is_mulh | in.is_mulhsu);
  assign mul_sign_y = in.rs2[31] & in.is_mulh;

  // A negative two's-complement operand contributes one subtraction of the other
  // operand at bit 32, so the signed high half is the unsigned product's with
  // two conditional subtracts and the low half untouched.
  logic [63:0] mul_unsigned;
  logic [31:0] mul_lo, mul_hi;
  assign mul_unsigned = in.rs1 * in.rs2;
  assign mul_lo = mul_unsigned[31:0];
  assign mul_hi = mul_unsigned[63:32] - (mul_sign_x ? in.rs2 : 32'b0)
                                      - (mul_sign_y ? in.rs1 : 32'b0);

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
    end else begin
      (* parallel_case, full_case *)
      case (state)
        init: begin
          out.valid <= in.valid;
         `ifdef RISCV_FORMAL
          out.rvfi <= in.rvfi;
         `endif
          out.rd <= in.rd;
          // A load leaves here with no result; the accessor merges the bus answer
          // in next cycle.
          out.rd_data <= 0;
          (* parallel_case, full_case *)
          case (1'b1)
            in.is_add: out.rd_data <= rs1 + rs2;
            in.is_sub: out.rd_data <= alu_sub[31:0];
            in.is_sll: out.rd_data <= shift_rev;
            in.is_slt: out.rd_data <= {31'b0, alu_lt};
            in.is_sltu: out.rd_data <= {31'b0, alu_ltu};
            in.is_xor: out.rd_data <= rs1 ^ rs2;
            in.is_srl || in.is_sra: out.rd_data <= shift_res;
            in.is_or: out.rd_data <= rs1 | rs2;
            in.is_and: out.rd_data <= rs1 & rs2;
            in.is_mul || in.is_mulh || in.is_mulhu || in.is_mulhsu: begin
             `ifndef RISCV_FORMAL_ALTOPS
              if (in.is_mul) begin
                out.rd_data <= mul_lo;
              end else begin
                out.rd_data <= mul_hi;
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
              op_is_div <= in.is_div;
              op_is_divu <= in.is_divu;
              op_is_rem <= in.is_rem;
              op_is_remu <= in.is_remu;
              op_sign_x <= in.rs1[31];
              op_sign_y <= in.rs2[31];
             `ifndef RISCV_FORMAL_ALTOPS
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
            // Loads, stores, fence and wfi compute nothing here; a CSR access or
            // a LUI arrives from decode as an add with rs2 zeroed.
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
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // Reset is assumed before the first edge and never after it: nothing in the
  // tree discharges either, and a reset mid-divide would zero out.rd_data where
  // the completion assertions expect a result.
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // Reset reaches `state` only at the first edge, so without this the basecase
  // could start at step 0 in `divide` with the divider's registers free.
  initial state = init;
  always_comb if (clocked) assume(!reset);

  // `is_valid_instr` and `is_amo` are left out on purpose: the first is every
  // no-result instruction's arm, the second the OR of nine flags already listed.
  // The eleven atomics fall to that no-result arm: an AMO's operands are the
  // memory word and rs2, which this stage never sees. An op flag added to the
  // struct and not here silently widens the environment.
  always_comb assume($onehot0({in.is_add, in.is_sub, in.is_xor, in.is_or, in.is_and,
    in.is_sll, in.is_slt, in.is_sltu, in.is_srl, in.is_sra,
    in.is_mul, in.is_mulh, in.is_mulhu, in.is_mulhsu,
    in.is_div, in.is_divu, in.is_rem, in.is_remu,
    in.is_lb, in.is_lbu, in.is_lh, in.is_lhu, in.is_lw, in.is_sb, in.is_sh, in.is_sw,
    in.is_amoswap, in.is_amoadd, in.is_amoxor, in.is_amoand, in.is_amoor,
    in.is_amomin, in.is_amomax, in.is_amominu, in.is_amomaxu,
    in.is_lr, in.is_sc}));

  // Every signed reference here is a signed net of its own, never an arm of a
  // conditional, where sign-context rules would evaluate it unsigned.
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

  // The bound is a hypothesis rather than a neighbouring assertion because
  // k-induction cannot use one assertion to discharge another; above it the
  // shifted remainder can carry into bit 32 and the bit stops being a borrow.
  always_comb
    if (div_rem < div_divisor) assert(rem_sub[32] == (rem_shifted < {1'b0, div_divisor}));
  always_comb
    if (div_rem < div_divisor && rem_sub[32]) assert(rem_shifted[32] == 1'b0);

  // The references extend by width-extending assignment rather than by
  // restating the RTL's sign expression, so a sign taken from the wrong bit
  // disagrees here.
  logic [32:0] rs1_sext33, rs2_sext33, rs1_zext33, rs2_zext33;
  assign rs1_sext33 = $signed(in.rs1);
  assign rs2_sext33 = $signed(in.rs2);
  assign rs1_zext33 = {1'b0, in.rs1};
  assign rs2_zext33 = {1'b0, in.rs2};
  logic [32:0] mul_op_x_ref, mul_op_y_ref;
  assign mul_op_x_ref = (in.is_mulh || in.is_mulhsu) ? rs1_sext33 : rs1_zext33;
  assign mul_op_y_ref = in.is_mulh ? rs2_sext33 : rs2_zext33;
  always_comb assert({mul_sign_x, in.rs1} == mul_op_x_ref);
  always_comb assert({mul_sign_y, in.rs2} == mul_op_y_ref);

  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(in.is_mul))
      assert(out.rd_data == $past(mul_lo));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(in.is_mulh))
      assert(out.rd_data == $past(mul_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(in.is_mulhu))
      assert(out.rd_data == $past(mul_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(in.is_mulhsu))
      assert(out.rd_data == $past(mul_hi));

  // A multiply is single-cycle whatever its operands. Guarded on $past(state)
  // == init, as the four above are, because `in` is free while a divide runs
  // and may name a multiply that never issued.
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init &&
        $past(in.is_mul || in.is_mulh || in.is_mulhu || in.is_mulhsu))
      assert(state == init);

  // Each multiplies by a constant, so the solver sees shifts and adds rather
  // than a second `bvmul` term. A miter against a signed 33x33 product returns
  // no verdict, and neither does an `in.rs2 == 32'hffffffff` lemma.
  logic [63:0] mul_result;
  assign mul_result = {mul_hi, mul_lo};
  always_comb if (in.rs1 == 32'b0) assert(mul_result == 64'b0);
  always_comb if (in.rs2 == 32'b0) assert(mul_result == 64'b0);
  always_comb if (in.rs2 == 32'h1 && !mul_sign_y)
    assert(mul_result == {{32{mul_sign_x}}, in.rs1});
  always_comb if (in.rs1 == 32'h1 && !mul_sign_x)
    assert(mul_result == {{32{mul_sign_y}}, in.rs2});

  // Proof-only copies of the operands the divider loaded, taken on the edge it
  // loads on, because `in` is free while it runs.
  logic [31:0] div_ghost_rs1, div_ghost_rs2;
  always_ff @(posedge clk)
    if (!reset && state == init) begin
      div_ghost_rs1 <= in.rs1;
      div_ghost_rs2 <= in.rs2;
    end

  logic [31:0] div_mag_x, div_mag_y;
  assign div_mag_x = (op_is_div || op_is_rem) && div_ghost_rs1[31] ? -div_ghost_rs1 : div_ghost_rs1;
  assign div_mag_y = (op_is_div || op_is_rem) && div_ghost_rs2[31] ? -div_ghost_rs2 : div_ghost_rs2;

  always_comb
    if (state == divide) assert($onehot({op_is_div, op_is_divu, op_is_rem, op_is_remu}));
  always_comb if (state == divide) assert(op_sign_x == div_ghost_rs1[31]);
  always_comb if (state == divide) assert(op_sign_y == div_ghost_rs2[31]);
  always_comb if (state == divide) assert(div_divisor == div_mag_y);

  // Without this k-induction may start from a wild counter that the invariant
  // below reads as an exact count of the iterations left.
  always_comb if (state == divide) assert(mul_div_counter <= 32);

  // A restriction on the proof, not the design, buying solver time on the
  // symbolic product. Guarded to the divide state on purpose: unguarded it is
  // proof-global, zeroed every multiply operand's high half, and let three
  // known multiplier defects pass.
  localparam [31:0] div_proof_cap = 32'h000000ff;
  always_comb if (state == divide) assume(div_mag_x <= div_proof_cap);
  always_comb if (state == divide) assume(div_mag_y <= div_proof_cap);

  // With n iterations left and k = 32 - n run: the dividend's top k bits are
  // divided, their quotient in div_quot's low k bits and div_rem the remainder,
  // and its other n bits still sit in div_quot's top. At n == 0 this is the
  // division identity.
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

  // Unreachable in the basecase: the real divide needs 33 cycles and `mode
  // prove` runs 20 basecase steps. A mutation that breaks one of these reports
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
