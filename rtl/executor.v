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

  logic [1:0]  state;
  localparam init = 2'b00;
  localparam divide = 2'b10;
  logic [6:0]  mul_div_counter;
  logic [63:0] mul_div_x, mul_div_y;
  logic [63:0] mul_div_store;
  logic stalled;
  always_comb
    stalled = state != init;

  // multiply from:
  logic mul_sign_x = in.rs1[0] & in.is_mulh;
  logic mul_sign_y = in.rs2[0] & (in.is_mulh | in.is_mulhsu);
  logic signed [63:0] multiply = {mul_sign_x, in.rs1} * {mul_sign_y, in.rs2};

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
              mul_div_counter <= 65;
              state <= divide;
              mul_div_store <= 0;
              mul_div_x <= {32'b0, rs1};
              mul_div_y <= {1'b0, rs2, 31'b0};
            end
            in.is_ecall || in.is_ebreak || in.is_csrrw || in.is_csrrs || in.is_csrrc: ;
            in.is_valid_instr: ;
          endcase // case (1'b1)
        end // case: init

        divide: begin
         `ifndef RISCV_FORMAL_ALTOPS
          if (mul_div_counter > 0) begin
            if (mul_div_x <= mul_div_y) begin
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
  // if we've been valid but stalled, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(executor_valid) && $past(executor_valid && !accessor_ready)) assert(!executor_valid);

  // if we've been valid but the next stage is busy, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(executor_valid) && $past(!accessor_ready)) assert(!executor_valid);

  // if we're stalled we aren't requesting anytthing, and we're not publishing anything
  always_ff @(posedge clk) if(clocked && $past(stalled)) assert(!executor_valid);
  always_ff @(posedge clk) if(clocked && !$past(reset) && $past(stalled)) assert(!executor_ready);
 `endif
endmodule
