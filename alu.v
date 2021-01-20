module alu (
  input logic         clk,
  input logic [31:0]  rs1,
  input logic [31:0]  rs2,
  input logic [4:0]   shamt,
  input logic         is_add,
  input logic         is_sub,
  input logic         is_xor,
  input logic         is_or,
  input logic         is_and,
  input logic         is_sll,
  input logic         is_slt,
  input logic         is_sltu,
  input logic         is_srl,
  input logic         is_sra,
  input logic         valid,
  output logic        ready,
  output logic [31:0] out
);
  logic [1:0] alu_state;
  localparam init = 2b'00;
  localparam multiply = 2b'01;
  localparam divide = 2b'10;
  always_ff @(posedge clk) begin
    if (!valid) begin
      ready <= 0;
    end else begin
      ready <= 1;
      (* parallel_case, full_case *)
      case(1'b1)
        is_add: out <= rs1 + rs2;
        is_sub: out <= rs1 - rs2;
        is_sll: out <= rs1 << shamt;
        is_slt: out <= {31'b0, $signed(rs1) < $signed(rs2)};
        is_sltu: out <= {31'b0, rs1 < rs2};
        is_xor: out <= rs1 ^ rs2;
        is_srl: out <= rs1 >> shamt;
        is_sra: out <= $signed(rs1) >>> shamt;
        is_or: out <= rs1 | rs2;
        is_and: out <= rs1 & rs2;
        is_mul || is_mulh || is_mulhu || is_mulhsu: begin
          ready <= 0;
          regs_rs1 <= rs1;
          regs_rs2 <= rs2;
          mul_div_counter <= is_mul ? 32 : 64;
          cpu_state <= multiply;
          mul_div_store <= 0;
          (* parallel_case, full_case *)
          case(1'b1)
            is_mul || is_mulhu: begin
              mul_div_x <= {32'b0,rs1};
              mul_div_y <= {32'b0,rs2};
            end

            is_mulh: begin
              mul_div_x <= {{32{rs1[31]}},rs1};
              mul_div_y <= {{32{rs2[31]}},rs2};
            end

            is_mulhsu: begin
              mul_div_x <= {{32{rs1[31]}},rs1};
              mul_div_y <= {{32'b0},rs2};
            end
          endcase
        end

        is_div || is_divu || is_rem || is_remu: begin
          ready <= 0;
          regs_rs1 <= rs1;
          regs_rs2 <= rs2;
          mul_div_counter <= 65;
          cpu_state <= divide;
          mul_div_store <= 0;
          mul_div_x <= {32'b0,rs1};
          mul_div_y <= {1'b0,rs2,31'b0};
        end
      endcase // case (1'b1)

         multiply: begin
         `ifndef RISCV_FORMAL_ALTOPS
          if (mul_div_counter > 0) begin
            mul_div_store <= mul_div_y[0] ? mul_div_store + mul_div_x : mul_div_store;
            mul_div_x <= mul_div_x << 1;
            mul_div_y <= mul_div_y >> 1;
            mul_div_counter <= mul_div_counter - 1;
          end else begin
            if (is_mul) begin
              reg_wdata <= mul_div_store[31:0];
            end else begin
              reg_wdata <= mul_div_store[63:32];
            end
            cpu_state <= reg_write;
          end
         `else
          cpu_state <= reg_write;
          (* parallel_case, full_case *)
          case (1'b1)
            is_mul: out <= (regs_rs1 + regs_rs2) ^ 32'h5876063e;
            is_mulh: out <= (regs_rs1 + regs_rs2) ^ 32'hf6583fb7;
            is_mulhu: out <= (regs_rs1 + regs_rs2) ^ 32'h949ce5e8;
            is_mulhsu: out <= (regs_rs1 - regs_rs2) ^ 32'hecfbe137;
          endcase
         `endif
        end

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
            case(1'b1)
              is_div: reg_wdata <= regs_rs1[31] != regs_rs2[31] ? -mul_div_store[31:0] : mul_div_store[31:0];
              is_divu: reg_wdata <= mul_div_store[31:0];
              is_rem: reg_wdata <= regs_rs1[31] ? -mul_div_x[31:0] : mul_div_x[31:0];
              is_remu: reg_wdata <= mul_div_x[31:0];
            endcase
            cpu_state <= reg_write;
          end
         `else
          cpu_state <= reg_write;
          (* parallel_case, full_case *)
          case (1'b1)
            is_div: reg_wdata <= (regs_rs1 - regs_rs2) ^ 32'h7f8529ec;
            is_divu: reg_wdata <= (regs_rs1 - regs_rs2) ^ 32'h10e8fd70;
            is_rem: reg_wdata <= (regs_rs1 - regs_rs2) ^ 32'h8da68fa5;
            is_remu: reg_wdata <= (regs_rs1 - regs_rs2) ^ 32'h3138d0e1;
          endcase
         `endif
        end
    end
  end
endmodule
