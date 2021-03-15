module alu (
  input  var logic        clk,
  input  var logic [31:0] rs1,
  input  var logic [31:0] rs2,
  input  var logic [4:0]  shamt,
  input  var logic        is_add,
  input  var logic        is_sub,
  input  var logic        is_xor,
  input  var logic        is_or,
  input  var logic        is_and,
  input  var logic        is_sll,
  input  var logic        is_slt,
  input  var logic        is_sltu,
  input  var logic        is_srl,
  input  var logic        is_sra,
  input  var logic        is_mul,
  input  var logic        is_mulh,
  input  var logic        is_mulhu,
  input  var logic        is_mulhsu,
  input  var logic        is_div,
  input  var logic        is_divu,
  input  var logic        is_rem,
  input  var logic        is_remu,
  input  var logic        valid,
  output var logic        ready,
  output var logic [31:0] out
);
  logic [1:0] alu_state;
  localparam init = 2'b00;
  localparam multiply = 2'b01;
  localparam divide = 2'b10;
  logic [4:0]  mul_div_counter;
  logic [31:0] mul_div_x, mul_div_y;
  logic [63:0] mul_div_store;

  always_ff @(posedge clk) begin
    if (!valid) begin
      ready <= 0;
      alu_state <= init;
    end else begin
      ready <= 1;
      (* parallel_case, full_case *)
      case(alu_state)
        init: begin
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
              mul_div_counter <= is_mul ? 32 : 64;
              alu_state <= multiply;
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
              mul_div_counter <= 65;
              alu_state <= divide;
              mul_div_store <= 0;
              mul_div_x <= {32'b0,rs1};
              mul_div_y <= {1'b0,rs2,31'b0};
            end
          endcase // case (1'b1)
        end // case: init

        multiply: begin
         `ifndef RISCV_FORMAL_ALTOPS
          if (mul_div_counter > 0) begin
            mul_div_store <= mul_div_y[0] ? mul_div_store + mul_div_x : mul_div_store;
            mul_div_x <= mul_div_x << 1;
            mul_div_y <= mul_div_y >> 1;
            mul_div_counter <= mul_div_counter - 1;
          end else begin
            if (is_mul) begin
              out <= mul_div_store[31:0];
            end else begin
              out <= mul_div_store[63:32];
            end
            alu_state <= init;
            ready <= 1;
          end
         `else
          (* parallel_case, full_case *)
          case (1'b1)
            is_mul: out <= (rs1 + rs2) ^ 32'h5876063e;
            is_mulh: out <= (rs1 + rs2) ^ 32'hf6583fb7;
            is_mulhu: out <= (rs1 + rs2) ^ 32'h949ce5e8;
            is_mulhsu: out <= (rs1 - rs2) ^ 32'hecfbe137;
          endcase
          alu_state <= init;
          ready <= 1;
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
              is_div: out <= rs1[31] != rs2[31] ? -mul_div_store[31:0] : mul_div_store[31:0];
              is_divu: out <= mul_div_store[31:0];
              is_rem: out <= rs1[31] ? -mul_div_x[31:0] : mul_div_x[31:0];
              is_remu: out <= mul_div_x[31:0];
            endcase
            alu_state <= init;
            ready <= 1;
          end
         `else
          (* parallel_case, full_case *)
          case (1'b1)
            is_div: out <= (rs1 - rs2) ^ 32'h7f8529ec;
            is_divu: out <= (rs1 - rs2) ^ 32'h10e8fd70;
            is_rem: out <= (rs1 - rs2) ^ 32'h8da68fa5;
            is_remu: out <= (rs1 - rs2) ^ 32'h3138d0e1;
          endcase
          alu_state <= init;
          ready <= 1;
         `endif
        end
     endcase // case (alu_state)
    end
  end
endmodule
