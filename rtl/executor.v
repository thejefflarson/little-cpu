`default_nettype none
module executor(
  input  var logic clk,
  input  var logic reset,
  // handshake
  input  var logic decoder_valid,
  output var logic executor_ready,
  output var logic executor_valid,
  input  var logic accessor_ready,
  // inputs
  input  var logic [4:0]  decoder_rd,
  input  var logic [31:0] decoder_reg_rs1,
  input  var logic [31:0] decoder_reg_rs2,
  input  var logic [31:0] decoder_rs1,
  input  var logic [31:0] decoder_rs2,
  input  var logic [31:0] decoder_mem_addr,
  input  var logic        is_valid_instr,
  input  var logic        is_add,
  input  var logic        is_sub,
  input  var logic        is_xor,
  input  var logic        is_or,
  input  var logic        is_and,
  input  var logic        is_mul,
  input  var logic        is_mulh,
  input  var logic        is_mulhu,
  input  var logic        is_mulhsu,
  input  var logic        is_div,
  input  var logic        is_divu,
  input  var logic        is_rem,
  input  var logic        is_remu,
  input  var logic        is_sll,
  input  var logic        is_slt,
  input  var logic        is_sltu,
  input  var logic        is_srl,
  input  var logic        is_sra,
  input  var logic        is_lui,
  input  var logic        is_lb,
  input  var logic        is_lbu,
  input  var logic        is_lh,
  input  var logic        is_lhu,
  input  var logic        is_lw,
  input  var logic        is_sb,
  input  var logic        is_sh,
  input  var logic        is_sw,
  input  var logic        is_ecall,
  input  var logic        is_ebreak,
  input  var logic        is_csrrw,
  input  var logic        is_csrrs,
  input  var logic        is_csrrc,
  // outputs
  output var logic [4:0]  executor_rd,
  output var logic [31:0] executor_rd_data,
  // forwards
  output var logic [31:0] executor_mem_addr,
  output var logic        executor_is_lui,
  output var logic        executor_is_lb,
  output var logic        executor_is_lbu,
  output var logic        executor_is_lh,
  output var logic        executor_is_lhu,
  output var logic        executor_is_lw,
  output var logic        executor_is_sb,
  output var logic        executor_is_sh,
  output var logic        executor_is_sw
);
  logic alu_wait, stalled;
  assign stalled = alu_wait;
  logic [31:0] rs1, rs2;
  assign rs1 = decoder_reg_rs1;
  assign rs2 = decoder_reg_rs2;
  // handshake
  always_ff @(posedge clk) begin
    if (reset) begin
      executor_valid <= 0;
    end else if (decoder_valid && !executor_valid && accessor_ready && !stalled) begin
      executor_valid <= executor_valid;
    end else if (!accessor_ready) begin
      executor_valid <= 0;
    end
  end

  // request something from the fetcher
  always_ff @(posedge clk) begin
    if (reset) begin
      executor_ready <= 0;
    end else if(!decoder_valid && !executor_valid && !stalled) begin
      executor_ready <= 1;
    end else begin
      executor_ready <= 0;
    end
  end

  logic state;
  localparam init = 2'b00;
  localparam multiply = 2'b01;
  localparam divide = 2'b10;
  logic [4:0]  mul_div_counter;
  logic [31:0] mul_div_x, mul_div_y;
  logic [63:0] mul_div_store;
  // state machine
  always_ff @(posedge clk) begin
    if (reset) begin
      alu_wait <= 0;
      state <= init;
    end else begin
      executor_rd_data <= 0;
      alu_wait <= 0;
      executor_mem_addr <= decoder_mem_addr;
      executor_rd <= decoder_rd;
      executor_rd_data <= 0;
      executor_is_lui <= is_lui;
      executor_is_lb <= is_lb;
      executor_is_lbu <= is_lbu;
      executor_is_lh <= is_lh;
      executor_is_lhu <= is_lhu;
      executor_is_lw <= is_lw;
      executor_is_sb <= is_sb;
      executor_is_sh <= is_sh;
      executor_is_sw <= is_sw;
      (* parallel_case, full_case *)
      case(state)
        init: begin
          (* parallel_case, full_case *)
          case(1'b1)
            is_add: executor_rd_data <= rs1 + rs2;
            is_sub: executor_rd_data <= rs1 - rs2;
            is_sll: executor_rd_data <= rs1 << rs2;
            is_slt: executor_rd_data <= {31'b0, $signed(rs1) < $signed(rs2)};
            is_sltu: executor_rd_data <= {31'b0, rs1 < rs2};
            is_xor: executor_rd_data <= rs1 ^ rs2;
            is_srl: executor_rd_data <= rs1 >> rs2;
            is_sra: executor_rd_data <= $signed(rs1) >>> rs2;
            is_or: executor_rd_data <= rs1 | rs2;
            is_and: executor_rd_data <= rs1 & rs2;

            is_mul || is_mulh || is_mulhu || is_mulhsu: begin
              alu_wait <= 1;
              mul_div_counter <= is_mul ? 32 : 64;
              state <= multiply;
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
              alu_wait <= 1;
              mul_div_counter <= 65;
              state <= divide;
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
              executor_rd_data <= mul_div_store[31:0];
            end else begin
              executor_rd_data <= mul_div_store[63:32];
            end
            state <= init;
            alu_wait <= 0;
          end
         `else
          (* parallel_case, full_case *)
          case (1'b1)
            is_mul: executor_rd_data <= (rs1 + rs2) ^ 32'h5876063e;
            is_mulh: executor_rd_data <= (rs1 + rs2) ^ 32'hf6583fb7;
            is_mulhu: executor_rd_data <= (rs1 + rs2) ^ 32'h949ce5e8;
            is_mulhsu: executor_rd_data <= (rs1 - rs2) ^ 32'hecfbe137;
          endcase
          state <= init;
          alu_wait <= 0;
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
              is_div: executor_rd_data <= rs1[31] != rs2[31] ? -mul_div_store[31:0] : mul_div_store[31:0];
              is_divu: executor_rd_data <= mul_div_store[31:0];
              is_rem: executor_rd_data <= rs1[31] ? -mul_div_x[31:0] : mul_div_x[31:0];
              is_remu: executor_rd_data <= mul_div_x[31:0];
            endcase
            state <= init;
            alu_wait <= 1;
          end
         `else
          (* parallel_case, full_case *)
          case (1'b1)
            is_div: executor_rd_data <= (rs1 - rs2) ^ 32'h7f8529ec;
            is_divu: executor_rd_data <= (rs1 - rs2) ^ 32'h10e8fd70;
            is_rem: executor_rd_data <= (rs1 - rs2) ^ 32'h8da68fa5;
            is_remu: executor_rd_data <= (rs1 - rs2) ^ 32'h3138d0e1;
          endcase
          state <= init;
          alu_wait <= 1;
         `endif
        end
      endcase;
    end
  end

  // state machine
 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked = 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always @(*) if(!clocked) assume(reset);
  // if we've been valid but stalled, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(executor_valid) && $past(executor_valid && !accessor_ready)) assert(!executor_valid);

  // if we've been valid but the next stage is busy, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(executor_valid) && $past(!accessor_ready)) assert(!executor_valid);

  // if we're stalled we aren't requesting anytthing, and we're not publishing anything
  always_ff @(posedge clk) if(clocked && $past(stalled)) assert(!executor_valid && !executor_ready);
 `endif
endmodule
