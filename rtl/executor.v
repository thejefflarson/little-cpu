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
  input  decoder_output in,
  // outputs
  output executor_output out,
  // forwards
  output executor_forward forward,
);
  logic [31:0] rs1, rs2;
  assign rs1 = in.reg_rs1;
  assign rs2 = in.reg_rs2;

  handshake handshake(
    .clk(clk),
    .reset(reset),
    .unit_ready(executor_ready),
    .input_valid(decoder_valid),
    .output_ready(accessor_ready),
    .unit_valid(executor_valid),
    .busy(stalled)
  );

  logic [1:0]  state;
  localparam init = 2'b00;
  localparam multiply = 2'b01;
  localparam divide = 2'b10;
  logic [6:0]  mul_div_counter;
  logic [63:0] mul_div_x, mul_div_y;
  logic [63:0] mul_div_store;
  logic stalled;
  always_ff @(posedge clk) begin
    stalled <= state != init;
  end

  typedef struct packed {
    logic        is_mul;
    `ifdef RISCV_FORMAL_ALTOPS
    logic        is_mulh;
    logic        is_mulhu;
    logic        is_mulhsu;
    logic [31:0] rs1;
    logic [31:0] rs2;
    `endif
  } mul_reg_args;
  mul_reg_args mul_reg;
  typedef struct packed {
    logic        is_div;
    logic        is_divu;
    logic        is_rem;
    logic        is_remu;
    logic [31:0] rs1;
    logic [31:0] rs2;
  } div_reg_args;
  div_reg_args div_reg;
  // state machine
  always_ff @(posedge clk) begin
    if (reset) begin
      state <= init;
      out <= 0;
      forward <= 0;
      div_reg <= 0;
      mul_reg <= 0;
      mul_div_counter <= 0;
      mul_div_store <= 0;
      mul_div_x <= 0;
      mul_div_y <= 0;
    end else if(decoder_valid || stalled) begin
      (* parallel_case, full_case *)
      case (state)
        init: begin
          out.rd <= in.rd;
          out.rd_data <= 0;
          forward.mem_addr <= in.mem_addr;
          forward.mem_data <= in.reg_rs2;
          forward.is_lui <= in.is_lui;
          forward.is_lb <= in.is_lb;
          forward.is_lbu <= in.is_lbu;
          forward.is_lh <= in.is_lh;
          forward.is_lhu <= in.is_lhu;
          forward.is_lw <= in.is_lw;
          forward.is_sb <= in.is_sb;
          forward.is_sh <= in.is_sh;
          forward.is_sw <= in.is_sw;
          (* parallel_case, full_case *)
          case(1'b1)
            in.is_add: out.rd_data <= rs1 + rs2;
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
              mul_div_counter <= in.is_mul ? 32 : 64;
              state <= multiply;
              mul_div_store <= 0;
              mul_reg.is_mul <= in.is_mul;
             `ifdef RISCV_FORMAL_ALTOPS
              mul_reg.is_mulhu <= in.is_mulhu;
              mul_reg.is_mulh <= in.is_mulh;
              mul_reg.is_mulhsu <= in.is_mulhsu;
              mul_reg.rs1 <= rs1;
              mul_reg.rs2 <= rs2;
              `endif
              (* parallel_case, full_case *)
              case(1'b1)
                in.is_mul || in.is_mulhu: begin
                  mul_div_x <= {32'b0,rs1};
                  mul_div_y <= {32'b0,rs2};
                end

                in.is_mulh: begin
                  mul_div_x <= {{32{rs1[31]}},rs1};
                  mul_div_y <= {{32{rs2[31]}},rs2};
                end

                in.is_mulhsu: begin
                  mul_div_x <= {{32{rs1[31]}},rs1};
                  mul_div_y <= {{32'b0},rs2};
                end
              endcase
            end

            in.is_div || in.is_divu || in.is_rem || in.is_remu: begin
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
            if (mul_reg.is_mul) begin
              out.rd_data <= mul_div_store[31:0];
            end else begin
              out.rd_data <= mul_div_store[63:32];
            end
            state <= init;
          end
         `else
          (* parallel_case, full_case *)
          case (1'b1)
            in.is_mul: out.rd_data <= (mul_reg.rs1 + mul_reg.rs2) ^ 32'h5876063e;
            in.is_mulh: out.rd_data <= (mul_reg.rs1 + mul_reg.rs2) ^ 32'hf6583fb7;
            in.is_mulhu: out.rd_data <= (mul_reg.rs1 + mul_reg.rs2) ^ 32'h949ce5e8;
            in.is_mulhsu: out.rd_data <= (mul_reg.rs1 - mul_reg.rs2) ^ 32'hecfbe137;
          endcase
          state <= init;
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
              div_reg.is_div: out.rd_data <= div_reg.rs1[31] != div_reg.rs2[31] ? -mul_div_store[31:0] : mul_div_store[31:0];
              div_reg.is_divu: out.rd_data <= mul_div_store[31:0];
              div_reg.is_rem: out.rd_data <= div_reg.rs1[31] ? -mul_div_x[31:0] : mul_div_x[31:0];
              div_reg.is_remu: out.rd_data <= mul_div_x[31:0];
            endcase
            state <= init;
          end
         `else
          (* parallel_case, full_case *)
          case (1'b1)
            div_reg.is_div: out.rd_data <= (div_reg.rs1 - div_reg.rs2) ^ 32'h7f8529ec;
            div_reg.is_divu: out.rd_data <= (div_reg.rs1 - div_reg.rs2) ^ 32'h10e8fd70;
            div_reg.is_rem: out.rd_data <= (div_reg.rs1 - div_reg.rs2) ^ 32'h8da68fa5;
            div_reg.is_remu: out.rd_data <= (div_reg.rs1 - div_reg.rs2) ^ 32'h3138d0e1;
          endcase
          state <= init;
         `endif
        end
      endcase;
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
