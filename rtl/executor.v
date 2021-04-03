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
  // forwards
  output var logic [31:0] executor_mem_addr,
  output var logic [4:0]  executor_rd,
  output var logic [31:0] executor_rd_data,
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
  // state machine
  always_ff @(posedge clk) begin
    alu_wait <= 0;
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
