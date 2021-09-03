`default_nettype none
module littlecpu(
  input  var logic        clk,
  input  var logic        reset,
  input  var logic        mem_valid,
  output var logic        mem_ready,
  output var logic [31:0] mem_addr,
  output var logic [31:0] mem_wdata,
  output var logic [3:0]  mem_wstrb,
  input  var logic [31:0] mem_rdata,
  output var logic        trap
 `ifdef RISCV_FORMAL
  ,
  output var logic        rvfi_valid,
  output var logic [63:0] rvfi_order,
  output var logic [31:0] rvfi_insn,
  output var logic        rvfi_trap,
  output var logic        rvfi_halt,
  output var logic        rvfi_intr,
  output var logic [ 1:0] rvfi_mode,
  output var logic [ 1:0] rvfi_ixl,
  output var logic [ 4:0] rvfi_rs1_addr,
  output var logic [ 4:0] rvfi_rs2_addr,
  output var logic [31:0] rvfi_rs1_rdata,
  output var logic [31:0] rvfi_rs2_rdata,
  output var logic [ 4:0] rvfi_rd_addr,
  output var logic [31:0] rvfi_rd_wdata,
  output var logic [31:0] rvfi_pc_rdata,
  output var logic [31:0] rvfi_pc_wdata,
  output var logic [31:0] rvfi_mem_addr,
  output var logic [ 3:0] rvfi_mem_rmask,
  output var logic [ 3:0] rvfi_mem_wmask,
  output var logic [31:0] rvfi_mem_rdata,
  output var logic [31:0] rvfi_mem_wdata,
  output var logic [63:0] rvfi_csr_mcycle_rmask,
  output var logic [63:0] rvfi_csr_mcycle_wmask,
  output var logic [63:0] rvfi_csr_mcycle_rdata,
  output var logic [63:0] rvfi_csr_mcycle_wdata,
  output var logic [63:0] rvfi_csr_minstret_rmask,
  output var logic [63:0] rvfi_csr_minstret_wmask,
  output var logic [63:0] rvfi_csr_minstret_rdata,
  output var logic [63:0] rvfi_csr_minstret_wdata
 `endif //  `ifdef RISCV_FORMAL
);
  logic        mem_instr;
  logic        fetcher_valid, fetcher_mem_ready, accessor_mem_ready;
  // possible hazard here
  assign mem_ready = fetcher_mem_ready || accessor_mem_ready;
  logic [31:0] pc;
  fetcher_output fetcher_out;
  fetcher fetcher(
    .clk(clk),
    .reset(reset),
    // handshake
    .mem_valid(mem_valid),
    .fetcher_valid(fetcher_valid),
    .fetcher_ready(fetcher_decoder_ready),
    // inputs
    .pc(pc),
    .mem_rdata(mem_rdata),
    // outputs
    .out(fetcher_out),
    .mem_instr(mem_instr),
    .mem_ready(fetcher_mem_ready),
    .mem_addr(mem_addr)
  );

  logic        fetcher_decoder_ready, fetcher_decoder_valid;
  fetcher_output  fetcher_decoder_out;
  skidbuffer #(.WIDTH($bits(fetcher_out))) fetcher_decoder(
    .clk(clk),
    .reset(reset),
    .input_ready(fetcher_decoder_ready),
    .input_valid(fetcher_valid),
    .input_data(fetcher_out),
    .output_ready(decoder_ready),
    .output_valid(fetcher_decoder_valid),
    .output_data(fetcher_decoder_out)
  );

  logic [31:0] reg_rs1, reg_rs2, wdata;
  logic [4:0]  waddr;
  logic        wen;
  regfile regfile(
    .clk(clk),
    // from the decoder
    .rs1(rs1),
    .rs2(rs2),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
  );

  logic        decoder_ready, decoder_valid;
  logic [31:0] decoder_reg_rs1, decoder_reg_rs2;
  logic [31:0] decoder_mem_addr;
  logic [4:0]  rs1, rs2, decoder_rd;
  logic is_valid_instr;
  logic is_add, is_sub, is_mul, is_mulh, is_mulhu, is_mulhsu, is_div, is_divu, is_rem, is_remu,
    is_xor, is_or, is_and, is_sll, is_slt, is_sltu, is_srl, is_sra, is_lui, is_lb, is_lbu, is_lhu,
    is_lh, is_lw, is_sb, is_sh, is_sw, is_ecall, is_ebreak, is_csrrw, is_csrrs, is_csrrc;
  decoder decoder(
    .clk(clk),
    .reset(reset),
    // handshake
    .fetcher_valid(fetcher_decoder_valid),
    .decoder_ready(decoder_ready),
    .decoder_valid(decoder_valid),
    .executor_ready(executor_ready),
    // inputs
    .in(fetcher_decoder_out),
    // The decoder is largely synchronous so these are assigned a clock cycle early
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    // forwards
    .decoder_reg_rs1(decoder_reg_rs1),
    .decoder_reg_rs2(decoder_reg_rs2),
    .decoder_rd(decoder_rd),
    // outputs
    // The whole trick! we update the program counter here to keep the pipeline filled
    .pc(pc),
     // rs1 and rs2 are not latched: used to get reg_rs1 and reg_rs2 from the reg file
    .rs1(rs1),
    .rs2(rs2),
    .decoder_mem_addr(decoder_mem_addr),
    .is_valid_instr(is_valid_instr),
    .is_add(is_add),
    .is_sub(is_sub),
    .is_xor(is_xor),
    .is_or(is_or),
    .is_and(is_and),
    .is_mul(is_mul),
    .is_mulh(is_mulh),
    .is_mulhu(is_mulhu),
    .is_mulhsu(is_mulhsu),
    .is_div(is_div),
    .is_divu(is_divu),
    .is_rem(is_rem),
    .is_remu(is_remu),
    .is_sll(is_sll),
    .is_slt(is_slt),
    .is_sltu(is_sltu),
    .is_srl(is_srl),
    .is_sra(is_sra),
    .is_lui(is_lui),
    .is_lb(is_lb),
    .is_lbu(is_lbu),
    .is_lh(is_lh),
    .is_lhu(is_lhu),
    .is_lw(is_lw),
    .is_sb(is_sb),
    .is_sh(is_sh),
    .is_sw(is_sw),
    .is_ecall(is_ecall),
    .is_ebreak(is_ebreak),
    .is_csrrw(is_csrrw),
    .is_csrrs(is_csrrs),
    .is_csrrc(is_csrrc)
  );

  logic executor_ready, executor_valid;
  logic [31:0] executor_rd_data, executor_mem_addr, executor_mem_data;
  logic [4:0]  executor_rd;
  executor executor(
    .clk(clk),
    .reset(reset),
    // handshake
    .decoder_valid(decoder_valid),
    .executor_ready(executor_ready),
    .executor_valid(executor_valid),
    .accessor_ready(accessor_ready),
    // inputs
    .decoder_rd(decoder_rd),
    .decoder_reg_rs1(decoder_reg_rs1),
    .decoder_reg_rs2(decoder_reg_rs2),
    .decoder_mem_addr(decoder_mem_addr),
    .is_valid_instr(is_valid_instr),
    .is_add(is_add),
    .is_sub(is_sub),
    .is_xor(is_xor),
    .is_or(is_or),
    .is_and(is_and),
    .is_mul(is_mul),
    .is_mulh(is_mulh),
    .is_mulhu(is_mulhu),
    .is_mulhsu(is_mulhsu),
    .is_div(is_div),
    .is_divu(is_divu),
    .is_rem(is_rem),
    .is_remu(is_remu),
    .is_sll(is_sll),
    .is_slt(is_slt),
    .is_sltu(is_sltu),
    .is_srl(is_srl),
    .is_sra(is_sra),
    .is_lui(is_lui),
    .is_lb(is_lb),
    .is_lbu(is_lbu),
    .is_lh(is_lh),
    .is_lhu(is_lhu),
    .is_lw(is_lw),
    .is_sb(is_sb),
    .is_sh(is_sh),
    .is_sw(is_sw),
    .is_ecall(is_ecall),
    .is_ebreak(is_ebreak),
    .is_csrrw(is_csrrw),
    .is_csrrs(is_csrrs),
    .is_csrrc(is_csrrc),
    // outputs
    .executor_rd(executor_rd),
    .executor_rd_data(executor_rd_data),
    // forwards
    .executor_mem_addr(executor_mem_addr),
    .executor_mem_data(executor_mem_data),
    .executor_is_lui(is_lui),
    .executor_is_lb(is_lb),
    .executor_is_lbu(is_lbu),
    .executor_is_lh(is_lh),
    .executor_is_lhu(is_lhu),
    .executor_is_lw(is_lw),
    .executor_is_sb(is_sb),
    .executor_is_sh(is_sh),
    .executor_is_sw(is_sw)
  );

  logic        accessor_ready, accessor_valid;
  logic [4:0]  accessor_rd;
  logic [31:0] accessor_rd_data;
  accessor accessor(
    .clk(clk),
    .reset(reset),
    // handshake
    .executor_valid(executor_valid),
    .accessor_ready(accessor_ready),
    .accessor_valid(accessor_valid),
    // forwards
    .executor_rd(executor_rd),
    .executor_rd_data(executor_rd_data),
    // inputs
    .executor_mem_addr(executor_mem_addr),
    .executor_mem_data(executor_mem_data),
    .executor_is_lui(is_lui),
    .executor_is_lb(is_lb),
    .executor_is_lbu(is_lbu),
    .executor_is_lh(is_lh),
    .executor_is_lhu(is_lhu),
    .executor_is_lw(is_lw),
    .executor_is_sb(is_sb),
    .executor_is_sh(is_sh),
    .executor_is_sw(is_sw),
    // memory access
    .mem_instr(mem_instr),
    .mem_ready(accessor_mem_ready),
    .mem_valid(mem_valid),
    .mem_wstrb(mem_wstrb),
    .mem_wdata(mem_wdata),
    // outputs
    .accessor_rd(accessor_rd),
    .accessor_rd_data(accessor_rd_data)
  );

  writeback writeback(
    .clk(clk),
    .reset(reset),
    // handshake
    .accessor_valid(accessor_valid),
    // inputs
    .accessor_rd(accessor_rd),
    .accessor_rd_data(accessor_rd_data),
    // outputs
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
  );
endmodule
