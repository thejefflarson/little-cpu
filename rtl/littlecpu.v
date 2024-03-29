`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module littlecpu(
  input  logic clk,
  input  logic reset,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
  output logic trap
  `ifdef RISCV_FORMAL
  ,
  output logic rvfi_valid,
  output logic [63:0] rvfi_order,
  output logic [31:0] rvfi_insn,
  output logic rvfi_trap,
  output logic rvfi_halt,
  output logic rvfi_intr,
  output logic [ 1:0] rvfi_mode,
  output logic [ 1:0] rvfi_ixl,
  output logic [ 4:0] rvfi_rs1_addr,
  output logic [ 4:0] rvfi_rs2_addr,
  output logic [31:0] rvfi_rs1_rdata,
  output logic [31:0] rvfi_rs2_rdata,
  output logic [ 4:0] rvfi_rd_addr,
  output logic [31:0] rvfi_rd_wdata,
  output logic [31:0] rvfi_pc_rdata,
  output logic [31:0] rvfi_pc_wdata,
  output logic [31:0] rvfi_mem_addr,
  output logic [ 3:0] rvfi_mem_rmask,
  output logic [ 3:0] rvfi_mem_wmask,
  output logic [31:0] rvfi_mem_rdata,
  output logic [31:0] rvfi_mem_wdata,
  output logic [63:0] rvfi_csr_mcycle_rmask,
  output logic [63:0] rvfi_csr_mcycle_wmask,
  output logic [63:0] rvfi_csr_mcycle_rdata,
  output logic [63:0] rvfi_csr_mcycle_wdata,
  output logic [63:0] rvfi_csr_minstret_rmask,
  output logic [63:0] rvfi_csr_minstret_wmask,
  output logic [63:0] rvfi_csr_minstret_rdata,
  output logic [63:0] rvfi_csr_minstret_wdata
  `endif //  `ifdef RISCV_FORMAL
  );
  assign trap = 0;
  logic  [31:0] pc;
  fetcher_output fetcher_out;
  fetcher fetcher(
    .clk(clk),
    .reset(reset),
    // inputs
    .pc(pc),
    .imem_data(imem_data),
    // outputs
    .out(fetcher_out),
    .imem_addr(imem_addr)
  );

  logic [31:0] reg_rs1, reg_rs2, wdata;
  logic [4:0]  rs1, rs2;
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

  decoder_output decoder_out;
  decoder decoder(
    .clk(clk),
    .reset(reset),
    // inputs
    .in(fetcher_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    // outputs
    .pc(pc),
    .rs1(rs1),
    .rs2(rs2),
    .out(decoder_out)
  );

  executor_output executor_out;
  executor executor(
    .clk(clk),
    .reset(reset),
    // inputs
    .in(decoder_out),
    // outputs
    .out(executor_out)
  );

  accessor_output accessor_out;
  accessor accessor(
    .clk(clk),
    .reset(reset),
    // inputs
    .in(executor_out),
    // memory access
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_wdata(mem_wdata),
    .mem_rdata(mem_rdata),
    // forwards
    .out(accessor_out)
  );

  logic writeback_ready;
  writeback writeback(
    .clk(clk),
    .reset(reset),
    // inputs
    .in(accessor_out),
    // outputs
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
  );
endmodule
