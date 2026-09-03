`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module writeback(
  input  logic clk,
  input  logic reset,
  input accessor_output in,
  output logic wen,
  output logic [4:0] waddr,
  output logic [31:0] wdata
 `ifdef RISCV_FORMAL
  ,
  output logic        rvfi_valid,
  output logic        rvfi_trap,
  output logic        rvfi_intr,
  output logic [63:0] rvfi_order,
  output logic [31:0] rvfi_insn,
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
  `ifdef RISCV_FORMAL_MEM_FAULT
  output logic        rvfi_mem_fault,
  output logic [ 3:0] rvfi_mem_fault_rmask,
  output logic [ 3:0] rvfi_mem_fault_wmask,
  `endif
  output logic [63:0] rvfi_csr_mcycle_rmask,
  output logic [63:0] rvfi_csr_mcycle_wmask,
  output logic [63:0] rvfi_csr_mcycle_rdata,
  output logic [63:0] rvfi_csr_mcycle_wdata,
  output logic [63:0] rvfi_csr_minstret_rmask,
  output logic [63:0] rvfi_csr_minstret_wmask,
  output logic [63:0] rvfi_csr_minstret_rdata,
  output logic [63:0] rvfi_csr_minstret_wdata,
  output logic [31:0] rvfi_csr_mscratch_rmask,
  output logic [31:0] rvfi_csr_mscratch_wmask,
  output logic [31:0] rvfi_csr_mscratch_rdata,
  output logic [31:0] rvfi_csr_mscratch_wdata
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  ,
  output logic [31:0] rvfi_csr_mcause_rmask,
  output logic [31:0] rvfi_csr_mcause_wmask,
  output logic [31:0] rvfi_csr_mcause_rdata,
  output logic [31:0] rvfi_csr_mcause_wdata
  `endif
 `endif
);
  // Struct-field reads are continuous assigns wherever they can be: inside an
  // `always_*` each one is a constant part-select iverilog cannot build a
  // precise sensitivity entry for, and it emits a `sorry:` note per read.
  logic        in_valid;
  logic [4:0]  in_rd;
  logic [31:0] in_rd_data;
  assign in_valid   = in.valid;
  assign in_rd      = in.rd;
  assign in_rd_data = in.rd_data;

  assign wen   = !reset && in_valid && (in_rd != 5'b0);
  // These masks are dead logic -- every consumer already tests `wen` -- and
  // they stay: deleting them puts the worst placement under the board clock.
  assign waddr = wen ? in_rd      : 5'b0;
  assign wdata = wen ? in_rd_data : 32'b0;

 `ifdef RISCV_FORMAL
  assign rvfi_trap = in.rvfi.trap;
  assign rvfi_intr = in.rvfi.intr;
 `ifdef RISCV_FORMAL_MEM_FAULT
  assign rvfi_mem_fault = in.rvfi.mem_fault;
  assign rvfi_mem_fault_rmask = in.rvfi.mem_fault_rmask;
  assign rvfi_mem_fault_wmask = in.rvfi.mem_fault_wmask;
  assign rvfi_mem_addr = in.rvfi.mem_fault ? in.rvfi.mem_fault_addr : in.rvfi_mem_addr;
 `else
  assign rvfi_mem_addr = in.rvfi_mem_addr;
 `endif

  always_comb begin
    rvfi_valid = !reset && in.valid;
    rvfi_insn = in.rvfi.insn;
    rvfi_rs1_addr = in.rvfi.rs1_addr;
    rvfi_rs2_addr = in.rvfi.rs2_addr;
    rvfi_rs1_rdata = in.rvfi.rs1_rdata;
    rvfi_rs2_rdata = in.rvfi.rs2_rdata;
    // RVFI requires rd_wdata to read zero for x0, whatever the ALU produced.
    rvfi_rd_addr = in.rd;
    rvfi_rd_wdata = |in.rd ? in.rd_data : 32'b0;
    rvfi_pc_rdata = in.rvfi.pc_rdata;
    rvfi_pc_wdata = in.rvfi.pc_wdata;
    rvfi_mem_rmask = in.rvfi_mem_rmask;
    rvfi_mem_wmask = in.rvfi_mem_wmask;
    rvfi_mem_rdata = in.rvfi_mem_rdata;
    rvfi_mem_wdata = in.rvfi_mem_wdata;
  end

  assign rvfi_csr_mcycle_rmask   = in.rvfi.csr_mcycle.rmask;
  assign rvfi_csr_mcycle_wmask   = in.rvfi.csr_mcycle.wmask;
  assign rvfi_csr_mcycle_rdata   = in.rvfi.csr_mcycle.rdata;
  assign rvfi_csr_mcycle_wdata   = in.rvfi.csr_mcycle.wdata;
  assign rvfi_csr_minstret_rmask = in.rvfi.csr_minstret.rmask;
  assign rvfi_csr_minstret_wmask = in.rvfi.csr_minstret.wmask;
  assign rvfi_csr_minstret_rdata = in.rvfi.csr_minstret.rdata;
  assign rvfi_csr_minstret_wdata = in.rvfi.csr_minstret.wdata;
  assign rvfi_csr_mscratch_rmask = in.rvfi.csr_mscratch.rmask;
  assign rvfi_csr_mscratch_wmask = in.rvfi.csr_mscratch.wmask;
  assign rvfi_csr_mscratch_rdata = in.rvfi.csr_mscratch.rdata;
  assign rvfi_csr_mscratch_wdata = in.rvfi.csr_mscratch.wdata;
 `ifdef RISCV_FORMAL_CSR_MCAUSE
  assign rvfi_csr_mcause_rmask   = in.rvfi.csr_mcause.rmask;
  assign rvfi_csr_mcause_wmask   = in.rvfi.csr_mcause.wmask;
  assign rvfi_csr_mcause_rdata   = in.rvfi.csr_mcause.rdata;
  assign rvfi_csr_mcause_wdata   = in.rvfi.csr_mcause.wdata;
 `endif

  always_ff @(posedge clk) begin
    if (reset) rvfi_order <= 64'b0;
    else if (rvfi_valid) rvfi_order <= rvfi_order + 64'b1;
  end
 `endif
endmodule
