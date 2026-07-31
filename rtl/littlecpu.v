`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module littlecpu(
  input  logic clk,
  input  logic reset,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  // ADR-0003: the second word of the dual-word combinational fetch window,
  // read at imem_addr + 4 so a 32-bit instruction straddling a 4-byte
  // boundary (pc % 4 == 2) can be fetched in the same cycle -- no aligner
  // FSM, no stall.
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
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
  output logic [63:0] rvfi_csr_minstret_wdata,
  output logic [31:0] rvfi_csr_mscratch_rmask,
  output logic [31:0] rvfi_csr_mscratch_wmask,
  output logic [31:0] rvfi_csr_mscratch_rdata,
  output logic [31:0] rvfi_csr_mscratch_wdata
  `endif //  `ifdef RISCV_FORMAL
  );
  // Declared here (ahead of the decoder instantiation below) because the trap
  // assignment below references it; iverilog requires declare-before-use.
  decoder_output decoder_out;
  // Declared here, ahead of the decoder/executor/accessor instantiations,
  // because decoder's scoreboard reads executor_out and accessor's pending
  // load, executor's stalled output feeds decoder's divider_stall, and
  // accessor's stalled output feeds both decoder's accessor_stall and
  // executor's accessor_stall (ADR-0004/ADR-0009); iverilog requires
  // declare-before-use.
  executor_output executor_out;
  logic divider_stalled, accessor_stalled;
  logic       accessor_pending_valid;
  logic [4:0] accessor_pending_rd;
  // ADR-0026: the CSR serialization drain predicate needs to see a *store*
  // in the accessor, which accessor_pending_valid (loads only) cannot show
  // it. accessor_out is declared further down, next to its instantiation, so
  // this carries its valid bit up to the decoder's port list.
  logic accessor_out_valid;
  // trap is asserted when the decoded instruction is unrecognized (illegal-instruction)
  // or when the accessor detects a misaligned memory access.  Gated by !reset so that
  // the pipeline flush state does not produce spurious traps, and by decoder_out.valid
  // so a hazard/divide bubble (which zeroes is_valid_instr along with everything else)
  // never looks like an illegal instruction (ADR-0009).
  logic mem_misaligned_trap;
  assign trap = !reset && ((decoder_out.valid && !decoder_out.is_valid_instr) || mem_misaligned_trap);
  logic  [31:0] pc;
  fetcher_output fetcher_out;
  fetcher fetcher(
    .clk(clk),
    .reset(reset),
    // inputs
    .pc(pc),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    // outputs
    .out(fetcher_out),
    .imem_addr(imem_addr),
    .imem_addr2(imem_addr2)
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

  // ADR-0005: the CSR file is a sibling of the decoder, read and written in
  // decode. Everything between them is combinational within the cycle the
  // accessing instruction issues.
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen;
  logic [31:0] csr_wdata, csr_rdata;
  logic        csr_implemented;
  logic        csr_instret;
 `ifdef RISCV_FORMAL
  rvfi_csr64 csr_rvfi_mcycle, csr_rvfi_minstret;
  rvfi_csr32 csr_rvfi_mscratch;
 `endif

  decoder decoder(
    .clk(clk),
    .reset(reset),
    // inputs
    .in(fetcher_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stalled),
    .accessor_stall(accessor_stalled),
    .accessor_pending_valid(accessor_pending_valid),
    .accessor_pending_rd(accessor_pending_rd),
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
   `ifdef RISCV_FORMAL
    .csr_rvfi_mcycle(csr_rvfi_mcycle),
    .csr_rvfi_minstret(csr_rvfi_minstret),
    .csr_rvfi_mscratch(csr_rvfi_mscratch),
   `endif
    // outputs
    .pc(pc),
    .rs1(rs1),
    .rs2(rs2),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .instret(csr_instret),
    .out(decoder_out)
  );

  csrs csrs(
    .clk(clk),
    .reset(reset),
    // inputs
    .addr(csr_addr),
    .ren(csr_ren),
    .wen(csr_wen),
    .wdata(csr_wdata),
    .instret(csr_instret),
    // outputs
    .rdata(csr_rdata),
    .implemented(csr_implemented)
   `ifdef RISCV_FORMAL
    ,
    .rvfi_mcycle(csr_rvfi_mcycle),
    .rvfi_minstret(csr_rvfi_minstret),
    .rvfi_mscratch(csr_rvfi_mscratch)
   `endif
  );

  executor executor(
    .clk(clk),
    .reset(reset),
    // inputs
    .in(decoder_out),
    .accessor_stall(accessor_stalled),
    // outputs
    .out(executor_out),
    .stalled(divider_stalled)
  );

  accessor_output accessor_out;
  assign accessor_out_valid = accessor_out.valid;
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
    // fault signals
    .mem_misaligned(mem_misaligned_trap),
    .stalled(accessor_stalled),
    .pending_valid(accessor_pending_valid),
    .pending_rd(accessor_pending_rd),
    // forwards
    .out(accessor_out)
  );

  writeback writeback(
    .clk(clk),
    .reset(reset),
    // inputs
    .in(accessor_out),
    // outputs
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
   `ifdef RISCV_FORMAL
    ,
    .rvfi_valid(rvfi_valid),
    .rvfi_order(rvfi_order),
    .rvfi_insn(rvfi_insn),
    .rvfi_rs1_addr(rvfi_rs1_addr),
    .rvfi_rs2_addr(rvfi_rs2_addr),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_rd_addr(rvfi_rd_addr),
    .rvfi_rd_wdata(rvfi_rd_wdata),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_pc_wdata(rvfi_pc_wdata),
    .rvfi_mem_addr(rvfi_mem_addr),
    .rvfi_mem_rmask(rvfi_mem_rmask),
    .rvfi_mem_wmask(rvfi_mem_wmask),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .rvfi_mem_wdata(rvfi_mem_wdata),
    .rvfi_csr_mcycle_rmask(rvfi_csr_mcycle_rmask),
    .rvfi_csr_mcycle_wmask(rvfi_csr_mcycle_wmask),
    .rvfi_csr_mcycle_rdata(rvfi_csr_mcycle_rdata),
    .rvfi_csr_mcycle_wdata(rvfi_csr_mcycle_wdata),
    .rvfi_csr_minstret_rmask(rvfi_csr_minstret_rmask),
    .rvfi_csr_minstret_wmask(rvfi_csr_minstret_wmask),
    .rvfi_csr_minstret_rdata(rvfi_csr_minstret_rdata),
    .rvfi_csr_minstret_wdata(rvfi_csr_minstret_wdata),
    .rvfi_csr_mscratch_rmask(rvfi_csr_mscratch_rmask),
    .rvfi_csr_mscratch_wmask(rvfi_csr_mscratch_wmask),
    .rvfi_csr_mscratch_rdata(rvfi_csr_mscratch_rdata),
    .rvfi_csr_mscratch_wdata(rvfi_csr_mscratch_wdata)
   `endif
  );

 `ifdef RISCV_FORMAL
  // The CSR fields are per-retire data now (rtl/csrs.v, threaded through the
  // shadow to rtl/writeback.v). These five are not, and stay constants:
  // rvfi_trap has nothing upstream computing it until trap entry lands (the
  // next M3 step, ADR-0011), and M-mode only / XLEN=32 / no interrupts
  // (CLAUDE.md: mie and mip are read-only zero) makes mode/ixl/intr
  // properties of the design rather than of an instruction.
  assign rvfi_trap = 1'b0;
  assign rvfi_halt = 1'b0;
  assign rvfi_intr = 1'b0;
  assign rvfi_mode = 2'd3;
  assign rvfi_ixl = 2'd1;
 `endif
endmodule
