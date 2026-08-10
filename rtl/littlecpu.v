`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module littlecpu(
  input  logic clk,
  input  logic reset,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  // The value `imem_addr` takes on the next edge. A synchronous memory latches
  // this on the same edge the PC moves, so its data is ready for the whole of
  // the cycle that needs it; a combinational memory -- test benches,
  // formal/wrapper.v -- leaves it unread and answers `imem_addr` directly.
  output logic [31:0] imem_addr_next,
  // The fetch and data buses share one address space, and the instruction
  // memory arbitrates: a load or store to the text range takes the read port
  // for that cycle and the fetch that lost it comes back as `fetch_stall`.
  // A memory that keeps its two buses separate ties `fetch_stall` low and
  // leaves `mem_ren` unread.
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  output logic        mem_ren,
  input  logic [31:0] mem_rdata,
  input  logic        fetch_stall,
  // The platform's machine-timer line. Registered at its source (rtl/timer.v
  // does it), because inside the core it is one gate away from the fetch loop.
  // A platform with no timer ties it low and the core never takes an interrupt.
  input  logic        irq_timer,
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
  // Declared up here rather than next to the module that drives each one. Later
  // stages feed signals back to earlier ones, so each of these is read by a
  // module written above its driver, and iverilog wants the name first.
  decoder_output decoder_out;
  executor_output executor_out;
  logic divider_stalled, accessor_stalled;
  logic       accessor_pending_valid;
  logic [4:0] accessor_pending_rd;
  // `accessor_pending_valid` only goes high for loads. The decoder needs this as
  // well, so it can tell a store is still in the accessor before it lets a CSR
  // access issue.
  logic accessor_out_valid;
  // High for one cycle per trap, not a level. The test benches watch it to catch
  // a trap taken before the handler address is installed: mtvec resets to 0 and
  // test/asm/sections.lds links .text at 0, so such a trap restarts the program
  // silently and would otherwise look like a timeout.
  logic decoder_trap_entry;
  assign trap = decoder_trap_entry;
  logic  [31:0] pc;
  logic  [31:0] next_pc;
  fetcher_output fetcher_out;
  fetcher fetcher(
    .clk(clk),
    .reset(reset),
    .pc(pc),
    .next_pc(next_pc),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .out(fetcher_out),
    .imem_addr(imem_addr),
    .imem_addr2(imem_addr2),
    .imem_addr_next(imem_addr_next)
  );

  logic [31:0] reg_rs1, reg_rs2, wdata;
  logic [4:0]  read_rs1, read_rs2;
  logic [4:0]  waddr;
  logic        wen;
  regfile regfile(
    .clk(clk),
    .rs1(read_rs1),
    .rs2(read_rs2),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
  );

  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen;
  logic [31:0] csr_wdata, csr_rdata;
  logic        csr_implemented;
  logic        csr_instret;
  logic        csr_trap_entry, csr_mret_entry;
  logic [31:0] csr_trap_cause, csr_trap_epc;
  logic [31:0] csr_mtvec, csr_mepc;
  logic        csr_interrupt_pending;
 `ifdef RISCV_FORMAL
  rvfi_csr64 csr_rvfi_mcycle, csr_rvfi_minstret;
  rvfi_csr32 csr_rvfi_mscratch;
 `endif

  decoder decoder(
    .clk(clk),
    .reset(reset),
    .in(fetcher_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stalled),
    .accessor_stall(accessor_stalled),
    .fetch_stall(fetch_stall),
    .accessor_pending_valid(accessor_pending_valid),
    .accessor_pending_rd(accessor_pending_rd),
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(csr_mtvec),
    .mepc(csr_mepc),
    .interrupt_pending(csr_interrupt_pending),
   `ifdef RISCV_FORMAL
    .csr_rvfi_mcycle(csr_rvfi_mcycle),
    .csr_rvfi_minstret(csr_rvfi_minstret),
    .csr_rvfi_mscratch(csr_rvfi_mscratch),
   `endif
    .pc(pc),
    .next_pc(next_pc),
    .read_rs1(read_rs1),
    .read_rs2(read_rs2),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .instret(csr_instret),
    .trap_entry(decoder_trap_entry),
    .trap_cause(csr_trap_cause),
    .trap_epc(csr_trap_epc),
    .mret_entry(csr_mret_entry),
    .out(decoder_out)
  );

  csrs csrs(
    .clk(clk),
    .reset(reset),
    .addr(csr_addr),
    .ren(csr_ren),
    .wen(csr_wen),
    .wdata(csr_wdata),
    .instret(csr_instret),
    .trap_entry(decoder_trap_entry),
    .trap_cause(csr_trap_cause),
    .trap_epc(csr_trap_epc),
    .mret_entry(csr_mret_entry),
    .irq_timer(irq_timer),
    .rdata(csr_rdata),
    .implemented(csr_implemented),
    .mtvec_value(csr_mtvec),
    .mepc_value(csr_mepc),
    .interrupt_pending(csr_interrupt_pending)
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
    .in(decoder_out),
    .accessor_stall(accessor_stalled),
    .out(executor_out),
    .stalled(divider_stalled)
  );

  accessor_output accessor_out;
  assign accessor_out_valid = accessor_out.valid;
  accessor accessor(
    .clk(clk),
    .reset(reset),
    .in(executor_out),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_wdata(mem_wdata),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .stalled(accessor_stalled),
    .pending_valid(accessor_pending_valid),
    .pending_rd(accessor_pending_rd),
    .out(accessor_out)
  );

  writeback writeback(
    .clk(clk),
    .reset(reset),
    .in(accessor_out),
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
   `ifdef RISCV_FORMAL
    ,
    .rvfi_valid(rvfi_valid),
    .rvfi_trap(rvfi_trap),
    .rvfi_intr(rvfi_intr),
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
  // These three never change. There is one privilege mode, registers are 32
  // bits wide and the core cannot be halted, so none of them depends on which
  // instruction just finished. `rvfi_intr` does, and comes out of writeback
  // with the rest of the retire.
  assign rvfi_halt = 1'b0;
  assign rvfi_mode = 2'd3;
  assign rvfi_ixl = 2'd1;
 `endif
endmodule
