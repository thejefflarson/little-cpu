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
  // ADR-0054: the value `imem_addr` will take on the next edge, published
  // combinationally this cycle so a SYNCHRONOUS instruction memory can latch
  // it and have `imem_data`/`imem_data2` ready for the whole of the cycle that
  // needs them. Every memory primitive on the target part is synchronous
  // (ADR-0044); this port is what keeps invariant 1 intact against that fact
  // without a fetch buffer, a stall or a flush.
  //
  // A memory that is genuinely combinational (test benches, the formal
  // environment) may leave it unread and drive `imem_data` off `imem_addr`
  // directly -- which is exactly what formal/wrapper.v does, so the riscv-formal
  // ladder sees this change as one extra, unread output port.
  output logic [31:0] imem_addr_next,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
  // SPIKE: the two ports the text-region arbiter needs. `mem_ren` tells an
  // idle bus from a real load; `fetch_stall` says last cycle's data access
  // took the ROM's read port, so this cycle's fetch window is garbage.
  output logic mem_ren,
  input  logic fetch_stall,
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
  // ADR-0028: `trap` is redefined, not deleted -- it is now a one-cycle "trap
  // entry committed this cycle" pulse, driven straight from the decode stage
  // that commits it (CLAUDE.md invariant 2). It used to be a combinational OR
  // of "the decoded instruction was unrecognised" with the accessor's
  // POST-DECODE misalignment signal, and nothing consumed it. Both halves of
  // that are gone: the accessor no longer detects misalignment (ADR-0011's
  // debt, discharged in rtl/decoder.v), and an unrecognised instruction is now
  // a real illegal-instruction trap rather than a silently suppressed one.
  //
  // It is kept rather than removed because deleting it would change the port
  // list in formal/wrapper.v, formal/dmemcheck.sv, formal/imemcheck.sv,
  // formal/cover.sv, formal/complete.sv and test/testbench.v for no functional
  // gain -- and because ADR-0029's harness check (a trap taken with mtvec == 0
  // is a silent restart at `_start`, since test/asm/sections.lds links .text
  // at 0) needs exactly this signal.
  logic decoder_trap_entry;
  assign trap = decoder_trap_entry;
  logic  [31:0] pc;
  // ADR-0054: decode owns the PC, so decode is where the NEXT one is known.
  // Declared here, ahead of both instantiations, because the fetcher reads it
  // and the decoder drives it; iverilog requires declare-before-use.
  logic  [31:0] next_pc;
  fetcher_output fetcher_out;
  fetcher fetcher(
    .clk(clk),
    .reset(reset),
    // inputs
    .pc(pc),
    .next_pc(next_pc),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    // outputs
    .out(fetcher_out),
    .imem_addr(imem_addr),
    .imem_addr2(imem_addr2),
    .imem_addr_next(imem_addr_next)
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
  // ADR-0005 / ADR-0028: the trap-entry side of the same decode/CSR pair. The
  // decoder detects and commits; the CSR file records mepc/mcause/mstatus and
  // hands back the two registers the decoder redirects the PC through.
  logic        csr_trap_entry, csr_mret_entry;
  logic [31:0] csr_trap_cause, csr_trap_epc;
  logic [31:0] csr_mtvec, csr_mepc;
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
    .fetch_stall(fetch_stall),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(csr_mtvec),
    .mepc(csr_mepc),
   `ifdef RISCV_FORMAL
    .csr_rvfi_mcycle(csr_rvfi_mcycle),
    .csr_rvfi_minstret(csr_rvfi_minstret),
    .csr_rvfi_mscratch(csr_rvfi_mscratch),
   `endif
    // outputs
    .pc(pc),
    .next_pc(next_pc),
    .rs1(rs1),
    .rs2(rs2),
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
    // inputs
    .addr(csr_addr),
    .ren(csr_ren),
    .wen(csr_wen),
    .wdata(csr_wdata),
    .instret(csr_instret),
    .trap_entry(decoder_trap_entry),
    .trap_cause(csr_trap_cause),
    .trap_epc(csr_trap_epc),
    .mret_entry(csr_mret_entry),
    // outputs
    .rdata(csr_rdata),
    .implemented(csr_implemented),
    .mtvec_value(csr_mtvec),
    .mepc_value(csr_mepc)
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
    .mem_ren(mem_ren),
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
    .rvfi_trap(rvfi_trap),
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
  // The CSR fields are per-retire data (rtl/csrs.v, threaded through the shadow
  // to rtl/writeback.v), and so is `rvfi_trap` now -- rtl/decoder.v computes it
  // where every trap is committed (CLAUDE.md invariant 2) and rtl/writeback.v
  // drives it at retire. These four are not per-retire data and stay constants:
  // M-mode only / XLEN=32 / no interrupts (mie and mip are read-only zero) and
  // no halt makes mode/ixl/intr/halt properties of the design rather than of an
  // instruction.
  assign rvfi_halt = 1'b0;
  assign rvfi_intr = 1'b0;
  assign rvfi_mode = 2'd3;
  assign rvfi_ixl = 2'd1;
 `endif
endmodule
