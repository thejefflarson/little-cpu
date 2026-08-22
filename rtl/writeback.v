`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module writeback(
  input  logic clk,
  input  logic reset,
  // inputs
  input accessor_output in,
  // outputs
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
  // The CSRs formal/checks.cfg's `[csrs]` list names. mcycle and minstret are
  // 64-bit RVFI CSRs -- mcycleh/minstreth address the upper half of the same
  // report rather than a second CSR.
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
  // Continuous assigns rather than an always_comb, for the reason this file
  // documents further down: a struct-field read inside an `always_*` is a
  // constant part-select iverilog cannot build a precise sensitivity entry for.
  logic        in_valid;
  logic [4:0]  in_rd;
  logic [31:0] in_rd_data;
  assign in_valid   = in.valid;
  assign in_rd      = in.rd;
  assign in_rd_data = in.rd_data;

  // Retire is `valid` reaching writeback: a bubble must never commit a
  // register write.
  assign wen   = !reset && in_valid && (in_rd != 5'b0);
  // The masks are dead logic and they stay. Every consumer in rtl/regfile.v --
  // both write-first terms, the array write and the write-through bypass --
  // already tests `wen`, so deleting them changes nothing and reads like a free
  // level off the loop into the branch comparator. It was built and swept at
  // sixteen seeds: the median period moved +2.8% and the worst placement went
  // under the board clock at 11.89 MHz. Deleting dead logic here is a measured
  // cost, not a saving.
  assign waddr = wen ? in_rd      : 5'b0;
  assign wdata = wen ? in_rd_data : 32'b0;

 `ifdef RISCV_FORMAL
  // What this core reports on a trapping retire, and what no oracle checks.
  // riscv-formal's checks/rvfi_insn_check.sv puts rd_addr, rd_wdata, pc_wdata
  // and every mem_* assertion inside `if (!spec_trap)`, so those checks cannot
  // tell a core that traps correctly from one that traps and also corrupts rd,
  // writes memory and redirects somewhere arbitrary. The convention is
  // rvfi_valid 1 (a trapping instruction does retire), rvfi_trap 1, rd_addr /
  // rd_wdata / mem_rmask / mem_wmask 0, and pc_wdata = mtvec (ADR-0028).
  //
  // None of it is enforced here. rtl/decoder.v suppresses `out.rd` and every
  // execution flag on a trapping issue, so the values below are zero because
  // nothing produced anything, and that redirect target is asserted in the
  // decoder's own component proof. `reg` and `pc_fwd`/`pc_bwd` catch a broken
  // one indirectly, but neither names traps in its title.
  //
  // A continuous assign rather than a line in the always_comb below: every
  // struct-field read inside an `always_comb` is a constant part-select
  // iverilog cannot build a precise sensitivity entry for, so it emits
  // `sorry: constant selects in always_* processes are not fully supported`.
  // The fallback is safe, but it is still a diagnostic. Keeping this read out
  // of the always_comb is what stops one more being emitted.
  assign rvfi_trap = in.rvfi.trap;
  // Same continuous-assign reason as rvfi_trap above.
  assign rvfi_intr = in.rvfi.intr;
 `ifdef RISCV_FORMAL_MEM_FAULT
  assign rvfi_mem_fault = in.rvfi.mem_fault;
  // Both clear is how checks/rvfi_fault_check.sv says a FETCH was refused. A
  // data access sets one or both, and decode sets them to the access it would
  // have made -- exactly, for the write mask, because
  // checks/rvfi_insn_check.sv compares that one against the spec model rather
  // than accepting a superset of it.
  assign rvfi_mem_fault_rmask = in.rvfi.mem_fault_rmask;
  assign rvfi_mem_fault_wmask = in.rvfi.mem_fault_wmask;
  // A refused access never reached rtl/accessor.v, so it has no transaction to
  // report -- and the check that reads the masks above reads this against the
  // spec model's effective address. Decode is where that address is known.
  // Continuous, for the same sensitivity reason as the three above.
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
    // rd_addr/rd_wdata are not carried in the rvfi shadow; `rd`/`rd_data` are
    // already the values needed. rd_wdata is masked to 0 alongside x0, matching
    // the regfile's own write suppression: a write "to" x0 must not report a
    // nonzero rd_wdata even though rd_data holds whatever the ALU produced.
    rvfi_rd_addr = in.rd;
    rvfi_rd_wdata = |in.rd ? in.rd_data : 32'b0;
    rvfi_pc_rdata = in.rvfi.pc_rdata;
    rvfi_pc_wdata = in.rvfi.pc_wdata;
    rvfi_mem_rmask = in.rvfi_mem_rmask;
    rvfi_mem_wmask = in.rvfi_mem_wmask;
    rvfi_mem_rdata = in.rvfi_mem_rdata;
    rvfi_mem_wdata = in.rvfi_mem_wdata;
  end

  // Captured in decode (ADR-0005 commits every CSR access there) and reported
  // here at retire. Continuous assigns for the same sensitivity reason as
  // rvfi_trap above: these twelve reads inside the always_comb are twelve more
  // `sorry:` notes.
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

  // The only sequential piece of RVFI state here: everything else is a
  // combinational read of `in`, which is already registered upstream.
  always_ff @(posedge clk) begin
    if (reset) rvfi_order <= 64'b0;
    else if (rvfi_valid) rvfi_order <= rvfi_order + 64'b1;
  end
 `endif
endmodule
