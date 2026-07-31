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
  // ADR-0006 / CLAUDE.md invariant 3: "retire is `valid` reaching
  // writeback, which gates wen and drives rvfi_valid" — this is that spot.
  // `rvfi_trap` stays hardwired 0 in littlecpu.v: nothing upstream computes
  // it yet (trap entry is the next M3 step, ADR-0011). The CSR fields no
  // longer are — rtl/csrs.v builds them and they ride the shadow down here
  // like every other per-retire value.
 `ifdef RISCV_FORMAL
  ,
  output logic        rvfi_valid,
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
  // The CSRs formal/checks.cfg's `[csrs]` list names. mcycle/minstret are
  // 64-bit RVFI CSRs (mcycleh/minstreth address the upper half of the same
  // report, not a second CSR); mscratch is XLEN.
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
 `endif
);
  always_comb begin
    if(reset) begin
      wen = 0;
      waddr = 0;
      wdata = 32'b0;
    end else begin
      // Retire is `valid` reaching writeback (CLAUDE.md invariant 3): a
      // bubble must never commit a register write, so wen is gated on both
      // valid and a non-zero destination.
      wen = in.valid && (in.rd != 0);
      waddr = in.rd;
      wdata = in.rd_data;
    end
  end // always_comb

 `ifdef RISCV_FORMAL
  // Combinational, same as wen/waddr/wdata above: `in` is already a
  // registered accessor_output, stable for exactly the one cycle it
  // retires, so there is nothing to re-register here except the running
  // order counter below.
  always_comb begin
    rvfi_valid = !reset && in.valid;
    rvfi_insn = in.rvfi.insn;
    rvfi_rs1_addr = in.rvfi.rs1_addr;
    rvfi_rs2_addr = in.rvfi.rs2_addr;
    rvfi_rs1_rdata = in.rvfi.rs1_rdata;
    rvfi_rs2_rdata = in.rvfi.rs2_rdata;
    // rd_addr/rd_wdata are not carried in the rvfi shadow -- `rd`/`rd_data`
    // are already exactly the values needed (ADR-0006: no need to duplicate
    // them). rd_wdata is masked to 0 alongside x0, matching the regfile's
    // own write-through (CLAUDE.md invariant 6): a write "to" x0 must never
    // report a nonzero rd_wdata even though rd_data itself is whatever the
    // ALU produced.
    rvfi_rd_addr = in.rd;
    rvfi_rd_wdata = |in.rd ? in.rd_data : 32'b0;
    rvfi_pc_rdata = in.rvfi.pc_rdata;
    rvfi_pc_wdata = in.rvfi.pc_wdata;
    rvfi_mem_addr = in.rvfi_mem_addr;
    rvfi_mem_rmask = in.rvfi_mem_rmask;
    rvfi_mem_wmask = in.rvfi_mem_wmask;
    rvfi_mem_rdata = in.rvfi_mem_rdata;
    rvfi_mem_wdata = in.rvfi_mem_wdata;
  end

  // Captured in decode (ADR-0005 commits every CSR access there), carried
  // down the shadow, reported here at retire like everything else -- but as
  // continuous assigns rather than inside the always_comb above, and that is
  // not a style choice. Every struct-field read in that block is a constant
  // part-select, which iverilog cannot build a precise sensitivity entry for
  // ("sorry: constant selects in always_* processes are not fully
  // supported"); it falls back to whole-struct sensitivity, which is safe but
  // is a diagnostic, and CLAUDE.md says warnings are errors and caps the
  // count. Adding twelve more reads in there added twelve more notes.
  // A continuous assign has an exact sensitivity, so this says the same thing
  // without them -- the same fix rtl/accessor.v and rtl/decoder.v already
  // apply to their own payload reads.
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

  // rvfi_order counts retires (RVFI spec: monotonically increasing, one per
  // rvfi_valid cycle). The only genuinely sequential piece of RVFI state.
  always_ff @(posedge clk) begin
    if (reset) rvfi_order <= 64'b0;
    else if (rvfi_valid) rvfi_order <= rvfi_order + 64'b1;
  end
 `endif
endmodule
