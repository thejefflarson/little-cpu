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
  // The CSR/trap fields (rvfi_trap, rvfi_csr_*) stay hardwired 0 in
  // littlecpu.v rather than threaded through here: nothing upstream of this
  // stage computes them yet (M3), so there is nothing per-instruction here
  // to drive them with.
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
  output logic [31:0] rvfi_mem_wdata
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
  // TODO: csrs

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

  // rvfi_order counts retires (RVFI spec: monotonically increasing, one per
  // rvfi_valid cycle). The only genuinely sequential piece of RVFI state.
  always_ff @(posedge clk) begin
    if (reset) rvfi_order <= 64'b0;
    else if (rvfi_valid) rvfi_order <= rvfi_order + 64'b1;
  end
 `endif

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
 `endif
endmodule
