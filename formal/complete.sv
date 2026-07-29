module rvfi_testbench (
  input var clk,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
);
  logic reset = 1;
  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES
  logic trap;

  // ADR-0006: "finish [complete.sv] by driving imem/dmem inputs rand-stable".
  // Without this, imem_data/mem_rdata are free every cycle (see wrapper.v),
  // which is fine for the single-retire insn_* checks but not for this
  // whole-ISA walk: a load that doesn't see back what an earlier store in
  // the same trace wrote would fail rvfi_isa_rv32imc's spec check for
  // reasons that have nothing to do with the core.
  //
  // imem is left fully free per cycle, deliberately not made rand-stable:
  // rtl/fetcher.v drives imem_addr = pc and out.instr = imem_data
  // combinationally with no cross-cycle latency (CLAUDE.md invariant 1), so
  // nothing here depends on two different cycles' fetches of the same
  // address agreeing.
  //
  // dmem gets a single-address write-through shadow -- the most recent
  // store only, not a full memory array -- which is enough to let the
  // common store-then-immediately-reload pattern through. A read of an
  // address some *earlier*, since-overwritten store touched stays free;
  // see dmemcheck.sv for the fuller version of this same argument (why
  // $past(mem_addr), and why the address-0/idle-default coincidence is
  // harmless: rtl/accessor.v only reads mem_rdata the cycle after a real
  // load request).
  logic [31:0] dmem_shadow;
  logic [31:0] dmem_shadow_addr;
  logic        dmem_shadow_valid = 0;
  always_ff @(posedge clk) begin
    if (!reset && mem_wstrb) begin
      dmem_shadow_addr <= mem_addr;
      if (mem_wstrb[0]) dmem_shadow[ 7: 0] <= mem_wdata[ 7: 0];
      if (mem_wstrb[1]) dmem_shadow[15: 8] <= mem_wdata[15: 8];
      if (mem_wstrb[2]) dmem_shadow[23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) dmem_shadow[31:24] <= mem_wdata[31:24];
      dmem_shadow_valid <= 1'b1;
    end
  end
  always_ff @(posedge clk) begin
    if (!reset && dmem_shadow_valid && !$past(mem_wstrb) &&
        $past(mem_addr) == dmem_shadow_addr)
      assume(mem_rdata == dmem_shadow);
  end

  // Instantiate the actual top-level CPU module (was stale "riscv" with Wishbone interface)
  littlecpu wrapper (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap),
    `RVFI_CONN
  );

  (* keep *) wire spec_valid;
  (* keep *) wire spec_trap;
  (* keep *) wire [4:0] spec_rs1_addr;
  (* keep *) wire [4:0] spec_rs2_addr;
  (* keep *) wire [4:0] spec_rd_addr;
  (* keep *) wire [`RISCV_FORMAL_XLEN   - 1:0] spec_rd_wdata;
  (* keep *) wire [`RISCV_FORMAL_XLEN   - 1:0] spec_pc_wdata;
  (* keep *) wire [`RISCV_FORMAL_XLEN   - 1:0] spec_mem_addr;
  (* keep *) wire [`RISCV_FORMAL_XLEN/8 - 1:0] spec_mem_rmask;
  (* keep *) wire [`RISCV_FORMAL_XLEN/8 - 1:0] spec_mem_wmask;
  (* keep *) wire [`RISCV_FORMAL_XLEN   - 1:0] spec_mem_wdata;

  rvfi_isa_rv32imc isa_spec (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_valid),
    .spec_trap(spec_trap),
    .spec_rs1_addr(spec_rs1_addr),
    .spec_rs2_addr(spec_rs2_addr),
    .spec_rd_addr(spec_rd_addr ),
    .spec_rd_wdata(spec_rd_wdata),
    .spec_pc_wdata(spec_pc_wdata),
    .spec_mem_addr(spec_mem_addr),
    .spec_mem_rmask(spec_mem_rmask),
    .spec_mem_wmask(spec_mem_wmask),
    .spec_mem_wdata(spec_mem_wdata)
  );

  // do the instruction check — all instruction classes including SYSTEM (ECALL/EBREAK/CSR)
  always_comb begin
    if (!reset && rvfi_valid && !rvfi_trap) begin
      assert(spec_valid && !spec_trap);
    end
  end
endmodule
