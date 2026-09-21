// nanocpu's analogue of formal/traps.sv: the only oracle that a trap lands on mtvec and
// saves the right mepc, mcause and mtval, since riscv-formal ships no spec model for
// SYSTEM at the pin. nano.v is monolithic -- no separate decoder to compose -- so this
// reads the real core's own RVFI report plus the debug-only CSR outputs added for
// exactly this proof, and re-derives what a trap should report from rvfi_insn and
// rvfi_rs1_rdata rather than from nano.v's own internal signals.

module rvfi_testbench (
  input var clk,
  output logic        mem_valid,
  output logic        mem_instr,
  input  logic        mem_ready,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
  // A plain, unconnected input is free in BMC: unlike the genchecks legs, this proof
  // states an interrupt-entry property too, so the line riscv-formal has no model for
  // stays live rather than tied off.
  input  logic        irq_meip
);
  logic reset = 1;
  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES
  logic trap;
  logic [31:0] dbg_mtvec, dbg_mepc, dbg_mcause, dbg_mtval, dbg_mstatus;

  riscv wrapper (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .irq_meip(irq_meip),
    .trap(trap),
    .rvfi_dbg_mtvec(dbg_mtvec),
    .rvfi_dbg_mepc(dbg_mepc),
    .rvfi_dbg_mcause(dbg_mcause),
    .rvfi_dbg_mtval(dbg_mtval),
    .rvfi_dbg_mstatus(dbg_mstatus),
    `RVFI_CONN
  );

  wire live = !reset && rvfi_valid;

  // Held from the previous retirement, not read live: an interrupt-redirected retire
  // can itself be a CSRRW to mtvec, whose new value must not retroactively apply to
  // the redirect that already happened before this instruction ever executed.
  logic [31:0] shadow_mtvec;
  always_ff @(posedge clk) begin
    if (reset) shadow_mtvec <= 32'b0;
    else if (live) shadow_mtvec <= dbg_mtvec;
  end

  // Re-derived from rvfi_insn/rvfi_rs1_rdata, never from nano.v's own signals: an
  // oracle built out of the thing under test proves nothing about it.
  wire [4:0] insn_opcode = rvfi_insn[6:2];
  wire       insn_uncompressed = rvfi_insn[1:0] == 2'b11;
  wire [2:0] insn_funct3 = rvfi_insn[14:12];
  // Matches nano.v's own is_load/is_store: the opcode alone is not enough, since a
  // reserved funct3 under either opcode is an illegal instruction, not an access nano
  // ever computes an address for. Naming x16-x31 in rd (a load) or rs1/rs2 (either) is
  // the same: RV32E's own illegal-instruction reason outranks a load/store's, so this
  // oracle excludes it rather than duplicate nano.v's rs1_valid/rs2_valid table.
  wire [4:0] insn_rd  = rvfi_insn[11:7];
  wire [4:0] insn_rs1 = rvfi_insn[19:15];
  wire [4:0] insn_rs2 = rvfi_insn[24:20];
  wire       e_illegal_free = !insn_rd[4] && !insn_rs1[4] && !insn_rs2[4];
  wire       is_load_op  = insn_uncompressed && insn_opcode == 5'b00000 && e_illegal_free &&
    (insn_funct3 == 3'b000 || insn_funct3 == 3'b001 || insn_funct3 == 3'b010 ||
     insn_funct3 == 3'b100 || insn_funct3 == 3'b101);
  wire       is_store_op = insn_uncompressed && insn_opcode == 5'b01000 && e_illegal_free &&
    (insn_funct3 == 3'b000 || insn_funct3 == 3'b001 || insn_funct3 == 3'b010);
  wire [31:0] i_immediate = {{20{rvfi_insn[31]}}, rvfi_insn[31:20]};
  wire [31:0] s_immediate = {{20{rvfi_insn[31]}}, rvfi_insn[31:25], rvfi_insn[11:7]};
  wire [31:0] load_addr  = $signed(i_immediate) + $signed(rvfi_rs1_rdata);
  wire [31:0] store_addr = $signed(s_immediate) + $signed(rvfi_rs1_rdata);

  wire lw_misaligned = is_load_op  && insn_funct3 == 3'b010 && load_addr[1:0]  != 2'b00;
  wire lh_misaligned = is_load_op  && (insn_funct3 == 3'b001 || insn_funct3 == 3'b101) &&
                        load_addr[0];
  wire sw_misaligned = is_store_op && insn_funct3 == 3'b010 && store_addr[1:0] != 2'b00;
  wire sh_misaligned = is_store_op && insn_funct3 == 3'b001 && store_addr[0];

  // Matches nano.v's own RAM_BASE/RAM_WORDS defaults; a build at other parameters is
  // not what this proof is over.
  localparam logic [31:0] RAM_BASE = 32'h0001_0000;
  localparam int          RAM_WORDS = 4096;
  wire [31:0] data_addr = is_store_op ? store_addr : load_addr;
  wire        in_range = data_addr >= RAM_BASE && data_addr < RAM_BASE + RAM_WORDS * 4;

  wire load_region_fault  = is_load_op  && !lw_misaligned && !lh_misaligned && !in_range;
  wire store_region_fault = is_store_op && !sw_misaligned && !sh_misaligned && !in_range;

  wire is_ecall  = rvfi_insn == 32'h0000_0073;
  wire is_ebreak = rvfi_insn == 32'h0010_0073;
  wire is_mret   = rvfi_insn == 32'h3020_0073;

  localparam logic [31:0] CAUSE_ILLEGAL      = 32'd2;
  localparam logic [31:0] CAUSE_BREAKPOINT   = 32'd3;
  localparam logic [31:0] CAUSE_LOAD_MIS     = 32'd4;
  localparam logic [31:0] CAUSE_LOAD_FAULT   = 32'd5;
  localparam logic [31:0] CAUSE_STORE_MIS    = 32'd6;
  localparam logic [31:0] CAUSE_STORE_FAULT  = 32'd7;
  localparam logic [31:0] CAUSE_ECALL_M      = 32'd11;
  localparam logic [31:0] CAUSE_EXTERNAL_IRQ = 32'h8000_000B;

  // Only the causes with a computable independent oracle: illegal-instruction's own
  // expected cause is "whatever nano did not otherwise account for", which this file
  // cannot re-derive without duplicating is_valid, so it is covered by the tval arm
  // below instead (an illegal instruction's mtval must be the instruction word) and by
  // ill_e.sv/complete.sv elsewhere in this suite.
  wire expected_trap = is_ebreak || is_ecall ||
      lw_misaligned || lh_misaligned || sw_misaligned || sh_misaligned ||
      load_region_fault || store_region_fault;

  logic [31:0] expected_cause, expected_tval;
  always_comb begin
    if (is_ebreak) begin
      expected_cause = CAUSE_BREAKPOINT;
      expected_tval  = 32'b0;
    end else if (is_ecall) begin
      expected_cause = CAUSE_ECALL_M;
      expected_tval  = 32'b0;
    end else if (lw_misaligned || lh_misaligned) begin
      expected_cause = CAUSE_LOAD_MIS;
      expected_tval  = load_addr;
    end else if (sw_misaligned || sh_misaligned) begin
      expected_cause = CAUSE_STORE_MIS;
      expected_tval  = store_addr;
    end else if (load_region_fault) begin
      expected_cause = CAUSE_LOAD_FAULT;
      expected_tval  = data_addr;
    end else begin
      expected_cause = CAUSE_STORE_FAULT;
      expected_tval  = data_addr;
    end
  end

  // The arm a forced-red probe covers first: any of the modelled reasons must
  // actually trap, not merely be tolerated by the conditional checks below it.
  always_comb if (live && expected_trap) begin
    assert(rvfi_trap);
  end

  // A retire that traps always lands on mtvec, with mepc pointing at the faulting
  // instruction itself.
  always_comb if (live && rvfi_trap) begin
    assert(rvfi_pc_wdata == dbg_mtvec);
    assert(dbg_mepc == rvfi_pc_rdata);
  end

  // The two arms a forced-red probe covers: a plain load or store outside the RAM
  // window must fault, with the cause matching which direction it was.
  always_comb if (live && rvfi_trap && expected_trap) begin
    assert(dbg_mcause == expected_cause);
  end

  // mtval: the one thing a trap saves that no self-reporting oracle sees, so this is
  // the only place it is checked at all. Scoped to the uncompressed load/store and
  // ecall/ebreak arms this oracle can compute independently; the illegal-instruction
  // arm (mtval == the raw word) is not modelled here, since telling an illegal
  // encoding apart from a legal one needs is_valid's whole table -- ill_e.sv and the
  // .S suite's rewritten mul.S/divide.S cover that arm instead.
  always_comb if (live && rvfi_trap && expected_trap) begin
    assert(dbg_mtval == expected_tval);
  end

  // An interrupt redirects between instructions, never mid-instruction, so the first
  // retire out of the handler always reads mtvec as its pc -- whether or not that
  // same instruction goes on to trap for a reason entirely its own.
  always_comb if (live && rvfi_intr) begin
    assert(rvfi_pc_rdata == shadow_mtvec);
  end

  // mret returns to mepc and restores MIE from MPIE, leaving MPIE set.
  always_comb if (live && !rvfi_trap && is_mret) begin
    assert(rvfi_pc_wdata == dbg_mepc);
    assert(dbg_mstatus[7] == 1'b1);
  end

  cover property (live && rvfi_trap && load_region_fault);
  cover property (live && rvfi_trap && store_region_fault);
  cover property (live && rvfi_intr);
  cover property (live && is_mret && !rvfi_trap);
endmodule
