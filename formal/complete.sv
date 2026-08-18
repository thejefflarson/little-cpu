module rvfi_testbench (
  input var clk,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  // The fetch window's second word, left free every cycle for the same reason
  // imem_data is.
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
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

  // A one-address write-through shadow of the data bus: the most recent store
  // only, not a memory array. Without it mem_rdata is free every cycle, and a
  // load that did not see back what an earlier store in the same trace wrote
  // would fail the spec check for reasons that have nothing to do with the core.
  // A read of an address some earlier, since-overwritten store touched is still
  // left free. dmemcheck.sv carries the fuller version of this argument, and
  // imem needs none of it because rtl/fetcher.v answers combinationally.
  //
  // Nothing discharges this assumption and there is no structural argument
  // behind it either, so do not invent a discharge. What limits the damage is
  // that it constrains a DUT input rather than the core, so it can only shrink
  // the trace set, never make the core's job easier on a trace that survives.
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

  // Unread here, since this environment answers imem_data in the same cycle,
  // but connected rather than left dangling.
  logic [31:0] imem_addr_next;
  // The address the core publishes for the platform to decode. Unread here:
  // `atomic_supported` is tied high, so no atomic can fault in this task.
  logic [31:0] atomic_addr;
  // The lock an arbiter would read. Unread here: one hart, one bus master.
  logic mem_lock;
  logic        mem_ren;
  logic        fetch_stall;

  imem_arbiter arbiter (
    .clock(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .fetch_stall(fetch_stall),
    .text_write()
  );

  littlecpu wrapper (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    // Tied off: this task's memory model answers every address, so there is no
    // window for a fetch to fall outside of.
    .imem_fault(1'b0),
    // Tied off high: this task's memory model answers every address, so every
    // address it answers is one a reservation may be held at, and one an atomic
    // is answered at.
    .mem_reservable(1'b1),
    .atomic_addr(atomic_addr),
    .atomic_supported(1'b1),
    // Tied off; formal/check-multihart-tie-off.py enforces it. formal/wrapper.v
    // carries the reason the riscv-formal side of the tree describes one hart.
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    // Tied off; formal/check-interrupt-tie-off.py enforces it. formal/wrapper.v
    // carries the reason the riscv-formal side of the tree runs with no
    // interrupt in the trace.
    .irq_timer(1'b0),
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

  // riscv-formal ships no spec model at all for two opcode classes at the pin,
  // so spec_valid is 0 on every such retire and the assertion below fails on
  // correct hardware. Measured with `insn_excluded` forced to 0: FAIL at step 5,
  // on a non-trapping FENCE retire the model has never heard of.
  //
  // Excluding an opcode weakens a check, which is admissible only if recorded.
  // formal/COMPLETE_EXCLUSIONS carries the same set as a baseline and
  // check-complete-exclusions.py compares the two both ways before this check
  // may run, re-deriving "no spec model at the pin" from the clone so an entry
  // that stops being true fails rather than quietly over-excluding.
  //
  // Each predicate keys on the encoding out of rvfi_insn and on nothing the core
  // decodes, so a core that wrongly decodes an ADD as a FENCE cannot excuse
  // itself from its own check. check-complete-exclusions.py enforces that shape
  // textually, because a one-line edit to use a decoder flag instead would not
  // look like it had broken anything.
  //
  // rvfi_insn[1:0] == 2'b11 is the width discriminator: RVFI reports a
  // compressed retire as a zero-extended 16-bit value, and no RVC encoding has
  // both low bits set.
  wire        insn_uncompressed = rvfi_insn[1:0] == 2'b11;
  wire [6:0]  insn_opcode       = rvfi_insn[6:0];

  // EXCLUDE MISC-MEM 0001111 fence fence.i
  //   No spec model at the pin. rtl/decoder.v makes both NOPs and they retire
  //   non-trapping, so unlike ecall/ebreak they are not already excused by the
  //   !rvfi_trap guard below. This is the class the failure above came from.
  wire exclude_misc_mem = insn_uncompressed && insn_opcode == 7'b0001111;

  // EXCLUDE SYSTEM 1110011 ecall ebreak mret wfi csrrw csrrs csrrc csrrwi csrrsi csrrci
  //   No spec model at the pin for any of the ten. `ecall`/`ebreak` are already
  //   excused by the !rvfi_trap guard and are named anyway, because this entry
  //   states which encodings have no oracle rather than which reach the
  //   assertion today. What checks them instead is this repo's own assertions:
  //   test/asm/trap.S, test/asm/csr.S, test/csr_tb.v, test/decoder_tb.v and the
  //   decoder and traps proofs.
  wire exclude_system = insn_uncompressed && insn_opcode == 7'b1110011;

  // EXCLUDE AMO 0101111 amoadd.w amoswap.w amoxor.w amoand.w amoor.w amomin.w amomax.w amominu.w amomaxu.w lr.w sc.w
  //   No spec model at the pin. insns/generate.py carries an `insn_amo`
  //   generator with every call site commented out, and it covers neither the
  //   min/max family nor lr/sc at all, so there is no isa_rv32ia*.txt to drive
  //   either. All eleven retire non-trapping, so like MISC-MEM they are not
  //   excused by the !rvfi_trap guard below. What checks them instead is this
  //   repo's own assertions: rtl/accessor.v's FORMAL block holds the nine
  //   read-modify-write functions against the operators they replaced and the
  //   reservation against the region bit, test/accessor_tb.v vectors the
  //   datapath and the transaction count, test/decoder_tb.v the eleven
  //   encodings and the three misalignment causes, and `dmemcheck` the
  //   two-cycle memory report against the pinned rvfi_dmem_check.
  wire exclude_amo = insn_uncompressed && insn_opcode == 7'b0101111;

  wire insn_excluded = exclude_misc_mem || exclude_system || exclude_amo;

  // There is no compressed entry, and that is a result. The one compressed
  // encoding this core implements that isa_rv32imc.txt does not name is C.EBREAK
  // (16'h9002), and it raises breakpoint -- so it reaches the assertion only if
  // the core reports it as a non-trapping retire, at which point spec_valid is 0
  // and this check fires. Excluding it would throw that away.

  // The !rvfi_trap term is core-supplied, unlike the exclusion predicate, and
  // has to be: a trapping retire has no spec-value obligation, and an illegal
  // encoding retires with rvfi_trap and has no spec model by definition, so
  // `assert(spec_valid)` cannot be hoisted out of the guard. The cost is that a
  // core claiming rvfi_trap on everything would pass this vacuously, which is
  // what the cover goals below rule out.
  always_comb begin
    if (!reset && rvfi_valid && !rvfi_trap && !insn_excluded) begin
      assert(spec_valid && !spec_trap);
    end
  end

  // Anti-vacuity: every non-excluded opcode class really does retire. sby's
  // `mode bmc` strips cover cells and `mode cover` strips assertions, so these
  // are inert in complete.sby's run and are the whole of complete_cover.sby's.
  //
  // `complete_live` repeats the assertion's guard as a wire rather than a macro.
  // A macro leaks into every file read after it in the same yosys invocation,
  // and sby reports a macro-expanded cover statement against the macro's source
  // range, so all twelve goals would resolve to overlapping spans and could not
  // be told apart in the summary.
  //
  // `insn_excluded` is redundant on the nine uncompressed goals and kept for
  // that reason: a future exclusion that shadowed one of these classes makes the
  // goal unreachable and fails this task, instead of leaving the assertion quiet
  // with nothing to say.
  wire complete_live = !reset && rvfi_valid && !rvfi_trap && !insn_excluded;
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0000011); // LOAD
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0010011); // OP-IMM
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0010111); // AUIPC
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0100011); // STORE
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0110011); // OP
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0110111); // LUI
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b1100011); // BRANCH
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b1100111); // JALR
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b1101111); // JAL
  cover property (complete_live && rvfi_insn[1:0] == 2'b00);                        // RVC quadrant 0
  cover property (complete_live && rvfi_insn[1:0] == 2'b01);                        // RVC quadrant 1
  cover property (complete_live && rvfi_insn[1:0] == 2'b10);                        // RVC quadrant 2

  // The thirteenth goal is the exclusion's own anti-vacuity control, and it is
  // the one that must NOT read `complete_live`: an excluded class is excluded
  // from the assertion, so a goal carrying `!insn_excluded` would be
  // unreachable by construction and would say nothing about whether the core
  // ever executes one. What this asks instead is that an AMO really does
  // retire without trapping, so the exclusion above is a live restriction on
  // this check rather than a line covering an instruction class nobody reaches.
  cover property (!reset && rvfi_valid && !rvfi_trap && exclude_amo);
endmodule
