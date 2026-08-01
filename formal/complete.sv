module rvfi_testbench (
  input var clk,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  // ADR-0003: the dual-word fetch window's second word. Left fully free
  // per cycle, same rationale as imem_data below (no cross-cycle latency to
  // model, CLAUDE.md invariant 1).
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
  //
  // ADR-0049 -- fact / discharge / scope for the `assume` twenty lines below,
  // which ADR-0049's census covered but could not annotate (this file was
  // carved out of that change and handed here):
  //
  //   FACT MODELLED. The data bus returns, one cycle after the request,
  //     whatever the most recent store to that same address wrote. That is
  //     store-to-load forwarding for one address, not a memory: a read of an
  //     address written by some earlier, since-overwritten store is left free.
  //
  //   DISCHARGED: NOWHERE, and there is no structural argument behind it
  //     either -- this is the weaker of the two answers ADR-0049 clause 2
  //     admits, and it is the honest one. The only writable memory in the tree
  //     is rtl/memory.v, which answers any address at or past 4*RAM with
  //     `mem_rdata <= mem_wdata` rather than with stored data, and `mem_addr`
  //     here is a free 32-bit value, so the modelled memory and the real
  //     module disagree over the overwhelming majority of the address space.
  //     test/mem_tb.v does not close this: it asserts only that an
  //     out-of-range read does not alias ram[0], never what one returns.
  //     ADR-0044 rules the placeholder out as a starting point and does not
  //     replace it, so when the real memory system is built there is no check
  //     anywhere that will hold it to what this proof assumed of it. That is
  //     ADR-0049's finding F4, restated at the site it is about; do not invent
  //     a discharge for it.
  //
  //   SCOPE. `complete`'s single assert(spec_valid && !spec_trap) -- this task
  //     has exactly one assertion, and the assume is guarded besides
  //     (dmem_shadow_valid, no store last cycle, same address). It is
  //     therefore no wider than what it was written for. Unlike
  //     rtl/executor.v's operand cap (ADR-0049 F1) it constrains a DUT INPUT
  //     rather than the core, so it narrows the environment and can only make
  //     the trace set smaller, never make the core's job easier on a trace
  //     that survives.
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
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
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

  // ----------------------------------------------------------------------
  // THE DECLARED EXCLUSION SET
  // ----------------------------------------------------------------------
  //
  // What this check says: every retire this core reports is an instruction
  // riscv-formal's whole-ISA spec model RECOGNISES (spec_valid) and agrees is
  // non-trapping (!spec_trap). It is the only thing in the tree that walks the
  // ISA as a whole rather than one instruction at a time -- the 70 insn_*
  // ladder checks each constrain rvfi_insn to their own encoding, so an
  // encoding no insn_* names is invisible to all of them.
  //
  // It cannot be that for the WHOLE ISA, because riscv-formal ships no spec
  // model at all for two opcode classes at formal/pin.mk's SHA. Not a weaker
  // model, none: there is no insns/insn_fence.v, no insn_ecall.v, no
  // insn_csrrw.v, and isa_rv32imc.txt names none of them. spec_valid is
  // therefore 0 on every such retire and the assertion fails on correct
  // hardware -- measured, `DONE (FAIL, rc=2)` at step 5 with imem_data
  // 0x0000800F, which is FENCE.
  //
  // EXCLUDING AN OPCODE FROM AN ASSERTION IS WEAKENING A CHECK. It is
  // legitimate here only under ADR-0010's rule one level up -- restrict the
  // proof, and RECORD the restriction -- which is why each entry below names
  // its encoding and its reason, why formal/COMPLETE_EXCLUSIONS carries the
  // same set as a baseline, and why formal/check-complete-exclusions.py
  // compares the two as sets in BOTH directions (ADR-0014's contract) before
  // this check is allowed to run. Adding an entry without moving the baseline
  // fails; removing one without moving the baseline fails; and the script also
  // re-derives "no spec model at the pin" from the clone, so an entry that
  // stops being true at a future pin fails rather than quietly over-excluding.
  //
  // THE PREDICATE KEYS ON THE ENCODING, NOT ON ANY CORE-SUPPLIED FLAG. It
  // reads rvfi_insn and nothing else -- no is_fence, no is_csr, nothing out of
  // rtl/decoder.v. A core that wrongly decodes an ADD as a FENCE must not be
  // able to excuse itself from its own check; with the test written this way,
  // such a core still retires an OP encoding and is still asserted on.
  // check-complete-exclusions.py enforces that shape textually, because it is
  // the kind of property a later one-line edit ("just use the shadow flag,
  // it's already there") destroys without looking like it did.
  //
  // rvfi_insn[1:0] == 2'b11 is the whole width discriminator: RVFI reports a
  // compressed retire as a zero-extended 16-bit value, and no RVC encoding has
  // both low bits set. cover.sv already counts long vs compressed retires with
  // the same test.
  wire        insn_uncompressed = rvfi_insn[1:0] == 2'b11;
  wire [6:0]  insn_opcode       = rvfi_insn[6:0];

  // EXCLUDE MISC-MEM 0001111 fence fence.i
  //   riscv-formal has no spec model for either at the pin. rtl/decoder.v
  //   makes both the NOPs ADR-0005 specifies -- one hart and no icache, so a
  //   conformant fence.i is a NOP (ADR-0002 as amended by ADR-0048) -- and
  //   they retire non-trapping, so unlike ecall/ebreak they are not already
  //   excused by the !rvfi_trap guard below. This is the class the measured
  //   failure above came from.
  wire exclude_misc_mem = insn_uncompressed && insn_opcode == 7'b0001111;

  // EXCLUDE SYSTEM 1110011 ecall ebreak mret wfi csrrw csrrs csrrc csrrwi csrrsi csrrci
  //   riscv-formal has no spec model for any of the ten at the pin -- the
  //   whole of M3's behaviour. CLAUDE.md's standing caveat says so in prose;
  //   this is that caveat mechanised. `mret`, `wfi` and the six csrr* forms
  //   retire non-trapping and would otherwise fail here. `ecall`/`ebreak` are
  //   already excused by the !rvfi_trap guard (they raise causes 11 and 3),
  //   and stay named anyway: the entry is a statement about which encodings
  //   have no oracle, not about which ones happen to reach the assertion
  //   today. What checks these instead: test/asm/trap.S, test/asm/csr.S,
  //   test/csr_tb.v, test/decoder_tb.v and rtl/decoder.v's component proof --
  //   this repo's own assertions, not an oracle (ADR-0033).
  wire exclude_system = insn_uncompressed && insn_opcode == 7'b1110011;

  wire insn_excluded = exclude_misc_mem || exclude_system;

  // NO COMPRESSED ENTRY, AND THAT IS A RESULT RATHER THAN AN OVERSIGHT. The
  // one compressed encoding this core implements that isa_rv32imc.txt does not
  // name is C.EBREAK (16'h9002, ADR-0048), and it raises BREAKPOINT, so it
  // reaches this assertion only if the core reports it as a NON-trapping
  // retire -- in which case spec_valid is 0 for 0x00009002 and this check
  // fires. Excluding it would have thrown that away. Every other compressed
  // encoding the decoder accepts has a model.

  // ----------------------------------------------------------------------
  // THE CHECK
  // ----------------------------------------------------------------------
  //
  // The !rvfi_trap term is core-supplied, unlike the exclusion predicate, and
  // it has to be: a trapping retire has no spec-value obligation (ADR-0028),
  // and under RISCV_FORMAL_ALIGNED_MEM the spec model reports spec_trap on a
  // misaligned lw that this core correctly refuses to perform. An illegal
  // encoding is the sharper case -- it retires with rvfi_trap and has no spec
  // model by definition, so `assert(spec_valid)` cannot be hoisted out of the
  // guard. The consequence is honest and worth stating: a core that claimed
  // rvfi_trap on everything would pass this vacuously. That is what
  // complete_cover.sby is for -- it witnesses a real non-trapping retire of
  // every opcode class this exclusion set does NOT cover, so "the assertion
  // was reached, class by class" is measured rather than assumed.
  always_comb begin
    if (!reset && rvfi_valid && !rvfi_trap && !insn_excluded) begin
      assert(spec_valid && !spec_trap);
    end
  end

  // ----------------------------------------------------------------------
  // ANTI-VACUITY: every non-excluded opcode class really does retire
  // ----------------------------------------------------------------------
  //
  // sby's `mode bmc` strips cover cells and `mode cover` strips assertions, so
  // these sit inert in complete.sby's run and are the whole point of
  // complete_cover.sby's. Each goal is a NON-TRAPPING, NON-EXCLUDED retire of
  // one opcode class -- i.e. a cycle on which the assertion above was live and
  // had something to say. Together they are the demonstration that the
  // exclusion set did not swallow the ISA.
  //
  // `complete_live` is character-for-character the assertion's guard, and it
  // is a wire rather than a `define` for two reasons: a macro defined in one
  // file leaks into every file read after it in the same yosys invocation, and
  // -- measured -- a macro-expanded cover statement is reported by sby against
  // the source range of the MACRO, so all twelve goals resolve to overlapping
  // spans and cannot be told apart in the PASS summary. One wire, twelve
  // one-line goals, twelve distinct locations.
  //
  // `insn_excluded` is redundant on the nine uncompressed goals (an opcode has
  // one value, and it is not one of the two excluded ones) and is kept for
  // exactly that reason: if a future entry did shadow one of these classes,
  // the goal would go unreachable and this task would fail, rather than the
  // assertion going quiet with nothing to say about it.
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
endmodule
