// Composed fetcher + decoder proof: the PC loop, with nothing assumed about it.
//
// rtl/decoder.v's own component task has to `assume(in.pc == pc)` -- it stands
// alone, so `in` is a free input and the solver would otherwise hand it a
// fetcher_pc unrelated to what the decoder itself last drove. That assumption
// is sound (it models rtl/fetcher.v's `out.pc = pc` plus littlecpu.v's pc->pc
// wiring, and ADR-0017 requires it to say so) but it makes the pc-increment
// assertions near-tautological: substitute it into `pc <= fetcher_pc + pc_inc`
// and you get `pc <= pc + pc_inc`, which is what is being asserted.
//
// So this task wires the two modules together for real, the way littlecpu.v
// does: decoder.pc drives fetcher.pc, and fetcher.out feeds decoder.in. The
// pc-increment assertions are restated against that closed loop, and the echo
// itself becomes an assertion rather than an assumption. Break rtl/fetcher.v's
// `out.pc = pc` and this task fails -- which the standalone decoder task
// cannot do, because it assumes exactly that away.
//
// One build-level rule makes this non-circular: components.sby reads the RTL
// WITHOUT -formal for this task (see the note there). Compiled with it, the
// decoder instance would carry its own standalone-task `assume(in.pc == pc)`
// into this proof -- assuming the very echo this task asserts, and reducing a
// broken fetcher to an unreachable state instead of a counterexample.
//
// Everything the decoder needs from stages this task does not instantiate
// (reg_rs1/rs2, executor_out, the accessor's stall and pending-load slots)
// stays a free input. reg_rs1 does reach pc arithmetic -- it is the jalr base
// and a branch comparand -- but only on cycles the increment assertion
// already excludes as jump/branch cycles, so leaving all of them
// unconstrained makes the proof stronger, not weaker.
`default_nettype none

module pcloop (
    input logic clk,
    input logic reset,
    input logic [31:0] imem_data,
    input logic [31:0] imem_data2,
    input logic [31:0] reg_rs1,
    input logic [31:0] reg_rs2,
    input executor_output executor_out,
    input logic divider_stall,
    input logic accessor_stall,
    input logic accessor_pending_valid,
    input logic [4:0] accessor_pending_rd,
    // ADR-0026's drain slot and the CSR file's read side. rtl/csrs.v is not
    // instantiated here -- this task is about the PC loop -- so both stay
    // free inputs, which is the stronger environment: the solver may claim
    // any CSR address is implemented and hand back any read value.
    input logic accessor_out_valid,
    input logic [31:0] csr_rdata,
    input logic csr_implemented,
    // ADR-0028's trap-entry pair, free for the same reason: rtl/csrs.v is not
    // instantiated here, so the solver may present any mtvec and any mepc.
    // That is the stronger environment for this task -- the PC-increment
    // assertions below exclude trap and mret cycles as jumps (see
    // f_jump_branch), so an arbitrary redirect target cannot make them pass;
    // it can only make them harder to satisfy on the cycles they do cover.
    input logic [31:0] mtvec,
    input logic [31:0] mepc
);
  logic [31:0] pc;
  logic [31:0] imem_addr, imem_addr2;
  fetcher_output fetcher_out;
  decoder_output decoder_out;
  logic [4:0] rs1, rs2;
  // Driven by the decoder, read by nothing here: rtl/csrs.v is the consumer
  // in the real pipeline (rtl/littlecpu.v).
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata;
  // Likewise driven by the decoder and read by nothing here: rtl/csrs.v is the
  // consumer in the real pipeline, and this task is about the PC loop.
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc;

  fetcher fetcher (
    .clk(clk),
    .reset(reset),
    .pc(pc),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .out(fetcher_out)
  );

  decoder decoder (
    .clk(clk),
    .reset(reset),
    .in(fetcher_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stall),
    .accessor_stall(accessor_stall),
    .accessor_pending_valid(accessor_pending_valid),
    .accessor_pending_rd(accessor_pending_rd),
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(mtvec),
    .mepc(mepc),
    .pc(pc),
    .rs1(rs1),
    .rs2(rs2),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .instret(instret),
    .trap_entry(trap_entry),
    .trap_cause(trap_cause),
    .trap_epc(trap_epc),
    .mret_entry(mret_entry),
    .out(decoder_out)
  );

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;

  // Reset is a once-at-the-start pulse everywhere in this design -- the same
  // assumption rtl/decoder.v and rtl/executor.v make in their own tasks, for
  // the same reason: without it the solver can reassert reset between the two
  // cycles these assertions compare and force pc back to 0 in between.
  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  // Every guard signal below is re-derived HERE, from pcloop's own ports and
  // wires -- never by hierarchical reference into the decoder instance.
  // yosys's Verilog frontend does not resolve `decoder.uncompressed`-style
  // references: it implicitly declares a fresh, undriven wire of that dotted
  // name in this module (the build log warned twice -- "Identifier
  // `\decoder.uncompressed' is implicitly declared", then "Wire
  // pcloop.\decoder.uncompressed is used but has no driver") and the solver
  // then drives the phantom wire freely. An earlier revision of this file
  // sampled those phantoms and got a "counterexample" in which pc correctly
  // stepped 0 -> 4 for an uncompressed word while its sampled history
  // claimed the instruction was compressed.
  //
  // Re-deriving is also the honest formulation: the width and jump guards
  // become an independent restatement of the fetched word's decode, so the
  // assertion compares the DUT against the instruction stream itself rather
  // than against the decoder's own opinion of it.

  // Width and control-flow facts of the word the fetcher presents this
  // cycle. Field positions mirror rtl/decoder.v (quadrant = instr[1:0],
  // opcode = instr[6:2], funct3 = instr[14:12], cfunct3 = instr[15:13]).
  // decoder.v's upper-half masking cannot make these disagree with the
  // decoder's view: every uncompressed match below requires quadrant ==
  // 2'b11, where the mask is the identity, and every compressed match reads
  // only instr[15:0], which the mask preserves.
  logic [31:0] f_instr;
  assign f_instr = fetcher_out.instr;
  logic f_uncompressed;
  assign f_uncompressed = f_instr[1:0] == 2'b11;
  // Exactly the instructions whose case arm in rtl/decoder.v's publish block
  // may write a non-sequential pc: jal/jalr, the six branches, and their
  // compressed forms (c.j, c.jal, c.jr, c.jalr, c.beqz, c.bnez). A branch
  // opcode with funct3 2'b01x matches none of the six instr_b* flags, takes
  // no case arm, and falls through to the sequential pc -- so it is
  // deliberately NOT excluded here.
  logic f_jump_branch;
  assign f_jump_branch =
      // jal / jalr (rtl/decoder.v instr_jal_op / instr_jalr_op)
      (f_uncompressed && f_instr[6:2] == 5'b11011) ||
      (f_uncompressed && f_instr[6:2] == 5'b11001 && f_instr[14:12] == 3'b000) ||
      // beq/bne/blt/bge/bltu/bgeu (instr_branch_op, the six real funct3s)
      (f_uncompressed && f_instr[6:2] == 5'b11000 &&
         f_instr[14:12] != 3'b010 && f_instr[14:12] != 3'b011) ||
      // c.j / c.jal / c.beqz / c.bnez (quadrant 01)
      (f_instr[1:0] == 2'b01 && (f_instr[15:13] == 3'b101 || f_instr[15:13] == 3'b001 ||
                                 f_instr[15:13] == 3'b110 || f_instr[15:13] == 3'b111)) ||
      // c.jr / c.jalr (quadrant 10, cfunct3 100, rs2 field 0, rs1 field != 0;
      // instr[12] only picks jr vs jalr, so the union drops it)
      (f_instr[1:0] == 2'b10 && f_instr[15:13] == 3'b100 && f_instr[6:2] == 5'b0 &&
         f_instr[11:7] != 5'b0);

  // A cycle on which the decoder may legitimately hold pc. Deliberately an
  // OVER-approximation of rtl/decoder.v's `stall`: the real hazard also
  // requires that the instruction consumes rs1/rs2 as register operands
  // (uses_rs1/uses_rs2), and restating that here would mean duplicating most
  // of decode. Skipping a superset of the stall cycles is sound for the
  // increment assertion (it only ever *excuses* cycles, and hazard implies
  // this signal); what it forgoes -- proving pc holds on a RAW-hazard cycle
  // -- is asserted where `stall` is visible, in rtl/decoder.v's own FORMAL
  // block. Built entirely from pcloop-scope nets: rs1/rs2 and decoder_out
  // are decoder output ports, the rest are this module's free inputs, and
  // the producer test transcribes rtl/decoder.v's live_producer().
  logic f_live_rs1, f_live_rs2, f_may_stall;
  assign f_live_rs1 = rs1 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs1) ||
       (executor_out.valid && executor_out.rd == rs1) ||
       (accessor_pending_valid && accessor_pending_rd == rs1));
  assign f_live_rs2 = rs2 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs2) ||
       (executor_out.valid && executor_out.rd == rs2) ||
       (accessor_pending_valid && accessor_pending_rd == rs2));
  // ADR-0026's fourth stall reason, transcribed the same way: a Zicsr access
  // is held in decode until the pipe drains, and `mret` joins it there
  // (CLAUDE.md invariant 5). Over-approximated like the rest -- the real term
  // also requires the pipe to be busy, and this deliberately does not look at
  // that -- so it only ever excuses more cycles, never fewer.
  //
  // Widened to the WHOLE SYSTEM opcode rather than the six csrr* funct3s,
  // because `mret` has funct3 == 0 and the narrower test missed it: a
  // serializing `mret` held the pc on a cycle this signal called quiet and the
  // increment assertion below failed. `ecall`/`ebreak` come along for the ride
  // and are covered by f_redirect anyway, since they now trap.
  logic f_system;
  assign f_system = f_uncompressed && f_instr[6:2] == 5'b11100;

  assign f_may_stall = divider_stall || accessor_stall || f_live_rs1 || f_live_rs2 ||
      f_system;

  // Trap entry and `mret` also write a non-sequential pc (ADR-0028: a trap is
  // a branch), so the sequential-advance assertion must not cover those
  // cycles either. Unlike everything above, these are NOT re-derived from
  // `f_instr`: "would the decoder consider this word illegal" is decode, and
  // restating it here would be the duplication this file refuses. They are
  // read off the decoder's own OUTPUT PORTS instead -- pcloop-scope nets, not
  // the phantom hierarchical references the note above warns about.
  //
  // Excusing a cycle on the DUT's own say-so would be a hole if nothing else
  // covered it, so nothing else is left uncovered: the two assertions at the
  // bottom of this block pin where the pc actually went on exactly these
  // cycles. A decoder that claimed a spurious trap to escape the increment
  // assertion would then have to land on mtvec to satisfy those.
  logic f_redirect;
  assign f_redirect = trap_entry || mret_entry;

  // Registered history, sampled at the same posedge that updates pc. That
  // same-edge sample is the phase relationship rtl/decoder.v:442 dictates:
  // `pc <= fetcher_pc + pc_inc` computes both addends combinationally from
  // the PRE-edge state (fetcher_pc = pc through the fetcher's echo, pc_inc
  // from the word the fetch window presents at that same pre-edge pc), so
  // the width that produced the post-edge pc is `uncompressed` as it stood
  // just before the edge -- exactly what a plain registered copy holds one
  // cycle later. Every history bit here copies a signal that is real and
  // driven on every non-reset cycle (fetcher_out is reset-muxed to zero, the
  // ports are always driven), and the one garbage sample possible -- the
  // pre-reset initial state at cycle 0 -- lands under prev_reset = 1, which
  // every assertion below excludes.
  logic [31:0] past_pc, prev_mtvec, prev_mepc;
  logic prev_reset, prev_may_stall, prev_hard_stall, prev_jump_branch, prev_uncompressed;
  logic prev_trap_entry, prev_mret_entry;
  always_ff @(posedge clk) begin
    past_pc           <= pc;
    prev_reset        <= reset;
    prev_may_stall    <= f_may_stall;
    prev_hard_stall   <= divider_stall || accessor_stall;
    prev_jump_branch  <= f_jump_branch || f_redirect;
    prev_uncompressed <= f_uncompressed;
    prev_trap_entry   <= trap_entry;
    prev_mret_entry   <= mret_entry;
    prev_mtvec        <= mtvec;
    prev_mepc         <= mepc;
  end

  // The echo, asserted rather than assumed. This one line is what turns the
  // standalone task's `assume(in.pc == pc)` into a proof obligation.
  always_comb if (clocked && !reset) assert(fetcher_out.pc == pc);

  // Sequential advance: on a cycle whose predecessor was quiet -- not reset,
  // no possible stall, not a jump or branch -- pc moved by exactly the width
  // of the word the fetcher presented, with the width judged independently
  // from that word itself. The advance provably routed through the real
  // fetcher: rtl/decoder.v:442 adds fetcher_pc, not its own pc, and only the
  // echo assertion above ties fetcher_pc back to past_pc.
  always_ff @(posedge clk)
    if (clocked && !prev_reset && !prev_may_stall && !prev_jump_branch)
      assert(pc == past_pc + (prev_uncompressed ? 32'd4 : 32'd2));

  // Deliberately NOT asserted here: `pc_inc == (uncompressed ? 4 : 2)` is a
  // verbatim restatement of rtl/decoder.v:343, so it would prove nothing
  // about the loop -- and the standalone decoder task already pins pc_inc's
  // two constants (mutating it to `uncompressed ? 4 : 4` fails there). What
  // this task adds is that the increment reaches pc *through a real fetcher*.

  // A divider/accessor freeze holds the PC (ADR-0009 / ADR-0015: upstream of
  // a stalling stage freezes). Under-approximated to the two unconditional
  // stall sources on purpose -- both are direct inputs here, and
  // rtl/decoder.v's first publish arm holds pc whenever either is up,
  // hazard or no hazard. The RAW-hazard hold is the standalone decoder
  // task's assertion, as above.
  always_ff @(posedge clk)
    if (clocked && prev_hard_stall && !prev_reset) assert(pc == past_pc);

  // The redirect cycles the increment assertion above steps around, covered
  // here instead of merely excused. A trap goes to mtvec and an mret goes to
  // mepc -- through the real closed loop, so the decoder's `pc` output and the
  // address the fetcher will present next cycle are the same thing (the echo
  // assertion above is what ties them together).
  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);
 `endif
endmodule

`default_nettype wire
