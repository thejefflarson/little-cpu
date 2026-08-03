// The fetcher and the decoder wired together as rtl/littlecpu.v wires them, so
// that the pc echo rtl/decoder.v's standalone task has to assume becomes an
// assertion here. Under that assumption its pc-increment assertions read
// `pc <= pc + pc_inc`, which is what they are trying to prove (ADR-0017).
//
// formal/components.sby must keep reading the RTL WITHOUT -formal for this
// task. Compiled with it the decoder brings its own `assume(in.pc == pc)`
// along, assumes the echo asserted here, and turns a broken fetcher into an
// unreachable state instead of a counterexample.
//
// Everything this task does not instantiate stays a free input, which is the
// stronger environment and costs nothing: reg_rs1, mtvec and mepc reach pc
// arithmetic only on cycles the increment assertion already excludes.
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
    input logic fetch_stall,
    input logic accessor_pending_valid,
    input logic [4:0] accessor_pending_rd,
    input logic accessor_out_valid,
    input logic [31:0] csr_rdata,
    input logic csr_implemented,
    input logic [31:0] mtvec,
    input logic [31:0] mepc
);
  logic [31:0] pc;
  logic [31:0] imem_addr, imem_addr2;
  logic [31:0] next_pc, imem_addr_next;
  fetcher_output fetcher_out;
  decoder_output decoder_out;
  logic [4:0] rs1, rs2;
  // rtl/csrs.v's half of the decoder's output, unread here but for f_redirect.
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata;
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc;

  fetcher fetcher (
    .clk(clk),
    .reset(reset),
    .pc(pc),
    .next_pc(next_pc),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
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
    .fetch_stall(fetch_stall),
    .accessor_pending_valid(accessor_pending_valid),
    .accessor_pending_rd(accessor_pending_rd),
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(mtvec),
    .mepc(mepc),
    .pc(pc),
    .next_pc(next_pc),
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

  // Assumed: reset is asserted before the first edge and never returns -- true
  // of every harness in the tree, asserted by none of them, and unguarded, so
  // it is in force over every assertion here rather than only the pc-history
  // pair it was written for (ADR-0049).
  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  // Never reach into the decoder instance for a guard signal: yosys does not
  // resolve `decoder.uncompressed`, it implicitly declares an undriven wire of
  // that name and lets the solver drive it freely. An earlier revision of this
  // file got a "counterexample" that way, in which pc stepped 0 -> 4 correctly
  // while the sampled history claimed the instruction was compressed.
  //
  // Re-deriving is also stronger, and decoder.v's upper-half masking cannot
  // make it disagree: every uncompressed match below requires quadrant ==
  // 2'b11, where the mask is the identity, and every compressed match reads
  // only instr[15:0].
  logic [31:0] f_instr;
  assign f_instr = fetcher_out.instr;
  logic f_uncompressed;
  assign f_uncompressed = f_instr[1:0] == 2'b11;
  // A branch opcode with funct3 2'b01x matches none of the six instr_b* flags,
  // takes no arm in the publish block and falls through to the sequential pc,
  // so it is deliberately not excluded here.
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

  // Every term here over-approximates rtl/decoder.v's `stall`, which also
  // requires the instruction to consume the operand; restating that would
  // duplicate most of decode. Excusing a superset of the stall cycles is sound
  // for the increment assertion, and the RAW-hazard hold it gives up is
  // asserted in rtl/decoder.v's own FORMAL block.
  logic f_live_rs1, f_live_rs2, f_may_stall;
  assign f_live_rs1 = rs1 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs1) ||
       (executor_out.valid && executor_out.rd == rs1) ||
       (accessor_pending_valid && accessor_pending_rd == rs1));
  assign f_live_rs2 = rs2 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs2) ||
       (executor_out.valid && executor_out.rd == rs2) ||
       (accessor_pending_valid && accessor_pending_rd == rs2));
  // The whole SYSTEM opcode rather than the six csrr* funct3s, because `mret`
  // serializes too and has funct3 == 0.
  logic f_system;
  assign f_system = f_uncompressed && f_instr[6:2] == 5'b11100;

  // This term does not hollow out the increment assertion, for a structural
  // reason: `operand_stall` holds the pc, so the same word is re-presented and
  // rs1/rs2 are unchanged on the cycle the instruction actually issues. Every
  // cycle the pc really advances on is therefore a cycle it calls quiet.
  logic [4:0] f_prev_rs1, f_prev_rs2;
  logic       f_read_taken, f_operand_fetch;
  always_ff @(posedge clk) begin
    if (reset) begin
      f_prev_rs1   <= 5'd0;
      f_prev_rs2   <= 5'd0;
      f_read_taken <= 1'b0;
    end else begin
      f_prev_rs1   <= rs1;
      f_prev_rs2   <= rs2;
      f_read_taken <= 1'b1;
    end
  end
  assign f_operand_fetch = !f_read_taken || f_prev_rs1 != rs1 || f_prev_rs2 != rs2;

  // `f_may_stall` must name every stall reason the decoder has -- six now. A
  // missing one makes the sequential-advance assertion cover a cycle the
  // decoder legitimately holds the pc on and the task goes red on correct RTL,
  // which is how ADR-0042's operand-fetch cycle left it failing. `f_system`
  // cannot cover `fence.i`: that is the SYSTEM opcode and this is MISC-MEM.
  logic f_fencei;
  assign f_fencei = f_uncompressed && f_instr[6:2] == 5'b00011;

  assign f_may_stall = divider_stall || accessor_stall || fetch_stall ||
      f_live_rs1 || f_live_rs2 || f_system || f_fencei || f_operand_fetch;

  // The one guard not re-derived from `f_instr`, because "would the decoder
  // consider this word illegal" is decode. Taking it off the DUT's own output
  // ports excuses cycles on the DUT's say-so, which is why the two assertions
  // at the bottom pin where the pc went on exactly those cycles: a decoder
  // claiming a spurious trap would then have to land on mtvec.
  logic f_redirect;
  assign f_redirect = trap_entry || mret_entry;

  // Sampled at the same posedge that updates pc, which is the right phase
  // because `pc <= fetcher_pc + pc_inc` computes both addends combinationally
  // from the pre-edge state.
  logic [31:0] past_pc, prev_mtvec, prev_mepc;
  logic prev_reset, prev_may_stall, prev_hard_stall, prev_jump_branch, prev_uncompressed;
  logic prev_trap_entry, prev_mret_entry, prev_fetch_stall;
  always_ff @(posedge clk) begin
    past_pc           <= pc;
    prev_reset        <= reset;
    prev_may_stall    <= f_may_stall;
    prev_hard_stall   <= divider_stall || accessor_stall;
    prev_fetch_stall  <= fetch_stall;
    prev_jump_branch  <= f_jump_branch || f_redirect;
    prev_uncompressed <= f_uncompressed;
    prev_trap_entry   <= trap_entry;
    prev_mret_entry   <= mret_entry;
    prev_mtvec        <= mtvec;
    prev_mepc         <= mepc;
  end

  always_comb if (clocked && !reset) assert(fetcher_out.pc == pc);

  // The fetch lockstep (ADR-0054). Nothing else holds the core to it: breaking
  // it has one symptom, the SoC fetching the wrong instruction, with no
  // elaboration error, no lint finding, and no ladder check in contact with the
  // port -- formal/wrapper.v answers imem_data combinationally and never reads
  // it.
  logic [31:0] past_imem_addr_next;
  always_ff @(posedge clk) past_imem_addr_next <= imem_addr_next;
  always_comb if (clocked) assert(imem_addr == past_imem_addr_next);

  // The advance provably routed through the real fetcher: rtl/decoder.v adds
  // `fetcher_pc`, not its own pc, and only the echo assertion above ties that
  // back to past_pc.
  always_ff @(posedge clk)
    if (clocked && !prev_reset && !prev_may_stall && !prev_jump_branch)
      assert(pc == past_pc + (prev_uncompressed ? 32'd4 : 32'd2));

  // `pc_inc == (uncompressed ? 4 : 2)` is deliberately NOT asserted here: it
  // restates rtl/decoder.v, and the standalone decoder task already pins both
  // constants.

  always_ff @(posedge clk)
    if (clocked && prev_hard_stall && !prev_reset) assert(pc == past_pc);

  // Holding the pc is what makes the steal a bubble rather than something
  // needing a kill signal (CLAUDE.md invariant 1): the same instruction is
  // presented again, so there is nothing to undo. Separate from the freeze
  // above because the two reach the pc through different publish arms.
  always_ff @(posedge clk)
    if (clocked && prev_fetch_stall && !prev_reset) assert(pc == past_pc);

  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);
 `endif
endmodule

`default_nettype wire
