// The fetcher and the decoder, wired together the way rtl/littlecpu.v wires
// them. On its own the decoder cannot prove its pc arithmetic. It has to assume
// the fetcher hands back the pc it was given, and under that assumption its own
// assertions say pc == pc + pc_inc. Here that echo is asserted instead.
//
// Keep reading the RTL without -formal for this task; formal/components.sby
// does. With -formal the decoder brings its own assume along, and it assumes
// the very thing this task asserts. A broken fetcher would come out as an
// unreachable state instead of a counterexample.
//
// Everything not instantiated here is a free input. That is safe. reg_rs1,
// mtvec and mepc only reach the pc on jump, branch, trap and mret cycles, and
// the increment assertion skips all of those.
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
    input logic fetch_stall,
    // Free, like everything else not instantiated here. It redirects the pc,
    // and the increment assertion skips a redirect because the decoder's own
    // `branch_jump` names the trap it raises.
    input logic imem_fault,
    // Free for the same reason and with the same effect: an atomic the platform
    // does not answer redirects the pc, and `branch_jump` names that trap too.
    input logic atomic_supported,
    input logic accessor_out_valid,
    // rtl/pairtable.v's answer, left free: nothing decode does with it can be
    // wrong, because `operand_stall` checks the guess against the pair the
    // issuing instruction really reads. Free is wider than the table, so the
    // excuse below covers every entry the table could hold and every one it
    // could not.
    input logic pair_hit,
    input logic [4:0] pair_rs1,
    input logic [4:0] pair_rs2,
    input logic [31:0] csr_rdata,
    input logic csr_implemented,
    input logic [31:0] mtvec,
    input logic [31:0] mepc,
    // Free, like everything else not instantiated here. An interrupt redirects
    // the pc, so the increment assertion has to skip that cycle -- and it does,
    // because the decoder's own `branch_jump` names it.
    input logic interrupt_pending
);
  logic [31:0] pc;
  logic [31:0] imem_addr, imem_addr2;
  logic [31:0] next_pc, imem_addr_next;
  // The address the decoder publishes for a platform to decode. Unread here:
  // `atomic_supported` is a free input, so the region decision is the
  // solver's rather than a map's.
  logic [31:0] atomic_addr;
  fetcher_output fetcher_out;
  decoder_output decoder_out;
  logic [4:0] rs1, rs2, read_rs1, read_rs2;
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
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    .accessor_out_valid(accessor_out_valid),
    .pair_hit(pair_hit),
    .pair_rs1(pair_rs1),
    .pair_rs2(pair_rs2),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(mtvec),
    .mepc(mepc),
    .interrupt_pending(interrupt_pending),
    .pc(pc),
    .next_pc(next_pc),
    .rs1(rs1),
    .rs2(rs2),
    .read_rs1(read_rs1),
    .read_rs2(read_rs2),
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

  // Assumed: reset is high before the first clock edge and low forever after.
  // Every harness in this tree drives it that way. Nothing proves it, so this
  // is a convention, not a fact.
  //
  // It is unguarded, so it holds over every assertion below, not just the two
  // that compare pc across a cycle. Those two are why it is here. Reset sets
  // next_pc to 0, so a solver free to raise reset again could zero pc between
  // the two cycles and satisfy them for the wrong reason.
  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  // Build the guard signals below from this module's own ports. Do not write
  // `decoder.uncompressed` or anything like it. yosys does not reach into the
  // instance: it makes a new wire with that name, nothing drives it, and the
  // solver picks the value. An earlier version of this file did that and got a
  // counterexample where pc stepped 0 -> 4 correctly while the recorded history
  // said the instruction was compressed.
  //
  // The decoder zeroes instr[31:16] when the low two bits are not 2'b11. That
  // cannot put these terms out of step with it. Every 32-bit match below needs
  // those bits to be 2'b11, and every compressed match reads only instr[15:0].
  logic [31:0] f_instr;
  assign f_instr = fetcher_out.instr;
  logic f_uncompressed;
  assign f_uncompressed = f_instr[1:0] == 2'b11;
  // A branch opcode with funct3 010 or 011 is none of the six branches. The
  // decoder gives it no arm and the pc advances normally, so do not exclude it.
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

  // These terms are wider than the decoder's real stall conditions. The decoder
  // also checks that the instruction uses the register, and copying that would
  // mean copying most of decode. Wider is safe: it only skips more cycles. The
  // cycles it skips are covered by rtl/decoder.v's own proof.
  logic f_live_rs1, f_live_rs2, f_may_stall;
  assign f_live_rs1 = rs1 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs1) ||
       (executor_out.valid && executor_out.rd == rs1));
  assign f_live_rs2 = rs2 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs2) ||
       (executor_out.valid && executor_out.rd == rs2));
  // The whole SYSTEM opcode, not just the six csrr* funct3 values. mret waits
  // for the pipeline too, and its funct3 is 0.
  logic f_system;
  assign f_system = f_uncompressed && f_instr[6:2] == 5'b11100;

  // Widening this one does not make the increment assertion useless. A stalled
  // decoder holds the pc, so the same instruction word comes back next cycle
  // and rs1/rs2 do not change. On the cycle the pc really moves, this is low.
  //
  // The two sides are deliberately different signals. What the register file was
  // asked for last cycle is `read_rs1`, which on an issuing cycle is a guess at
  // the next instruction's pair; what has to match it is the pair the current
  // instruction decodes to. Register the guess on both sides and this is true on
  // nearly every issuing cycle, which would excuse the pc from advancing almost
  // everywhere and leave the assertion below asking nothing. That warning is not
  // left to this comment to enforce: the two cover goals further down have to
  // reach that assertion, and the second of them is unreachable under exactly
  // that edit.
  logic [4:0] f_prev_rs1, f_prev_rs2;
  logic       f_read_taken, f_operand_fetch;
  always_ff @(posedge clk) begin
    if (reset) begin
      f_prev_rs1   <= 5'd0;
      f_prev_rs2   <= 5'd0;
      f_read_taken <= 1'b0;
    end else begin
      f_prev_rs1   <= read_rs1;
      f_prev_rs2   <= read_rs2;
      f_read_taken <= 1'b1;
    end
  end
  assign f_operand_fetch = !f_read_taken || f_prev_rs1 != rs1 || f_prev_rs2 != rs2;

  // The control for the paragraph above, run by formal/pcloop_cover.sby. A
  // comment cannot stop the excuse from widening; the two cover goals near the
  // increment assertion can, and this is what the second of them reads. It says
  // the pair presented to the register file changed from the one presented
  // before it -- which is what an excuse built from two registered copies of the
  // guess would be, so that excuse and this signal cannot both be low on the
  // same cycle.
  logic f_pair_moved;
  assign f_pair_moved = read_rs1 != f_prev_rs1 || read_rs2 != f_prev_rs2;

  // List every reason decode can stall. Miss one and the assertion below covers
  // a cycle where the pc is allowed to hold, then fails there instead of
  // finding a real bug. This has happened before: the register file read became
  // synchronous, that added a stall, and this list did not know about it.
  //
  // f_system does not cover fence.i. That term matches SYSTEM; fence.i is
  // MISC-MEM.
  logic f_fencei;
  assign f_fencei = f_uncompressed && f_instr[6:2] == 5'b00011;

  // The atomic wait is the one reason not decidable from the word being fetched:
  // it is raised the cycle AFTER an AMO issues, for the cycle rtl/accessor.v
  // needs the bus to write back. So it reads `decoder_out`, which is a port of
  // the instance and not a reach-in. Wider than the decoder's own term, which
  // also requires the divide to be over.
  logic f_amo_wait;
  assign f_amo_wait = decoder_out.valid &&
      (decoder_out.is_amoswap || decoder_out.is_amoadd || decoder_out.is_amoxor ||
       decoder_out.is_amoand  || decoder_out.is_amoor  || decoder_out.is_amomin ||
       decoder_out.is_amomax  || decoder_out.is_amominu || decoder_out.is_amomaxu);

  assign f_may_stall = divider_stall || fetch_stall ||
      f_live_rs1 || f_live_rs2 || f_system || f_fencei || f_operand_fetch ||
      f_amo_wait;

  // Read off the decoder's outputs rather than decoded here. Working out
  // whether a word is illegal is most of decode, and copying it would defeat
  // the point of this file.
  //
  // That lets the decoder excuse its own cycles, so the last two assertions
  // below check where the pc actually went on them. Claim a trap you did not
  // take and you still have to land on mtvec.
  logic f_redirect;
  assign f_redirect = trap_entry || mret_entry;

  // Sampled on the same edge that updates pc. That is the right edge: pc is
  // computed from values that had settled before it.
  logic [31:0] past_pc, prev_mtvec, prev_mepc;
  logic prev_reset, prev_may_stall, prev_hard_stall, prev_jump_branch, prev_uncompressed;
  logic prev_trap_entry, prev_mret_entry, prev_fetch_stall, prev_pair_moved;
  always_ff @(posedge clk) begin
    past_pc           <= pc;
    prev_reset        <= reset;
    prev_may_stall    <= f_may_stall;
    prev_hard_stall   <= divider_stall;
    prev_fetch_stall  <= fetch_stall;
    prev_jump_branch  <= f_jump_branch || f_redirect;
    prev_uncompressed <= f_uncompressed;
    prev_trap_entry   <= trap_entry;
    prev_mret_entry   <= mret_entry;
    prev_mtvec        <= mtvec;
    prev_mepc         <= mepc;
    prev_pair_moved   <= f_pair_moved;
  end

  always_comb if (clocked && !reset) assert(fetcher_out.pc == pc);

  // The memory latches its address a cycle early, so imem_addr_next this cycle
  // must be imem_addr next cycle. rtl/decoder.v checks the same thing one level
  // up, on pc and next_pc. This one is on the fetcher's output ports, so it
  // also covers the two word-alignment masks: change one and forget the other
  // and only this fails. No generated riscv-formal check reads either port.
  logic [31:0] past_imem_addr_next;
  always_ff @(posedge clk) past_imem_addr_next <= imem_addr_next;
  always_comb if (clocked) assert(imem_addr == past_imem_addr_next);

  // The increment goes through the real fetcher. rtl/decoder.v adds fetcher_pc,
  // not its own pc, and only the echo assertion above ties fetcher_pc back to
  // past_pc.
  //
  // The guard is a signal rather than written out twice because the two cover
  // goals below have to sit on exactly it. A cover under a copy could be
  // narrowed apart from the assertion and go on reporting that a cycle nobody
  // checks any more is reachable.
  logic f_increment_checked;
  assign f_increment_checked =
      clocked && !prev_reset && !prev_may_stall && !prev_jump_branch;

  always_ff @(posedge clk)
    if (f_increment_checked)
      assert(pc == past_pc + (prev_uncompressed ? 32'd4 : 32'd2));

  // pc_inc == (uncompressed ? 4 : 2) is not asserted here. It would just
  // restate rtl/decoder.v, whose own proof already pins both constants.

  // Only cover mode looks at these; formal/pcloop_cover.sby is that run, and
  // formal/Makefile makes it a prerequisite of the proof so neither can be run
  // without the other.
  //
  // The first fails if any term of f_may_stall becomes universal -- an excuse
  // that fires everywhere leaves the assertion above guarded by something
  // always false and passing having asked nothing. The second fails if
  // f_operand_fetch stops comparing what was presented against the pair the
  // instruction decodes to, which is the one edit that would make the excuse
  // near-universal without making it constant.
  always_ff @(posedge clk)
    if (f_increment_checked) begin
      increment_reached: cover (1'b1);
      increment_reached_on_moved_pair: cover (prev_pair_moved);
    end

  always_ff @(posedge clk)
    if (clocked && prev_hard_stall && !prev_reset) assert(pc == past_pc);

  // Holding the pc is what makes a stolen fetch cycle harmless. The same
  // instruction comes back next cycle, so nothing issued and nothing needs
  // undoing. Separate from the freeze above: the two take different arms of the
  // publish block.
  always_ff @(posedge clk)
    if (clocked && prev_fetch_stall && !prev_reset) assert(pc == past_pc);

  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);
 `endif
endmodule

`default_nettype wire
