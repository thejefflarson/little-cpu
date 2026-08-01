// ADR-0006: ported from the wave-0 (rtl/riscv.v-era) harness, which spoke a
// picorv32-style mem_valid/mem_instr/mem_ready handshake. `littlecpu` (the
// module that replaced `riscv` -- deleted in `1709433`) has no handshake:
// fetch is combinational and unconditional (CLAUDE.md invariant 1), and the
// data bus is a plain address/strobe/data bus with a fixed one-cycle load
// turnaround (ADR-0015), not a request/ready protocol. imem_data and
// mem_rdata are simply free every cycle, the same as riscv-formal's other
// non-handshake cores (cores/nerv, cores/serv model the equivalent with an
// explicit ready/ack bit; littlecpu has none to model).
module rvfi_wrapper (
  input var clock, reset,
  `RVFI_OUTPUTS
);
  `RVFI_WIRES

  (* keep *) `rvformal_rand_reg [31:0] imem_data;
  // ADR-0003: the dual-word fetch window's second word, resampled every cycle
  // exactly like imem_data above -- littlecpu never stalls or waits on either
  // (CLAUDE.md invariant 1), so it carries no handshake. Both are constrained
  // by exactly one thing, the same-address stability assumption further down;
  // that block is where to read about why it exists (ADR-0042). This comment
  // used to say the two ports needed "no more constraint than the single-word
  // port already had", which was true only while the regfile read was
  // combinational.
  (* keep *) `rvformal_rand_reg [31:0] imem_data2;
  (* keep *) `rvformal_rand_reg [31:0] mem_rdata;

  (* keep *) logic [31:0] imem_addr;
  (* keep *) logic [31:0] imem_addr2;
  (* keep *) logic [31:0] mem_addr;
  (* keep *) logic [31:0] mem_wdata;
  (* keep *) logic [3:0]  mem_wstrb;
  (* keep *) logic        trap;

  // INSTRUCTION MEMORY IS A FUNCTION OF ITS ADDRESS (ADR-0042, ADR-0017).
  //
  // The structural fact being modelled: a ROM asked twice for the same address,
  // with no write port anywhere in this design that could reach it, answers the
  // same both times. That is a property of memory, not a convenience -- and
  // formal/imemcheck.sv already relies on it, in the stronger `rand_const_reg`
  // form (a fixed address whose data is fixed for the whole trace).
  //
  // WHY IT IS NEEDED NOW, AND WAS NOT BEFORE. Until ADR-0042 the core read its
  // registers combinationally, so nothing it did depended on imem_data holding
  // still: the comment on RISCV_FAIRNESS below said littlecpu "has no port an
  // adversarial environment could hold to defeat forward progress," and that
  // was true. A registered regfile read changes it. Decode now presents an
  // address in one cycle and consumes the operand in the next, and it decides
  // the two belong to the same instruction by comparing rs1/rs2 -- which are
  // combinational out of imem_data. Left fully free, the environment may hand
  // the held PC a different instruction word every cycle forever, decode never
  // issues, and `hang` and `liveness_ch0` produce counterexamples at k = 30.
  // Measured: without the assumption below, exactly those two checks go red on
  // an otherwise 82-pass ladder, both as real counterexamples rather than
  // timeouts.
  //
  // WHAT IT COSTS. This is an assumption, so it can only make checks easier,
  // and that is the honest price: a defect that manifests only when the same
  // address yields different data in consecutive cycles is now invisible here.
  // No such defect is possible against real memory, which is the point -- but
  // it is a real narrowing of the environment and belongs in the ADR, not only
  // in a comment.
  //
  // The form is the WEAKEST one sufficient for the property: stability across
  // consecutive cycles, not a full functional model. A full model needs an
  // unbounded array; consecutive-cycle stability is implied by it, is all the
  // operand-fetch cycle actually needs, and leaves the environment free to
  // answer differently for an address revisited later.
  logic [31:0] past_imem_addr, past_imem_data;
  logic [31:0] past_imem_addr2, past_imem_data2;
  logic        past_imem_valid = 0;

  always @(posedge clock) begin
    past_imem_addr  <= imem_addr;
    past_imem_data  <= imem_data;
    past_imem_addr2 <= imem_addr2;
    past_imem_data2 <= imem_data2;
    past_imem_valid <= !reset;
  end

  always @* begin
    if (past_imem_valid && !reset) begin
      if (imem_addr  == past_imem_addr)  assume (imem_data  == past_imem_data);
      if (imem_addr2 == past_imem_addr2) assume (imem_data2 == past_imem_data2);
    end
  end

  littlecpu dut (
    .clk(clock),
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

 `ifdef RISCV_FAIRNESS
  // picorv32/nerv/serv each need this block to bound an *external* signal
  // (mem_ready, or a bus ack) that the formal environment controls and could
  // otherwise withhold forever, starving the liveness check of a next
  // retire. littlecpu exposes no such signal: imem_data/mem_rdata above are
  // free every cycle, but nothing in the core *waits* on their value --
  // fetch is combinational, and the two internal stall sources this design
  // has are driven entirely by the core's own counters, not by anything an
  // adversarial environment supplies:
  //   - rtl/executor.v's divider: `mul_div_counter` (rtl/executor.v:169,
  //     `mul_div_counter <= 32`) counts strictly down from 32 every cycle
  //     once loaded, with no way for imem_data/mem_rdata's value to hold it.
  //   - rtl/accessor.v's load-response turnaround: `stalled` is high for
  //     exactly the one cycle after a load's request (ADR-0015 invariant
  //     I3 -- the following cycle `in.valid` is guaranteed 0, so `stalled`
  //     falls unconditionally).
  // There is nothing for this block to assume: littlecpu has no port an
  // adversarial environment could hold to defeat forward progress, so
  // RISCV_FAIRNESS's usual job (bound the stall) has no signal to act on
  // here. Left as an explicit empty block, per ADR-0017, rather than a
  // silently omitted `ifdef` -- a future reader should see this was a
  // decision, not an oversight, if the liveness check ever surprises them.
 `endif
endmodule
