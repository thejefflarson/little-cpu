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
  // ADR-0003: the dual-word fetch window's second word, free every cycle
  // exactly like imem_data above -- littlecpu never stalls or waits on
  // either (CLAUDE.md invariant 1), so it needs no more constraint here
  // than the single-word port already had.
  (* keep *) `rvformal_rand_reg [31:0] imem_data2;
  (* keep *) `rvformal_rand_reg [31:0] mem_rdata;

  (* keep *) logic [31:0] imem_addr;
  (* keep *) logic [31:0] imem_addr2;
  (* keep *) logic [31:0] mem_addr;
  (* keep *) logic [31:0] mem_wdata;
  (* keep *) logic [3:0]  mem_wstrb;
  (* keep *) logic        trap;

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
