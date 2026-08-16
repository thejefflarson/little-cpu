// There is no bus handshake to model. Fetch is combinational and unconditional,
// and the data bus is a plain address/strobe/data bus with a fixed one-cycle
// load turnaround, so `imem_data` and `mem_rdata` are simply free every cycle.
module rvfi_wrapper (
  input var clock, reset,
  `RVFI_OUTPUTS
);
  `RVFI_WIRES

  (* keep *) `rvformal_rand_reg [31:0] imem_data;
  // The fetch window's second word, resampled every cycle exactly like
  // `imem_data`. Both are constrained by one thing only, the same-address
  // stability assumption further down.
  (* keep *) `rvformal_rand_reg [31:0] imem_data2;
  (* keep *) `rvformal_rand_reg [31:0] mem_rdata;

  (* keep *) logic [31:0] imem_addr;
  (* keep *) logic [31:0] imem_addr2;
  // The fetch address one cycle early, for a synchronous memory. Unread here:
  // this environment answers `imem_data` against `imem_addr` in the same cycle,
  // so every check sees a combinational fetch bus and never the port the
  // shipping SoC uses.
  (* keep *) logic [31:0] imem_addr_next;
  (* keep *) logic [31:0] mem_addr;
  (* keep *) logic [31:0] mem_wdata;
  (* keep *) logic [3:0]  mem_wstrb;
  (* keep *) logic        mem_ren;
  (* keep *) logic        trap;

  (* keep *) logic fetch_stall;
  logic text_write;

  // The memory has nothing at the address it is answering. Free, like the data
  // it accompanies, because this environment models no address map -- what
  // ranges rtl/imemory.v calls text is test/imem_tb.v's question, not these
  // checks'. What they need is the one thing a memory that answers nothing
  // always does, which is the assumption below.
  (* keep *) `rvformal_rand_reg imem_fault;

  // Whether the data address this cycle is reservable main memory. Free, for
  // the same reason `imem_fault` is: this environment models no address map,
  // and a core that only works on a platform that answers everywhere is not
  // what these checks should accept. Nothing has to be assumed about it --
  // it decides only whether a store-conditional may succeed, and failing one
  // spuriously is permitted everywhere.
  (* keep *) `rvformal_rand_reg mem_reservable;

  // Whether an atomic's address is memory that answers one. Free for the same
  // reason, and this is the one that makes the load and store arms of
  // checks/rvfi_fault_check.sv reachable at all: with it tied high the core
  // raises causes 5 and 7 nowhere, and that check would be asking only about
  // fetches. Nothing is assumed about it either -- an atomic decode either
  // faults with it or issues without it, and both are this core's behaviour.
  (* keep *) `rvformal_rand_reg atomic_supported;
  wire [31:0] atomic_addr;

  // Assumed: on a cycle it reports having nothing, the instruction memory
  // answers zero on both fetch ports.
  //
  // rtl/imemory.v gates both of them on the same range test that raises the
  // fault, so this is structural rather than believed. It is also what makes the
  // instruction-access-fault arm of checks/rvfi_fault_check.sv say something
  // about this core: that arm requires `insn == 0`, and without this the
  // environment could report a fault alongside a word it invented.
  always @* begin
    if (imem_fault) begin
      assume (imem_data  == 32'b0);
      assume (imem_data2 == 32'b0);
    end
  end

  imem_arbiter arbiter (
    .clock(clock),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .fetch_stall(fetch_stall),
    .text_write(text_write)
  );

  // Two cycles after a text store the banks answer with the new contents, so the
  // same-address compare below has to be dropped there as well as on the stolen
  // cycle. The array is written on the store's own edge and the fetch address is
  // published a cycle early, which is what puts it two cycles out.
  logic [1:0] text_write_age = 0;
  always @(posedge clock)
    text_write_age <= {text_write_age[0], text_write};

  // Assumed: asked for the same address two cycles running, instruction memory
  // answers the same both times.
  //
  // Nothing in this repo discharges it. The backing is structural -- both fetch
  // ports read one array in rtl/imemory.v -- and it is believed rather than
  // proved. It sits in the harness every generated check instantiates, so it is
  // in force over all 85 of them, not only the two it was written for.
  //
  // Why the core needs it: decode presents a register address pair in one cycle
  // and consumes the answer in the next, and it decides the two belong to the
  // same instruction by comparing rs1/rs2, which come straight out of
  // `imem_data`. Left free, the environment can hand the held pc a different
  // instruction word every cycle forever and decode never issues. Measured
  // without it: `hang` and `liveness_ch0` are the only two red, both real
  // counterexamples at k = 30.
  //
  // What it costs: a defect that shows up only when one address answers two
  // different ways in consecutive cycles is invisible to every check. No
  // memory does that, but the narrowing is real.
  //
  // Stability across two cycles rather than a full memory model, because that is
  // all the operand-fetch cycle needs and it leaves an address revisited later
  // free to answer differently.
  //
  // The stolen cycle and the two-cycles-after-a-store cycle drop the compare
  // rather than model what memory answers there. That is the wide side, and an
  // assumption can only make checks easier, so a depth floor derived under it is
  // the safe one.
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
    if (past_imem_valid && !reset && !fetch_stall && !text_write_age[1]) begin
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
    .imem_addr_next(imem_addr_next),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault),
    .mem_reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    // Tied off, and formal/check-interrupt-tie-off.py is what says so. Every
    // depth in formal/checks.cfg is derived from F and G measured with no
    // interrupt in the trace, and riscv-formal ships no model at the pin of
    // what an interrupt does to mcause, mepc or mstatus -- only the two pc
    // checks read `rvfi_intr`, and only to stop expecting continuity. Left
    // free, the generated checks would not be checking a weaker property, they
    // would be checking a different machine against a spec that does not
    // describe it. formal/traps.sv is where the interrupt is proved.
    .irq_timer(1'b0),
    .trap(trap),
    `RVFI_CONN
  );

 `ifdef RISCV_FAIRNESS
  // Other cores use this block to bound a bus ack the environment could
  // withhold forever, starving the liveness check of a next retire. There is
  // nothing here to bound. `imem_data` and `mem_rdata` are free every cycle but
  // the core never waits on their value, and every stall it does have is driven
  // by its own state: the divider counts down from 32 on its own, the load
  // turnaround is one cycle by construction, and `fetch_stall` comes from the
  // arbiter above, which reads the core's own bus rather than being chosen.
  //
  // Left as an explicit empty block so that a liveness surprise later reads as a
  // decision rather than an omission.
 `endif
endmodule
