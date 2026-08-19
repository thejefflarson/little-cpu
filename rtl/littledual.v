`timescale 1 ns / 1 ps
`default_nettype none
// Two harts, one text storage, one data RAM, one machine timer and one bus
// arbiter. rtl/littledualsoc.v gives this pins and `make dual-ecp5-timing`
// places that; test/dual_testbench.v runs it. It is NOT a configuration of
// rtl/littlesoc.v and does not replace it: the up5k SoC ships one hart, and two
// fetch windows need 32 block RAMs against that part's 30, so the dual
// configuration builds on ECP5 only.
//
// TWO HARTS IS THE NUMBER, not a parameter with 2 as its default.
// rtl/busarbiter.v is two request bits, two lock bits and a two-bit grant, and
// its wait bound is proved for exactly that. `NHARTS` below is a localparam so
// the packed widths and the generate loop can be written once, not an
// invitation to raise it.
//
// ---- how a hart gets the data bus ------------------------------------------
//
// The arbiter's grant is REGISTERED, so it has to be asked a cycle before the
// cycle it covers. Decode is the stage that knows a cycle early: `bus_request`
// is high on a cycle that would publish a memory instruction, and the
// transaction that instruction makes goes out the cycle after, from the
// decoder's `out`. So the invariant this top maintains is not "at most one
// transaction" directly -- it is
//
//     AT MOST ONE HART PUBLISHES A MEMORY INSTRUCTION PER CYCLE
//
// which gives at most one transaction one cycle later. `grant` is
// one-hot-or-zero and `bus_wait[h]` is raised for every hart that asked and did
// not get it, so that much follows from the arbiter alone.
//
// An AMO is the case that needs the second term. It publishes once and puts TWO
// transactions on the bus -- the read on the cycle after it issues, the
// read-modify-write on the cycle after that -- so nothing anywhere may publish
// on the cycle its read is out. Its own decode is already spending that cycle,
// and `mem_lock` is high on exactly it, so routing the lock into the OTHER
// hart's wait is what covers the write cycle. The lock reaches the arbiter as
// well, where it holds the grant, and there it lands a cycle later than this
// term needs -- for the same reason the grant is registered. Read the two
// together: the lock says "the bus is busy next cycle too", the grant says "you
// may publish".
//
// ---- why the two harts' buses can be ORed ----------------------------------
//
// rtl/accessor.v drives address, write data and strobes to zero on a cycle it
// is not requesting, so the two harts' bus outputs join with an OR exactly the
// way rtl/littlesoc.v joins its three memories' read data. That is sound only
// under the invariant above, and an OR of two live masters is silent -- so
// test/dual_testbench.v checks it every cycle rather than trusting it.
module littledual #(
  // The text window, in words. The integrator's number, the way it is for one
  // hart: the simulated machine's ROM is deliberately larger than the part's.
  parameter integer ROM_WORDS = 2048,
  parameter INIT_EVEN = "",
  parameter INIT_ODD  = ""
) (
  input  logic       clk,
  // One line per hart, low hart first. Hardware holds both together;
  // test/dual_testbench.v drives them apart, which is how it demonstrates that
  // a run measuring one hart looks different from a run measuring two.
  input  logic [1:0] reset,
  output logic [1:0] trap
 `ifdef RISCV_FORMAL
  ,
  // One retire channel per hart, packed low hart first. Each is one core's
  // stream and nothing joins them: test/monitor.sim.v is a per-hart oracle and
  // reads one of these.
  output logic [1:0]   rvfi_valid,
  output logic [127:0] rvfi_order,
  output logic [63:0]  rvfi_insn,
  output logic [1:0]   rvfi_trap,
  output logic [1:0]   rvfi_halt,
  output logic [1:0]   rvfi_intr,
  output logic [9:0]   rvfi_rs1_addr,
  output logic [9:0]   rvfi_rs2_addr,
  output logic [63:0]  rvfi_rs1_rdata,
  output logic [63:0]  rvfi_rs2_rdata,
  output logic [9:0]   rvfi_rd_addr,
  output logic [63:0]  rvfi_rd_wdata,
  output logic [63:0]  rvfi_pc_rdata,
  output logic [63:0]  rvfi_pc_wdata,
  output logic [63:0]  rvfi_mem_addr,
  output logic [7:0]   rvfi_mem_rmask,
  output logic [7:0]   rvfi_mem_wmask,
  output logic [63:0]  rvfi_mem_rdata,
  output logic [63:0]  rvfi_mem_wdata
 `endif
);
  localparam int NHARTS = 2;

  // Per-hart nets, packed low hart first, the way rtl/imemory.v and rtl/timer.v
  // pack theirs. Declared before the modules that drive them, because later
  // stages feed earlier ones and iverilog wants the name first.
  logic [32*NHARTS-1:0] imem_addr, imem_addr2, imem_addr_next;
  logic [32*NHARTS-1:0] imem_data, imem_data2;
  logic [NHARTS-1:0]    fetch_stall, imem_fault;
  logic [32*NHARTS-1:0] atomic_addr;
  logic [NHARTS-1:0]    atomic_supported;
  logic [NHARTS-1:0]    irq_timer;
  logic [NHARTS-1:0]    bus_request, bus_wait, mem_lock, grant;
  logic [32*NHARTS-1:0] hart_mem_addr, hart_mem_wdata;
  logic [4*NHARTS-1:0]  hart_mem_wstrb;
  logic [NHARTS-1:0]    hart_mem_ren;

  // The shared data bus. One master a cycle, so these are ORs and not muxes.
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren, mem_reservable;
  logic [31:0] imem_mem_rdata, dmem_mem_rdata, timer_mem_rdata;
  assign mem_addr  = hart_mem_addr[31:0]  | hart_mem_addr[63:32];
  assign mem_wdata = hart_mem_wdata[31:0] | hart_mem_wdata[63:32];
  assign mem_wstrb = hart_mem_wstrb[3:0]  | hart_mem_wstrb[7:4];
  assign mem_ren   = hart_mem_ren[0]      | hart_mem_ren[1];
  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata | timer_mem_rdata;

  busarbiter arbiter (
    .clk(clk),
    // The bench drives the two resets apart, so this is the AND: the arbiter
    // comes out of reset once any hart has, and a hart held in reset asks for
    // nothing and is granted nothing.
    .reset(&reset),
    .request(bus_request),
    .mem_lock(mem_lock),
    .grant(grant)
  );

  for (genvar h = 0; h < NHARTS; h++) begin : l_hart
    localparam int OTHER = 1 - h;

    // Asked for the bus and did not get it, or the other hart's atomic owns the
    // next cycle of it. Neither term is raised for a hart that is not asking,
    // so a hart with no memory instruction in decode never waits.
    assign bus_wait[h] = bus_request[h] && (!grant[h] || mem_lock[OTHER]);

    littlecpu #(
      .HART_ID(h[31:0]),
      .LS_TEXT_WORDS(ROM_WORDS)
    ) core (
      .clk(clk),
      .reset(reset[h]),
      .imem_addr(imem_addr[32*h+31:32*h]),
      .imem_data(imem_data[32*h+31:32*h]),
      .imem_addr2(imem_addr2[32*h+31:32*h]),
      .imem_data2(imem_data2[32*h+31:32*h]),
      .imem_addr_next(imem_addr_next[32*h+31:32*h]),
      .mem_addr(hart_mem_addr[32*h+31:32*h]),
      .mem_wdata(hart_mem_wdata[32*h+31:32*h]),
      .mem_wstrb(hart_mem_wstrb[4*h+3:4*h]),
      .mem_ren(hart_mem_ren[h]),
      // Every hart sees the whole bus. A read matters only to the hart whose
      // transaction it answers -- rtl/accessor.v captures it for its own load
      // and for nothing else -- and the reservability of the address on the bus
      // is a question about that same transaction.
      .mem_rdata(mem_rdata),
      .fetch_stall(fetch_stall[h]),
      .imem_fault(imem_fault[h]),
      .mem_reservable(mem_reservable),
      .atomic_addr(atomic_addr[32*h+31:32*h]),
      .atomic_supported(atomic_supported[h]),
      .bus_wait(bus_wait[h]),
      // The other hart's write, which is what a foreign write is here. Taken
      // from that hart's own port rather than off the shared bus, so a hart
      // never snoops its own store back and drops its own reservation.
      .snoop_write(|hart_mem_wstrb[4*OTHER+3:4*OTHER]),
      .snoop_addr(hart_mem_addr[32*OTHER+31:32*OTHER]),
      .mem_lock(mem_lock[h]),
      .bus_request(bus_request[h]),
      .irq_timer(irq_timer[h]),
      .trap(trap[h])
     `ifdef RISCV_FORMAL
      , .rvfi_valid(rvfi_valid[h]),
      .rvfi_order(rvfi_order[64*h+63:64*h]),
      .rvfi_insn(rvfi_insn[32*h+31:32*h]),
      .rvfi_trap(rvfi_trap[h]),
      .rvfi_halt(rvfi_halt[h]),
      .rvfi_intr(rvfi_intr[h]),
      // Constants inside the core and unread by either monitor.
      .rvfi_mode(),
      .rvfi_ixl(),
      .rvfi_rs1_addr(rvfi_rs1_addr[5*h+4:5*h]),
      .rvfi_rs2_addr(rvfi_rs2_addr[5*h+4:5*h]),
      .rvfi_rs1_rdata(rvfi_rs1_rdata[32*h+31:32*h]),
      .rvfi_rs2_rdata(rvfi_rs2_rdata[32*h+31:32*h]),
      .rvfi_rd_addr(rvfi_rd_addr[5*h+4:5*h]),
      .rvfi_rd_wdata(rvfi_rd_wdata[32*h+31:32*h]),
      .rvfi_pc_rdata(rvfi_pc_rdata[32*h+31:32*h]),
      .rvfi_pc_wdata(rvfi_pc_wdata[32*h+31:32*h]),
      .rvfi_mem_addr(rvfi_mem_addr[32*h+31:32*h]),
      .rvfi_mem_rmask(rvfi_mem_rmask[4*h+3:4*h]),
      .rvfi_mem_wmask(rvfi_mem_wmask[4*h+3:4*h]),
      .rvfi_mem_rdata(rvfi_mem_rdata[32*h+31:32*h]),
      .rvfi_mem_wdata(rvfi_mem_wdata[32*h+31:32*h]),
      // The CSR channels are riscv-formal's and no harness here reads them.
      .rvfi_csr_mcycle_rmask(), .rvfi_csr_mcycle_wmask(),
      .rvfi_csr_mcycle_rdata(), .rvfi_csr_mcycle_wdata(),
      .rvfi_csr_minstret_rmask(), .rvfi_csr_minstret_wmask(),
      .rvfi_csr_minstret_rdata(), .rvfi_csr_minstret_wdata(),
      .rvfi_csr_mscratch_rmask(), .rvfi_csr_mscratch_wmask(),
      .rvfi_csr_mscratch_rdata(), .rvfi_csr_mscratch_wdata()
     `endif
    );
  end

  // One storage, one fetch window per hart. A load is answered out of window
  // 0's copy and steals only that window; a write stalls every window.
  imemory #(
    .ROM_WORDS(ROM_WORDS),
    .NHARTS(NHARTS),
    .INIT_EVEN(INIT_EVEN),
    .INIT_ODD(INIT_ODD)
  ) imem (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(imem_mem_rdata),
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault)
  );

  // `atomic_addr` is the one pair of this memory's ports that is per hart:
  // every hart asks about its own instruction in decode at the same time, where
  // the bus-side ports describe the one transaction on the bus.
  memory #(.NHARTS(NHARTS)) dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dmem_mem_rdata),
    .reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported)
  );

  // One `mtime`, one `mtimecmp` and one `mtip` line per hart, in a window the
  // single-hart map already reserves.
  timer #(.NHARTS(NHARTS)) mtimer (
    .clk(clk),
    .reset(&reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(timer_mem_rdata),
    .mtip(irq_timer)
  );
endmodule // littledual
