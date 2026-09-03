`timescale 1 ns / 1 ps
`default_nettype none
// Two harts on one text storage, data RAM, timer and bus arbiter. Two is the
// number, not a default: rtl/busarbiter.v is proved for exactly two. Its grant
// is registered, so a hart asks from decode a cycle before its transaction.
module littledual #(
  parameter integer ROM_WORDS = 2048,
  parameter INIT_EVEN = "",
  parameter INIT_ODD  = ""
) (
  input  logic       clk,
  // One per hart, low hart first.
  input  logic [1:0] reset,
  output logic [1:0] trap
 `ifdef RISCV_FORMAL
  ,
  // One retire channel per hart, packed low hart first.
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
  output logic [63:0]  rvfi_mem_wdata,
 `ifdef RISCV_FORMAL_MEM_FAULT
  output logic [1:0]   rvfi_mem_fault,
 `endif
  // Which harts have a transaction on the shared bus this cycle.
  output logic [1:0]   probe_bus_active,
  output logic [63:0]  probe_imem_addr
 `endif
);
  localparam int NHARTS = 2;

  // Per-hart nets, packed low hart first.
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

  // At most one hart publishes a memory instruction per cycle, so at most one
  // has a transaction out and three of the four bus outputs join with an OR.
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren, mem_reservable;
  logic [31:0] imem_mem_rdata, dmem_mem_rdata, timer_mem_rdata;
  assign mem_addr  = hart_mem_addr[31:0] | hart_mem_addr[63:32];
  assign mem_wstrb = hart_mem_wstrb[3:0] | hart_mem_wstrb[7:4];
  assign mem_ren   = hart_mem_ren[0]     | hart_mem_ren[1];

  // `mem_wdata` cannot be ORed: rtl/accessor.v publishes rs2 on it for every
  // issuing instruction, not only a store. ORed, one hart's rs2 lands in the
  // other's store whenever a non-memory instruction issues beside it, with
  // neither a read enable nor a strobe raised to say so.
  assign mem_wdata = |hart_mem_wstrb[3:0] ? hart_mem_wdata[31:0]
                                          : hart_mem_wdata[63:32];
  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata | timer_mem_rdata;

 `ifdef RISCV_FORMAL
  for (genvar h = 0; h < NHARTS; h++) begin : l_probe
    assign probe_bus_active[h] = hart_mem_ren[h] || |hart_mem_wstrb[4*h+3:4*h];
  end
  assign probe_imem_addr = imem_addr;
 `endif

  busarbiter arbiter (
    .clk(clk),
    // In reset only while both harts are; a hart in reset asks for nothing.
    .reset(&reset),
    .request(bus_request),
    .mem_lock(mem_lock),
    .grant(grant)
  );

  for (genvar h = 0; h < NHARTS; h++) begin : l_hart
    localparam int OTHER = 1 - h;

    // An AMO publishes once and makes two transactions, its read and then its
    // write-back; `mem_lock` is high between them, so the other hart waits
    // through the second even when the arbiter has already granted it.
    assign bus_wait[h] = bus_request[h] && (!grant[h] || mem_lock[OTHER]);

   `ifdef RISCV_FORMAL
    logic [1:0]  u_mode, u_ixl;
    logic [63:0] u_mcycle_rmask, u_mcycle_wmask, u_mcycle_rdata, u_mcycle_wdata;
    logic [63:0] u_minstret_rmask, u_minstret_wmask, u_minstret_rdata, u_minstret_wdata;
    logic [31:0] u_mscratch_rmask, u_mscratch_wmask, u_mscratch_rdata, u_mscratch_wdata;
   `endif
   `ifdef RISCV_FORMAL_MEM_FAULT
    logic [3:0]  u_mem_fault_rmask, u_mem_fault_wmask;
   `endif

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
      .mem_rdata(mem_rdata),
      .fetch_stall(fetch_stall[h]),
      .imem_fault(imem_fault[h]),
      .mem_reservable(mem_reservable),
      .atomic_addr(atomic_addr[32*h+31:32*h]),
      .atomic_supported(atomic_supported[h]),
      .bus_wait(bus_wait[h]),
      // Taken from the other hart's own port rather than the shared bus, so a
      // hart never snoops its own store and drops its own reservation.
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
      // Named rather than left empty: test/port_connect_test.py refuses an
      // empty connection at a littlecpu site.
      .rvfi_mode(u_mode),
      .rvfi_ixl(u_ixl),
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
     `ifdef RISCV_FORMAL_MEM_FAULT
      .rvfi_mem_fault(rvfi_mem_fault[h]),
      .rvfi_mem_fault_rmask(u_mem_fault_rmask), .rvfi_mem_fault_wmask(u_mem_fault_wmask),
     `endif
      .rvfi_csr_mcycle_rmask(u_mcycle_rmask), .rvfi_csr_mcycle_wmask(u_mcycle_wmask),
      .rvfi_csr_mcycle_rdata(u_mcycle_rdata), .rvfi_csr_mcycle_wdata(u_mcycle_wdata),
      .rvfi_csr_minstret_rmask(u_minstret_rmask), .rvfi_csr_minstret_wmask(u_minstret_wmask),
      .rvfi_csr_minstret_rdata(u_minstret_rdata), .rvfi_csr_minstret_wdata(u_minstret_wdata),
      .rvfi_csr_mscratch_rmask(u_mscratch_rmask), .rvfi_csr_mscratch_wmask(u_mscratch_wmask),
      .rvfi_csr_mscratch_rdata(u_mscratch_rdata), .rvfi_csr_mscratch_wdata(u_mscratch_wdata)
     `endif
    );
  end

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

  // `atomic_addr` is per hart because every hart asks about its own
  // decode-stage instruction at once; the bus ports carry the one granted
  // transaction.
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
