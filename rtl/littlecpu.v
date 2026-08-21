`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module littlecpu #(
  // This hart's mhartid, handed straight to rtl/csrs.v. Nothing else in the
  // core reads it: it is a self-description register, so a wrong value changes
  // no cycle and no retire, and test/csr_tb.v is the only place the read is
  // graded away from the default.
  parameter logic [31:0] HART_ID = 32'd0,
  // The data bus's memory map, read by the load/store locality counters under
  // `RISCV_FORMAL` at the bottom of this file and by nothing else: no port of
  // this module and no cycle of the datapath depends on any of it. The RAM's
  // base and size and the timer's and the UART's bases are rtl/memory.v's,
  // rtl/timer.v's and rtl/uart.v's own parameter defaults, restated here
  // because a module cannot read another module's parameters;
  // test/memmap_test.sh is what compares the copies.
  // The text window's size is the integrator's, because the simulated machine's
  // ROM is deliberately larger than the part's -- so each integrator hands this
  // the same number it hands its `imemory`.
  parameter integer      LS_TEXT_WORDS = 2048,
  parameter logic [31:0] LS_RAM_BASE   = 32'h0001_0000,
  parameter integer      LS_RAM_WORDS  = 16384,
  parameter logic [31:0] LS_TIMER_BASE = 32'h0002_0000,
  parameter logic [31:0] LS_UART_BASE  = 32'h0002_0020
) (
  input  logic clk,
  input  logic reset,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  // The value `imem_addr` takes on the next edge. A synchronous memory latches
  // this on the same edge the PC moves, so its data is ready for the whole of
  // the cycle that needs it; a combinational memory -- test benches,
  // formal/wrapper.v -- leaves it unread and answers `imem_addr` directly.
  output logic [31:0] imem_addr_next,
  // The fetch and data buses share one address space, and the instruction
  // memory arbitrates: a load or store to the text range takes the read port
  // for that cycle and the fetch that lost it comes back as `fetch_stall`.
  // A memory that keeps its two buses separate ties `fetch_stall` low and
  // leaves `mem_ren` unread.
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  output logic        mem_ren,
  input  logic [31:0] mem_rdata,
  input  logic        fetch_stall,
  // The instruction memory has nothing at the address it is answering. It
  // arrives with the word, so decode commits it as an instruction access fault
  // in the cycle it is looking at that word. A memory that answers everywhere
  // ties it low.
  input  logic        imem_fault,
  // The data address this cycle names reservable main memory, so `lr.w` there
  // may hold a reservation. A platform whose memory answers everywhere ties it
  // high; one that ties it low never lets a store-conditional succeed, which is
  // a spurious failure and permitted anywhere.
  input  logic        mem_reservable,
  // The address an atomic in decode would use, and the platform's answer about
  // it. This pair is the fetch bus's shape on the data side: the platform
  // decodes its own map and hands back one bit, and the bit accompanies the
  // address rather than a response, so decode can commit the fault with every
  // other cause. Low means an lr.w there raises cause 5 and an AMO or sc.w
  // cause 7. A platform whose memory answers atomics everywhere ties it high.
  output logic [31:0] atomic_addr,
  input  logic        atomic_supported,
  // The shared-bus surface. `bus_wait` says the bus is somebody else's this
  // cycle: decode publishes nothing, the pc holds, and the instruction asks
  // again next cycle. `snoop_write`/`snoop_addr` are the write another master
  // is making, which clears a reservation on that word. `mem_lock` is high on
  // the cycle an AMO reads and low on the cycle it writes back, which is how an
  // arbiter that registers its grant learns to keep the bus here for the second
  // cycle.
  //
  // A platform with one bus master ties both inputs low and leaves both outputs
  // unread, and only the INPUTS are free that way: they fold before mapping and
  // the cell census does not move. An unread output is still a net for ABC to
  // map around, and these moved the SoC's mapped count by tens of cells -- so
  // each arrived with a placement sweep rather than with an equal netlist.
  input  logic        bus_wait,
  input  logic        snoop_write,
  input  logic [31:0] snoop_addr,
  output logic        mem_lock,
  // Decode's request for the shared data bus, a cycle before the transaction it
  // asks for. `bus_wait` is what comes back, and the platform -- not this
  // module -- ANDs the two: a hart that is not granted publishes nothing and
  // asks again next cycle.
  output logic        bus_request,
  // The platform's machine-timer line. Registered at its source (rtl/timer.v
  // does it), because inside the core it is one gate away from the fetch loop.
  // A platform with no timer ties it low and the core never takes an interrupt.
  input  logic        irq_timer,
  output logic trap
  `ifdef RISCV_FORMAL
  ,
  output logic rvfi_valid,
  output logic [63:0] rvfi_order,
  output logic [31:0] rvfi_insn,
  output logic rvfi_trap,
  output logic rvfi_halt,
  output logic rvfi_intr,
  output logic [ 1:0] rvfi_mode,
  output logic [ 1:0] rvfi_ixl,
  output logic [ 4:0] rvfi_rs1_addr,
  output logic [ 4:0] rvfi_rs2_addr,
  output logic [31:0] rvfi_rs1_rdata,
  output logic [31:0] rvfi_rs2_rdata,
  output logic [ 4:0] rvfi_rd_addr,
  output logic [31:0] rvfi_rd_wdata,
  output logic [31:0] rvfi_pc_rdata,
  output logic [31:0] rvfi_pc_wdata,
  output logic [31:0] rvfi_mem_addr,
  output logic [ 3:0] rvfi_mem_rmask,
  output logic [ 3:0] rvfi_mem_wmask,
  output logic [31:0] rvfi_mem_rdata,
  output logic [31:0] rvfi_mem_wdata,
  `ifdef RISCV_FORMAL_MEM_FAULT
  // Guarded by the same macro riscv-formal declares these three under, so a
  // harness that does not ask for them does not connect ports that are not in
  // its `RVFI_CONN` either. Both sim legs are such a harness.
  output logic        rvfi_mem_fault,
  output logic [ 3:0] rvfi_mem_fault_rmask,
  output logic [ 3:0] rvfi_mem_fault_wmask,
  `endif
  output logic [63:0] rvfi_csr_mcycle_rmask,
  output logic [63:0] rvfi_csr_mcycle_wmask,
  output logic [63:0] rvfi_csr_mcycle_rdata,
  output logic [63:0] rvfi_csr_mcycle_wdata,
  output logic [63:0] rvfi_csr_minstret_rmask,
  output logic [63:0] rvfi_csr_minstret_wmask,
  output logic [63:0] rvfi_csr_minstret_rdata,
  output logic [63:0] rvfi_csr_minstret_wdata,
  output logic [31:0] rvfi_csr_mscratch_rmask,
  output logic [31:0] rvfi_csr_mscratch_wmask,
  output logic [31:0] rvfi_csr_mscratch_rdata,
  output logic [31:0] rvfi_csr_mscratch_wdata
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  // The comma LEADS, and that is what makes both arms of this `ifdef` legal: the
  // port above cannot carry a trailing one, because with the macro absent it is
  // the last in the list. iverilog and svlint both reject the other spelling and
  // yosys accepts it, so this is a shape only the second frontend catches.
  ,
  output logic [31:0] rvfi_csr_mcause_rmask,
  output logic [31:0] rvfi_csr_mcause_wmask,
  output logic [31:0] rvfi_csr_mcause_rdata,
  output logic [31:0] rvfi_csr_mcause_wdata
  `endif
  `endif //  `ifdef RISCV_FORMAL
  );
  // The shapes rtl/imemory.v, rtl/memory.v, rtl/timer.v and rtl/uart.v each
  // refuse to elaborate at, restated at the site that copies their map. A window
  // that is not a power of two on its own boundary is one no memory here
  // implements, and the block arithmetic below would go on classifying addresses
  // against it without a word. `make window-test` forces all five of these.
  localparam int LS_TEXT_ADDR_BITS = $clog2(LS_TEXT_WORDS);
  localparam int LS_RAM_ADDR_BITS  = $clog2(LS_RAM_WORDS);
  if (LS_TEXT_WORDS != (1 << LS_TEXT_ADDR_BITS)) begin : l_ls_text_words_power_of_two
    $fatal(1, "littlecpu: LS_TEXT_WORDS must be a power of two");
  end
  if (LS_RAM_WORDS != (1 << LS_RAM_ADDR_BITS)) begin : l_ls_ram_words_power_of_two
    $fatal(1, "littlecpu: LS_RAM_WORDS must be a power of two");
  end
  if (|LS_RAM_BASE[LS_RAM_ADDR_BITS+1:0]) begin : l_ls_ram_base_aligned
    $fatal(1, "littlecpu: LS_RAM_BASE must be aligned to LS_RAM_WORDS words");
  end
  if (|LS_TIMER_BASE[3:0]) begin : l_ls_timer_base_aligned
    $fatal(1, "littlecpu: LS_TIMER_BASE must be 16-byte aligned");
  end
  if (|LS_UART_BASE[2:0]) begin : l_ls_uart_base_aligned
    $fatal(1, "littlecpu: LS_UART_BASE must be 8-byte aligned");
  end

  // Declared up here rather than next to the module that drives each one. Later
  // stages feed signals back to earlier ones, so each of these is read by a
  // module written above its driver, and iverilog wants the name first.
  decoder_output decoder_out;
  executor_output executor_out;
  logic divider_stalled;
  // A store writes no register, so the decode scoreboard never sees one. The
  // decoder needs this to tell a store is still in the accessor before it lets
  // a CSR access issue.
  logic accessor_out_valid;
  // High for one cycle per trap, not a level. The test benches watch it to catch
  // a trap taken before the handler address is installed: mtvec resets to 0 and
  // test/asm/sections.lds links .text at 0, so such a trap restarts the program
  // silently and would otherwise look like a timeout.
  logic decoder_trap_entry;
  assign trap = decoder_trap_entry;
  logic  [31:0] pc;
  logic  [31:0] next_pc;
  fetcher_output fetcher_out;
  fetcher fetcher(
    .clk(clk),
    .reset(reset),
    .pc(pc),
    .next_pc(next_pc),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .out(fetcher_out),
    .imem_addr(imem_addr),
    .imem_addr2(imem_addr2),
    .imem_addr_next(imem_addr_next)
  );

  logic [31:0] reg_rs1, reg_rs2, wdata;
  logic [4:0]  read_rs1, read_rs2;
  logic [4:0]  waddr;
  logic        wen;
  regfile regfile(
    .clk(clk),
    .rs1(read_rs1),
    .rs2(read_rs2),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
  );

  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen;
  logic [31:0] csr_wdata, csr_rdata;
  logic        csr_implemented;
  logic        csr_instret;
  logic        csr_trap_entry, csr_mret_entry;
  logic [31:0] csr_trap_cause, csr_trap_epc;
  logic [31:0] csr_mtvec, csr_mepc;
  logic        csr_interrupt_pending;
 `ifdef RISCV_FORMAL
  rvfi_csr64 csr_rvfi_mcycle, csr_rvfi_minstret;
  rvfi_csr32 csr_rvfi_mscratch;
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  rvfi_csr32 csr_rvfi_mcause;
  `endif
  // Instrumentation only, both of them: the register number this instruction
  // reads as its base, and the cycle it issues on. The datapath asks the
  // register file for a different pair on an issuing cycle, so `probe_rs1` is
  // the decoded number rather than the presented one.
  logic [4:0] probe_rs1;
  logic       probe_ls_issuing;
 `endif

  decoder decoder(
    .clk(clk),
    .reset(reset),
    .in(fetcher_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stalled),
    .fetch_stall(fetch_stall),
    .bus_wait(bus_wait),
    .bus_request(bus_request),
    .imem_fault(imem_fault),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(csr_mtvec),
    .mepc(csr_mepc),
    .interrupt_pending(csr_interrupt_pending),
   `ifdef RISCV_FORMAL
    .csr_rvfi_mcycle(csr_rvfi_mcycle),
    .csr_rvfi_minstret(csr_rvfi_minstret),
    .csr_rvfi_mscratch(csr_rvfi_mscratch),
    `ifdef RISCV_FORMAL_CSR_MCAUSE
    .csr_rvfi_mcause(csr_rvfi_mcause),
    `endif
    .rs1(probe_rs1),
    .probe_ls_issuing(probe_ls_issuing),
   `endif
    .pc(pc),
    .next_pc(next_pc),
    .read_rs1(read_rs1),
    .read_rs2(read_rs2),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .instret(csr_instret),
    .trap_entry(decoder_trap_entry),
    .trap_cause(csr_trap_cause),
    .trap_epc(csr_trap_epc),
    .mret_entry(csr_mret_entry),
    .out(decoder_out)
  );

  csrs #(.HART_ID(HART_ID)) csrs(
    .clk(clk),
    .reset(reset),
    .addr(csr_addr),
    .ren(csr_ren),
    .wen(csr_wen),
    .wdata(csr_wdata),
    .instret(csr_instret),
    .trap_entry(decoder_trap_entry),
    .trap_cause(csr_trap_cause),
    .trap_epc(csr_trap_epc),
    .mret_entry(csr_mret_entry),
    .irq_timer(irq_timer),
    .rdata(csr_rdata),
    .implemented(csr_implemented),
    .mtvec_value(csr_mtvec),
    .mepc_value(csr_mepc),
    .interrupt_pending(csr_interrupt_pending)
   `ifdef RISCV_FORMAL
    ,
    .rvfi_mcycle(csr_rvfi_mcycle),
    .rvfi_minstret(csr_rvfi_minstret),
    .rvfi_mscratch(csr_rvfi_mscratch)
    `ifdef RISCV_FORMAL_CSR_MCAUSE
    , .rvfi_mcause(csr_rvfi_mcause)
    `endif
   `endif
  );

  executor executor(
    .clk(clk),
    .reset(reset),
    .in(decoder_out),
    .out(executor_out),
    .stalled(divider_stalled)
  );

  accessor_output accessor_out;
  assign accessor_out_valid = accessor_out.valid;
  // The bus transaction goes out from `decoder_out`, a stage ahead of the
  // struct whose answer it becomes. `divider_stalled` low is exactly the cycle
  // the executor takes that instruction, and presenting it on any other cycle
  // would drive the bus twice for one store.
  accessor accessor(
    .clk(clk),
    .reset(reset),
    .launch(decoder_out),
    .launch_taken(!divider_stalled),
    .in(executor_out),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_wdata(mem_wdata),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .mem_reservable(mem_reservable),
    .snoop_write(snoop_write),
    .snoop_addr(snoop_addr),
    .mem_lock(mem_lock),
    .out(accessor_out)
  );

  writeback writeback(
    .clk(clk),
    .reset(reset),
    .in(accessor_out),
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
   `ifdef RISCV_FORMAL
    ,
    .rvfi_valid(rvfi_valid),
    .rvfi_trap(rvfi_trap),
    .rvfi_intr(rvfi_intr),
    .rvfi_order(rvfi_order),
    .rvfi_insn(rvfi_insn),
    .rvfi_rs1_addr(rvfi_rs1_addr),
    .rvfi_rs2_addr(rvfi_rs2_addr),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_rd_addr(rvfi_rd_addr),
    .rvfi_rd_wdata(rvfi_rd_wdata),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_pc_wdata(rvfi_pc_wdata),
    .rvfi_mem_addr(rvfi_mem_addr),
    .rvfi_mem_rmask(rvfi_mem_rmask),
    .rvfi_mem_wmask(rvfi_mem_wmask),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .rvfi_mem_wdata(rvfi_mem_wdata),
   `ifdef RISCV_FORMAL_MEM_FAULT
    .rvfi_mem_fault(rvfi_mem_fault),
    .rvfi_mem_fault_rmask(rvfi_mem_fault_rmask),
    .rvfi_mem_fault_wmask(rvfi_mem_fault_wmask),
   `endif
    .rvfi_csr_mcycle_rmask(rvfi_csr_mcycle_rmask),
    .rvfi_csr_mcycle_wmask(rvfi_csr_mcycle_wmask),
    .rvfi_csr_mcycle_rdata(rvfi_csr_mcycle_rdata),
    .rvfi_csr_mcycle_wdata(rvfi_csr_mcycle_wdata),
    .rvfi_csr_minstret_rmask(rvfi_csr_minstret_rmask),
    .rvfi_csr_minstret_wmask(rvfi_csr_minstret_wmask),
    .rvfi_csr_minstret_rdata(rvfi_csr_minstret_rdata),
    .rvfi_csr_minstret_wdata(rvfi_csr_minstret_wdata),
    .rvfi_csr_mscratch_rmask(rvfi_csr_mscratch_rmask),
    .rvfi_csr_mscratch_wmask(rvfi_csr_mscratch_wmask),
    .rvfi_csr_mscratch_rdata(rvfi_csr_mscratch_rdata),
    .rvfi_csr_mscratch_wdata(rvfi_csr_mscratch_wdata)
    `ifdef RISCV_FORMAL_CSR_MCAUSE
    ,
    .rvfi_csr_mcause_rmask(rvfi_csr_mcause_rmask),
    .rvfi_csr_mcause_wmask(rvfi_csr_mcause_wmask),
    .rvfi_csr_mcause_rdata(rvfi_csr_mcause_rdata),
    .rvfi_csr_mcause_wdata(rvfi_csr_mcause_wdata)
    `endif
   `endif
  );

 `ifdef RISCV_FORMAL
  // These three never change. There is one privilege mode, registers are 32
  // bits wide and the core cannot be halted, so none of them depends on which
  // instruction just finished. `rvfi_intr` does, and comes out of writeback
  // with the rest of the retire.
  assign rvfi_halt = 1'b0;
  assign rvfi_mode = 2'd3;
  assign rvfi_ixl = 2'd1;

  // Two questions about issuing loads and stores, counted rather than argued.
  // Between them they price a whole family of load/store region tests in one
  // `make cycles` run, and both have come out far from what reading the code
  // suggested. test/stall_report.py prints them under the cycle table; nothing
  // here is read by the core, and outside a `RISCV_FORMAL` build none of it
  // exists at all.
  //
  // EDGE PROXIMITY. A 12-bit offset reaches 2 KB either way, so an access whose
  // base register sits in the 2 KB block beside a region's first or last block
  // may cross that edge while one a block further in cannot. That is the set a
  // region test answered from the base register alone would have to stall on.
  //
  // BYPASS COINCIDENCE. A writeback commits to the base register in the issue
  // cycle, through the register file's write-through bypass. Anything computed
  // about that register's value a cycle earlier describes the value before the
  // write, so this is the floor on what a design precomputing during the
  // operand-fetch cycle would have to spend a cycle redoing.
  localparam int LS_BLOCK_BITS = 11;              // 2 KB: a 12-bit offset's reach
  localparam int LS_BLOCK_NUM_BITS = 32 - LS_BLOCK_BITS;
  // rtl/timer.v's four words and rtl/uart.v's two, the regions smaller than a
  // block. Both sit inside the same block today, so yosys folds the UART's four
  // comparisons into the timer's; they are written out because what makes them
  // equal is where the UART happens to be, and a region that moved would need
  // them.
  localparam logic [31:0] LS_TIMER_BYTES = 32'd16;
  localparam logic [31:0] LS_UART_BYTES  = 32'd8;

  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_TEXT_LO = '0;
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_TEXT_HI =
    (LS_TEXT_WORDS * 4 - 1) >> LS_BLOCK_BITS;
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_RAM_LO = LS_RAM_BASE >> LS_BLOCK_BITS;
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_RAM_HI =
    (LS_RAM_BASE + LS_RAM_WORDS * 4 - 1) >> LS_BLOCK_BITS;
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_TIMER_LO = LS_TIMER_BASE >> LS_BLOCK_BITS;
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_TIMER_HI =
    (LS_TIMER_BASE + LS_TIMER_BYTES - 1) >> LS_BLOCK_BITS;
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_UART_LO = LS_UART_BASE >> LS_BLOCK_BITS;
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_UART_HI =
    (LS_UART_BASE + LS_UART_BYTES - 1) >> LS_BLOCK_BITS;

  logic [LS_BLOCK_NUM_BITS-1:0] ls_block;
  assign ls_block = reg_rs1[31:LS_BLOCK_BITS];

  // Four blocks per region: its first and its last, and the block outside each
  // of those. Written out rather than folded into a function the four regions
  // share, because iverilog builds a continuous assign's sensitivity list from
  // the call's arguments. `1'b1` and not `1`: an integer literal would widen the
  // whole comparison to 32 bits, and the block below zero has to come out as the
  // block at the top of memory, which a negative offset really does reach.
  logic ls_at_edge;
  assign ls_at_edge =
    ls_block == LS_TEXT_LO - 1'b1  || ls_block == LS_TEXT_LO  ||
    ls_block == LS_TEXT_HI         || ls_block == LS_TEXT_HI + 1'b1 ||
    ls_block == LS_RAM_LO - 1'b1   || ls_block == LS_RAM_LO   ||
    ls_block == LS_RAM_HI          || ls_block == LS_RAM_HI + 1'b1 ||
    ls_block == LS_TIMER_LO - 1'b1 || ls_block == LS_TIMER_LO ||
    ls_block == LS_TIMER_HI        || ls_block == LS_TIMER_HI + 1'b1 ||
    ls_block == LS_UART_LO - 1'b1  || ls_block == LS_UART_LO  ||
    ls_block == LS_UART_HI         || ls_block == LS_UART_HI + 1'b1;

  // `wen` is already low for a write to x0 and `waddr` is zero when it is low,
  // so a match here is always a real write to a real base register.
  logic ls_at_bypass;
  assign ls_at_bypass = wen && waddr == probe_rs1;

  logic [31:0] probe_ls_issues, probe_ls_edges, probe_ls_bypasses;
  always_ff @(posedge clk) begin
    if (reset) begin
      probe_ls_issues   <= 32'd0;
      probe_ls_edges    <= 32'd0;
      probe_ls_bypasses <= 32'd0;
    end else if (probe_ls_issuing) begin
      probe_ls_issues <= probe_ls_issues + 32'd1;
      if (ls_at_edge)    probe_ls_edges    <= probe_ls_edges + 32'd1;
      if (ls_at_bypass)  probe_ls_bypasses <= probe_ls_bypasses + 32'd1;
    end
  end
 `endif
endmodule
