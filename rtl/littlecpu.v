`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module littlecpu #(
  parameter logic [31:0] HART_ID = 32'd0,
  // Restated because a module cannot read another's parameters.
  parameter integer      LS_TEXT_WORDS = 2048,
  parameter logic [31:0] LS_RAM_BASE   = 32'h0001_0000,
  parameter integer      LS_RAM_WORDS  = 16384,
  parameter logic [31:0] LS_TIMER_BASE = 32'h0002_0000,
  parameter logic [31:0] LS_UART_BASE  = 32'h0002_0020,
  parameter logic [31:0] LS_FLASH_BASE = 32'h0002_0028
) (
  input  logic clk,
  input  logic reset,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  // The value `imem_addr` takes on the next edge, so a synchronous memory can latch it a
  // cycle early.
  output logic [31:0] imem_addr_next,
  // A text-range load or store takes the fetch port; the fetch that lost it comes back as
  // `fetch_stall`.
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  output logic        mem_ren,
  input  logic [31:0] mem_rdata,
  input  logic        fetch_stall,
  input  logic        imem_fault,
  input  logic        mem_reservable,
  // Arrives with the address, so decode commits the fault there.
  output logic [31:0] atomic_addr,
  input  logic        atomic_supported,
  // `snoop_*` is that initiator's write, which clears a reservation on its word.
  input  logic        bus_wait,
  input  logic        snoop_write,
  input  logic [31:0] snoop_addr,
  output logic        mem_lock,
  // A cycle before the transaction; the platform ANDs it against its grant.
  output logic        bus_request,
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
  // The comma leads: yosys accepts a trailing one and iverilog does not.
  ,
  output logic [31:0] rvfi_csr_mcause_rmask,
  output logic [31:0] rvfi_csr_mcause_wmask,
  output logic [31:0] rvfi_csr_mcause_rdata,
  output logic [31:0] rvfi_csr_mcause_wdata
  `endif
  `endif //  `ifdef RISCV_FORMAL
  );
  // The region arithmetic would go on classifying addresses against a window that is not
  // a power of two on its own boundary.
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
  if (|LS_FLASH_BASE[2:0]) begin : l_ls_flash_base_aligned
    $fatal(1, "littlecpu: LS_FLASH_BASE must be 8-byte aligned");
  end

  decoder_output decoder_out;
  executor_output executor_out;
  logic divider_stalled;
  // A store writes no register, so the scoreboard cannot see one still in the accessor.
  logic accessor_out_valid;
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
  logic [31:0] csr_trap_cause, csr_trap_epc, csr_trap_tval;
  logic [31:0] csr_mtvec, csr_mepc;
  logic        csr_interrupt_pending;
 `ifdef RISCV_FORMAL
  rvfi_csr64 csr_rvfi_mcycle, csr_rvfi_minstret;
  rvfi_csr32 csr_rvfi_mscratch;
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  rvfi_csr32 csr_rvfi_mcause;
  `endif
  // Instrumentation only. `probe_rs1` is the issuing instruction's own rs1, not the pair
  // presented to the register file.
  logic [4:0] probe_rs1;
  logic       probe_ls_issuing;
 `endif

  decoder #(
    .LS_TEXT_WORDS(LS_TEXT_WORDS),
    .LS_RAM_BASE(LS_RAM_BASE),
    .LS_RAM_WORDS(LS_RAM_WORDS),
    .LS_TIMER_BASE(LS_TIMER_BASE),
    .LS_UART_BASE(LS_UART_BASE),
    .LS_FLASH_BASE(LS_FLASH_BASE)
  ) decoder(
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
    .trap_tval(csr_trap_tval),
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
    .trap_tval(csr_trap_tval),
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
  // Launches from `decoder_out` a stage early, on the one cycle the executor takes it;
  // any other cycle would present a store twice.
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
  assign rvfi_halt = 1'b0;
  assign rvfi_mode = 2'd3;
  assign rvfi_ixl = 2'd1;

  // Two counters for `make cycles`, unread by the core.
  localparam int LS_BLOCK_BITS = 11;              // 2 KB: a 12-bit offset's reach
  localparam int LS_BLOCK_NUM_BITS = 32 - LS_BLOCK_BITS;
  localparam logic [31:0] LS_TIMER_BYTES = 32'd16;
  localparam logic [31:0] LS_UART_BYTES  = 32'd8;
  localparam logic [31:0] LS_FLASH_BYTES = 32'd8;

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
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_FLASH_LO = LS_FLASH_BASE >> LS_BLOCK_BITS;
  localparam logic [LS_BLOCK_NUM_BITS-1:0] LS_FLASH_HI =
    (LS_FLASH_BASE + LS_FLASH_BYTES - 1) >> LS_BLOCK_BITS;

  logic [LS_BLOCK_NUM_BITS-1:0] ls_block;
  assign ls_block = reg_rs1[31:LS_BLOCK_BITS];

  // Not a shared function: iverilog derives a continuous assign's sensitivity from the
  // call's arguments.
  logic ls_at_edge;
  assign ls_at_edge =
    ls_block == LS_TEXT_LO - 1'b1  || ls_block == LS_TEXT_LO  ||
    ls_block == LS_TEXT_HI         || ls_block == LS_TEXT_HI + 1'b1 ||
    ls_block == LS_RAM_LO - 1'b1   || ls_block == LS_RAM_LO   ||
    ls_block == LS_RAM_HI          || ls_block == LS_RAM_HI + 1'b1 ||
    ls_block == LS_TIMER_LO - 1'b1 || ls_block == LS_TIMER_LO ||
    ls_block == LS_TIMER_HI        || ls_block == LS_TIMER_HI + 1'b1 ||
    ls_block == LS_UART_LO - 1'b1  || ls_block == LS_UART_LO  ||
    ls_block == LS_UART_HI         || ls_block == LS_UART_HI + 1'b1 ||
    ls_block == LS_FLASH_LO - 1'b1 || ls_block == LS_FLASH_LO ||
    ls_block == LS_FLASH_HI        || ls_block == LS_FLASH_HI + 1'b1;

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
