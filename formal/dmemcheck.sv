`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_ALIGNED_MEM
`include "rvfi_macros.vh"
`include "rvfi_channel.sv"
`include "rvfi_dmem_check.sv"

module testbench (
  input clk
);
  logic reset = 1;
  logic trap;

  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES

  logic [31:0] dmem_addr;

  rvfi_dmem_check checker_inst (
    .clock(clk),
    .reset(reset),
    .enable(1'b1),
    .dmem_addr(dmem_addr),
    `RVFI_CONN
  );

  logic [31:0] imem_addr;
  logic [31:0] imem_data;
  logic [31:0] imem_addr2;
  logic [31:0] imem_data2;
  logic [31:0] imem_addr_next;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren;
  logic [31:0] mem_rdata;
  logic        fetch_stall;

  imem_arbiter arbiter (
    .clock(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .fetch_stall(fetch_stall),
    .text_write()
  );

  // rvfi_dmem_check does its own tracking and does not need this. What this is
  // for: mem_rdata is otherwise a free input every cycle, and no design could
  // satisfy the check against a memory that answers anything it likes. It reads
  // the bus directly because rvfi_dmem_check only sees cycles that retire a
  // load or a store. Watching mem_wstrb alone is enough, because rtl/accessor.v
  // sets it to 0 on every cycle that is not a store.
  logic [31:0] dmem_data;
  always_ff @(posedge clk) begin
    if (!reset && mem_addr == dmem_addr) begin
      if (mem_wstrb[0]) dmem_data[ 7: 0] <= mem_wdata[ 7: 0];
      if (mem_wstrb[1]) dmem_data[15: 8] <= mem_wdata[15: 8];
      if (mem_wstrb[2]) dmem_data[23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) dmem_data[31:24] <= mem_wdata[31:24];
    end
  end

  // Assumed: the bus returns whatever was last written to that address, one
  // cycle after the request.
  //
  // test/mem_tb.v checks that rtl/memory.v really does this, but only inside
  // the mapped region. Outside it the real memory drops the write and reads
  // zero, where this keeps the value. dmem_addr is a free 32-bit input, so over
  // most of the address space this assumes a memory the core does not have.
  //
  // It reaches only the one rvfi_dmem_check assertion below. It constrains an
  // input to the core rather than an output, so it can narrow what the solver
  // may try but it cannot excuse a bug.
  //
  // Compared against last cycle's mem_addr, because that is the cycle the data
  // arriving now was asked for.
  //
  // An AMO puts its write on the bus at the same address on the very cycle its
  // read is answered, so the block above and this one fire in the same time
  // step there. The read still sees the old word, and what makes that true is
  // that the shadow's update is non-blocking; neither may become blocking.
  always_ff @(posedge clk) begin
    if (!reset && $past(mem_addr) == dmem_addr && !$past(mem_wstrb))
      assume(dmem_data == mem_rdata);
  end

  littlecpu uut (
    .clk(clk),
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
    // Tied off: this task's memory model answers every address, so there is no
    // window for a fetch to fall outside of.
    .imem_fault(1'b0),
    // Tied off high: this task's memory model answers every address, so every
    // address it answers is one a reservation may be held at.
    .mem_reservable(1'b1),
    // Tied off; formal/check-interrupt-tie-off.py enforces it. formal/wrapper.v
    // carries the reason the riscv-formal side of the tree runs with no
    // interrupt in the trace.
    .irq_timer(1'b0),
    .trap(trap),
    `RVFI_CONN
  );
endmodule
