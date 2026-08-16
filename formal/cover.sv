module testbench (
  input var clk,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata
);
  logic reset = 1;
  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES
  logic trap;
  // The fetch address one cycle early. Unread here -- this environment answers
  // `imem_data` freely against `imem_addr` in the same cycle -- but connected
  // rather than left dangling, so every instantiation of the core names every
  // port.
  logic [31:0] imem_addr_next;
  // The address the core publishes for the platform to decode. Unread here:
  // `atomic_supported` is tied high, so no atomic can fault in this task.
  logic [31:0] atomic_addr;
  logic        mem_ren;
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
    // address it answers is one a reservation may be held at, and one an atomic
    // is answered at.
    .mem_reservable(1'b1),
    .atomic_addr(atomic_addr),
    .atomic_supported(1'b1),
    // Tied off; formal/check-interrupt-tie-off.py enforces it. formal/wrapper.v
    // carries the reason the riscv-formal side of the tree runs with no
    // interrupt in the trace.
    .irq_timer(1'b0),
    .trap(trap),
    `RVFI_CONN
  );
  integer mem_read = 0;
  integer mem_write = 0;
  integer long_insns = 0;
  integer comp_insns = 0;
  always_ff @(posedge clk) begin
    if(!reset && rvfi_valid) begin
      if(rvfi_mem_rmask)
        mem_read <= mem_read + 1;
      if(rvfi_mem_wmask)
        mem_write <= mem_write + 1;
      if(rvfi_insn[1:0] == 3)
        long_insns <= long_insns + 1;
      if(rvfi_insn[1:0] != 3)
        comp_insns <= comp_insns + 1;
    end
  end // always_ff @ (posedge clk)

  cover property (mem_read);
  cover property (mem_write);
  cover property (long_insns);
  cover property (comp_insns);
  cover property (mem_read >= 2 && mem_write >= 2 && long_insns >= 2 && comp_insns >= 2);
endmodule
