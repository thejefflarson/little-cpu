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
  // The fetch address one cycle early.
  logic [31:0] imem_addr_next;
  // The address the core publishes for the platform to decode.
  logic [31:0] atomic_addr;
  // The lock an arbiter would read.
  logic mem_lock;
  logic bus_request;
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
    .imem_fault(1'b0),
    .mem_reservable(1'b1),
    .atomic_addr(atomic_addr),
    .atomic_supported(1'b1),
    // Tied off; formal/check-multihart-tie-off.py enforces it.
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    .bus_request(bus_request),
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
