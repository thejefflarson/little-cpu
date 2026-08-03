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
  // ADR-0054: the fetch address one cycle early, for a synchronous memory.
  // Unread here -- this environment answers `imem_data` freely against
  // `imem_addr` in the same cycle -- but connected rather than left dangling,
  // so every instantiation of the core names every port.
  logic [31:0] imem_addr_next;
  // ADR-0059's steal, tied low: this environment answers `imem_data` freely
  // against `imem_addr` in the same cycle and models no memory, so a steal
  // would only push the five goals further out. formal/wrapper.v is where the
  // arbiter is transcribed and driven.
  logic        mem_ren;
  logic        fetch_stall = 1'b0;

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
