module testbench (
  input var clk,
  output logic        mem_valid,
  output logic        mem_instr,
  input  logic        mem_ready,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
);
  logic reset = 1;
  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES
  logic trap;

  riscv wrapper (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
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
