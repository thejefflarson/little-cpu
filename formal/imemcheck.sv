`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_ALIGNED_MEM
`include "rvfi_macros.vh"
`include "rvfi_channel.sv"
`include "rvfi_imem_check.sv"

module testbench (
  input clk
);
  logic reset = 1;
  logic trap;

  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES

  logic [31:0] imem_addr;
  logic [15:0] imem_data;

  rvfi_imem_check checker_inst (
    .clock(clk),
    .reset(reset),
    .enable(1'b1),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    `RVFI_CONN
  );

  logic [31:0] uut_imem_addr;
  logic [31:0] uut_imem_data;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;

  // No mem_valid/mem_ready to gate on: rtl/fetcher.v:15/25 drives
  // imem_addr = pc and out.instr = imem_data combinationally, unconditionally,
  // every non-reset cycle (CLAUDE.md invariant 1 -- fetch never stalls or
  // waits). checker_inst's imem_addr/imem_data are `rand_const_reg`s: fixed
  // for the whole trace, not resampled per cycle, so this comparison needs
  // no cycle-history bookkeeping either -- whenever the DUT's fetch this
  // cycle targets the checker's fixed address, its data must agree,
  // full stop.
  always_comb begin
    if (!reset) begin
      if (uut_imem_addr == imem_addr)
        assume(uut_imem_data[15:0] == imem_data);
      if (uut_imem_addr + 2 == imem_addr)
        assume(uut_imem_data[31:16] == imem_data);
    end
  end

  littlecpu uut (
    .clk(clk),
    .reset(reset),
    .imem_addr(uut_imem_addr),
    .imem_data(uut_imem_data),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap),
    `RVFI_CONN
  );
endmodule
