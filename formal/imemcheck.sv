`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_ALIGNED_MEM
`include "rvfi_macros.vh"
`include "rvfi_channel.sv"
`include "rvfi_imem_check.sv"

module testbench (
  input clk,
  input   mem_ready,
  output  mem_valid,
  output  mem_instr,
  output [31:0] mem_addr,
  output [31:0] mem_wdata,
  output [3:0]  mem_wstrb,
  input  [31:0] mem_rdata
);
  reg resetn = 0;
  wire trap;

  always @(posedge clk)
    resetn <= 1;

  `RVFI_WIRES

  wire [31:0] imem_addr;
  wire [15:0] imem_data;

  rvfi_imem_check checker_inst (
    .clock(clk),
    .reset(!resetn),
    .enable(1'b1),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    `RVFI_CONN
  );

  always @* begin
    if (resetn && mem_valid && mem_ready) begin
      if (mem_addr == imem_addr)
        assume(mem_rdata[15:0] == imem_data);
      if (mem_addr + 2 == imem_addr)
        assume(mem_rdata[31:16] == imem_data);
    end
  end

  riscv uut (
    .clk(clk),
    .reset(resetn),
    .trap(trap),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    `RVFI_CONN
  );

  reg [4:0] mem_wait = 0;
  always @(posedge clk) begin
    mem_wait <= {mem_wait, mem_valid && !mem_ready};
  end
endmodule
