`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_ALIGNED_MEM
`include "rvfi_macros.vh"
`include "rvfi_channel.sv"
`include "rvfi_dmem_check.sv"

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
  logic reset = 1;
  logic trap;

  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES

  logic [31:0] dmem_addr;
  logic [31:0] dmem_data;

  rvfi_dmem_check checker_inst (
    .clock(clk),
    .reset(reset),
    .enable(1'b1),
    .dmem_addr(dmem_addr),
    `RVFI_CONN
  );

  always_ff @(posedge clk) begin
    if (!reset && mem_valid && mem_ready && mem_addr == dmem_addr) begin
      if (mem_wstrb[0]) dmem_data[ 7: 0] <= mem_wdata[ 7: 0];
      if (mem_wstrb[1]) dmem_data[15: 8] <= mem_wdata[15: 8];
      if (mem_wstrb[2]) dmem_data[23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) dmem_data[31:24] <= mem_wdata[31:24];
    end
  end

  always_comb begin
    if (!reset && mem_valid && mem_ready && mem_addr == dmem_addr && !mem_wstrb)
      assume(dmem_data == mem_rdata);
  end

  riscv uut (
    .clk(clk),
    .reset(reset),
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
endmodule
