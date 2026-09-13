`timescale 1 ns / 1 ps
// A bare wrapper around `riscv` for nano_exec_cxxrtl.cc: the bus is tied off since every vector pokes `cpu_state`/`instr`/`regs` directly from C++ instead.
module nano_exec_tb (
  input logic clk,
  input logic reset
);
  logic        mem_valid;
  logic        mem_instr;
  logic        trap;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;

  riscv dut (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(1'b0),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(32'b0),
    .trap(trap)
  );
endmodule
