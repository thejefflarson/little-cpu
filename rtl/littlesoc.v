`timescale 1 ns / 1 ps
`default_nettype none
module littlesoc (
  input  clk,
  input  reset,
  output flash_cs,
  output flash_clk,
  inout  flash_io0,
  inout  flash_io1,
  inout  flash_io2,
  inout  flash_io3
);
  logic int_flash_clk;
  assign flash_clk = int_flash_clk;
  // TODO SPI mem
  always_ff @(posedge clk) begin
   if (reset) begin
     flash_cs <= 0;
     int_flash_clk <= 0;
   end else begin
     flash_cs <= 1;
     int_flash_clk <= !int_flash_clk;
   end
  end

  // Internal mem
  logic        mem_valid;
  logic        trap;
  logic        mem_ready;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  memory memory (
    .clk(clk),
    .mem_valid(mem_valid),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata)
  );

  logic [31:0] imem_addr;
  logic [31:0] imem_data;
  imemory imemory(
    .imem_addr(imem_addr[15:2]),
    .imem_data(imem_data)
  );

  littlecpu riscv (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .mem_valid(mem_valid),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap)
  );
endmodule // littlesoc
