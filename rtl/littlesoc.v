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
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata)
  );

  logic [31:0] imem_addr;
  logic [31:0] imem_data_raw;
  logic [31:0] imem_data;
  imemory imemory(
    .imem_addr(imem_addr[15:2]),
    .imem_data(imem_data_raw)
  );
  // Return zero (illegal instruction) if PC >= 0x10000 so out-of-range PCs
  // do not silently alias back into the ROM via bit truncation.
  assign imem_data = (|imem_addr[31:16]) ? 32'b0 : imem_data_raw;

  // ADR-0003: the second, adjacent word of the dual-word fetch window --
  // same ROM, a second combinational read port at imem_addr2 (rtl/fetcher.v
  // drives imem_addr2 = imem_addr + 4).
  logic [31:0] imem_addr2;
  logic [31:0] imem_data2_raw;
  logic [31:0] imem_data2;
  imemory imemory2(
    .imem_addr(imem_addr2[15:2]),
    .imem_data(imem_data2_raw)
  );
  assign imem_data2 = (|imem_addr2[31:16]) ? 32'b0 : imem_data2_raw;

  littlecpu riscv (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap)
  );
endmodule // littlesoc
