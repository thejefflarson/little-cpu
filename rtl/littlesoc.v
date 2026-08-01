`timescale 1 ns / 1 ps
`default_nettype none
// The SoC wrapper: the core plus the two memories it talks to, and nothing
// else. What used to sit above the instances was scaffolding for an SPI flash
// controller nobody ever wrote -- a chip select and a free-running `flash_clk`
// divider driving four `inout` flash pins, plus `mem_valid`/`mem_ready` wires
// no module read -- and it is gone. The real memory system (SPRAM data RAM,
// ROM banking, byte strobes) and the pinout that goes with it are the work
// that will give this module ports again; until then it deliberately has none
// beyond clk/reset.
//
// This is NOT the synthesis top for area. ADR-0038 measures `littlecpu` with
// its memories external, because the two memories below are placeholders whose
// real implementation will be SPRAM/EBR and will not consume logic cells.
module littlesoc (
  input  clk,
  input  reset
);
  // Internal mem
  logic        trap;
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
