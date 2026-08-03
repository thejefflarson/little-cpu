`timescale 1 ns / 1 ps
`default_nettype none
// The design that places, measured by `make soc-timing`. `make fit` measures
// `littlecpu` with its memories external; the two numbers are separate on
// purpose and must not be merged (ADR-0038).
//
// Nothing on either bus is ever refused -- out of range reads as zero -- which
// is what keeps every trap in decode (invariant 2).
//
// The ROM is initialised from the bitstream and the SPRAM cannot be, so a
// program's `.data` is not there at power-on and every `.S` program in test/asm
// has one. Running a real program needs a copy stub in `.text` or ADR-0044's
// SPI-flash boot path.
module littlesoc (
  input  logic clk,
  input  logic btn_n,
  // A design with no observable output is one yosys deletes: the previous
  // version of this module reported 4 logic cells and 0% utilisation while doing
  // exactly that. These two are passive taps that reach every stage of the core,
  // and neither adds an MMIO region.
  output logic ledr_n,
  output logic ledg_n
);
  // The FPGA comes out of configuration with no reset pulse of its own.
  // Registering `reset` is a timing decision as well as a metastability one:
  // `make soc-timing`'s first run measured the design's longest path starting at
  // the `btn_n` pad.
  logic [3:0] por_count = 4'b0;
  logic       por_done  = 1'b0;
  logic [1:0] btn_sync  = 2'b0;
  logic       reset     = 1'b1;
  always_ff @(posedge clk) begin
    if (!por_done) begin
      por_count <= por_count + 4'd1;
      if (por_count == 4'hf) por_done <= 1'b1;
    end
    btn_sync <= {btn_sync[0], btn_n};
    reset    <= !por_done || !btn_sync[1];
  end

  logic        trap;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [31:0] imem_mem_rdata, dmem_mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren, fetch_stall;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic [31:0] imem_data, imem_data2;

  littlecpu riscv (
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
    .trap(trap)
  );

  // `imem_addr`/`imem_addr2` are deliberately unread: the ROM is synchronous, so
  // it is addressed off `imem_addr_next` a cycle early. Both ports stay on the
  // core because that is the bus the riscv-formal ladder speaks.
  //
  // 2048 words = 8 KB = 16 EBRs of the part's 30, with rtl/regfile.v taking 4.
  imemory #(
    .ROM_WORDS(2048),
    .INIT_EVEN("soc/rom_even.hex"),
    .INIT_ODD("soc/rom_odd.hex")
  ) imem (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(imem_mem_rdata),
    .fetch_stall(fetch_stall)
  );

  // 16384 words = 64 KB = two `SB_SPRAM256KA` of the part's four.
  memory #(.BASE(32'h0001_0000), .RAM_WORDS(16384)) dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dmem_mem_rdata)
  );

  // An OR rather than a select, one less level on the data path. The ranges do
  // not overlap, and a store leaves the RAM's registered read data unchanged
  // (rtl/memory.v) while the ROM answers zero, so this cannot mix two live
  // values.
  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata;

  logic store_bit, trap_seen;
  always_ff @(posedge clk) begin
    if (reset) begin
      store_bit <= 1'b0;
      trap_seen <= 1'b0;
    end else begin
      if (|mem_wstrb) store_bit <= mem_wdata[0];
      if (trap)       trap_seen <= 1'b1;
    end
  end
  assign ledg_n = !store_bit;
  assign ledr_n = !trap_seen;
endmodule // littlesoc
