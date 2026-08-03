`timescale 1 ns / 1 ps
`default_nettype none
// The whole chip: core, both memories, four pins. `make soc-timing` measures
// this. `make fit` measures the core on its own with the memories outside it, so
// its cell count is a different design's and the two do not compare.
//
// Neither memory can turn an access down. Anything out of range reads as zero.
// So no memory error can turn up after decode has already let an instruction
// through, and there is no path for one to come back on if it did.
//
// The ROM is initialised from the bitstream and the SPRAM cannot be, so a
// program's `.data` is not there at power-on and every `.S` program in test/asm
// has one. Running a real program needs a stub in `.text` that copies the data
// image out of ROM, or a boot path that reads it from external SPI flash.
module littlesoc (
  input  logic clk,
  input  logic btn_n,
  // Give yosys a design whose output nothing can see and it deletes the lot. An
  // earlier version of this module did exactly that and reported 4 logic cells.
  // These two only watch signals the core already drives, so they add no
  // registers to the memory map and change nothing about how it runs.
  output logic ledr_n,
  output logic ledg_n
);
  // The FPGA comes out of configuration with no reset of its own, so this makes
  // one. Keep `reset` registered. The button is asynchronous and needs the two
  // flops anyway, but the register also keeps the pin out of the logic behind
  // it: `make soc-timing`'s first run found the longest path in the whole design
  // starting at `btn_n`.
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

  // `imem_addr` and `imem_addr2` are left unconnected on purpose. The ROM needs
  // its address a cycle early, so it takes `imem_addr_next` instead. The core
  // keeps both ports because the formal checks read them.
  //
  // 2048 words = 8 KB = 16 block RAMs of the part's 30. rtl/regfile.v takes 4.
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

  // An OR instead of a mux, which is one less level of logic here. The two
  // ranges do not overlap. On a store the RAM holds its last read value and the
  // ROM gives zero, so this can never mix two real values together.
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
