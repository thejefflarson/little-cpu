`timescale 1 ns / 1 ps
`default_nettype none
// The data RAM: single-port SPRAM, byte-strobed, synchronous read.
//
// `SB_SPRAM256KA` is 16 bits wide with a 14-bit address and FOUR-BIT NIBBLE
// write masks, so a 32-bit word is two instances side by side and this core's
// 4-bit BYTE strobe becomes eight nibble masks. ADR-0044 flagged that mapping
// as a real detail rather than a rename, and it is: get it wrong and `sb`
// writes a halfword.
//
// The mapping is not written out here, though, and that is deliberate. Yosys
// infers `SB_SPRAM256KA` from the shape below (`synth_ice40 -spram`) and does
// the width and mask splitting itself, so this file stays a plain behavioural
// memory that both simulators run directly -- no vendor primitive, no
// simulation model of one, and no second description of the same array that
// could drift from the synthesised one. `make soc-timing` reports the SPRAM
// count, which is how the inference is checked rather than assumed.
//
// ---- the one non-obvious line: `rdata` is NO-CHANGE on a write -------------
//
// yosys's SPRAM rule (`ice40/spram.txt`) declares `rdwr no_change`: the read
// port holds its value on a cycle the port writes. The obvious spelling --
// `rdata <= ram[addr]` unconditionally, alongside the byte writes -- is
// read-first, which the hardware CANNOT do, so yosys silently declines SPRAM
// and maps the array to 128 `SB_RAM40_4K` instead. That is four times the
// part's entire block RAM, reported as a normal synthesis run. MEASURED, both
// ways; the `else` below is what makes the difference.
//
// Nothing observes the difference in behaviour. rtl/accessor.v reads
// `mem_rdata` only on the cycle after a LOAD request (ADR-0015's one-cycle
// turnaround), and decode is bubbled on that cycle, so no store is ever in
// flight when a load's data is being captured.
module memory #(
  // Base of the mapped region, subtracted before indexing. ADR-0008's map puts
  // RAM at a non-zero base so a store through a null pointer lands outside it
  // rather than silently on real data.
  parameter logic [31:0] BASE = 32'h0001_0000,
  // 32-bit words. 16384 = 64 KB = two `SB_SPRAM256KA`; the part has four, so
  // this can double without touching anything else.
  parameter integer RAM_WORDS = 16384
) (
  input  logic        clk,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata
);
  localparam int ADDR_BITS = $clog2(RAM_WORDS);

  logic [31:0] ram[0:RAM_WORDS-1];

  logic [31:0] offset;
  assign offset = mem_addr - BASE;
  logic in_range;
  assign in_range = mem_addr >= BASE && offset < 32'(RAM_WORDS) * 32'd4;
  logic [ADDR_BITS-1:0] index;
  assign index = offset[ADDR_BITS+1:2];

  // An out-of-range access is dropped and reads as zero. It must not ALIAS a
  // mapped word, which is what indexing on truncated address bits alone would
  // do; test/mem_tb.v checks exactly that, at the boundary and far away.
  // The outer split IS the no-change property: a write cycle writes and leaves
  // `mem_rdata` alone, a non-write cycle reads. An out-of-range write is
  // dropped inside the write arm rather than falling into the read arm.
  always_ff @(posedge clk) begin
    if (|mem_wstrb) begin
      if (in_range) begin
        if (mem_wstrb[0]) ram[index][7:0]   <= mem_wdata[7:0];
        if (mem_wstrb[1]) ram[index][15:8]  <= mem_wdata[15:8];
        if (mem_wstrb[2]) ram[index][23:16] <= mem_wdata[23:16];
        if (mem_wstrb[3]) ram[index][31:24] <= mem_wdata[31:24];
      end
    end else begin
      mem_rdata <= in_range ? ram[index] : 32'b0;
    end
  end
endmodule
