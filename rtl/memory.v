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
  parameter integer RAM_WORDS = 16384,
  // One `atomic_addr`/`atomic_supported` pair per hart, packed low hart first.
  // The bus-side ports stay scalar because the bus is shared and only one
  // master drives it per cycle; this pair is the exception because it is asked
  // in DECODE, where every hart is asking about its own instruction at once.
  parameter integer NHARTS = 1
) (
  input  logic        clk,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  // This is the region a reservation may be held in. It is the range test below
  // exported rather than a second one: what makes an address reservable here is
  // that this memory really answers it, so a store-conditional cannot report
  // success for a write that went nowhere. Combinational, and it accompanies the
  // request rather than the response for the same reason the fetch bus's refusal
  // does -- the reservation is taken on the cycle the request goes out.
  output logic        reservable,
  // The same question about a SECOND address, two stages earlier: the one an
  // atomic's rs1 names while decode is still deciding whether to trap it. Decode
  // must know before it chooses the next pc, so it cannot wait for that address
  // to arrive on `mem_addr`. Answering both out of one window is what keeps the
  // reservation the accessor refuses and the fault decode raises talking about
  // the same memory.
  input  logic [32*NHARTS-1:0] atomic_addr,
  output logic [NHARTS-1:0]    atomic_supported
);
  localparam int ADDR_BITS = $clog2(RAM_WORDS);

  // `in_range` below is an equality on the bits above the window and `index` is
  // read straight off the address, neither of which needs the subtraction they
  // replace. Both agree with "at or after BASE and inside the RAM" only while
  // the window is a power-of-two number of words sitting on a multiple of its
  // own size. Off either, the equality names a region the RAM does not fill and
  // `index` aliases the addresses in the gap onto real words -- silently, where
  // the subtract-and-compare was merely slow. So both are elaboration checks
  // rather than comments.
  if (RAM_WORDS != (1 << ADDR_BITS)) begin : l_ram_words_power_of_two
    $fatal(1, "memory: RAM_WORDS must be a power of two");
  end
  if (|BASE[ADDR_BITS+1:0]) begin : l_base_aligned
    $fatal(1, "memory: BASE must be aligned to RAM_WORDS words");
  end

  logic [31:0] ram[0:RAM_WORDS-1];

  logic in_range;
  assign in_range = mem_addr[31:ADDR_BITS+2] == BASE[31:ADDR_BITS+2];
  logic [ADDR_BITS-1:0] index;
  assign index = mem_addr[ADDR_BITS+1:2];
  assign reservable = in_range;
  assign atomic_supported[0] = atomic_addr[31:ADDR_BITS+2] == BASE[31:ADDR_BITS+2];

  // The harts above the first. Hart 0 keeps its own line above rather than
  // becoming an arm of this loop, and the loop is left to elaborate to nothing
  // at the default, because rtl/imemory.v measured both of those spellings on
  // the shipping netlist and folding hart 0 in was the more expensive one.
  for (genvar h = 1; h < NHARTS; h++) begin : l_atomic
    assign atomic_supported[h] =
      atomic_addr[32*h+31:32*h+ADDR_BITS+2] == BASE[31:ADDR_BITS+2];
  end

  // An out-of-range access is dropped and reads as zero. It must not ALIAS a
  // mapped word, which is what indexing on truncated address bits alone would
  // do; test/mem_tb.v checks exactly that, at the boundary and far away.
  //
  // THE FLAT SPELLING IS DELIBERATE, AND IT IS A MEASURED DECISION. The nested
  // form -- `if (|wstrb) begin if (in_range) ... end else ...` -- says the
  // no-change property more directly and was written that way first. It costs
  // **11 logic cells and 3.6% of the SoC's critical path** (88.51 -> 91.67 ns,
  // 41 -> 53 logic levels), measured by building both, twice each. This module
  // is not on that path at all: 11 cells of difference anywhere in the netlist
  // is enough to redistribute placement. The same trade was declined twice
  // before: a shifter merge worth 19 cells SAVED, and a counter narrowing
  // costing 37. Spending 11 cells and 3.6% of the only timing number
  // this project has, on a design already 6% short of its declared 12 MHz,
  // cuts the same way. Read ADR-0054 before rewriting it back.
  always_ff @(posedge clk) begin
    if (in_range && |mem_wstrb) begin
      if (mem_wstrb[0]) ram[index][7:0]   <= mem_wdata[7:0];
      if (mem_wstrb[1]) ram[index][15:8]  <= mem_wdata[15:8];
      if (mem_wstrb[2]) ram[index][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) ram[index][31:24] <= mem_wdata[31:24];
    end else if (!(|mem_wstrb)) begin
      mem_rdata <= in_range ? ram[index] : 32'b0;
    end
  end
endmodule
