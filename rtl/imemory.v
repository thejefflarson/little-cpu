`timescale 1 ns / 1 ps
`default_nettype none
// The instruction ROM: two interleaved banks of 32-bit words, read
// synchronously off the fetch address published one cycle early (ADR-0054).
//
// It is a ROM in BLOCK RAM, and both halves of that are decisions
// (ADR-0054 §"ROM in BRAM"). `SB_SPRAM256KA` is the only large memory on this
// part and it has NO INIT capability at all, so a ROM there needs a runtime
// boot path copying from external SPI flash -- a different product. BRAM is
// initialisable from the bitstream, and its 15 KB ceiling is therefore the
// ceiling on program size.
//
// ---- why two banks, and why they are 32 bits wide and not 16 --------------
//
// ADR-0003's fetch window reads the word at `pc` AND the word after it, every
// cycle, so a 32-bit instruction straddling a 4-byte boundary costs nothing.
// The previous placeholder got the second word by instantiating the whole ROM
// TWICE (rtl/littlesoc.v), which doubles storage to add a port and is most of
// the 43 KB overage ADR-0044 measured.
//
// Banking replaces the duplication: consecutive words live in different banks,
// so `word W` and `word W+1` are always one read from each. Storage is the ROM
// size, once.
//
// ADR-0044 names this technique at 16-BIT granularity -- even halfwords in one
// bank, odd in the other. That is the right split for a memory that returns the
// 32-bit window itself. This core's fetch interface asks for two adjacent
// WORDS and does its own windowing (rtl/fetcher.v), so the parity that matters
// here is word parity, and 16-bit banks would need two reads from each bank
// rather than one -- duplication again, one level down. Word-granularity
// interleaving is the same technique against the interface this core actually
// has, and it is also the one that leaves `imem_data`/`imem_data2` meaning
// exactly what `formal/imemcheck.sv` checks them to mean.
//
// ---- why the address arrives a cycle early --------------------------------
//
// There is no combinational-read memory primitive on this part (ADR-0044), and
// invariant 1 requires decode to see the instruction at `pc` in the same cycle
// it decides the next one. So the address register is loaded with the NEXT
// fetch address at every posedge, in lockstep with the PC itself: during the
// cycle in which `imem_addr` names a word, this module's registered outputs
// already hold it. `formal/pcloop.sv` asserts that lockstep rather than
// leaving it to be read off two files.
module imemory #(
  // Total 32-bit words of ROM. Split evenly across the two banks below, so
  // this must be even and each bank must be a whole number of `SB_RAM40_4K`
  // depths (256 words) for the mapping to be free of leftover logic.
  //
  // 2048 words = 8 KB = 16 EBRs, which with rtl/regfile.v's 4 leaves 10 of the
  // part's 30 spare. See ADR-0054 for what that ceiling costs: it holds 55 of
  // this repo's 56 test programs, and `test/asm/rvc.S` (12,232 bytes, almost
  // all of it `.skip` padding for a page-boundary straddle) is the one it does
  // not. test/testbench.v overrides it, because simulation has no EBRs.
  parameter integer ROM_WORDS = 2048,
  // Bank image files, in `$readmemh` format: even-indexed words in the first,
  // odd-indexed in the second. Two files rather than one plus a de-interleaving
  // loop, because a loop copying between arrays in an `initial` block is not
  // something yosys turns into memory init -- it would elaborate and then
  // synthesise to an empty ROM. soc/rom_banks.py writes the pair.
  parameter INIT_EVEN = "",
  parameter INIT_ODD  = ""
) (
  input  logic        clk,
  // ADR-0054: the word address rtl/fetcher.v will present as `imem_addr` on
  // the NEXT edge. This module latches it, so its outputs answer that cycle.
  input  logic [31:0] imem_addr_next,
  output logic [31:0] imem_data,
  output logic [31:0] imem_data2
);
  localparam int BANK_WORDS = ROM_WORDS / 2;
  localparam int BANK_BITS  = $clog2(BANK_WORDS);

  logic [31:0] rom_even[0:BANK_WORDS-1];
  logic [31:0] rom_odd [0:BANK_WORDS-1];
  // Guarded, so a consumer that fills the banks some other way -- the cxxrtl
  // runner pokes them through `debug_items` -- can leave the pair empty rather
  // than pointing at a file that has to exist. `$readmemh("")` is an error, not
  // a no-op, in both frontends.
  generate if (INIT_EVEN != "") begin : l_rom_init
    initial $readmemh(INIT_EVEN, rom_even);
    initial $readmemh(INIT_ODD,  rom_odd);
  end endgenerate

  // Word index W of the next fetch, and the two bank indices it selects.
  // Word W lives in bank W[0] at index W >> 1; word W+1 lives in the other
  // bank, at the SAME index if W is even and at one past it if W is odd.
  // That "+ W[0]" is the entire cost of banking.
  logic [BANK_BITS:0]   word_index;
  logic [BANK_BITS-1:0] even_index, odd_index;
  assign word_index = imem_addr_next[BANK_BITS+2:2];
  assign odd_index  = word_index[BANK_BITS:1];
  assign even_index = word_index[BANK_BITS:1] + {{(BANK_BITS-1){1'b0}}, word_index[0]};

  // Out of range reads as zero, which decodes to an illegal instruction, so a
  // wild PC faults instead of silently aliasing back into the ROM through bit
  // truncation. Both words are tested: the last word of ROM is in range while
  // the word after it is not. Registered alongside the data they mask, because
  // the address they are computed from is a cycle ahead of it.
  //
  // BOTH TESTS ARE ON THE WORD INDEX, and the second is `< ROM_WORDS - 1`
  // rather than `word + 1 < ROM_WORDS`. That is a timing decision, measured:
  // the address arriving here is the freshly computed next PC, so an
  // incrementer in front of the comparator puts a second 32-bit carry chain in
  // series with the one that produced it -- and `make soc-timing`'s first run
  // found exactly that chain at the END of the critical path. Comparing against
  // a constant one lower is the same predicate with no adder.
  //
  // It also removes a corner: `word + 1` wraps at the top of the address space,
  // so the old form called the second word of a fetch at 0xfffffffc "in range"
  // and answered it from word 0. This form faults both words there.
  logic [29:0] next_word;
  assign next_word = imem_addr_next[31:2];

  logic [31:0] even_data, odd_data;
  logic        odd_first, in_range, in_range2;
  always_ff @(posedge clk) begin
    even_data <= rom_even[even_index];
    odd_data  <= rom_odd[odd_index];
    odd_first <= word_index[0];
    in_range  <= next_word < 30'(ROM_WORDS);
    in_range2 <= next_word < 30'(ROM_WORDS) - 30'd1;
  end

  logic [31:0] window_lo, window_hi;
  assign window_lo = odd_first ? odd_data  : even_data;
  assign window_hi = odd_first ? even_data : odd_data;
  assign imem_data  = in_range  ? window_lo : 32'b0;
  assign imem_data2 = in_range2 ? window_hi : 32'b0;
endmodule
