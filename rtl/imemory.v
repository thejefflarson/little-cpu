`timescale 1 ns / 1 ps
`default_nettype none
// The instruction memory: two interleaved banks of 32-bit words, read
// synchronously off the fetch address published one cycle early (ADR-0054).
// The data bus reaches it too, through the banks' own write port and by
// borrowing their read port for a cycle.
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
  output logic [31:0] imem_data2,
  // The data bus, so text is readable and writable from a store or a load.
  // The range decode and the arbitration between the two buses live here
  // rather than in each consumer: rtl/littlesoc.v and test/testbench.v run
  // different `ROM_WORDS`, so a range test written outside this module would
  // put the two legs on different maps.
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  // High on the cycle a load's request is on the bus. The idle bus presents
  // address 0, which is inside the text range, so without it every idle cycle
  // would steal a fetch.
  input  logic        mem_ren,
  // Zero unless the previous cycle was a text-range load, so a consumer can
  // OR this with the data RAM's answer.
  output logic [31:0] mem_rdata,
  // High for the cycle whose fetch window was taken by a data access. The
  // fetch that lost the read port has to be repeated.
  output logic        fetch_stall
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
  logic [29:0]          next_word;
  logic [BANK_BITS:0]   word_index;
  logic [BANK_BITS-1:0] even_index, odd_index;
  assign next_word  = imem_addr_next[31:2];
  assign word_index = next_word[BANK_BITS:0];
  assign odd_index  = word_index[BANK_BITS:1];
  assign even_index = odd_index + {{(BANK_BITS-1){1'b0}}, word_index[0]};

  // The data side. One word, so it needs one bank -- the one its word parity
  // names -- at the index that parity strips off.
  //
  // The range test is the same shape as the fetch one below: a word index
  // against a constant, with no adder in front of it. `mem_addr` is
  // word-aligned for every load and store (rtl/accessor.v), so the low two
  // bits carry nothing the byte strobes do not.
  logic [29:0]          data_word;
  logic [BANK_BITS-1:0] data_index;
  logic                 data_odd, text_range;
  assign data_word  = mem_addr[31:2];
  assign data_index = data_word[BANK_BITS:1];
  assign data_odd   = data_word[0];
  assign text_range = data_word < 30'(ROM_WORDS);

  // The steal. A text access takes the banks' single read port for the edge,
  // so the fetch presented that cycle is answered with the data word instead
  // and has to be repeated -- `fetch_stall` below says so. A store steals too:
  // it holds the write port, and a fetch read of the word being written is a
  // collision the part does not define.
  logic text_access, text_write_even, text_write_odd;
  assign text_access     = (mem_ren || |mem_wstrb) && text_range;
  assign text_write_even = |mem_wstrb && text_range && !data_odd;
  assign text_write_odd  = |mem_wstrb && text_range &&  data_odd;

  logic [BANK_BITS-1:0] even_raddr, odd_raddr;
  assign even_raddr = text_access ? data_index : even_index;
  assign odd_raddr  = text_access ? data_index : odd_index;

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
  //
  // The read below is unconditional and the write is a second, separately
  // addressed port: that is the shape yosys maps to an EBR's own read and write
  // ports. Both buses are answered from these registers, so a data read arrives
  // a cycle after its address the same way a fetch does.
  logic [31:0] even_data, odd_data;
  logic        odd_first, in_range, in_range2;
  logic        data_hit, data_hit_odd;
  always_ff @(posedge clk) begin
    even_data <= rom_even[even_raddr];
    odd_data  <= rom_odd[odd_raddr];
    odd_first <= word_index[0];
    in_range  <= next_word < 30'(ROM_WORDS);
    in_range2 <= next_word < 30'(ROM_WORDS) - 30'd1;

    fetch_stall  <= text_access;
    data_hit     <= mem_ren && text_range;
    data_hit_odd <= data_odd;

    if (text_write_even) begin
      if (mem_wstrb[0]) rom_even[data_index][7:0]   <= mem_wdata[7:0];
      if (mem_wstrb[1]) rom_even[data_index][15:8]  <= mem_wdata[15:8];
      if (mem_wstrb[2]) rom_even[data_index][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) rom_even[data_index][31:24] <= mem_wdata[31:24];
    end
    if (text_write_odd) begin
      if (mem_wstrb[0]) rom_odd[data_index][7:0]   <= mem_wdata[7:0];
      if (mem_wstrb[1]) rom_odd[data_index][15:8]  <= mem_wdata[15:8];
      if (mem_wstrb[2]) rom_odd[data_index][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) rom_odd[data_index][31:24] <= mem_wdata[31:24];
    end
  end

  logic [31:0] window_lo, window_hi;
  assign window_lo = odd_first ? odd_data  : even_data;
  assign window_hi = odd_first ? even_data : odd_data;
  assign imem_data  = in_range  ? window_lo : 32'b0;
  assign imem_data2 = in_range2 ? window_hi : 32'b0;

  // Zero out of range, and zero on the cycle after a store, so a consumer can
  // OR this with the data RAM's `mem_rdata` instead of decoding the map twice.
  assign mem_rdata = data_hit ? (data_hit_odd ? odd_data : even_data) : 32'b0;
endmodule
