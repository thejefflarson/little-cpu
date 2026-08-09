`timescale 1 ns / 1 ps
`default_nettype none
// A ROM in block RAM. Fetch asks for two neighbouring words every cycle, so an
// instruction sitting across a word boundary costs nothing extra. Two banks
// split by word parity serve both from one copy of the ROM: neighbouring words
// are always in different banks, so each bank supplies one of them.
//
// Splitting by halfword would suit a memory that picks the 32 bits out itself.
// rtl/fetcher.v does that part, and asks for whole words, so halfword banks
// would mean two reads from each bank instead of one.
//
// Block RAM rather than `SB_SPRAM256KA` because SPRAM cannot be filled from the
// bitstream. A ROM there could only be written by code already running. So the
// block RAM on the part is the limit on how big a program can be.
//
// The read takes a cycle. So the address register is loaded with the address
// fetch wants *next*, on the same edge the PC moves to it. Decode still sees the
// instruction at `pc` in the cycle it works out where to go next. If those two
// ever slip apart, every instruction the core runs is the wrong one, and nothing
// else would show it -- `formal/pcloop.sv` checks it.
module imemory #(
  // Must be a power of two -- the range tests below depend on it, and the check
  // beside them stops the build otherwise. Each bank should also be a whole
  // number of `SB_RAM40_4K` depths (256 words), or the mapping picks up leftover
  // logic.
  parameter integer ROM_WORDS = 2048,
  // Two files rather than one plus a de-interleaving loop: yosys does not turn a
  // loop copying between arrays in an `initial` block into memory init, so that
  // would elaborate and then synthesise to an empty ROM.
  parameter INIT_EVEN = "",
  parameter INIT_ODD  = ""
) (
  input  logic        clk,
  input  logic [31:0] imem_addr_next,
  // Low on a cycle the core is not ready for a new window. The banks hold
  // their output registers, so the same two words come back without the address
  // having to be held -- which is what keeps the core's stall off the address
  // path. A data access takes the read port regardless, and the fetch it
  // displaces comes back as `fetch_stall` below.
  input  logic        imem_ren,
  output logic [31:0] imem_data,
  output logic [31:0] imem_data2,
  // The range decode lives here because rtl/littlesoc.v and test/testbench.v run
  // different `ROM_WORDS`, so a range test written outside this module would put
  // the two legs on different maps.
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  // The idle bus presents address 0, which is inside the text range, so without
  // this every idle cycle would steal a fetch.
  input  logic        mem_ren,
  // Zero unless the previous cycle was a text-range load, so a consumer can OR
  // this with the data RAM's answer.
  output logic [31:0] mem_rdata,
  output logic        fetch_stall
);
  localparam int BANK_WORDS = ROM_WORDS / 2;
  localparam int BANK_BITS  = $clog2(BANK_WORDS);
  localparam int ROM_BITS   = $clog2(ROM_WORDS);

  // Both range tests below are reductions on the address bits above the ROM,
  // which is `< ROM_WORDS` only at a power-of-two depth. At any other depth
  // `$clog2` rounds up, the reduction admits the addresses between the top of
  // the ROM and the next power of two, and those index off the end of the banks
  // instead of reading zero -- so a PC that runs off the end aliases real code
  // rather than trapping. The old comparison was merely slow at those depths;
  // this one is wrong, so it fails elaboration instead.
  if (ROM_WORDS != (1 << ROM_BITS)) begin : l_rom_words_power_of_two
    $fatal(1, "imemory: ROM_WORDS must be a power of two");
  end

  logic [31:0] rom_even[0:BANK_WORDS-1];
  logic [31:0] rom_odd [0:BANK_WORDS-1];
  // Guarded because `$readmemh("")` is an error rather than a no-op in both
  // frontends, and the cxxrtl runner fills the banks through `debug_items`.
  generate if (INIT_EVEN != "") begin : l_rom_init
    initial $readmemh(INIT_EVEN, rom_even);
    initial $readmemh(INIT_ODD,  rom_odd);
  end endgenerate

  logic [29:0]          next_word;
  logic [BANK_BITS:0]   word_index;
  logic [BANK_BITS-1:0] even_index, odd_index;
  assign next_word  = imem_addr_next[31:2];
  assign word_index = next_word[BANK_BITS:0];
  assign odd_index  = word_index[BANK_BITS:1];
  assign even_index = odd_index + {{(BANK_BITS-1){1'b0}}, word_index[0]};

  // The low two bits are dropped because rtl/accessor.v word-aligns every load
  // and store, so they carry nothing the byte strobes do not.
  logic [29:0]          data_word;
  logic [BANK_BITS-1:0] data_index;
  logic                 data_odd, text_range;
  assign data_word  = mem_addr[31:2];
  assign data_index = data_word[BANK_BITS:1];
  assign data_odd   = data_word[0];
  // Same reduction as the fetch-side test below: `< ROM_WORDS` on a power of two
  // is "the bits above the ROM are all zero".
  assign text_range = ~|data_word[29:ROM_BITS];

  // A store takes the port too, not just a load. Reading a word while it is
  // being written gives an answer the part does not define.
  logic text_access, text_write_even, text_write_odd;
  assign text_access     = (mem_ren || |mem_wstrb) && text_range;
  assign text_write_even = |mem_wstrb && text_range && !data_odd;
  assign text_write_odd  = |mem_wstrb && text_range &&  data_odd;

  logic [BANK_BITS-1:0] even_raddr, odd_raddr;
  assign even_raddr = text_access ? data_index : even_index;
  assign odd_raddr  = text_access ? data_index : odd_index;

  // Out of range reads as zero. Zero is an illegal instruction, so a PC that
  // runs off the end traps instead of wrapping round and reading real code.
  //
  // Both tests are reductions, not magnitude comparisons: `ROM_WORDS` is a power
  // of two, so `< ROM_WORDS` is exactly "the bits above the ROM are all zero"
  // and `< ROM_WORDS - 1` is that and "not the last word". Written as `<`, the
  // second one compares against a non-power-of-two constant, which yosys can
  // only build as a carry chain longer than a tile column -- so it becomes carry
  // segments with general routing between them, and it measured a quarter of the
  // whole period. The two reductions below are independent and evaluate in
  // parallel.
  //
  // Do not restore `word + 1 < ROM_WORDS`: the adder would sit in front of the
  // test, the address arriving here is the next PC which an adder just produced,
  // and at the top of the address space `word + 1` wraps to zero and reads word
  // zero back.
  logic next_in_rom, next_is_last;
  assign next_in_rom  = ~|next_word[29:ROM_BITS];
  assign next_is_last = &next_word[ROM_BITS-1:0];

  // A read under an enable, with the write on its own address: that is the shape
  // yosys turns into a block RAM's two ports, and `SB_RAM40_4K` holds `RDATA`
  // while `RE` is low. The five registers move together because they describe
  // one window: which bank came first and whether either word is in range are
  // answers about the address that was read, not about the address being
  // presented now.
  //
  // A text-range access takes the port whether the fetch wanted it or not, so it
  // reads even when the core asked for nothing. That overwrites the held window,
  // and `fetch_stall` is what says so; the cycle after, the core asks again.
  logic bank_re;
  assign bank_re = imem_ren || text_access;

  logic [31:0] even_data, odd_data;
  logic        odd_first, in_range, in_range2;
  logic        data_hit, data_hit_odd;
  always_ff @(posedge clk) begin
    if (bank_re) begin
      even_data <= rom_even[even_raddr];
      odd_data  <= rom_odd[odd_raddr];
      odd_first <= word_index[0];
      in_range  <= next_in_rom;
      in_range2 <= next_in_rom && !next_is_last;
    end

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

  assign mem_rdata = data_hit ? (data_hit_odd ? odd_data : even_data) : 32'b0;
endmodule
