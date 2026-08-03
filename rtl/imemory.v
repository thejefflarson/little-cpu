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
  // Must be even, and each bank a whole number of `SB_RAM40_4K` depths (256
  // words), or the mapping picks up leftover logic.
  parameter integer ROM_WORDS = 2048,
  // Two files rather than one plus a de-interleaving loop: yosys does not turn a
  // loop copying between arrays in an `initial` block into memory init, so that
  // would elaborate and then synthesise to an empty ROM.
  parameter INIT_EVEN = "",
  parameter INIT_ODD  = ""
) (
  input  logic        clk,
  input  logic [31:0] imem_addr_next,
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
  assign text_range = data_word < 30'(ROM_WORDS);

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
  // Keep `in_range2` as `< ROM_WORDS - 1`. Written the obvious way, `word + 1 <
  // ROM_WORDS`, the adder sits in front of the comparator -- and the address
  // arriving here is the next PC, which was itself just computed by an adder.
  // That puts two carry chains in a row on the longest path in the design.
  // `make soc-timing`'s first run found exactly that. The form here also handles
  // the top of the address space, where `word + 1` wraps round to 0 and would
  // read word 0 back.
  //
  // Read every cycle, with the write on its own address: that is the shape yosys
  // turns into a block RAM's two ports.
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

  assign mem_rdata = data_hit ? (data_hit_odd ? odd_data : even_data) : 32'b0;
endmodule
