`timescale 1 ns / 1 ps
`default_nettype none
// A ROM in block RAM, banked by *word* parity so ADR-0003's two-word fetch
// window comes out of one copy. ADR-0044 names the technique at halfword
// granularity, which suits a memory that windows the 32 bits itself; this core
// windows them in rtl/fetcher.v, so halfword banks would need two reads a bank.
//
// Block RAM rather than `SB_SPRAM256KA` because SPRAM has no INIT capability, so
// a ROM there would need an SPI-flash boot path. BRAM's 15 KB ceiling is
// therefore the ceiling on program size (ADR-0054).
//
// The read is synchronous, so the address register is loaded with the *next*
// fetch address, in lockstep with the PC. That is what keeps invariant 1 true on
// a part with no combinational-read memory; `formal/pcloop.sv` asserts it.
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

  // A store steals as well as a load: it holds the write port, and a fetch read
  // of the word being written is a collision the part does not define.
  logic text_access, text_write_even, text_write_odd;
  assign text_access     = (mem_ren || |mem_wstrb) && text_range;
  assign text_write_even = |mem_wstrb && text_range && !data_odd;
  assign text_write_odd  = |mem_wstrb && text_range &&  data_odd;

  logic [BANK_BITS-1:0] even_raddr, odd_raddr;
  assign even_raddr = text_access ? data_index : even_index;
  assign odd_raddr  = text_access ? data_index : odd_index;

  // Out of range reads as zero, which decodes to an illegal instruction, so a
  // wild PC faults instead of aliasing back into the ROM through truncation.
  //
  // Keep `in_range2` as `< ROM_WORDS - 1` rather than `word + 1 < ROM_WORDS`.
  // The address arriving here is the freshly computed next PC, so an incrementer
  // in front of the comparator puts a second carry chain in series with the one
  // that produced it -- `make soc-timing`'s first run found exactly that at the
  // end of the critical path. It also faults both words at the top of the
  // address space, where `word + 1` wraps and answers from word 0.
  //
  // The unconditional read plus a separately addressed write is the shape yosys
  // maps to an EBR's own two ports.
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
