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
  // One fetch window per hart, all reading ONE storage. Neither part infers a
  // true dual-port primitive from the second read port: yosys replicates each
  // bank and drives every copy from the same write, so a store lands in all of
  // them on one edge and each copy keeps the one-read-one-write shape a block
  // RAM has. That is what makes text SHARED rather than mirrored -- there is no
  // cycle in which two windows read different words at one address.
  parameter integer NHARTS = 1,
  // Two files rather than one plus a de-interleaving loop: yosys does not turn a
  // loop copying between arrays in an `initial` block into memory init, so that
  // would elaborate and then synthesise to an empty ROM.
  parameter INIT_EVEN = "",
  parameter INIT_ODD  = ""
) (
  input  logic                 clk,
  // One address, one pair of words, one stall bit and one fault bit per window,
  // packed low window first. At the default one hart every width here is the
  // scalar it has always been, so a single-hart integrator connects the nets it
  // always did.
  input  logic [32*NHARTS-1:0] imem_addr_next,
  output logic [32*NHARTS-1:0] imem_data,
  output logic [32*NHARTS-1:0] imem_data2,
  // The range decode lives here because rtl/littlesoc.v and test/testbench.v run
  // different `ROM_WORDS`, so a range test written outside this module would put
  // the two legs on different maps.
  input  logic [31:0]          mem_addr,
  input  logic [31:0]          mem_wdata,
  input  logic [3:0]           mem_wstrb,
  // The idle bus presents address 0, which is inside the text range, so without
  // this every idle cycle would steal a fetch.
  input  logic                 mem_ren,
  // Zero unless the previous cycle was a text-range load, so a consumer can OR
  // this with the data RAM's answer.
  output logic [31:0]          mem_rdata,
  output logic [NHARTS-1:0]    fetch_stall,
  // The word arriving this cycle is not in the ROM. Decode raises it as an
  // instruction access fault; without it a PC past the end of text is only a
  // word of zeroes, which is an illegal instruction and the wrong cause.
  output logic [NHARTS-1:0]    imem_fault
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

  // Out of range reads as zero, and says so on `imem_fault`, so a PC that runs
  // off the end traps with the cause the privileged spec names rather than as
  // an illegal instruction.
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

  // Read every cycle, with the write on its own address: that is the shape yosys
  // turns into a block RAM's two ports.
  logic [31:0] even_data, odd_data;
  logic        odd_first, in_range, in_range2;
  logic        data_hit, data_hit_odd;
  always_ff @(posedge clk) begin
    even_data <= rom_even[even_raddr];
    odd_data  <= rom_odd[odd_raddr];
    odd_first <= word_index[0];
    in_range  <= next_in_rom;
    in_range2 <= next_in_rom && !next_is_last;

    fetch_stall[0] <= text_access;
    data_hit       <= mem_ren && text_range;
    data_hit_odd   <= data_odd;

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
  assign imem_data [31:0] = in_range  ? window_lo : 32'b0;
  assign imem_data2[31:0] = in_range2 ? window_hi : 32'b0;
  assign imem_fault[0] = !in_range;

  assign mem_rdata = data_hit ? (data_hit_odd ? odd_data : even_data) : 32'b0;

  // The windows above the first, which are that window without the data port.
  //
  // WHY WINDOW 0 IS NOT AN ARM OF THIS LOOP, AND WHY THE WRITE TEST IS SPELLED
  // OUT AGAIN BELOW. Both restatements were measured against the single-hart
  // SoC's mapped netlist, which is what lets a tied-off change skip a
  // sixteen-seed sweep: folding window 0 into the loop is +30 cells, and merely
  // naming `|mem_wstrb && text_range` once and using it twice is +64. Neither
  // changes the logic; both change the nets ABC maps over. The shipping build
  // has to elaborate to today's design, so change either copy and change the
  // other.
  for (genvar h = 1; h < NHARTS; h++) begin : l_window
    logic [29:0]          w_next_word;
    logic [BANK_BITS:0]   w_word_index;
    logic [BANK_BITS-1:0] w_even_index, w_odd_index;
    logic                 w_next_in_rom, w_next_is_last;
    logic [31:0]          w_even_data, w_odd_data, w_window_lo, w_window_hi;
    logic                 w_odd_first, w_in_range, w_in_range2;

    assign w_next_word    = imem_addr_next[32*h+31:32*h+2];
    assign w_word_index   = w_next_word[BANK_BITS:0];
    assign w_odd_index    = w_word_index[BANK_BITS:1];
    assign w_even_index   = w_odd_index + {{(BANK_BITS-1){1'b0}}, w_word_index[0]};
    assign w_next_in_rom  = ~|w_next_word[29:ROM_BITS];
    assign w_next_is_last = &w_next_word[ROM_BITS-1:0];

    // A load is answered out of window 0's copy, so it takes that copy's read
    // port and no other and this window goes on fetching. A WRITE lands in every
    // copy on one edge, and both parts define a same-address write-during-read
    // as returning invalid data on the reading port rather than the old word --
    // Zifencei permits fetching the old or the new instruction, never garbage --
    // so a write is published to every window.
    always_ff @(posedge clk) begin
      w_even_data <= rom_even[w_even_index];
      w_odd_data  <= rom_odd[w_odd_index];
      w_odd_first <= w_word_index[0];
      w_in_range  <= w_next_in_rom;
      w_in_range2 <= w_next_in_rom && !w_next_is_last;

      fetch_stall[h] <= |mem_wstrb && text_range;
    end

    assign w_window_lo = w_odd_first ? w_odd_data  : w_even_data;
    assign w_window_hi = w_odd_first ? w_even_data : w_odd_data;
    assign imem_data [32*h+31:32*h] = w_in_range  ? w_window_lo : 32'b0;
    assign imem_data2[32*h+31:32*h] = w_in_range2 ? w_window_hi : 32'b0;
    assign imem_fault[h] = !w_in_range;
  end
endmodule
