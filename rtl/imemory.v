`timescale 1 ns / 1 ps
`default_nettype none
// A ROM in block RAM, banked by word parity so the two neighbouring words fetch
// asks for every cycle come out of one copy of the storage. The read takes a
// cycle, so the address presented is the one fetch wants next.
module imemory #(
  // Each bank should be a whole number of 256-word `SB_RAM40_4K` depths, or the
  // mapping picks up leftover logic.
  parameter integer ROM_WORDS = 2048,
  // One fetch window per hart, all reading ONE storage: yosys replicates each
  // bank per read port and drives every copy from the same write.
  parameter integer NHARTS = 1,
  // Two files: yosys does not turn an `initial` loop copying between arrays
  // into memory init.
  parameter INIT_EVEN = "",
  parameter INIT_ODD  = ""
) (
  input  logic                 clk,
  input  logic [32*NHARTS-1:0] imem_addr_next,
  output logic [32*NHARTS-1:0] imem_data,
  output logic [32*NHARTS-1:0] imem_data2,
  input  logic [31:0]          mem_addr,
  input  logic [31:0]          mem_wdata,
  input  logic [3:0]           mem_wstrb,
  // The idle bus presents address 0, which is inside the text range, so without
  // this every idle cycle would steal a fetch.
  input  logic                 mem_ren,
  // Zero unless the previous cycle was a text-range load, so the SoC can OR it
  // with the data RAM's answer.
  output logic [31:0]          mem_rdata,
  output logic [NHARTS-1:0]    fetch_stall,
  // The word arriving this cycle is outside the ROM.
  output logic [NHARTS-1:0]    imem_fault
);
  localparam int BANK_WORDS = ROM_WORDS / 2;
  localparam int BANK_BITS  = $clog2(BANK_WORDS);
  localparam int ROM_BITS   = $clog2(ROM_WORDS);

  // Both range tests below are reductions on the address bits above the ROM,
  // which is `< ROM_WORDS` only at a power of two; at any other depth they admit
  // addresses that index off the end of the banks and alias real code.
  if (ROM_WORDS != (1 << ROM_BITS)) begin : l_rom_words_power_of_two
    $fatal(1, "imemory: ROM_WORDS must be a power of two");
  end

  logic [31:0] rom_even[0:BANK_WORDS-1];
  logic [31:0] rom_odd [0:BANK_WORDS-1];
  // Guarded because `$readmemh("")` is an error rather than a no-op in both
  // frontends.
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

  logic [29:0]          data_word;
  logic [BANK_BITS-1:0] data_index;
  logic                 data_odd, text_range;
  assign data_word  = mem_addr[31:2];
  assign data_index = data_word[BANK_BITS:1];
  assign data_odd   = data_word[0];
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

  // Do not spell `next_is_last` as `word + 1 < ROM_WORDS`: the adder would sit
  // in the fetch loop, and at the top of the address space `word + 1` wraps to
  // zero and reads word zero back.
  logic next_in_rom, next_is_last;
  assign next_in_rom  = ~|next_word[29:ROM_BITS];
  assign next_is_last = &next_word[ROM_BITS-1:0];

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

  // The windows above the first, without the data port. Window 0 is spelled on
  // its own above on purpose: folding it into this loop changes no logic and
  // moves the single-hart SoC's mapped netlist by tens of cells. Change one
  // copy and change the other.
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

    // A load reads window 0's copy alone, so only a write, which lands in every
    // copy, stalls this window.
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
