`timescale 1 ns / 1 ps
`default_nettype none
// rtl/imemory.v for one hart with the text store in SPRAM, shadowed at the top by a block-RAM boot ROM.
// SPRAM has no initial contents, so the first fetch after reset reads the boot ROM. Yosys maps the two
// arrays to SPRAM only while the write address is the read address signal (`even_raddr`, `odd_raddr`);
// a separate write index maps them to block RAM instead.
module imemory_spram #(
  parameter integer ROM_WORDS = 16384,
  parameter integer BOOT_WORDS = 256,
  parameter BOOT_INIT_EVEN = "",
  parameter BOOT_INIT_ODD  = ""
) (
  input  logic        clk,
  input  logic [31:0] imem_addr_next,
  output logic [31:0] imem_data,
  output logic [31:0] imem_data2,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  input  logic        mem_ren,
  output logic [31:0] mem_rdata,
  output logic        fetch_stall,
  output logic        imem_fault
);
  localparam int BANK_WORDS     = ROM_WORDS / 2;
  localparam int BANK_BITS      = $clog2(BANK_WORDS);
  localparam int ROM_BITS       = $clog2(ROM_WORDS);
  localparam int BOOT_BANK_BITS = $clog2(BOOT_WORDS / 2);

  if (ROM_WORDS != (1 << ROM_BITS) || BOOT_WORDS != (1 << $clog2(BOOT_WORDS)) || BOOT_WORDS < 4
      || BOOT_WORDS >= ROM_WORDS) begin : l_shape
    $fatal(1, "imemory_spram: ROM_WORDS and BOOT_WORDS must be powers of two with a boot ROM inside the window");
  end

  logic [31:0] rom_even[0:BANK_WORDS-1];
  logic [31:0] rom_odd [0:BANK_WORDS-1];
  logic [31:0] boot_even[0:BOOT_WORDS/2-1];
  logic [31:0] boot_odd [0:BOOT_WORDS/2-1];
  if (BOOT_INIT_EVEN != "") begin : l_boot_init
    initial $readmemh(BOOT_INIT_EVEN, boot_even);
    initial $readmemh(BOOT_INIT_ODD,  boot_odd);
  end

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

  // A store takes the port too and skips the read beside it: the stolen fetch is re-presented.
  logic text_access, text_write, text_write_even, text_write_odd;
  assign text_access     = (mem_ren || |mem_wstrb) && text_range;
  assign text_write      = |mem_wstrb && text_range;
  assign text_write_even = text_write && !data_odd;
  assign text_write_odd  = text_write &&  data_odd;

  logic [BANK_BITS-1:0] even_raddr, odd_raddr;
  assign even_raddr = text_access ? data_index : even_index;
  assign odd_raddr  = text_access ? data_index : odd_index;

  logic next_in_rom, next_is_last;
  assign next_in_rom  = ~|next_word[29:ROM_BITS];
  assign next_is_last = &next_word[ROM_BITS-1:0];

  logic [31:0] even_text, odd_text, boot_even_q, boot_odd_q, even_data, odd_data;
  logic        even_in_boot, odd_in_boot;
  logic        odd_first, in_range, in_range2, data_hit, data_hit_odd;
  always_ff @(posedge clk) begin
    odd_first <= word_index[0];
    in_range  <= next_in_rom;
    in_range2 <= next_in_rom && !next_is_last;

    fetch_stall  <= text_access;
    data_hit     <= mem_ren && text_range;
    data_hit_odd <= data_odd;
  end

  always_ff @(posedge clk) begin
    if (!text_write) begin
      even_text    <= rom_even[even_raddr];
      odd_text     <= rom_odd[odd_raddr];
      boot_even_q  <= boot_even[even_raddr[BOOT_BANK_BITS-1:0]];
      boot_odd_q   <= boot_odd[odd_raddr[BOOT_BANK_BITS-1:0]];
      even_in_boot <= &even_raddr[BANK_BITS-1:BOOT_BANK_BITS];
      odd_in_boot  <= &odd_raddr[BANK_BITS-1:BOOT_BANK_BITS];
    end
    if (text_write_even) begin
      if (mem_wstrb[0]) rom_even[even_raddr][7:0]   <= mem_wdata[7:0];
      if (mem_wstrb[1]) rom_even[even_raddr][15:8]  <= mem_wdata[15:8];
      if (mem_wstrb[2]) rom_even[even_raddr][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) rom_even[even_raddr][31:24] <= mem_wdata[31:24];
    end
    if (text_write_odd) begin
      if (mem_wstrb[0]) rom_odd[odd_raddr][7:0]   <= mem_wdata[7:0];
      if (mem_wstrb[1]) rom_odd[odd_raddr][15:8]  <= mem_wdata[15:8];
      if (mem_wstrb[2]) rom_odd[odd_raddr][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) rom_odd[odd_raddr][31:24] <= mem_wdata[31:24];
    end
  end
  assign even_data = even_in_boot ? boot_even_q : even_text;
  assign odd_data  = odd_in_boot  ? boot_odd_q  : odd_text;

  logic [31:0] window_lo, window_hi;
  assign window_lo = odd_first ? odd_data  : even_data;
  assign window_hi = odd_first ? even_data : odd_data;
  assign imem_data  = in_range  ? window_lo : 32'b0;
  assign imem_data2 = in_range2 ? window_hi : 32'b0;
  assign imem_fault = !in_range;

  assign mem_rdata = data_hit ? (data_hit_odd ? odd_data : even_data) : 32'b0;
endmodule
