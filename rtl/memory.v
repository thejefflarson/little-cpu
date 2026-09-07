`timescale 1 ns / 1 ps
`default_nettype none
// The data RAM: single-port SPRAM, byte-strobed, synchronous read.
module memory #(
  parameter logic [31:0] BASE = 32'h0001_0000,
  parameter integer RAM_WORDS = 16384,
  parameter integer NHARTS = 1
) (
  input  logic        clk,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  output logic        reservable,
  input  logic [32*NHARTS-1:0] atomic_addr,
  output logic [NHARTS-1:0]    atomic_supported
);
  localparam int ADDR_BITS = $clog2(RAM_WORDS);

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

  for (genvar h = 1; h < NHARTS; h++) begin : l_atomic
    assign atomic_supported[h] =
      atomic_addr[32*h+31:32*h+ADDR_BITS+2] == BASE[31:ADDR_BITS+2];
  end

  logic [31:0] ram_q;
  logic        in_range_q;
  always_ff @(posedge clk) begin
    if (in_range && |mem_wstrb) begin
      if (mem_wstrb[0]) ram[index][7:0]   <= mem_wdata[7:0];
      if (mem_wstrb[1]) ram[index][15:8]  <= mem_wdata[15:8];
      if (mem_wstrb[2]) ram[index][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) ram[index][31:24] <= mem_wdata[31:24];
    end else if (!(|mem_wstrb)) begin
      ram_q      <= ram[index];
      in_range_q <= in_range;
    end
  end
  // A mux on the output, never a synchronous constant: soc/bram_reset_check.py says why.
  assign mem_rdata = in_range_q ? ram_q : 32'b0;
endmodule
