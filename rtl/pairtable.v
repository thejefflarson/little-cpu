`timescale 1 ns / 1 ps
`default_nettype none
// The pair read after this address last time -- a guess `operand_stall` already checks.
module pairtable #(
  parameter integer INDEX_BITS = 8,
  parameter integer TAG_BITS   = 6
) (
  input  logic        clk,
  input  logic [31:0] read_pc,
  output logic        hit,
  output logic [4:0]  hit_rs1,
  output logic [4:0]  hit_rs2,
  input  logic        wen,
  input  logic [31:0] write_pc,
  input  logic [4:0]  write_rs1,
  input  logic [4:0]  write_rs2
);
  localparam integer ENTRIES = 1 << INDEX_BITS;
  localparam integer WIDTH   = TAG_BITS + 11;

  logic [WIDTH-1:0] entries[ENTRIES-1:0];
  // Zeroed: an undefined array pushes an X into the regfile address port, unseen by cxxrtl.
  initial begin
    for (int i = 0; i < ENTRIES; i++) entries[i] = '0;
  end

  logic [INDEX_BITS-1:0] read_index, write_index;
  logic [TAG_BITS-1:0]   read_tag, write_tag;
  assign read_index  = read_pc[INDEX_BITS:1];
  assign read_tag    = read_pc[INDEX_BITS+TAG_BITS:INDEX_BITS+1];
  assign write_index = write_pc[INDEX_BITS:1];
  assign write_tag   = write_pc[INDEX_BITS+TAG_BITS:INDEX_BITS+1];

  logic [WIDTH-1:0]    entry;
  logic [TAG_BITS-1:0] held_tag;
  always_ff @(posedge clk) begin
    entry    <= entries[read_index];
    held_tag <= read_tag;
    if (wen) entries[write_index] <= {write_tag, 1'b1, write_rs1, write_rs2};
  end

  assign hit     = entry[10] && entry[WIDTH-1:11] == held_tag;
  assign hit_rs1 = entry[9:5];
  assign hit_rs2 = entry[4:0];
endmodule
