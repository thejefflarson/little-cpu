`timescale 1 ns / 1 ps
`default_nettype none
module imemory #(
  // in four byte words, this will take up half of the embedded memory in an up5k with room for the
  // register file.
  parameter integer ROM = 15872
) (
  input logic [13:0]  imem_addr,
  output logic [31:0] imem_data
);
  initial $readmemh("./rom.mem", rom);
  logic [31:0] rom[0:ROM-1];
  always_comb
    imem_data = rom[imem_addr];
endmodule
