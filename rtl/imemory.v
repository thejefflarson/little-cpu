`default_nettype none
module imemory #(
  // in four byte words, this will take up half of the embedded memory in an up5k with room for the
  // register file.
  parameter integer ROM = 15872
) (
  input  logic clk,
  input  logic [31:0] imem_addr,
  output logic [31:0] imem_data
);
  initial $readmemh("./rom.mem", rom);
  logic [31:0] rom[ROM-1:0];
  always_ff @(posedge clk) begin
    imem_data <= rom[imem_addr[31:2]];
  end
endmodule
