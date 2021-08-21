`default_nettype none
module memory #(
  // in four byte words, this will take up all of the embedded memory in an up5k with room for the
  // register file.
  parameter integer RAM = 31744
) (
  input  var logic        clk,
  input  var logic        mem_ready,
  output var logic        mem_valid,
  input  var logic [31:0] mem_addr,
  input  var logic [31:0] mem_wdata,
  input  var logic [3:0]  mem_wstrb,
  output var logic [31:0] mem_rdata
);
  initial $readmemh("./rom.mem", ram);
  logic [31:0] ram[RAM-1:0];
  always_ff @(posedge clk) begin
      mem_valid <= 0;
      if (mem_ready && !mem_valid) begin
        if (mem_addr < 4*RAM) begin
          mem_rdata <= ram[mem_addr >> 2];
          if(mem_wstrb[0]) ram[mem_addr >> 2][7:0] <= mem_wdata[7:0];
          if(mem_wstrb[1]) ram[mem_addr >> 2][15:8] <= mem_wdata[15:8];
          if(mem_wstrb[2]) ram[mem_addr >> 2][23:16] <= mem_wdata[23:16];
          if(mem_wstrb[3]) ram[mem_addr >> 2][31:24] <= mem_wdata[31:24];
          mem_valid <= 1;
        end
      end
  end
endmodule
