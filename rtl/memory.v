module memory #(
  parameter integer WORDS = 256
) (
  input  var logic        clk,
  input  var logic        reset,
  input  var logic        mem_ready,
  output var logic        mem_valid,
  input  var logic [31:0] mem_addr,
  input  var logic [31:0] mem_wdata,
  input  var logic [3:0]  mem_wstrb,
  output var logic [31:0] mem_rdata
);
  logic [31:0] memory[0:WORDS-1];
  always_ff @(posedge clk) begin
    if (reset) begin
      for(integer i = 0; i < WORDS; i = i + 1) memory[i] <= 32'b0;
    end else begin
      mem_valid <= 0;
      if (mem_ready && !mem_valid) begin
        if (mem_addr < 4*WORDS) begin
          mem_rdata <= memory[mem_addr >> 2];
          if(mem_wstrb[0]) memory[mem_addr >> 2][7:0] <= mem_wdata[7:0];
          if(mem_wstrb[1]) memory[mem_addr >> 2][15:8] <= mem_wdata[15:8];
          if(mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
          if(mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
          mem_valid <= 1;
        end
      end
    end
  end
endmodule
