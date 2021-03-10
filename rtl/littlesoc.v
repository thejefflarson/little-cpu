`default_nettype none
module littlesoc (
  input  var clk,
  input  var reset,
  output var flash_cs,
  output var flash_clk,
  inout  tri flash_io0,
  inout  tri flash_io1,
  inout  tri flash_io2,
  inout  tri flash_io3
);
  logic int_flash_clk;
  assign flash_clk = int_flash_clk;
  // TODO SPI mem
  always_ff @(posedge clk) begin
   if (reset) begin
     int_flash_clk <= 0;
   end else begin
     int_flash_clk <= !int_flash_clk;
   end
  end

  // Internal mem
  logic        mem_valid;
  logic        mem_instr;
  logic        mem_ready;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  mem mem (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata)
  );

  riscv riscv (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata)
  );
endmodule // littlesoc

module mem #(
  parameter integer WORDS = 256
) (
  input  var logic        clk,
  input  var logic        reset,
  input  var logic        mem_valid,
  output var logic        mem_ready,
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
      mem_ready <= 0;
      if (mem_valid && !mem_ready) begin
        if (mem_addr < 4*WORDS) begin
          mem_rdata <= memory[mem_addr >> 2];
          if(mem_wstrb[0]) memory[mem_addr >> 2][7:0] <= mem_wdata[7:0];
          if(mem_wstrb[1]) memory[mem_addr >> 2][15:8] <= mem_wdata[15:8];
          if(mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
          if(mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
          mem_ready <= 1;
        end
      end
    end
  end
endmodule
