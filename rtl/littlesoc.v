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
     flash_cs <= 0;
     int_flash_clk <= 0;
   end else begin
     flash_cs <= 1;
     int_flash_clk <= !int_flash_clk;
   end
  end

  // Internal mem
  logic        mem_valid;
  logic        trap;
  logic        mem_ready;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  memory memory (
    .clk(clk),
    .mem_valid(mem_valid),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata)
  );

  littlecpu riscv (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap)
  );
endmodule // littlesoc
