`timescale 1 ns / 1 ps
`default_nettype none
// rtl/littlesoc.v with a 64 KB text window running test/window64/prog.S (its ROM read from soc/rom_*.hex in the
// working directory). Ends on the first store to RAM's first word, on a trap, or when the cycle budget runs out.
module window64_sim;
  parameter integer BUDGET = 4000;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  littlesoc #(.ROM_WORDS(16384)) soc (
    .clk(clk),
    .btn_n(1'b1),
    .entropy_raw(1'b0),
    .ledr_n(),
    .ledg_n(),
    .uart_tx(),
    .spi_sck(),
    .spi_mosi(),
    .spi_miso(1'b1),
    .spi_cs_n()
  );

  integer cycles = 0;
  always @(posedge clk) begin
    cycles <= cycles + 1;
    if (|soc.mem_wstrb && soc.mem_addr == 32'h0001_0000) begin
      $display("STORE %0d cycles=%0d", soc.mem_wdata, cycles);
      $finish;
    end
    if (soc.trap) begin
      $display("TRAP cycles=%0d", cycles);
      $finish;
    end
    if (cycles >= BUDGET) begin
      $display("TIMEOUT");
      $finish;
    end
  end
endmodule
