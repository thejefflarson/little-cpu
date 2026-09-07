`timescale 1 ns / 1 ps
`default_nettype none
module icesugar_pro_top (
  input  logic clk_pin,
  output logic led_r_n,
  output logic led_g_n,
  output logic uart_tx
);
  logic spi_sck, spi_mosi, spi_cs_n;

  littlesoc #(.CLOCK_HZ(25_000_000)) soc (
    .clk(clk_pin),
    .btn_n(1'b1),
    .ledr_n(led_r_n),
    .ledg_n(led_g_n),
    .uart_tx(uart_tx),
    .spi_sck(spi_sck),
    .spi_mosi(spi_mosi),
    .spi_miso(1'b0),
    .spi_cs_n(spi_cs_n)
  );
endmodule
`default_nettype wire
