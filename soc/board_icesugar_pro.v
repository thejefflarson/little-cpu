`timescale 1 ns / 1 ps
`default_nettype none
module icesugar_pro_top (
  input  logic clk_pin,
  output logic led_r_n,
  output logic led_g_n,
  output logic uart_tx
);
  // The pad is PAD_HZ and the core runs at CORE_HZ; test/pll_clock_test.py recomputes the
  // second from the first. `btn_n` is the released-button input, so lock gates `reset`.
  localparam integer PAD_HZ  = 25_000_000;
  localparam integer CORE_HZ = 30_000_000;

  logic spi_sck, spi_mosi, spi_cs_n;
  logic clk_core, pll_locked;

  icesugar_pro_pll pll (
    .clk_pad(clk_pin),
    .clk_core(clk_core),
    .locked(pll_locked)
  );

  littlesoc #(.CLOCK_HZ(CORE_HZ)) soc (
    .clk(clk_core),
    .btn_n(pll_locked),
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
