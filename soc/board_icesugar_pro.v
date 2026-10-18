`timescale 1 ns / 1 ps
`default_nettype none
module icesugar_pro_top (
  input  logic clk_pin,
  output logic led_r_n,
  output logic led_g_n,
  output logic uart_tx,
  output logic flash_cs_n,
  output logic flash_mosi,
  input  logic flash_miso
);
  // The pad is PAD_HZ and the core runs at CORE_HZ; test/pll_clock_test.py recomputes the
  // second from the first. `btn_n` is the released-button input, so lock gates `reset`.
  localparam integer PAD_HZ  = 25_000_000;
  localparam integer CORE_HZ = 30_000_000;

  logic spi_sck;
  logic clk_core, pll_locked, entropy_raw;

  icesugar_pro_pll pll (
    .clk_pad(clk_pin),
    .clk_core(clk_core),
    .locked(pll_locked)
  );

  icesugar_pro_entropy entropy (.raw(entropy_raw));
  USRMCLK flash_clk (.USRMCLKI(spi_sck), .USRMCLKTS(1'b0));

  littlesoc #(.CLOCK_HZ(CORE_HZ)) soc (
    .clk(clk_core),
    .btn_n(pll_locked),
    .entropy_raw(entropy_raw),
    .ledr_n(led_r_n),
    .ledg_n(led_g_n),
    .uart_tx(uart_tx),
    .spi_sck(spi_sck),
    .spi_mosi(flash_mosi),
    .spi_miso(flash_miso),
    .spi_cs_n(flash_cs_n)
  );
endmodule
`default_nettype wire
