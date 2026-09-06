// rtl/littlesoc.v on a MuseLab iCESugar-Pro: an ECP5 LFE5U-25F in a caBGA256,
// 25 MHz on P6, flashed by dropping a .bit on the iCELink volume or with
// `icesprog -w`.
//
// CLOCK_HZ is the whole reason this board can report a number. rtl/uart.v
// derives its divisor from it, and the default is the up5k's 12 MHz -- left
// alone here, 25 MHz would put the line near 240000 baud and nothing would
// decode it.
//
// The flash controller's three outputs stay internal. Its pins are the
// programmer's, and this board's iCELink drives them; an output constrained
// onto one would fight the debugger mid-session.
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
    // No user button is wired here, so the power-on counter is the only reset,
    // the same as the UPduino.
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
