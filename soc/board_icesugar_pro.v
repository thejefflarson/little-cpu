// rtl/littlesoc.v on a MuseLab iCESugar-Pro: an ECP5 LFE5U-25F in a caBGA256,
// 25 MHz on P6, flashed by dropping a .bit on the iCELink volume.
//
// THE UART AND THE FLASH ARE DELIBERATELY NOT PORTS. This board's schematic
// does not document which pins reach its USB-CDC bridge, and nextpnr runs here
// with `--lpf-allow-unconstrained`, so an output exposed and unconstrained gets
// whatever pad the placer likes -- on a real board that is a pin driving
// something. They stay internal until a pin is known to be right, which makes
// this a blink bitstream and not the SoC as it would ship.
//
// The UART is also at the wrong baud on this board: rtl/uart.v derives its
// divisor from a CLOCK_HZ parameter that defaults to 12 MHz and rtl/littlesoc.v
// has no parameter to override it, so at 25 MHz it would emit near 240000 baud.
// Routing a pin here means threading CLOCK_HZ through the SoC first.
`timescale 1 ns / 1 ps
`default_nettype none
module icesugar_pro_top (
  input  logic clk_pin,
  output logic led_r_n,
  output logic led_g_n
);
  logic uart_tx;
  logic spi_sck, spi_mosi, spi_cs_n;

  littlesoc soc (
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
