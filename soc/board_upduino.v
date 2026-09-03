// The UPduino v3.x around rtl/littlesoc.v. Its pin 14 is the USB serial line
// and the configuration flash's data-out at once, and the FPGA keeps running
// while `iceprog` reads that flash, so the UART drives pin 14 only while pin
// 16, the flash's chip select, reads high.
`timescale 1 ns / 1 ps
`default_nettype none
module upduino_top #(
  // 0 takes the 12 MHz crystal off pin 20, which reaches the FPGA only with
  // R16 (silkscreened OSC) shorted; 1 takes `SB_HFOSC` divided to 12 MHz.
  parameter bit INTERNAL_OSC = 1'b0
) (
  input  logic clk_pin,
  output logic ledr_n,
  output logic ledg_n,
  // Pin 14, and pin 16 read only.
  inout  wire  spi_miso_txd,
  inout  wire  spi_ssn
);
  logic clk;

  generate
    if (INTERNAL_OSC) begin : g_internal
      // CLKHF_DIV 0b10 is 48 MHz / 4.
      SB_HFOSC #(.CLKHF_DIV("0b10")) hfosc (
        .CLKHFPU(1'b1),
        .CLKHFEN(1'b1),
        .CLKHF(clk)
      );
    end else begin : g_crystal
      assign clk = clk_pin;
    end
  endgenerate

  logic uart_tx;
  logic ssn_pin;
  logic miso_enable;

  // The flash's data-out drives only while its chip select is low, so the
  // level of pin 16 says whether that driver is off whoever holds the pin up.
  miso_share_enable enable_gate (
    .clk(clk),
    .now(ssn_pin),
    .enable(miso_enable)
  );

  // PIN_TYPE 6'b1010_01: an output behind an output enable, and a simple input.
  localparam logic [5:0] SHARED_PIN = 6'b1010_01;

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_miso_txd (
    .PACKAGE_PIN(spi_miso_txd),
    .OUTPUT_ENABLE(miso_enable),
    .D_OUT_0(uart_tx),
    .D_IN_0()
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_ssn (
    .PACKAGE_PIN(spi_ssn),
    .OUTPUT_ENABLE(1'b0),
    .D_OUT_0(1'b0),
    .D_IN_0(ssn_pin)
  );

  littlesoc soc (
    .clk(clk),
    // No user button on this board; the power-on counter is the only reset.
    .btn_n(1'b1),
    .ledr_n(ledr_n),
    .ledg_n(ledg_n),
    .uart_tx(uart_tx),
    // The on-chip flash controller's pins are not wired: pin 16 reading high
    // cannot tell a host idle with its select parked high from no host at all.
    .spi_sck(),
    .spi_mosi(),
    .spi_miso(1'b1),
    .spi_cs_n()
  );
endmodule
