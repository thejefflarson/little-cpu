// The UPduino v3.x around rtl/littlesoc.v.
`timescale 1 ns / 1 ps
`default_nettype none
module upduino_top #(
  parameter bit INTERNAL_OSC = 1'b0
) (
  input  logic clk_pin,
  output logic ledr_n,
  output logic ledg_n,
  // Pin 14 is the UART line and the flash's MISO at once.
  inout  wire  spi_miso_txd,
  inout  wire  spi_sck,
  inout  wire  spi_ssn,
  inout  wire  spi_mosi
);
  logic clk;

  generate
    if (INTERNAL_OSC) begin : g_internal
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
  logic miso_pin;
  logic soc_sck, soc_mosi, soc_cs_n;
  logic flash_grant;

  miso_share_enable enable_gate (
    .clk(clk),
    .now(ssn_pin),
    .enable(miso_enable)
  );

  pin_lockout lockout (
    .clk(clk),
    .want(!soc_cs_n),
    .busy(soc_sck),
    .release_in(ssn_pin),
    .grant(flash_grant)
  );

  localparam logic [5:0] SHARED_PIN = 6'b1010_01;

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_miso_txd (
    .PACKAGE_PIN(spi_miso_txd),
    .OUTPUT_ENABLE(miso_enable && soc_cs_n),
    .D_OUT_0(uart_tx),
    .D_IN_0(miso_pin)
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_ssn (
    .PACKAGE_PIN(spi_ssn),
    .OUTPUT_ENABLE(flash_grant),
    .D_OUT_0(1'b0),
    .D_IN_0(ssn_pin)
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_sck (
    .PACKAGE_PIN(spi_sck),
    .OUTPUT_ENABLE(flash_grant),
    .D_OUT_0(soc_sck),
    .D_IN_0()
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_mosi (
    .PACKAGE_PIN(spi_mosi),
    .OUTPUT_ENABLE(flash_grant),
    .D_OUT_0(soc_mosi),
    .D_IN_0()
  );

  logic entropy_raw;
  SB_LFOSC lfosc (
    .CLKLFPU(1'b1),
    .CLKLFEN(1'b1),
    .CLKLF(entropy_raw)
  );

  littlesoc soc (
    .clk(clk),
    .btn_n(1'b1),
    .entropy_raw(entropy_raw),
    .ledr_n(ledr_n),
    .ledg_n(ledg_n),
    .uart_tx(uart_tx),
    .spi_sck(soc_sck),
    .spi_mosi(soc_mosi),
    .spi_miso(miso_pin),
    .spi_cs_n(soc_cs_n)
  );
endmodule
