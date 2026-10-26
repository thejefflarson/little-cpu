// The UPduino v3.x around rtl/littlesoc.v.
`timescale 1 ns / 1 ps
`default_nettype none
module upduino_top #(
  parameter bit INTERNAL_OSC = 1'b0
) (
  input  logic clk_pin,
  output logic ledr_n,
  output logic ledg_n,
  // Pin 14 is the UART line and the flash's data input at once; pin 17 is the flash's data output.
  inout  wire  spi_mosi_txd,
  inout  wire  spi_sck,
  inout  wire  spi_ssn,
  inout  wire  spi_miso
);
  // The divide sets trng's edge period (about 9,600 cycles); test/entropy_geometry_test.py grades it.
  localparam integer CORE_HZ = 12_000_000;
  localparam integer ENTROPY_DIV_LOG2 = 3;

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

  logic uart_tx, miso_pin, ssn_pin;
  logic soc_sck, soc_mosi, soc_cs_n;
  logic txd_oe, txd_d, sck_oe, ssn_oe, ssn_d;

  flash_share share (
    .clk(clk),
    .cs_n(soc_cs_n),
    .sck(soc_sck),
    .mosi(soc_mosi),
    .uart_tx(uart_tx),
    .ssn_in(ssn_pin),
    .txd_oe(txd_oe),
    .txd_d(txd_d),
    .sck_oe(sck_oe),
    .ssn_oe(ssn_oe),
    .ssn_d(ssn_d)
  );

  localparam logic [5:0] SHARED_PIN = 6'b1010_01;

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_mosi_txd (
    .PACKAGE_PIN(spi_mosi_txd),
    .OUTPUT_ENABLE(txd_oe),
    .D_OUT_0(txd_d),
    .D_IN_0()
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_sck (
    .PACKAGE_PIN(spi_sck),
    .OUTPUT_ENABLE(sck_oe),
    .D_OUT_0(soc_sck),
    .D_IN_0()
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_ssn (
    .PACKAGE_PIN(spi_ssn),
    .OUTPUT_ENABLE(ssn_oe),
    .D_OUT_0(ssn_d),
    .D_IN_0(ssn_pin)
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_miso (
    .PACKAGE_PIN(spi_miso),
    .OUTPUT_ENABLE(1'b0),
    .D_OUT_0(1'b0),
    .D_IN_0(miso_pin)
  );

  logic lfosc_out;
  SB_LFOSC lfosc (
    .CLKLFPU(1'b1),
    .CLKLFEN(1'b1),
    .CLKLF(lfosc_out)
  );

  logic [ENTROPY_DIV_LOG2-1:0] entropy_count = '0;
  always_ff @(posedge lfosc_out) entropy_count <= entropy_count + 1'b1;
  logic entropy_raw;
  assign entropy_raw = entropy_count[ENTROPY_DIV_LOG2-1];

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
