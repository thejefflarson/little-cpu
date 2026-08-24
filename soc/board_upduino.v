// The UPduino v3.x carried into rtl/littlesoc.v's shape. A board file and not
// core RTL: everything here is a fact about one piece of hardware, and nothing
// under rtl/ should have to know which board it lands on.
//
// Three things this board does not share with the iCEBreaker that
// soc/littlesoc.pcf was written for:
//
// NO BUTTON. `littlesoc` takes `btn_n` as an asynchronous active-low reset
// beside its power-on counter. There is no user button here, so it is tied
// released and the POR is the only reset -- which is what actually resets the
// part on the iCEBreaker too, since the button is a convenience. Tied rather
// than left to a pin, because a floating input through those two sync flops
// would hold the core in reset at random.
//
// THE CRYSTAL IS NOT CONNECTED FROM THE FACTORY. The 12 MHz oscillator reaches
// the FPGA on pin 20 only if R16 -- silkscreened OSC -- is shorted. Set
// INTERNAL_OSC to build against `SB_HFOSC` instead, which needs no iron. Read
// its comment below before choosing: the two are not interchangeable for the
// UART.
//
// THE LEDs ARE THE RGB LED. Same active-low sense `littlesoc` already drives,
// on the three dedicated pins the RGB driver block also uses. Driving them as
// ordinary IO is what the vendor's own example does; cut R28 if the current
// draw is unwanted.
//
// ---- FOUR PINS, TWO OWNERS, AND WHY NONE OF THEM MOVED --------------------
//
// The vendor's own constraint file gives one pin two names each:
//
//     serial_txd 14  = spi_miso 14      serial_rxd 15 = spi_sck 15
//     spi_ssn    16                     spi_mosi 17
//
// So the FPGA's only path to the USB serial port is pin 14, and pin 14 is also
// the flash's data output. There is no third pin to move the UART to that a
// host can read -- the other headers go nowhere but a wire -- and there is no
// way to read the flash without pin 14 either. The two have to share it, and
// the only question is who drives it when.
//
// THE FLASH'S CHIP SELECT IS THE ARBITER, and it is the one signal both owners
// can see. While the on-chip master holds the flash selected, pin 14 is an
// input and the flash drives it. While the HOST holds it selected -- `iceprog`
// drives pin 16 from the FTDI, and the FPGA is still running its old design
// throughout, because this board's CRESET is not wired to the programmer --
// pin 14 is released as well, and the flash has the wire to itself. The rest of
// the time nobody is talking to the flash, so the UART drives pin 14 and the
// serial port works exactly as it did.
//
// That last arm is the bug this arrangement fixes rather than a nicety. With
// the UART driving pin 14 unconditionally, a board replaying a report fought
// the flash for the wire while `iceprog` read it: `cdone` high after a reset
// that should have lowered it, a JEDEC id arriving a byte late as
// `FF EF 70 16`, and then a write error. soc/run_suite_board.sh retried four
// times to get around it. Nothing here has to retry.
//
// THE PULL-UPS ARE LOAD-BEARING. Three of these four pins are released most of
// the time, and a released pin is an input with nothing on it: the UART's idle
// level is high, and a chip select read low by accident selects the flash. Both
// are the `SB_IO` pull-up below, not the board's -- the vendor's file says to
// drive pin 16 high unless the flash is in use, which is what a part without a
// pull-up on it needs.
`timescale 1 ns / 1 ps
`default_nettype none
module upduino_top #(
  // 0 takes the 12 MHz crystal off pin 20 and requires R16 shorted.
  // 1 takes `SB_HFOSC`, the internal oscillator, divided to 12 MHz.
  //
  // THE UART IS WHAT DECIDES THIS. `SB_HFOSC` is trimmed to about +/-10%, and
  // rtl/uart.v divides the clock by 104 for 115200 baud, which the header
  // states a 0.16% error for. Ten percent of clock error swamps that and the
  // FTDI at the other end will read garbage. So: internal to see the core run
  // with no soldering, crystal for anything that has to be read.
  parameter bit INTERNAL_OSC = 1'b0
) (
  input  logic clk_pin,
  output logic ledr_n,
  output logic ledg_n,
  // The four shared pins above. Declared `inout` because three of them really
  // are bidirectional here and the fourth is read back.
  inout  wire  spi_miso_txd,
  inout  wire  spi_sck,
  inout  wire  spi_ssn,
  inout  wire  spi_mosi
);
  logic clk;

  generate
    if (INTERNAL_OSC) begin : g_internal
      // CLKHF_DIV divides 48 MHz: 0b00 is 48, 0b01 24, 0b10 12, 0b11 6. Both
      // enables are tied on because nothing here powers the oscillator down.
      // `clk_pin` is unused in this arm; soc/upduino.pcf constrains it with
      // -nowarn so the unused port is not an error.
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
  logic soc_sck, soc_mosi, soc_miso, soc_cs_n;

  // The on-chip master owns the flash exactly while it holds the select low.
  logic own_flash;
  assign own_flash = !soc_cs_n;

  // Pin 16's level, read back whenever the master is not driving it. That is
  // where the host's chip select shows up.
  logic ssn_in;

  // PIN_TYPE 6'b1010_01: a simple output behind an output enable, and a simple
  // input. The same eight bits for all four, because all four are the same
  // shape -- what differs is only what raises the enable.
  localparam logic [5:0] SHARED_PIN = 6'b1010_01;

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_miso_txd (
    .PACKAGE_PIN(spi_miso_txd),
    // The UART has pin 14 only while neither owner holds the select low.
    .OUTPUT_ENABLE(!own_flash && ssn_in),
    .D_OUT_0(uart_tx),
    .D_IN_0(soc_miso)
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_sck (
    .PACKAGE_PIN(spi_sck),
    .OUTPUT_ENABLE(own_flash),
    .D_OUT_0(soc_sck),
    .D_IN_0()
  );

  // Driven low to select, released to deselect: the pull-up is what deselects
  // it, and releasing is also what lets the level above be read.
  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_ssn (
    .PACKAGE_PIN(spi_ssn),
    .OUTPUT_ENABLE(own_flash),
    .D_OUT_0(1'b0),
    .D_IN_0(ssn_in)
  );

  SB_IO #(.PIN_TYPE(SHARED_PIN), .PULLUP(1'b1)) io_mosi (
    .PACKAGE_PIN(spi_mosi),
    .OUTPUT_ENABLE(own_flash),
    .D_OUT_0(soc_mosi),
    .D_IN_0()
  );

  littlesoc soc (
    .clk(clk),
    .btn_n(1'b1),
    .ledr_n(ledr_n),
    .ledg_n(ledg_n),
    .uart_tx(uart_tx),
    .spi_sck(soc_sck),
    .spi_mosi(soc_mosi),
    .spi_miso(soc_miso),
    .spi_cs_n(soc_cs_n)
  );
endmodule
