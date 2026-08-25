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
// ---- PIN 14 IS SHARED, AND ONLY PIN 14 IS WIRED HERE -----------------------
//
// The vendor's own constraint file gives pin 14 two names: `serial_txd` and
// `spi_miso`. So the FPGA's only route to the USB serial port is also the
// on-board configuration flash's own data-out line, and there is no other pin
// that reaches a host. Driving pin 14 unconditionally fought that flash chip
// for the wire during a real flashing session -- `cdone` read high after a
// reset that should have lowered it, a JEDEC id arrived a byte late as
// `FF EF 70 16`, and then a write failed -- because this board's CRESET is not
// wired to the programmer, so the FPGA keeps running, and keeps driving pin
// 14, throughout `iceprog`'s own access to the same wire.
//
// THE FIX READS PIN 16 RATHER THAN GUESSING. Pin 16 is the flash's own chip
// select, and an SPI slave's output stage is a deterministic function of its
// own chip select: LOW, it drives; HIGH, it is high-impedance, no matter who
// or what is holding that pin high. So "does pin 16 read high" is a sound
// question at DC -- the level pin 16 settles to does not depend on whether a
// pull-up or `iceprog` itself is what holds it there. It is NOT sound to ask
// only through a synchroniser, though: soc/miso_share_enable.v gates turn-ON
// through one (two cycles, because `now` crosses from a source this design
// has no clock relationship with), but turns the enable OFF combinationally,
// on the raw read, the instant pin 16 goes low. A synchronised-only enable
// would keep this design driving pin 14 for up to two clock periods after
// the flash's own driver is already live on it -- two push-pull drivers on
// one pin, on every chip-select assertion a host makes. Nothing here ever
// writes pin 16, so there is no loop through the pad to freeze against,
// unlike the mechanism in soc/pin_lockout.v this board does not use.
//
// PINS 15 AND 17 ARE NOT WIRED HERE AT ALL. `rtl/spiflash.v` -- the on-chip
// controller for the same flash -- ships on `littlesoc`'s bus and is
// reachable in simulation, but this board does not connect it to a physical
// pin. Its own chip select is a different question from the flash's, and
// `released` (soc/pin_lockout.v's own predicate) cannot tell a host that is
// idle with the flash's chip select parked high from one that was never
// there -- `iceprog` parks it exactly that way, as a push-pull output,
// between its own transactions, which is most of a session and not a corner
// of it. Granting the on-chip controller those two pins on that predicate
// risks driving them into the FTDI's own driver. `soc/pin_lockout.v` still
// ships, bounded so a hung on-chip request cannot hold the pins forever once
// something DOES wire a requester to it, but wiring it to this board's real
// pins is a future ticket's, and it owes a real-board measurement of the
// contention window described above, not another RTL argument.
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
  // Pin 14, shared with the flash's data-out. Pin 16, the flash's chip
  // select, read only -- see the note above for why this board never drives
  // it.
  inout  wire  spi_miso_txd,
  inout  wire  spi_ssn
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
  logic ssn_pin;
  logic miso_enable;

  // Turn-on synchronised, turn-off combinational -- see soc/miso_share_enable.v
  // and the header comment above for why the two directions need different
  // treatment. `ssn_pin` is read here only, so there is no loop through the
  // pad for this design's own drive to close: nothing below ever writes pin
  // 16, unlike the mechanism in soc/pin_lockout.v this board does not use.
  miso_share_enable enable_gate (
    .clk(clk),
    .now(ssn_pin),
    .enable(miso_enable)
  );

  // PIN_TYPE 6'b1010_01: a simple output behind an output enable, and a
  // simple input. Pin 14 uses both halves; pin 16 only the input half, with
  // its output enable tied low so this design never drives it.
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
    .btn_n(1'b1),
    .ledr_n(ledr_n),
    .ledg_n(ledg_n),
    .uart_tx(uart_tx),
    .spi_sck(),
    .spi_mosi(),
    .spi_miso(1'b1),
    .spi_cs_n()
  );
endmodule
