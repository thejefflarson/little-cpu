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
  output logic uart_tx
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

  littlesoc soc (
    .clk(clk),
    .btn_n(1'b1),
    .ledr_n(ledr_n),
    .ledg_n(ledg_n),
    .uart_tx(uart_tx)
  );
endmodule
