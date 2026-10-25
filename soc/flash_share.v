`timescale 1 ns / 1 ps
`default_nettype none
// Output enables for the UPduino's shared pins; the pin roles and the order of events are in ADR-0135's
// 2026-10-24 amendment. Pin 14 carries the UART's line and the flash's data input, pin 17 only its output.
module flash_share #(
  parameter int IDLE_LIMIT = 128
) (
  input  logic clk,
  input  logic cs_n,
  input  logic sck,
  input  logic mosi,
  input  logic uart_tx,
  input  logic ssn_in,
  output logic txd_oe,
  output logic txd_d,
  output logic sck_oe,
  output logic ssn_oe,
  output logic ssn_d
);
  logic grant, grant_q, miso_enable;

  initial grant_q = 1'b0;

  always_ff @(posedge clk) grant_q <= grant;

  miso_share_enable enable_gate (
    .clk(clk),
    .now(ssn_in),
    .enable(miso_enable)
  );

  pin_lockout #(.IDLE_LIMIT(IDLE_LIMIT)) lockout (
    .clk(clk),
    .want(!cs_n),
    .busy(sck),
    .release_in(ssn_in),
    .grant(grant)
  );

  assign txd_oe = miso_enable || grant;
  assign txd_d  = grant ? mosi : uart_tx;
  assign sck_oe = grant;
  assign ssn_oe = grant || grant_q;
  assign ssn_d  = !grant;
endmodule
