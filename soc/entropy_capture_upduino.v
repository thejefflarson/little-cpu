`timescale 1 ns / 1 ps
`default_nettype none
// SB_LFOSC with a capture in place of the core: STREAM 0 sends edge intervals, 1 sample-bit frames.
// The capture starts 2^HOLDOFF_LOG2 cycles after configuration so a host has time to listen;
// FAB_LOG2 divides the oscillator in fabric before its edges are timed.
module entropy_capture_upduino #(
  parameter bit INTERNAL_OSC = 1'b0,
  parameter bit STREAM       = 1'b0,
  parameter integer HOLDOFF_LOG2 = 4,
  parameter integer FAB_LOG2     = 0,
  parameter bit     FILL         = 1'b0
) (
  input  logic clk_pin,
  output logic ledr_n,
  output logic ledg_n,
  inout  wire  spi_miso_txd,
  inout  wire  spi_ssn
);
  localparam integer CLOCK_HZ = 12_000_000;

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

  logic lfosc_out, entropy_raw;
  SB_LFOSC lfosc (
    .CLKLFPU(1'b1),
    .CLKLFEN(1'b1),
    .CLKLF(lfosc_out)
  );

  generate
    if (FAB_LOG2 == 0) begin : g_direct
      assign entropy_raw = lfosc_out;
    end else begin : g_divided
      logic [FAB_LOG2-1:0] count = '0;
      always_ff @(posedge lfosc_out) count <= count + 1'b1;
      assign entropy_raw = count[FAB_LOG2-1];
    end
  endgenerate

  logic [2:0] sync = '0;
  always_ff @(posedge clk) sync <= {sync[1:0], entropy_raw};
  logic edge_seen;
  assign edge_seen = sync[1] && !sync[2];

  logic [HOLDOFF_LOG2-1:0] ready_count = '0;
  logic       running;
  assign running = &ready_count;
  always_ff @(posedge clk) if (!running) ready_count <= ready_count + 1'b1;

  logic uart_tx;
  generate
    if (STREAM) begin : g_stream
      entropy_bit_framer #(.CLOCK_HZ(CLOCK_HZ), .FILL(FILL)) framer (
        .clk(clk),
        .running(running),
        .edge_seen(edge_seen),
        .uart_tx(uart_tx)
      );
    end else begin : g_intervals
      entropy_interval_capture #(.CLOCK_HZ(CLOCK_HZ)) capture (
        .clk(clk),
        .running(running),
        .edge_seen(edge_seen),
        .uart_tx(uart_tx)
      );
    end
  endgenerate

  logic ssn_pin;
  logic miso_enable;
  miso_share_enable enable_gate (
    .clk(clk),
    .now(ssn_pin),
    .enable(miso_enable)
  );

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

  assign ledr_n = 1'b1;
  assign ledg_n = !running;
endmodule
`default_nettype wire
