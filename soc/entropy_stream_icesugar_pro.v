`timescale 1 ns / 1 ps
`default_nettype none
// The seed source of the board with a gapless sample-bit stream in place of the core.
module entropy_stream_icesugar_pro #(
  parameter integer OSC_DIV  = 128,  // OSCG divider, 2..128
  parameter integer FAB_LOG2 = 9,    // fabric divide-by-2^FAB_LOG2, at least 1
  parameter integer CORE_PAD = 0     // 1 clocks the capture from the 25 MHz pad, 0 from the PLL
) (
  input  logic clk_pin,
  output logic led_r_n,
  output logic led_g_n,
  output logic uart_tx
);
  localparam integer CLOCK_HZ = CORE_PAD ? 25_000_000 : 30_000_000;

  logic clk_core, pll_locked;
  icesugar_pro_pll pll (
    .clk_pad(clk_pin),
    .clk_core(clk_core),
    .locked(pll_locked)
  );

  logic clk;
  assign clk = CORE_PAD ? clk_pin : clk_core;

  logic osc_slow;
  icesugar_pro_entropy #(.DIV(OSC_DIV), .DIV_LOG2(FAB_LOG2)) entropy (.raw(osc_slow));

  logic [2:0] sync = '0;
  always_ff @(posedge clk) sync <= {sync[1:0], osc_slow};

  logic [3:0] ready_count = '0;
  logic       running;
  assign running = &ready_count;
  always_ff @(posedge clk) begin
    if (!(CORE_PAD != 0 || pll_locked)) ready_count <= '0;
    else if (!running) ready_count <= ready_count + 1'b1;
  end

  entropy_bit_framer #(.CLOCK_HZ(CLOCK_HZ)) framer (
    .clk(clk),
    .running(running),
    .edge_seen(sync[1] && !sync[2]),
    .uart_tx(uart_tx)
  );

  assign led_r_n = 1'b1;
  assign led_g_n = !running;
endmodule
`default_nettype wire
