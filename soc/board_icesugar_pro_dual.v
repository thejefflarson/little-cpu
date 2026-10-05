`timescale 1 ns / 1 ps
`default_nettype none
module icesugar_pro_dual_top (
  input  logic clk_pin,
  output logic led_r_n,
  output logic led_g_n,
  output logic uart_tx
);
  localparam integer PAD_HZ  = 25_000_000;
  localparam integer CORE_HZ = 30_000_000;

  logic clk_core, pll_locked;

  icesugar_pro_pll pll (
    .clk_pad(clk_pin),
    .clk_core(clk_core),
    .locked(pll_locked)
  );

  littledualsoc #(.CLOCK_HZ(CORE_HZ)) soc (
    .clk(clk_core),
    .btn_n(pll_locked),
    .ledr_n(led_r_n),
    .ledg_n(led_g_n),
    .uart_tx(uart_tx)
  );
endmodule
`default_nettype wire
