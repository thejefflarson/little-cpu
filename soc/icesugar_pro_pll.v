`timescale 1 ns / 1 ps
`default_nettype none
module icesugar_pro_pll (
  input  wire clk_pad,   // 25 MHz
  output wire clk_core,  // 30 MHz
  output wire locked
);
(* FREQUENCY_PIN_CLKI="25" *)
(* FREQUENCY_PIN_CLKOP="30" *)
(* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
EHXPLLL #(
    .PLLRST_ENA("DISABLED"),
    .INTFB_WAKE("DISABLED"),
    .STDBY_ENABLE("DISABLED"),
    .DPHASE_SOURCE("DISABLED"),
    .OUTDIVIDER_MUXA("DIVA"),
    .OUTDIVIDER_MUXB("DIVB"),
    .OUTDIVIDER_MUXC("DIVC"),
    .OUTDIVIDER_MUXD("DIVD"),
    .CLKI_DIV(5),
    .CLKOP_ENABLE("ENABLED"),
    .CLKOP_DIV(20),
    .CLKOP_CPHASE(9),
    .CLKOP_FPHASE(0),
    .FEEDBK_PATH("CLKOP"),
    .CLKFB_DIV(6)
  ) pll_i (
    .RST(1'b0),
    .STDBY(1'b0),
    .CLKI(clk_pad),
    .CLKOP(clk_core),
    .CLKFB(clk_core),
    .CLKINTFB(),
    .PHASESEL0(1'b0),
    .PHASESEL1(1'b0),
    .PHASEDIR(1'b1),
    .PHASESTEP(1'b1),
    .PHASELOADREG(1'b1),
    .PLLWAKESYNC(1'b0),
    .ENCLKOP(1'b0),
    .LOCK(locked)
  );
endmodule
`default_nettype wire
