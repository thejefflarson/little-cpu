`timescale 1 ns / 1 ps
`default_nettype none
// The dual chip: rtl/littledual.v plus the four pins and the power-on reset,
// which is all rtl/littlesoc.v is on top of its own complex. Kept separate from
// that complex so test/dual_testbench.v can instantiate the same dual wiring
// with a bigger ROM, no init files and the two resets driven apart -- one copy
// of the arbitration, placed and simulated.
//
// The pins are rtl/littlesoc.v's, so soc/littlesoc.lpf constrains this too. Do
// not add a pin without adding it there: nextpnr is run with
// `--lpf-allow-unconstrained`, so an unconstrained pin is placed wherever and
// says nothing.
//
// ECP5 ONLY. Two fetch windows are two copies of the banked ROM -- 32 block
// RAMs against the up5k's 30 -- so there is no up5k build of this and
// `make soc-timing` still measures the single-hart SoC it always did.
module littledualsoc (
  input  logic clk,
  input  logic btn_n,
  // Watchers, the way rtl/littlesoc.v's are: a design whose output nothing can
  // see is a design yosys deletes.
  output logic ledr_n,
  output logic ledg_n
);
  // rtl/littlesoc.v's reset, generated once and broadcast to both harts. There
  // is no per-hart release on hardware: both come out of reset together and
  // test/crt0.S sends them different ways on `mhartid`.
  logic [3:0] por_count = 4'b0;
  logic       por_done  = 1'b0;
  logic [1:0] btn_sync  = 2'b0;
  logic       reset     = 1'b1;
  always_ff @(posedge clk) begin
    if (!por_done) begin
      por_count <= por_count + 4'd1;
      if (por_count == 4'hf) por_done <= 1'b1;
    end
    btn_sync <= {btn_sync[0], btn_n};
    reset    <= !por_done || !btn_sync[1];
  end

  logic [1:0] trap;
  littledual #(
    .ROM_WORDS(2048),
    .INIT_EVEN("soc/rom_even.hex"),
    .INIT_ODD("soc/rom_odd.hex")
  ) complex (
    .clk(clk),
    .reset({reset, reset}),
    .trap(trap)
  );

  // Either hart trapping lights the red LED. There are two harts and one LED,
  // and which one trapped is a question for the bench and not for a pin.
  logic trap_seen;
  always_ff @(posedge clk) begin
    if (reset) trap_seen <= 1'b0;
    else if (|trap) trap_seen <= 1'b1;
  end
  assign ledr_n = !trap_seen;
  assign ledg_n = !por_done;
endmodule // littledualsoc
