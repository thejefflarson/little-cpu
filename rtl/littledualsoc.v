`timescale 1 ns / 1 ps
`default_nettype none
// rtl/littledual.v with its pins and a power-on reset. ECP5 only: two fetch
// windows are two copies of the banked ROM, 32 block RAMs against the up5k's
// 30.
module littledualsoc (
  // Named for rtl/littlesoc.v's own ports, so soc/littlesoc.lpf constrains this
  // top too -- only `clk` is located there; nextpnr places the rest anywhere
  // (`--lpf-allow-unconstrained`).
  input  logic clk,
  input  logic btn_n,
  output logic ledr_n,
  output logic ledg_n
);
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

  logic trap_seen;
  always_ff @(posedge clk) begin
    if (reset) trap_seen <= 1'b0;
    else if (|trap) trap_seen <= 1'b1;
  end
  assign ledr_n = !trap_seen;
  assign ledg_n = !por_done;
endmodule // littledualsoc
