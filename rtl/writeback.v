`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module writeback(
  input  logic clk,
  input  logic reset,
  // inputs
  input accessor_output in,
  // outputs
  output logic wen,
  output logic [4:0] waddr,
  output logic [31:0] wdata
);
  always_comb begin
    if(reset) begin
      wen = 0;
      waddr = 0;
      wdata = 32'b0;
    end else begin
      // Retire is `valid` reaching writeback (CLAUDE.md invariant 3): a
      // bubble must never commit a register write, so wen is gated on both
      // valid and a non-zero destination.
      wen = in.valid && (in.rd != 0);
      waddr = in.rd;
      wdata = in.rd_data;
    end
  end // always_comb
  // TODO: csrs

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
 `endif
endmodule
