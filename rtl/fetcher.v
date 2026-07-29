`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module fetcher(
  input  logic clk,
  input  logic reset,
  // inputs
  input  logic [31:0] pc,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  // outputs
  output fetcher_output out
);

  assign imem_addr = pc;
  always_comb begin
    if (reset) begin
      out.valid = 1'b0;
      out.pc = 32'b0;
      out.instr = 32'b0;
    end else begin
      // Fetch never presents a wrong-path instruction (CLAUDE.md invariant 1),
      // so it is valid on every non-reset cycle.
      out.valid = 1'b1;
      out.instr = imem_data;
      out.pc = pc;
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always_comb if(!clocked) assume(reset);

 `endif
endmodule
