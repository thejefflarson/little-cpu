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
  always_ff @(posedge clk) begin
    if (reset)
      out.pc <= 32'b0;
      out.instr <= 32'b0;
    else begin
      out.instr <= imem_data;
      out.pc <= pc;
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always_comb if(!clocked) assume(reset);

  always_ff @(posedge clk) if(clocked && $past(reset)) assert(out.pc == 32'b0);
  // TODO: validate
 `endif
endmodule
