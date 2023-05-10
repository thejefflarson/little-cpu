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
  always_ff @(posedge clk) begin
    if(reset) begin
      wen <= 0;
      waddr <= 0;
      wdata <= 32'b0;
    end else begin
      if (1) begin
        wen <= 1;
        waddr <= in.rd;
        wdata <= in.rd_data;
      end else begin
        wen <= 0;
      end
    end
  end // always_ff @ (posedge clk)
 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always_ff @(posedge clk) if(clocked && $past(accessor_valid)) assert(wen == 1);
 `endif
endmodule
