`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// Windows a decoded instruction out of the queue's head pair; `pop` fires when next_pc leaves the word `pc` names, and `pop2` with it when a straddling instruction leaves its second word too.
module fetcher(
  input  logic clk,
  input  logic reset,
  input  logic [31:0] pc,
  input  logic [31:0] next_pc,
  input  logic [31:0] q0,
  input  logic [31:0] q1,
  output logic         pop,
  output logic         pop2,
  output fetcher_output out
);

  assign pop = next_pc[31:2] != pc[31:2];

  logic [63:0] fetch_pair;
  assign fetch_pair = {q1, q0} >> (pc[1] ? 16 : 0);
  logic [31:0] windowed_instr;
  assign windowed_instr = fetch_pair[31:0];
  logic straddles;
  assign straddles = pc[1] && windowed_instr[1:0] == 2'b11;
  assign pop2 = pop && straddles && next_pc[31:2] != pc[31:2] + 30'd1;

  logic [31:0] next_word;
  assign next_word = (windowed_instr[1:0] == 2'b11) ? fetch_pair[63:32]
                                                    : fetch_pair[47:16];

  always_comb begin
    if (reset) begin
      out.valid = 1'b0;
      out.pc = 32'b0;
      out.instr = 32'b0;
      out.next_instr = 32'b0;
    end else begin
      out.valid = 1'b1;
      out.instr = windowed_instr;
      out.next_instr = next_word;
      out.pc = pc;
    end
  end

endmodule
