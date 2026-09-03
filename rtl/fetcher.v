`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module fetcher(
  input  logic clk,
  input  logic reset,
  input  logic [31:0] pc,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  // `imem_addr` one cycle early, so a synchronous memory answers in the cycle
  // `imem_addr` names the word. Only the first word address is published; the
  // memory computes the `+ 4` itself.
  input  logic [31:0] next_pc,
  output logic [31:0] imem_addr_next,
  output fetcher_output out
);

  assign imem_addr  = {pc[31:2], 2'b00};
  assign imem_addr2 = imem_addr + 4;
  assign imem_addr_next = {next_pc[31:2], 2'b00};

  // Named continuous assigns rather than part-selects inside the always_comb
  // below: iverilog cannot build a precise sensitivity entry for a constant
  // select there.
  logic [63:0] fetch_pair;
  assign fetch_pair = {imem_data2, imem_data} >> (pc[1] ? 16 : 0);
  logic [31:0] windowed_instr;
  assign windowed_instr = fetch_pair[31:0];

  // The word after the instruction. Decode uses it as a guess at the next
  // register pair and checks the guess, so it need not be right.
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
      // Fetch never presents a wrong-path instruction, so it is valid on
      // every non-reset cycle.
      out.valid = 1'b1;
      out.instr = windowed_instr;
      out.next_instr = next_word;
      out.pc = pc;
    end
  end

endmodule
