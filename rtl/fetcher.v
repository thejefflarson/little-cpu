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
  // The same two fetch addresses, one cycle early. `next_pc` is the decoder's
  // combinational next PC, so `imem_addr_next` is by construction the value
  // `imem_addr` takes on the next edge, and a synchronous memory that latches
  // it answers `imem_addr` for the whole of the cycle in which `imem_addr`
  // names that word -- which is what lets fetch stay combinational from
  // decode's point of view on a part whose every memory primitive is
  // synchronous.
  //
  // Only the first word address is published: the second is `+ 4`, which the
  // memory already computes for its own bank indexing.
  input  logic [31:0] next_pc,
  output logic [31:0] imem_addr_next,
  output fetcher_output out
);

  // Two adjacent words, windowed combinationally. A 32-bit instruction can
  // straddle a 4-byte boundary when pc % 4 == 2, so fetch always reads the
  // word-aligned pair straddling pc and windows the 32 bits starting at pc out
  // of the two -- stateless, so a straddle costs nothing: no aligner FSM, no
  // buffer, no stall.
  assign imem_addr  = {pc[31:2], 2'b00};
  assign imem_addr2 = imem_addr + 4;
  assign imem_addr_next = {next_pc[31:2], 2'b00};

  // Named continuous assigns, not part-selects inside the always_comb below:
  // iverilog cannot build a precise sensitivity entry for a constant select
  // there and emits `sorry:`. rtl/writeback.v is the one file that carries
  // those; do not add more anywhere else.
  logic [63:0] fetch_pair;
  assign fetch_pair = {imem_data2, imem_data} >> (pc[1] ? 16 : 0);
  logic [31:0] windowed_instr;
  assign windowed_instr = fetch_pair[31:0];

  // The window is 64 bits and the instruction being presented is either 2 or 4
  // bytes of it, so the word after that instruction is already here -- a second
  // slice of the same shift, not a second read. Decode asks the register file
  // for this word's register numbers a cycle early and checks what it asked
  // for, so nothing here has to be right. It goes out raw: rtl/regsel.v takes
  // any 32-bit word, and a compressed successor's upper half is masked there,
  // in the same module that decides which fields are register numbers.
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
