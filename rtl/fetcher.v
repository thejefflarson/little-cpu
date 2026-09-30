`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// The ROM is addressed a cycle ahead with `next_pc`'s word, so its output register always
// holds the window `{w[p], w[p+1]}` at `pc` and the only miss is a stolen read.
module fetcher (
  input  logic clk,
  input  logic reset,
  input  logic [31:0] pc,
  input  logic [31:0] next_pc,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  output logic [31:0] imem_addr_next,
  input  logic        imem_stall,
  input  logic        imem_fault,
  output logic        fetch_stall,
  output logic        fault,
  output fetcher_output out
);
  assign fetch_stall    = imem_stall;
  assign fault          = imem_fault;
  assign imem_addr_next = reset ? 32'b0 : {next_pc[31:2], 2'b00};
  assign imem_addr      = {pc[31:2], 2'b00};
  assign imem_addr2     = imem_addr + 32'd4;

  logic [47:0] fetch_pair;
  assign fetch_pair = {imem_data2[15:0], imem_data} >> (pc[1] ? 16 : 0);
  logic [31:0] windowed_instr;
  assign windowed_instr = fetch_pair[31:0];

  always_comb begin
    if (reset) begin
      out.valid = 1'b0;
      out.pc = 32'b0;
      out.instr = 32'b0;
    end else begin
      out.valid = 1'b1;
      out.instr = windowed_instr;
      out.pc = pc;
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;

  logic [31:0] past_imem_addr_next;
  always_ff @(posedge clk) past_imem_addr_next <= imem_addr_next;
  always_comb if (clocked) assert(imem_addr == past_imem_addr_next);

  logic prev_fetch_stall;
  always_ff @(posedge clk) prev_fetch_stall <= fetch_stall;
  always_ff @(posedge clk) if (clocked && !reset)
    miss_then_hit: cover (prev_fetch_stall && !fetch_stall);
 `endif
endmodule
