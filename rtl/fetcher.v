`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// Whole words are requested from the ROM by a registered address that runs ahead of decode, and
// land in a three-word queue decode reads from registers only. Nothing decode computes this
// cycle reaches the ROM address: a redirect arrives a register late, as does the guess.
module fetcher (
  input  logic clk,
  input  logic reset,
  input  logic [31:0] pc,
  input  logic        issuing,
  input  logic        refetch,
  input  logic        redirect,
  input  logic [31:0] redirect_target,
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
  logic [31:0] q0, q1, q2;
  logic        f0, f1, f2;
  logic [1:0]  count;
  // `req_valid`: last cycle's address was a real request, answered now; `req_addr` is its word.
  logic        req_valid;
  logic [29:0] req_addr;
  logic        refetch_q;

  logic flush, retry, accept, room, request;
  assign flush   = redirect || refetch_q;
  assign retry   = req_valid && imem_stall && !flush;
  assign accept  = req_valid && !imem_stall && !flush;
  assign room    = count != 2'd3 && !(count == 2'd2 && req_valid);
  assign request = flush || retry || room;

  logic [29:0] next_req, redirect_word, pc_word;
  assign redirect_word = redirect_target[31:2];
  assign pc_word       = pc[31:2];
  assign next_req = redirect  ? redirect_word :
                    refetch_q ? pc_word :
                    retry     ? req_addr :
                                req_addr + 30'd1;
  assign imem_addr_next = reset ? 32'b0 : {next_req, 2'b00};
  assign imem_addr      = {req_addr, 2'b00};
  assign imem_addr2     = 32'b0;

  logic [15:0] head_lo;
  logic        uncompressed, straddle;
  assign head_lo      = pc[1] ? q0[31:16] : q0[15:0];
  assign uncompressed = head_lo[1:0] == 2'b11;
  assign straddle     = pc[1] && uncompressed;

  logic avail;
  assign avail = !flush && count != 2'd0 && !(straddle && count == 2'd1);
  assign fetch_stall = !avail;
  assign fault = f0 || (straddle && f1);

  logic [31:0] window;
  assign window = pc[1] ? {q1[15:0], q0[31:16]} : q0;
  assign out.valid = avail && !reset;
  assign out.pc    = pc;
  assign out.instr = window;

  logic pop;
  assign pop = issuing && (pc[1] || uncompressed);

  logic [1:0] after_pop;
  assign after_pop = count - {1'b0, pop};
  logic wr0, wr1, wr2;
  assign wr0 = accept && after_pop == 2'd0;
  assign wr1 = accept && after_pop == 2'd1;
  assign wr2 = accept && after_pop == 2'd2;

  always_ff @(posedge clk) begin
    if (reset) begin
      req_valid <= 1'b0;
      refetch_q <= 1'b1;
      count     <= 2'd0;
    end else begin
      req_valid <= request;
      refetch_q <= issuing && refetch;
      count     <= flush ? 2'd0 : after_pop + {1'b0, accept};
    end
    if (request) req_addr <= next_req;

    if (wr0) begin q0 <= imem_data; f0 <= imem_fault; end
    else if (pop) begin q0 <= q1; f0 <= f1; end
    if (wr1) begin q1 <= imem_data; f1 <= imem_fault; end
    else if (pop) begin q1 <= q2; f1 <= f2; end
    if (wr2) begin q2 <= imem_data; f2 <= imem_fault; end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;

  always_comb if (clocked) assert(count <= 2'd3);
  always_comb if (clocked) assert({1'b0, count} + {2'b0, req_valid} <= 3'd3);
  always_comb if (clocked && flush) assert(!avail);
 `endif
endmodule
