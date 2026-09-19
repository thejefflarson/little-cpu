`timescale 1 ns / 1 ps
`default_nettype none
// The depth-2 skid named as rtl/fetchqueue.v's fallback if the up5k does not place at
// depth 4: exactly one word pair in flight, so a push always lands in the one empty pair
// and a read is always mem0/mem1 -- no head pointer, no array index, no output mux. Room
// opens only once BOTH words of the resident pair are gone, so a word crossing (`pop`)
// drops q_valid until the next pair lands; decode pays that stall every crossing, unlike
// depth 4's double buffering. Same req_valid contract as rtl/fetchqueue.v.
module fetchqueue2 (
  input  logic         clk,
  input  logic         reset,
  input  logic         flush,
  input  logic         req_valid,
  input  logic [31:0]  imem_data,
  input  logic [31:0]  imem_data2,
  input  logic         imem_fault,
  input  logic         pop,
  output logic [31:0]  q0,
  output logic         q0_fault,
  output logic [31:0]  q1,
  output logic         q1_fault,
  output logic         q_valid,
  output logic [2:0]   count,
  output logic         room
);
  logic [31:0] mem0, mem1;
  logic        fault0, fault1;
  // 0, 1 (mem0 already popped, mem1 still live) or 2 words resident.
  logic [1:0]  cnt;

  logic do_pop;
  assign do_pop = pop && (cnt != 2'd0);

  always_ff @(posedge clk) begin
    if (reset || flush) begin
      cnt <= 2'd0;
    end else begin
      if (req_valid) begin
        mem0   <= imem_data;
        fault0 <= imem_fault;
        mem1   <= imem_data2;
        fault1 <= imem_fault;
      end else if (do_pop && cnt == 2'd2) begin
        // The word that survives a single pop slides down to mem0 -- the "no output
        // mux" shape reads mem0 unconditionally, so the slide has to happen in the
        // register instead of in the read path.
        mem0   <= mem1;
        fault0 <= fault1;
      end
      cnt <= cnt - (do_pop ? 2'd1 : 2'd0) + (req_valid ? 2'd2 : 2'd0);
    end
  end

  assign q0       = mem0;
  assign q0_fault = fault0;
  assign q1       = mem1;
  assign q1_fault = fault1;
  // A word crossing (`pop`) only ever fires exactly at a word boundary, so the word
  // decode reaches immediately after one (cnt drops to 1, mem0 holding the slid-down
  // survivor) is never itself a straddle into the not-yet-fetched next pair -- q1 is
  // read only when a straddling instruction genuinely needs it, which cannot happen on
  // the very cycle a crossing was just taken. So one live word is enough to proceed;
  // gating on both, as the depth-4 queue's q_valid does, deadlocks here -- decode can
  // never pop the second word without first being allowed to read it.
  assign q_valid  = cnt != 2'd0;
  assign count    = {1'b0, cnt};

  logic [1:0] committed;
  assign committed = cnt + (req_valid ? 2'd2 : 2'd0);
  assign room = committed == 2'd0;

 `ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;

  always_comb assume(!req_valid || cnt == 2'd0);

  always_comb if (clocked) assert(cnt <= 2'd2);
  always_ff @(posedge clk)
    if (clocked) assert(!$past(flush) || cnt == 2'd0);
 `endif
endmodule
