`timescale 1 ns / 1 ps
`default_nettype none
// A 4-word FIFO of individually fetched words: q0/q1 are the pair at the issuing pc.
// req_valid marks a genuine due response, never a stolen-cycle retry -- the caller
// drives it as `waiting && !fetch_stall`, `waiting` next-stating `fetch_stall ? 1 : launch`.
module fetchqueue (
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
  localparam int DEPTH = 4;

  logic [31:0] mem      [0:DEPTH-1];
  logic        fault_mem[0:DEPTH-1];
  logic [1:0]  head, tail;
  logic [2:0]  cnt;

  logic do_pop;
  assign do_pop = pop && (cnt != 3'd0);

  always_ff @(posedge clk) begin
    if (reset || flush) begin
      head <= 2'd0;
      tail <= 2'd0;
      cnt  <= 3'd0;
    end else begin
      if (do_pop) head <= head + 2'd1;
      if (req_valid) begin
        mem[tail]              <= imem_data;
        fault_mem[tail]        <= imem_fault;
        mem[tail + 2'd1]       <= imem_data2;
        fault_mem[tail + 2'd1] <= imem_fault;
        tail <= tail + 2'd2;
      end
      cnt <= cnt - (do_pop ? 3'd1 : 3'd0) + (req_valid ? 3'd2 : 3'd0);
    end
  end

  assign q0       = mem[head];
  assign q0_fault = fault_mem[head];
  assign q1       = mem[head + 2'd1];
  assign q1_fault = fault_mem[head + 2'd1];
  assign q_valid  = cnt >= 3'd2;
  assign count    = cnt;

  logic [2:0] committed;
  assign committed = cnt + (req_valid ? 3'd2 : 3'd0);
  assign room = committed <= (DEPTH - 2);

 `ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;

  always_comb assume(!req_valid || cnt <= (DEPTH - 2));

  // `cnt` has no initial value, so an unclocked assert here is free to fail on the
  // very first step before any reset has run -- found composing this module into
  // formal/pcloop.sv for the first time.
  always_comb if (clocked) assert(cnt <= DEPTH);
  always_ff @(posedge clk)
    if (clocked) assert(!$past(flush) || cnt == 3'd0);
 `endif
endmodule
