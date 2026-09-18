`timescale 1 ns / 1 ps
`default_nettype none
// A 4-word FIFO of individually fetched words: q0/q1 are "the word at the issuing pc" and
// "the one after it". `req_pending` clearing on a flush drops a stale response.
module fetchqueue (
  input  logic         clk,
  input  logic         reset,
  input  logic         flush,
  input  logic [31:0]  fetch_pc,
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
  localparam logic [31:0] SENTINEL = 32'hffff_ffff;

  logic [31:0] mem      [0:DEPTH-1];
  logic        fault_mem[0:DEPTH-1];
  logic [1:0]  head, tail;
  logic [2:0]  cnt;

  logic [31:0] prev_fetch_pc;
  logic        addr_changed;
  logic        req_pending;

  always_ff @(posedge clk)
    prev_fetch_pc <= (reset || flush) ? SENTINEL : fetch_pc;
  assign addr_changed = fetch_pc != prev_fetch_pc;

  always_ff @(posedge clk)
    if (reset || flush) req_pending <= 1'b0;
    else req_pending <= addr_changed;

  logic do_pop;
  assign do_pop = pop && (cnt != 3'd0);

  always_ff @(posedge clk) begin
    if (reset || flush) begin
      head <= 2'd0;
      tail <= 2'd0;
      cnt  <= 3'd0;
    end else begin
      if (do_pop) head <= head + 2'd1;
      if (req_pending) begin
        mem[tail]              <= imem_data;
        fault_mem[tail]        <= imem_fault;
        mem[tail + 2'd1]       <= imem_data2;
        fault_mem[tail + 2'd1] <= imem_fault;
        tail <= tail + 2'd2;
      end
      cnt <= cnt - (do_pop ? 3'd1 : 3'd0) + (req_pending ? 3'd2 : 3'd0);
    end
  end

  assign q0       = mem[head];
  assign q0_fault = fault_mem[head];
  assign q1       = mem[head + 2'd1];
  assign q1_fault = fault_mem[head + 2'd1];
  assign q_valid  = cnt >= 3'd2;
  assign count    = cnt;

  // Reserves two words each for a due response and a request landing this cycle.
  logic [2:0] committed;
  assign committed = cnt + (req_pending ? 3'd2 : 3'd0) + (addr_changed ? 3'd2 : 3'd0);
  assign room = committed <= (DEPTH - 2);

 `ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;

  always_comb assert(cnt <= DEPTH);
  always_ff @(posedge clk)
    if (clocked) assert(!$past(flush) || cnt == 3'd0);
 `endif
endmodule
