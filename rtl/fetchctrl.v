`timescale 1 ns / 1 ps
`default_nettype none
// Drives fetchqueue.v from registers only: fetch_pc takes a registered redirect target,
// else advances by one word pair when the queue has room, else holds. A redirect's own
// target reaches imemory a cycle after decode computes it, and imemory answers a cycle
// after that, so two responses already in flight when the redirect fires are wrong-path
// and must never reach the queue -- flush stays asserted for both of those cycles, not
// just the one the redirect is captured in.
module fetchctrl (
  input  logic         clk,
  input  logic         reset,
  // Decode's own redirect decision and target, both combinational this cycle.
  input  logic         redirect,
  input  logic [31:0]  redirect_target,
  output logic [31:0]  fetch_pc,
  input  logic [31:0]  imem_data,
  input  logic [31:0]  imem_data2,
  input  logic         imem_fault,
  // A text load or store stole imemory's read port this cycle: the answer imemory gives
  // next cycle belongs to that access, not to fetch_pc, and must not be queued.
  input  logic         fetch_stall,
  input  logic         pop,
  output logic [31:0]  q0,
  output logic         q0_fault,
  output logic [31:0]  q1,
  output logic         q1_fault,
  output logic         buffer_empty,
  // Set the cycle a redirect fires, held until the queue holds a fresh pair again. decode
  // ANDs this with its own buffer_empty to attribute an empty-buffer cycle to a discard
  // in flight rather than to any of the eight stall reasons.
  output logic         redirect_recovering
);
  logic redirect_apply, redirect_apply_d1;
  logic [31:0] redirect_target_reg;
  logic waiting, launch, room, q_valid, flush;
  logic [2:0] queue_count;
  // A steal is confirmed one cycle after it happens, by which cycle fetch_pc has already
  // advanced to a second, honestly-requested address -- that response is also discarded
  // and re-requested once the retry lands, so the two never arrive out of order. stolen_pc
  // is only written when fetch_pc is about to move past a presentation that is not itself
  // a retry (redirect_apply or room), never while fetch_stall is retrying: two steals in a
  // row must keep retrying the FIRST address, not the second one the earlier bug drifted
  // onto, silently dropping the first from the fetch stream. formal/imemcheck.sv's own
  // free imem_arbiter, unconstrained by any real bus's transaction spacing, is what forced
  // this rather than the shipping suite's own (always single-steal) traffic.
  logic [31:0] stolen_pc;
  logic        fetch_stall_d1;

  logic req_valid;
  assign req_valid = waiting && !fetch_stall && !fetch_stall_d1;
  assign flush = redirect_apply || redirect_apply_d1;
  assign launch = redirect_apply || room;
  assign buffer_empty = !q_valid || redirect_apply;

  fetchqueue fq (
    .clk(clk),
    .reset(reset),
    .flush(flush),
    .req_valid(req_valid),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .imem_fault(imem_fault),
    .pop(pop),
    .q0(q0),
    .q0_fault(q0_fault),
    .q1(q1),
    .q1_fault(q1_fault),
    .q_valid(q_valid),
    .count(queue_count),
    .room(room)
  );

  always_ff @(posedge clk) begin
    if (reset) begin
      fetch_pc          <= 32'b0;
      redirect_apply     <= 1'b0;
      redirect_apply_d1  <= 1'b0;
      waiting            <= 1'b0;
      fetch_stall_d1     <= 1'b0;
      redirect_recovering <= 1'b0;
      // Undriven on reset otherwise: a steal on the very first post-reset cycle would
      // retry into whatever this register powered up holding. cxxrtl zero-initializes
      // registers, which hid this from every simulation leg; only BMC's free step-0
      // register value surfaced it.
      stolen_pc          <= 32'b0;
      redirect_target_reg <= 32'b0;
    end else begin
      redirect_apply_d1 <= redirect_apply;
      redirect_apply    <= redirect;
      redirect_target_reg <= redirect_target;
      waiting        <= fetch_stall ? 1'b1 : launch;
      fetch_stall_d1 <= fetch_stall;
      // Cleared off buffer_empty, not q_valid: q_valid still reads true for one cycle
      // after a redirect (the flush zeroing it lands on the NEXT edge), and clearing off
      // it directly let that cycle's own genuine discard read as an ordinary empty buffer
      // instead of the redirect recovery it is.
      if (redirect) redirect_recovering <= 1'b1;
      else if (!buffer_empty) redirect_recovering <= 1'b0;
      if (redirect_apply) begin
        fetch_pc  <= redirect_target_reg;
        stolen_pc <= redirect_target_reg;
      end else if (fetch_stall) begin
        fetch_pc <= stolen_pc;
        // stolen_pc UNCHANGED: this is the one branch that must not overwrite it, or a
        // second steal in a row retries the address the first one just drifted onto
        // instead of the one still waiting.
      end else begin
        // Covers both a room-having advance (stolen_pc becomes the address fetch_pc is
        // about to leave) and a no-room hold (fetch_pc does not move, so this is a
        // same-value refresh) -- either way stolen_pc tracks whatever fetch_pc is
        // CURRENTLY presenting, ready to be the retry target the moment a steal is
        // confirmed against it.
        stolen_pc <= fetch_pc;
        if (room) fetch_pc <= fetch_pc + 32'd8;
      end
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;

  // The environment's own obligation, checked where it is driven rather than merely
  // assumed inside fetchqueue.v: a genuine response is never claimed on a cycle the
  // queue has no room for one.
  always_comb if (clocked) assert(!req_valid || queue_count <= 3'd2);
 `endif
endmodule
