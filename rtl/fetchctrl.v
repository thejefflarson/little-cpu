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
  output logic         buffer_empty
);
  logic redirect_apply, redirect_apply_d1;
  logic [31:0] redirect_target_reg;
  logic waiting, launch, room, q_valid, flush;
  logic [2:0] queue_count;
  // A steal is confirmed one cycle after it happens, by which cycle fetch_pc has already
  // advanced to a second, honestly-requested address -- that response is also discarded
  // and re-requested once the retry lands, so the two never arrive out of order.
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
      stolen_pc      <= fetch_pc;
      if (redirect_apply)  fetch_pc <= redirect_target_reg;
      else if (fetch_stall) fetch_pc <= stolen_pc;
      else if (room)         fetch_pc <= fetch_pc + 32'd8;
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
