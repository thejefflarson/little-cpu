`timescale 1 ns / 1 ps
`default_nettype none
// Drives fetchqueue.v from registers only: fetch_pc advances, holds, retries a steal, or takes a registered redirect target two cycles out; flush spans both cycles a redirect leaves in flight.
module fetchctrl (
  input  logic         clk,
  input  logic         reset,
  input  logic         redirect,
  input  logic [31:0]  redirect_target,
  output logic [31:0]  fetch_pc,
  input  logic [31:0]  imem_data,
  input  logic [31:0]  imem_data2,
  input  logic         imem_fault,
  input  logic         fetch_stall,
  input  logic         pop,
  output logic [31:0]  q0,
  output logic         q0_fault,
  output logic [31:0]  q1,
  output logic         q1_fault,
  output logic         buffer_empty,
  output logic         redirect_recovering
);
  logic redirect_apply, redirect_apply_d1;
  logic [31:0] redirect_target_reg;
  logic waiting, launch, room, q_valid, flush;
  logic [2:0] queue_count;
  // Updates only when fetch_pc leaves a non-retry presentation, never while fetch_stall retries, or two steals in a row silently drop the first stolen address.
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
      // Undriven otherwise, a steal on the first post-reset cycle retries a garbage address.
      stolen_pc          <= 32'b0;
      redirect_target_reg <= 32'b0;
    end else begin
      redirect_apply_d1 <= redirect_apply;
      redirect_apply    <= redirect;
      redirect_target_reg <= redirect_target;
      waiting        <= fetch_stall ? 1'b1 : launch;
      fetch_stall_d1 <= fetch_stall;
      // Cleared off buffer_empty, not q_valid: q_valid still reads true for one cycle after a redirect before flush's zeroing lands.
      if (redirect) redirect_recovering <= 1'b1;
      else if (!buffer_empty) redirect_recovering <= 1'b0;
      if (redirect_apply) begin
        fetch_pc  <= redirect_target_reg;
        stolen_pc <= redirect_target_reg;
      end else if (fetch_stall) begin
        fetch_pc <= stolen_pc;
      end else begin
        stolen_pc <= fetch_pc;
        if (room) fetch_pc <= fetch_pc + 32'd8;
      end
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;

  always_comb if (clocked) assert(!req_valid || queue_count <= 3'd2);
 `endif
endmodule
