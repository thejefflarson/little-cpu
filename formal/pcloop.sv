// The fetch queue, its controller, the fetcher and the decoder, wired as rtl/littlecpu.v.
`default_nettype none

module pcloop (
    input logic clk,
    input logic reset,
    input logic [31:0] imem_data,
    input logic [31:0] imem_data2,
    input logic [31:0] reg_rs1,
    input logic [31:0] reg_rs2,
    input executor_output executor_out,
    input logic divider_stall,
    input logic fetch_stall,
    input logic text_write,
    // Free: a hart not granted the shared bus holds the pc, the same way an unanswered
    // atomic redirects it; the increment assertion skips both cycles.
    input logic bus_wait,
    input logic imem_fault,
    input logic atomic_supported,
    input logic accessor_out_valid,
    input logic [31:0] csr_rdata,
    input logic csr_implemented,
    input logic [31:0] mtvec,
    input logic [31:0] mepc,
    input logic interrupt_pending
);
  logic [31:0] pc;
  logic [31:0] next_pc;
  logic redirect;
  logic [31:0] atomic_addr;
  fetcher_output fetcher_out;
  decoder_output decoder_out;
  logic [4:0] rs1, rs2, read_rs1, read_rs2;
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata;
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc, trap_tval;

  logic [31:0] fetch_pc;
  logic [31:0] queue_q0, queue_q1;
  logic        queue_q0_fault, queue_q1_fault;
  logic        buffer_empty;
  logic        redirect_recovering;
  logic        kill;
  logic        mispredict;
  logic        fetcher_pop;
  logic        predicted_active;
  logic [31:0] predicted_src_pc;
  logic [31:0] predicted_target;
  logic        predict_resolved;

  fetcher fetcher (
    .clk(clk),
    .reset(reset),
    .pc(pc),
    .next_pc(next_pc),
    .q0(queue_q0),
    .q1(queue_q1),
    .pop(fetcher_pop),
    .out(fetcher_out)
  );

  fetchctrl fetchctrl (
    .clk(clk),
    .reset(reset),
    .redirect(redirect),
    .redirect_target(next_pc),
    .fetch_pc(fetch_pc),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .imem_fault(imem_fault),
    .fetch_stall(fetch_stall),
    .text_write(text_write),
    .pop(fetcher_pop),
    .q0(queue_q0),
    .q0_fault(queue_q0_fault),
    .q1(queue_q1),
    .q1_fault(queue_q1_fault),
    .buffer_empty(buffer_empty),
    .redirect_recovering(redirect_recovering),
    .predicted_active(predicted_active),
    .predicted_src_pc(predicted_src_pc),
    .predicted_target(predicted_target),
    .predict_resolved(predict_resolved)
  );

  decoder decoder (
    .clk(clk),
    .reset(reset),
    .in(fetcher_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stall),
    .buffer_empty(buffer_empty),
    .redirect_recovering(redirect_recovering),
    .predicted_active(predicted_active),
    .predicted_src_pc(predicted_src_pc),
    .predicted_target(predicted_target),
    .predict_resolved(predict_resolved),
    .mispredict(mispredict),
    .kill(kill),
    .bus_wait(bus_wait),
    .bus_request(bus_request),
    .imem_fault(queue_q0_fault),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(mtvec),
    .mepc(mepc),
    .interrupt_pending(interrupt_pending),
    .pc(pc),
    .next_pc(next_pc),
    .redirect(redirect),
    .rs1(rs1),
    .rs2(rs2),
    .read_rs1(read_rs1),
    .read_rs2(read_rs2),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .instret(instret),
    .trap_entry(trap_entry),
    .trap_cause(trap_cause),
    .trap_epc(trap_epc),
    .trap_tval(trap_tval),
    .mret_entry(mret_entry),
    .out(decoder_out)
  );

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;

  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  logic [31:0] f_instr;
  assign f_instr = fetcher_out.instr;
  logic f_uncompressed;
  assign f_uncompressed = f_instr[1:0] == 2'b11;
  logic f_jump_branch;
  assign f_jump_branch =
      (f_uncompressed && f_instr[6:2] == 5'b11011) ||
      (f_uncompressed && f_instr[6:2] == 5'b11001 && f_instr[14:12] == 3'b000) ||
      (f_uncompressed && f_instr[6:2] == 5'b11000 &&
         f_instr[14:12] != 3'b010 && f_instr[14:12] != 3'b011) ||
      (f_instr[1:0] == 2'b01 && (f_instr[15:13] == 3'b101 || f_instr[15:13] == 3'b001 ||
                                 f_instr[15:13] == 3'b110 || f_instr[15:13] == 3'b111)) ||
      (f_instr[1:0] == 2'b10 && f_instr[15:13] == 3'b100 && f_instr[6:2] == 5'b0 &&
         f_instr[11:7] != 5'b0);

  logic f_live_rs1, f_live_rs2, f_may_stall;
  assign f_live_rs1 = rs1 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs1) ||
       (executor_out.valid && executor_out.rd == rs1));
  assign f_live_rs2 = rs2 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs2) ||
       (executor_out.valid && executor_out.rd == rs2));
  logic f_system;
  assign f_system = f_uncompressed && f_instr[6:2] == 5'b11100;

  logic [4:0] f_prev_rs1, f_prev_rs2;
  logic       f_read_taken, f_operand_fetch;
  always_ff @(posedge clk) begin
    if (reset) begin
      f_prev_rs1   <= 5'd0;
      f_prev_rs2   <= 5'd0;
      f_read_taken <= 1'b0;
    end else begin
      f_prev_rs1   <= read_rs1;
      f_prev_rs2   <= read_rs2;
      f_read_taken <= 1'b1;
    end
  end
  assign f_operand_fetch = !f_read_taken || f_prev_rs1 != rs1 || f_prev_rs2 != rs2;

  logic f_pair_moved;
  assign f_pair_moved = read_rs1 != f_prev_rs1 || read_rs2 != f_prev_rs2;

  logic f_fencei;
  assign f_fencei = f_uncompressed && f_instr[6:2] == 5'b00011;

  logic f_amo_wait;
  assign f_amo_wait = decoder_out.valid && decoder_out.is_amo;

  logic f_load_store;
  assign f_load_store =
      (f_uncompressed && (f_instr[6:2] == 5'b00000 || f_instr[6:2] == 5'b01000)) ||
      ((f_instr[1:0] == 2'b00 || f_instr[1:0] == 2'b10) &&
         (f_instr[15:13] == 3'b010 || f_instr[15:13] == 3'b110));

  assign f_may_stall = divider_stall || buffer_empty || bus_wait ||
      f_live_rs1 || f_live_rs2 || f_system || f_fencei || f_operand_fetch ||
      f_amo_wait || f_load_store;

  logic f_redirect;
  assign f_redirect = trap_entry || mret_entry;

  logic [31:0] past_pc, prev_mtvec, prev_mepc;
  logic prev_reset, prev_may_stall, prev_hard_stall, prev_jump_branch, prev_uncompressed;
  logic prev_trap_entry, prev_mret_entry, prev_buffer_empty, prev_pair_moved, prev_bus_wait;
  always_ff @(posedge clk) begin
    past_pc           <= pc;
    prev_reset        <= reset;
    prev_may_stall    <= f_may_stall;
    prev_hard_stall   <= divider_stall;
    prev_buffer_empty <= buffer_empty;
    prev_bus_wait     <= bus_wait;
    prev_jump_branch  <= f_jump_branch || f_redirect;
    prev_uncompressed <= f_uncompressed;
    prev_trap_entry   <= trap_entry;
    prev_mret_entry   <= mret_entry;
    prev_mtvec        <= mtvec;
    prev_mepc         <= mepc;
    prev_pair_moved   <= f_pair_moved;
  end

  always_comb if (clocked && !reset) assert(fetcher_out.pc == pc);

  logic f_increment_checked;
  assign f_increment_checked =
      clocked && !prev_reset && !prev_may_stall && !prev_jump_branch;

  always_ff @(posedge clk)
    if (f_increment_checked)
      assert(pc == past_pc + (prev_uncompressed ? 32'd4 : 32'd2));

  always_ff @(posedge clk)
    if (f_increment_checked) begin
      increment_reached: cover (1'b1);
      increment_reached_on_moved_pair: cover (prev_pair_moved);
    end

  always_ff @(posedge clk)
    if (clocked && prev_hard_stall && !prev_reset) assert(pc == past_pc);

  always_ff @(posedge clk)
    if (clocked && prev_buffer_empty && !prev_reset) assert(pc == past_pc);

  always_ff @(posedge clk)
    if (clocked && prev_bus_wait && !prev_reset) assert(pc == past_pc);

  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);

  // Property 1: fetch_pc advances by one word pair, holds, retries, takes a guessed target a
  // cycle after fetchctrl forms it, or a redirect target two cycles after decode computes it.
  logic [31:0] past_fetch_pc, past2_fetch_pc;
  logic [31:0] past_next_pc_r, past2_next_pc_r;
  logic        past_redirect_r, past2_redirect_r;
  logic        past_predicted_active;
  always_ff @(posedge clk) begin
    past_fetch_pc    <= fetch_pc;
    past2_fetch_pc   <= past_fetch_pc;
    past_next_pc_r   <= next_pc;
    past2_next_pc_r  <= past_next_pc_r;
    past_redirect_r  <= redirect;
    past2_redirect_r <= past_redirect_r;
    past_predicted_active <= predicted_active;
  end

  logic f_fetch_pc_advanced, f_fetch_pc_held, f_fetch_pc_retried, f_fetch_pc_redirected,
        f_fetch_pc_guessed;
  assign f_fetch_pc_advanced   = fetch_pc == past_fetch_pc + 32'd8;
  assign f_fetch_pc_held       = fetch_pc == past_fetch_pc;
  assign f_fetch_pc_retried    = fetch_pc == past2_fetch_pc;
  assign f_fetch_pc_redirected = past2_redirect_r && fetch_pc == past2_next_pc_r;
  assign f_fetch_pc_guessed    = predicted_active && !past_predicted_active &&
                                  fetch_pc == predicted_target;

  logic f_fetch_pc_prev2_ok;
  always_ff @(posedge clk) if (reset) f_fetch_pc_prev2_ok <= 1'b0;
    else f_fetch_pc_prev2_ok <= clocked && !reset;

  always_comb if (clocked && !reset && f_fetch_pc_prev2_ok)
    assert(f_fetch_pc_advanced || f_fetch_pc_held || f_fetch_pc_retried ||
           f_fetch_pc_redirected || f_fetch_pc_guessed);

  // Property 2: the buffer pops only on the cycle pc actually leaves the word it names,
  // restated independently of fetcher.v's own `pop`.
  always_comb if (clocked && !reset)
    assert(fetcher_pop == (next_pc[31:2] != pc[31:2]));

  // Property 3: a word that never reaches decode never issues, graded against last cycle's
  // buffer occupancy since decoder_out.valid reports what issued THEN (out is registered); a
  // divider hold republishes out unchanged and is not a fresh issue.
  always_comb if (clocked && !prev_reset && !prev_hard_stall)
    assert(!prev_buffer_empty || !decoder_out.valid);

  // The strengthened half, over `kill` rather than `buffer_empty` so a future narrowing of the
  // latter's other causes stays covered: restates decoder's own `kill => !issuing` one cycle
  // later, against the composed queue's `redirect_recovering`.
  logic prev_kill;
  always_ff @(posedge clk) prev_kill <= kill;
  always_comb if (clocked && !prev_reset && !prev_hard_stall)
    assert(!prev_kill || !decoder_out.valid);
 `endif
endmodule

`default_nettype wire
