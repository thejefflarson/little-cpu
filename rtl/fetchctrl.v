`timescale 1 ns / 1 ps
`default_nettype none
// Drives fetchqueue.v from registers only, and forms a static BTFN/jal guess off the fetched pair.
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
  output logic         redirect_recovering,
  output logic         predicted_active,
  output logic [31:0]  predicted_src_pc,
  output logic [31:0]  predicted_target,
  input  logic         predict_resolved
);
  logic redirect_apply, redirect_apply_d1;
  logic [31:0] redirect_target_reg;
  logic waiting, launch, room, q_valid, flush, req_valid;
  logic [2:0] queue_count;
  // The retry address: held across a steal so a second one in a row does not drift.
  logic [31:0] stolen_pc;
  logic        fetch_stall_d1;
  logic        fetch_odd;
  // One cycle behind fetch_pc always, unlike stolen_pc (whose redirect_apply arm jumps early).
  logic [31:0] fetch_addr_d1;
  logic        predict_commit_d1;

  logic [15:0] lane0, lane1, lane2, lane3;
  assign lane0 = imem_data[15:0];
  assign lane1 = imem_data[31:16];
  assign lane2 = imem_data2[15:0];
  assign lane3 = imem_data2[31:16];
  logic lane0_wide, lane1_wide, lane2_wide, lane3_wide;
  assign lane0_wide = lane0[1:0] == 2'b11;
  assign lane1_wide = lane1[1:0] == 2'b11;
  assign lane2_wide = lane2[1:0] == 2'b11;
  assign lane3_wide = lane3[1:0] == 2'b11;

  logic boundary0, boundary1, boundary2, boundary3, boundary4;
  assign boundary0 = !fetch_odd;
  assign boundary1 = fetch_odd || (boundary0 && !lane0_wide);
  assign boundary2 = (boundary0 && lane0_wide) || (boundary1 && !lane1_wide);
  assign boundary3 = (boundary1 && lane1_wide) || (boundary2 && !lane2_wide);
  assign boundary4 = (boundary2 && lane2_wide) || (boundary3 && !lane3_wide);

  logic [31:0] pair_base;
  assign pair_base = {fetch_addr_d1[31:2], 2'b00};

  logic cand_a_eligible, cand_b_eligible;
  assign cand_a_eligible = boundary2 && lane2_wide;
  assign cand_b_eligible = boundary3 && !lane3_wide;

  logic cand_a_jal, cand_a_branch, cand_a_taken;
  logic [31:0] cand_a_imm, cand_a_target;
  assign cand_a_jal    = cand_a_eligible && imem_data2[6:0] == 7'b1101111;
  assign cand_a_branch = cand_a_eligible && imem_data2[6:0] == 7'b1100011;
  assign cand_a_imm = cand_a_jal
    ? {{12{imem_data2[31]}}, imem_data2[19:12], imem_data2[20], imem_data2[30:21], 1'b0}
    : {{20{imem_data2[31]}}, imem_data2[7], imem_data2[30:25], imem_data2[11:8], 1'b0};
  assign cand_a_target = pair_base + 32'd4 + cand_a_imm;
  // A target inside the candidate's own word would leave pop with nothing to advance over.
  logic [29:0] cand_word;
  assign cand_word = pair_base[31:2] + 30'd1;
  logic cand_a_same_word;
  assign cand_a_same_word = cand_a_target[31:2] == cand_word;
  assign cand_a_taken  = !cand_a_same_word &&
    (cand_a_jal || (cand_a_branch && imem_data2[31]));

  // A compressed jump or branch in the second word: at lane 2 (whatever lane 3 holds, since
  // pop leaves the word once the guess is taken) or at lane 3. Quadrant 01 only; c.sw and
  // c.swsp share the funct3 codes.
  logic [15:0] cand_c_lane;
  logic        cand_c_at2, cand_c_eligible;
  assign cand_c_at2      = boundary2 && !lane2_wide && lane2[1:0] == 2'b01 &&
                           (lane2[15:13] == 3'b101 || lane2[15:13] == 3'b001 ||
                            ((lane2[15:13] == 3'b110 || lane2[15:13] == 3'b111) && lane2[12]));
  assign cand_c_lane     = cand_c_at2 ? lane2 : lane3;
  assign cand_c_eligible = cand_c_at2 || (cand_b_eligible && lane3[1:0] == 2'b01);
  logic cand_c_cj, cand_c_cjal, cand_c_cbeqz, cand_c_cbnez;
  logic [31:0] cand_c_imm, cand_c_target;
  assign cand_c_cj     = cand_c_eligible && cand_c_lane[15:13] == 3'b101;
  assign cand_c_cjal   = cand_c_eligible && cand_c_lane[15:13] == 3'b001;
  assign cand_c_cbeqz  = cand_c_eligible && cand_c_lane[15:13] == 3'b110;
  assign cand_c_cbnez  = cand_c_eligible && cand_c_lane[15:13] == 3'b111;
  assign cand_c_imm = (cand_c_cj || cand_c_cjal)
    ? {{20{cand_c_lane[12]}}, cand_c_lane[12], cand_c_lane[8], cand_c_lane[10],
       cand_c_lane[9], cand_c_lane[6], cand_c_lane[7], cand_c_lane[2], cand_c_lane[11],
       cand_c_lane[5], cand_c_lane[4], cand_c_lane[3], 1'b0}
    : {{23{cand_c_lane[12]}}, cand_c_lane[12], cand_c_lane[6:5], cand_c_lane[2],
       cand_c_lane[11:10], cand_c_lane[4:3], 1'b0};
  logic [31:0] cand_c_src;
  assign cand_c_src    = pair_base + (cand_c_at2 ? 32'd4 : 32'd6);
  assign cand_c_target = cand_c_src + cand_c_imm;
  logic cand_c_same_word, cand_c_taken;
  assign cand_c_same_word = cand_c_target[31:2] == cand_word;
  assign cand_c_taken  = !cand_c_same_word && (cand_c_cj || cand_c_cjal ||
                          ((cand_c_cbeqz || cand_c_cbnez) && cand_c_lane[12]));

  logic predict_found, fetch_odd_next;
  logic [31:0] predict_src, predict_tgt;
  assign predict_found = cand_a_taken || cand_c_taken;
  assign predict_src   = cand_a_taken ? (pair_base + 32'd4) : cand_c_src;
  assign predict_tgt   = cand_a_taken ? cand_a_target : cand_c_target;
  assign fetch_odd_next = !boundary4;

  assign req_valid = waiting && !fetch_stall && !fetch_stall_d1;
  assign flush = redirect_apply || redirect_apply_d1;
  assign launch = redirect_apply || room;
  assign buffer_empty = !q_valid || redirect_apply;

  logic predict_trusted, predict_commit;
  assign predict_trusted = req_valid && predict_found;
  // One guess in flight, never off the pair a redirect is discarding: that pair's data
  // arrives the cycle after redirect_apply drops, so both cycles are excluded.
  assign predict_commit  = predict_trusted && !predicted_active &&
                           !redirect_apply && !redirect_apply_d1;

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
      predicted_active    <= 1'b0;
      predicted_src_pc    <= 32'b0;
      predicted_target    <= 32'b0;
      fetch_odd           <= 1'b0;
      fetch_addr_d1       <= 32'b0;
      predict_commit_d1   <= 1'b0;
    end else begin
      redirect_apply_d1 <= redirect_apply;
      redirect_apply    <= redirect;
      redirect_target_reg <= redirect_target;
      fetch_addr_d1     <= fetch_pc;
      predict_commit_d1 <= predict_commit;
      // !predict_commit drops the pair's own naive successor, already in flight.
      waiting        <= fetch_stall ? 1'b1 : (launch && !predict_commit);
      fetch_stall_d1 <= fetch_stall;
      // Cleared off buffer_empty, not q_valid, which reads true one cycle too long.
      if (redirect) redirect_recovering <= 1'b1;
      else if (!buffer_empty) redirect_recovering <= 1'b0;
      if (redirect_apply || predict_resolved) predicted_active <= 1'b0;
      else if (predict_commit) begin
        predicted_active <= 1'b1;
        predicted_src_pc <= predict_src;
        predicted_target <= predict_tgt;
      end
      // fetch_pc already holds the jump target by the cycle its pair's data arrives.
      if (redirect_apply_d1 || predict_commit_d1) fetch_odd <= fetch_pc[1];
      else if (req_valid) fetch_odd <= fetch_odd_next;
      if (redirect_apply) begin
        fetch_pc  <= redirect_target_reg;
        stolen_pc <= redirect_target_reg;
      end else if (fetch_stall) begin
        fetch_pc <= stolen_pc;
      end else if (predict_commit) begin
        // The retry address too: a steal this cycle must re-present the target, never the
        // successor the guess abandons.
        fetch_pc  <= predict_tgt;
        stolen_pc <= predict_tgt;
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

  logic past_predict_commit;
  always_ff @(posedge clk) past_predict_commit <= !reset && predict_commit;
  // The cycle after a commit both the fetch address and the retry address are the target.
  always_comb if (clocked && past_predict_commit)
    assert(predicted_active && fetch_pc == predicted_target && stolen_pc == predicted_target);
 `endif
endmodule
