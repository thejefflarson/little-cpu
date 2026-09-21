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
  input  logic         pop2,
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
  logic [31:0] stolen_pc;
  logic        fetch_stall_d1;
  logic        fetch_odd;
  logic [15:0] prev_lane3;
  logic        straddle_in;
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

  // stolen_pc is last cycle's fetch_pc on every response a guess may commit off: the arms that
  // write it anything else (a redirect, a steal, a commit) each make the next response one
  // req_valid or predict_commit refuses.
  logic [31:0] pair_base;
  assign pair_base = {stolen_pc[31:2], 2'b00};

  // A compressed jump or backward branch at each lane; quadrant 01 only, since c.sw and c.swsp
  // share the funct3 codes.
  logic lane0_c, lane1_c, lane2_c, lane3_c;
  assign lane0_c = lane0[1:0] == 2'b01 && (lane0[15:13] == 3'b101 || lane0[15:13] == 3'b001 ||
                   ((lane0[15:13] == 3'b110 || lane0[15:13] == 3'b111) && lane0[12]));
  assign lane1_c = lane1[1:0] == 2'b01 && (lane1[15:13] == 3'b101 || lane1[15:13] == 3'b001 ||
                   ((lane1[15:13] == 3'b110 || lane1[15:13] == 3'b111) && lane1[12]));
  assign lane2_c = lane2[1:0] == 2'b01 && (lane2[15:13] == 3'b101 || lane2[15:13] == 3'b001 ||
                   ((lane2[15:13] == 3'b110 || lane2[15:13] == 3'b111) && lane2[12]));
  assign lane3_c = lane3[1:0] == 2'b01 && (lane3[15:13] == 3'b101 || lane3[15:13] == 3'b001 ||
                   ((lane3[15:13] == 3'b110 || lane3[15:13] == 3'b111) && lane3[12]));
  logic [31:0] word_s0, word_s1;
  assign word_s0 = {lane0, prev_lane3};
  assign word_s1 = {lane2, lane1};
  logic word0_j, word1_j, word_s0_j, word_s1_j;
  assign word0_j   = imem_data[6:0]  == 7'b1101111 || (imem_data[6:0]  == 7'b1100011 && imem_data[31]);
  assign word1_j   = imem_data2[6:0] == 7'b1101111 || (imem_data2[6:0] == 7'b1100011 && imem_data2[31]);
  assign word_s0_j = word_s0[6:0]    == 7'b1101111 || (word_s0[6:0]    == 7'b1100011 && word_s0[31]);
  assign word_s1_j = word_s1[6:0]    == 7'b1101111 || (word_s1[6:0]    == 7'b1100011 && word_s1[31]);

  // The first taken candidate in program order wins. One ending in the first word queues that
  // word alone, so the target's pair follows it; a straddling one is left by fetcher's pop2.
  logic sel_s0, sel_l0, sel_w0, sel_l1, sel_s1, sel_l2, sel_w1, sel_l3, sel_first, sel_early;
  assign sel_s0 = straddle_in && word_s0_j;
  assign sel_l0 = boundary0 && !lane0_wide && lane0_c;
  assign sel_w0 = boundary0 &&  lane0_wide && word0_j;
  assign sel_l1 = boundary1 && !lane1_wide && lane1_c && !sel_s0 && !sel_l0;
  assign sel_s1 = boundary1 &&  lane1_wide && word_s1_j && !sel_s0 && !sel_l0;
  assign sel_first = sel_s0 || sel_l0 || sel_w0 || sel_l1;
  assign sel_early = sel_first || sel_s1;
  assign sel_l2 = boundary2 && !lane2_wide && lane2_c && !sel_early;
  assign sel_w1 = boundary2 &&  lane2_wide && word1_j && !sel_early;
  assign sel_l3 = boundary3 && !lane3_wide && lane3_c && !sel_early && !sel_l2;

  logic [15:0] cand_c;
  logic [31:0] cand_w;
  logic        cand_wide, cand_straddle, cand_jal, cand_cj;
  logic [2:0]  cand_off;
  assign cand_c    = sel_l0 ? lane0 : sel_l1 ? lane1 : sel_l2 ? lane2 : lane3;
  assign cand_w    = sel_s0 ? word_s0 : sel_w0 ? imem_data : sel_s1 ? word_s1 : imem_data2;
  assign cand_straddle = sel_s0 || sel_s1;
  assign cand_wide = cand_straddle || sel_w0 || sel_w1;
  assign cand_jal  = cand_w[6:0] == 7'b1101111;
  assign cand_cj   = cand_c[15:13] == 3'b101 || cand_c[15:13] == 3'b001;
  assign cand_off  = sel_s0 ? 3'b111 : (sel_l0 || sel_w0) ? 3'd0 : (sel_l1 || sel_s1) ? 3'd1 :
                     (sel_l2 || sel_w1) ? 3'd2 : 3'd3;

  logic [31:0] cand_imm;
  always_comb begin
    case (1'b1)
      cand_wide && cand_jal:
        cand_imm = {{12{cand_w[31]}}, cand_w[19:12], cand_w[20], cand_w[30:21], 1'b0};
      cand_wide:
        cand_imm = {{20{cand_w[31]}}, cand_w[7], cand_w[30:25], cand_w[11:8], 1'b0};
      cand_cj:
        cand_imm = {{20{cand_c[12]}}, cand_c[12], cand_c[8], cand_c[10], cand_c[9], cand_c[6],
                    cand_c[7], cand_c[2], cand_c[11], cand_c[5], cand_c[4], cand_c[3], 1'b0};
      default:
        cand_imm = {{23{cand_c[12]}}, cand_c[12], cand_c[6:5], cand_c[2], cand_c[11:10],
                    cand_c[4:3], 1'b0};
    endcase
  end

  // A target inside a word the candidate occupies would leave pop nothing to advance over.
  logic cand_same_word;
  assign cand_same_word = cand_imm == 32'd0 ||
                          (cand_imm == 32'd2 && !cand_off[0]) ||
                          (cand_imm == 32'hffff_fffe && cand_off[0]) ||
                          (cand_straddle && (cand_imm == 32'd2 || cand_imm == 32'd4));

  logic predict_found, predict_half, fetch_odd_next;
  logic [31:0] predict_src, predict_tgt;
  assign predict_found = (sel_early || sel_l2 || sel_w1 || sel_l3) && !cand_same_word;
  assign predict_half  = sel_first;
  assign predict_src   = pair_base + {{28{cand_off[2]}}, cand_off, 1'b0};
  assign predict_tgt   = predict_src + cand_imm;
  assign fetch_odd_next = !boundary4;

  assign req_valid = waiting && !fetch_stall && !fetch_stall_d1;
  assign flush = redirect_apply || redirect_apply_d1;
  assign launch = redirect_apply || room;
  assign buffer_empty = !q_valid || redirect_apply;

  logic predict_trusted, predict_commit;
  assign predict_trusted = req_valid && predict_found;
  // One guess in flight, never off the pair a redirect is discarding, whose data arrives the
  // cycle after redirect_apply drops.
  assign predict_commit  = predict_trusted && !predicted_active &&
                           !redirect_apply && !redirect_apply_d1;

  fetchqueue fq (
    .clk(clk),
    .reset(reset),
    .flush(flush),
    .req_valid(req_valid),
    .req_half(predict_commit && predict_half),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .imem_fault(imem_fault),
    .pop(pop),
    .pop2(pop2),
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
      stolen_pc          <= 32'b0;
      redirect_target_reg <= 32'b0;
      predicted_active    <= 1'b0;
      predicted_src_pc    <= 32'b0;
      predicted_target    <= 32'b0;
      fetch_odd           <= 1'b0;
      prev_lane3          <= 16'b0;
      straddle_in         <= 1'b0;
      predict_commit_d1   <= 1'b0;
    end else begin
      redirect_apply_d1 <= redirect_apply;
      redirect_apply    <= redirect;
      redirect_target_reg <= redirect_target;
      predict_commit_d1 <= predict_commit;
      // !predict_commit drops the pair's own naive successor, already in flight.
      waiting        <= fetch_stall ? 1'b1 : (launch && !predict_commit);
      fetch_stall_d1 <= fetch_stall;
      if (redirect) redirect_recovering <= 1'b1;
      else if (!buffer_empty) redirect_recovering <= 1'b0;
      if (redirect_apply || predict_resolved) predicted_active <= 1'b0;
      else if (predict_commit) begin
        predicted_active <= 1'b1;
        predicted_src_pc <= predict_src;
        predicted_target <= predict_tgt;
      end
      if (redirect_apply_d1 || predict_commit_d1) begin
        fetch_odd   <= fetch_pc[1];
        straddle_in <= 1'b0;
      end else if (req_valid) begin
        fetch_odd   <= fetch_odd_next;
        straddle_in <= boundary3 && lane3_wide;
        prev_lane3  <= lane3;
      end
      if (redirect_apply) begin
        fetch_pc  <= redirect_target_reg;
        stolen_pc <= redirect_target_reg;
      end else if (fetch_stall) begin
        fetch_pc <= stolen_pc;
      end else if (predict_commit) begin
        // The retry address too, or a steal this cycle re-presents the abandoned successor.
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
  always_comb if (clocked && past_predict_commit)
    assert(predicted_active && fetch_pc == predicted_target && stolen_pc == predicted_target);

  logic [31:0] past_fetch_pc;
  always_ff @(posedge clk) past_fetch_pc <= fetch_pc;
  always_comb if (clocked && req_valid && !redirect_apply_d1)
    assert(stolen_pc == past_fetch_pc);
 `endif
endmodule
