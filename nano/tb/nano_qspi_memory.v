`timescale 1 ns / 1 ps
// A behavioural QSPI-flash-and-PSRAM timing model standing in for nano_memory.v's zero-wait one; only mem_ready's timing differs. See the ADR for the modelled timings.
module nano_qspi_memory #(
  parameter int WORDS = 20480,
  parameter int PREFETCH_DEPTH = 0,
  // 0: no loop buffer. 1: one aligned LOOP_WINDOW-parcel tagged block. 2: a fully-assoc cache of the last LOOP_WINDOW parcels delivered, index 0 the most recent.
  parameter int LOOP_KIND = 0,
  parameter int LOOP_WINDOW = 0,
  parameter int PREAMBLE_CYCLES = 24,
  parameter int PARCEL_CYCLES = 8,
  parameter int PSRAM_LOAD_CYCLES = 44,
  parameter int PSRAM_STORE_CYCLES = 33
) (
  input  logic        clk,
  input  logic        reset,
  input  logic        mem_valid,
  input  logic        mem_instr,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [ 3:0] mem_wstrb,
  output logic        mem_ready,
  output logic [31:0] mem_rdata,
  output logic        reason_parcel_wait,
  output logic        reason_redirect_preamble,
  output logic        reason_loop_hit,
  output logic        reason_handshake,
  output logic        reason_psram_wait,
  output logic        stream_fault
);
  initial begin
    if (PARCEL_CYCLES < 1) $fatal(1, "PARCEL_CYCLES must be at least 1");
    if (PREAMBLE_CYCLES < 2) $fatal(1, "PREAMBLE_CYCLES must be at least 2");
    if (PSRAM_LOAD_CYCLES < 1) $fatal(1, "PSRAM_LOAD_CYCLES must be at least 1");
    if (PSRAM_STORE_CYCLES < 1) $fatal(1, "PSRAM_STORE_CYCLES must be at least 1");
    if (PREFETCH_DEPTH < 0 || PREFETCH_DEPTH == 1)
      $fatal(1, "PREFETCH_DEPTH must be 0 or at least 2: a 32-bit instruction needs two parcels queued");
    if (LOOP_KIND < 0 || LOOP_KIND > 2) $fatal(1, "LOOP_KIND must be 0, 1 or 2");
    if (LOOP_KIND != 0 && LOOP_WINDOW <= 0)
      $fatal(1, "LOOP_WINDOW must be positive when LOOP_KIND is not 0");
    if (LOOP_KIND == 1 && (LOOP_WINDOW & (LOOP_WINDOW - 1)) != 0)
      $fatal(1, "LOOP_WINDOW must be a power of two for LOOP_KIND 1");
  end

  localparam int SLOTS = LOOP_WINDOW <= 0 ? 1 : LOOP_WINDOW;
  localparam int SLOTBITS = SLOTS <= 1 ? 1 : $clog2(SLOTS);

  logic [31:0] mem [0:WORDS-1];
  logic [31:0] word_addr;
  assign word_addr = mem_addr[31:2];
  logic [31:0] low_word, high_word;
  assign low_word = mem[word_addr];
  assign high_word = mem[word_addr + 1];
  assign mem_rdata = mem_addr[1] ? {high_word[15:0], low_word[31:16]} : low_word;

  // Written out, not read via a function in a continuous assign: both frontends silently under-evaluate that shape's sensitivity.
  int unsigned target_index;
  assign target_index = mem_addr[31:1];
  logic [31:0] target_word;
  assign target_word = mem[target_index >> 1];
  logic [15:0] target_parcel;
  assign target_parcel = target_index[0] ? target_word[31:16] : target_word[15:0];
  int unsigned target_len;
  assign target_len = target_parcel[1:0] == 2'b11 ? 32'd2 : 32'd1;
  int unsigned target_last;
  assign target_last = target_index + target_len - 1;

  logic stream_open;
  int unsigned expect_index;
  // The flash queue's head, apart from the core's own position: a loop-buffer hit moves the core, not the stream.
  int unsigned fifo_head;
  logic arrived_valid;
  int unsigned arrived_index;
  int unsigned parcel_timer;
  logic preamble_pending;
  int unsigned preamble_timer;
  int unsigned preamble_target;

  int unsigned next_to_produce;
  assign next_to_produce = arrived_valid ? arrived_index + 1 : preamble_target;
  int produce_lead;
  assign produce_lead = next_to_produce - fifo_head;
  logic produce_room;
  assign produce_room = PREFETCH_DEPTH == 0 ? mem_valid : produce_lead < PREFETCH_DEPTH;

  logic already_aimed;
  assign already_aimed = (stream_open || preamble_pending) && fifo_head == target_index;

  // LOOP_KIND 1: one tag and a valid bit per slot; every miss re-tags, one inside the tagged block included.
  logic tag_window_valid;
  logic [31:SLOTBITS] tag_window_tag;
  logic [SLOTS-1:0] tag_window_bits;
  int unsigned target_index_p1;
  assign target_index_p1 = target_index + 1;
  logic tag_ready0, tag_ready1;
  assign tag_ready0 = tag_window_valid &&
    target_index[31:SLOTBITS] == tag_window_tag &&
    tag_window_bits[target_index[SLOTBITS-1:0]];
  assign tag_ready1 = tag_window_valid &&
    target_index_p1[31:SLOTBITS] == tag_window_tag &&
    tag_window_bits[target_index_p1[SLOTBITS-1:0]];

  logic [SLOTS-1:0] cam_valid;
  int unsigned cam_idx [0:SLOTS-1];
  logic cam_has0, cam_has1;
  always_comb begin
    cam_has0 = 1'b0;
    cam_has1 = 1'b0;
    for (int i = 0; i < SLOTS; i++) begin
      if (cam_valid[i] && cam_idx[i] == target_index) cam_has0 = 1'b1;
      if (cam_valid[i] && cam_idx[i] == target_index + 1) cam_has1 = 1'b1;
    end
  end

  logic loop_hit_full;
  assign loop_hit_full =
    LOOP_KIND == 1 ? (tag_ready0 && (target_len == 1 || tag_ready1)) :
    LOOP_KIND == 2 ? (cam_has0 && (target_len == 1 || cam_has1)) :
    1'b0;

  logic xfer_active;
  logic xfer_aimed;
  logic xfer_loophit;
  logic xfer_load;
  int unsigned psram_left;
  int unsigned psram_left_eff;
  assign psram_left_eff = xfer_active ? psram_left
    : (mem_wstrb == 4'b0000 ? PSRAM_LOAD_CYCLES : PSRAM_STORE_CYCLES) - 1;

  // The three *_now signals recompute fresh on a transaction's first cycle; the xfer_* registers only latch that decision starting the next.
  logic redirect_now;
  assign redirect_now = mem_valid && mem_instr && !already_aimed;
  logic loop_hit_now;
  assign loop_hit_now = xfer_active ? xfer_loophit : (mem_valid && mem_instr && loop_hit_full);
  logic preamble_active_now;
  assign preamble_active_now = xfer_active ? preamble_pending
    : (redirect_now && !loop_hit_full);
  logic aimed_now;
  assign aimed_now = xfer_active ? xfer_aimed : already_aimed;

  // A loop-buffer hit is an on-chip SRAM read: it answers the same cycle it is recognized, like a streaming fetch whose parcel has already arrived.
  assign mem_ready = mem_valid && (
    !mem_instr ? psram_left_eff == 0 :
    loop_hit_now ? 1'b1 :
    aimed_now && arrived_valid && arrived_index >= target_last);

  assign reason_loop_hit = mem_valid && mem_instr && loop_hit_now;
  assign reason_redirect_preamble = mem_valid && mem_instr && !loop_hit_now && preamble_active_now;
  assign reason_handshake = mem_valid && mem_instr && !loop_hit_now && !preamble_active_now && mem_ready;
  assign reason_parcel_wait = mem_valid && mem_instr && !loop_hit_now && !preamble_active_now && !mem_ready;
  assign reason_psram_wait = mem_valid && !mem_instr;
  // Independent of the aim logic: a fetch the buffer did not serve must lie inside what this flash run has streamed.
  assign stream_fault = mem_valid && mem_instr && mem_ready && !loop_hit_now &&
    !(arrived_valid && target_index >= preamble_target && target_last <= arrived_index);

  always_ff @(posedge clk) begin
    if (reset) begin
      stream_open <= 1'b0;
      expect_index <= 0;
      fifo_head <= 0;
      arrived_valid <= 1'b0;
      arrived_index <= 0;
      parcel_timer <= PARCEL_CYCLES;
      preamble_pending <= 1'b0;
      preamble_timer <= 0;
      preamble_target <= 0;
      tag_window_valid <= 1'b0;
      tag_window_tag <= 0;
      tag_window_bits <= '0;
      cam_valid <= '0;
      xfer_active <= 1'b0;
      xfer_aimed <= 1'b0;
      xfer_loophit <= 1'b0;
      xfer_load <= 1'b0;
      psram_left <= 0;
    end else begin
      // Ordered before the redirect-trigger block: on the same cycle a redirect abandons this stream, that block's arrived_valid clear must win the write race here.
      if (preamble_pending) begin
        if (preamble_timer != 0) begin
          preamble_timer <= preamble_timer - 1;
        end else begin
          preamble_pending <= 1'b0;
          stream_open <= 1'b1;
          arrived_valid <= 1'b0;
          parcel_timer <= PARCEL_CYCLES - 1;
        end
      end else if (stream_open && produce_room) begin
        if (parcel_timer != 0) begin
          parcel_timer <= parcel_timer - 1;
        end else begin
          arrived_valid <= 1'b1;
          arrived_index <= next_to_produce;
          parcel_timer <= PARCEL_CYCLES - 1;
          if (LOOP_KIND == 1 && tag_window_valid &&
              next_to_produce[31:SLOTBITS] == tag_window_tag) begin
            tag_window_bits[next_to_produce[SLOTBITS-1:0]] <= 1'b1;
          end
        end
      end

      if (mem_valid && !xfer_active) begin
        xfer_active <= 1'b1;
        xfer_loophit <= 1'b0;
        // True for any ordinary fetch: either it was already aimed, or it is setting up a redirect that (by construction) targets this exact transaction, so it too will be aimed once its own wait ends.
        xfer_aimed <= mem_instr;
        if (!mem_instr) begin
          stream_open <= 1'b0;
          preamble_pending <= 1'b0;
          xfer_load <= mem_wstrb == 4'b0000;
          psram_left <= (mem_wstrb == 4'b0000 ? PSRAM_LOAD_CYCLES : PSRAM_STORE_CYCLES) - 1;
        end else if (loop_hit_full) begin
          xfer_loophit <= 1'b1;
        end else if (!already_aimed) begin
          stream_open <= 1'b0;
          arrived_valid <= 1'b0;
          preamble_pending <= 1'b1;
          preamble_timer <= PREAMBLE_CYCLES - 2;
          preamble_target <= target_index;
          fifo_head <= target_index;
          if (LOOP_KIND == 1) begin
            tag_window_valid <= 1'b1;
            tag_window_tag <= target_index[31:SLOTBITS];
            tag_window_bits <= '0;
          end
        end
      end else if (mem_valid && xfer_active) begin
        if (!mem_instr && psram_left != 0) psram_left <= psram_left - 1;
      end else begin
        xfer_active <= 1'b0;
      end

      if (mem_valid && mem_ready) begin
        xfer_active <= 1'b0;
        if (mem_instr) begin
          expect_index <= target_index + target_len;
          if (target_index == fifo_head) fifo_head <= target_index + target_len;
          if (LOOP_KIND == 2) begin
            if (target_len == 1) begin
              for (int i = SLOTS - 1; i > 0; i--) begin
                cam_idx[i] <= cam_idx[i - 1];
                cam_valid[i] <= cam_valid[i - 1];
              end
              cam_idx[0] <= target_index;
              cam_valid[0] <= 1'b1;
            end else if (SLOTS > 1) begin
              for (int i = SLOTS - 1; i > 1; i--) begin
                cam_idx[i] <= cam_idx[i - 2];
                cam_valid[i] <= cam_valid[i - 2];
              end
              cam_idx[0] <= target_index;
              cam_valid[0] <= 1'b1;
              cam_idx[1] <= target_index + 1;
              cam_valid[1] <= 1'b1;
            end
          end
        end else begin
          // The flash's chip select dropped when this PSRAM access began, so its resync starts now rather than when the next fetch issues.
          stream_open <= 1'b0;
          arrived_valid <= 1'b0;
          preamble_pending <= 1'b1;
          preamble_timer <= PREAMBLE_CYCLES - 2;
          preamble_target <= expect_index;
          fifo_head <= expect_index;
        end
        if (mem_wstrb[0]) mem[word_addr][7:0]   <= mem_wdata[7:0];
        if (mem_wstrb[1]) mem[word_addr][15:8]  <= mem_wdata[15:8];
        if (mem_wstrb[2]) mem[word_addr][23:16] <= mem_wdata[23:16];
        if (mem_wstrb[3]) mem[word_addr][31:24] <= mem_wdata[31:24];
      end
    end
  end
endmodule
