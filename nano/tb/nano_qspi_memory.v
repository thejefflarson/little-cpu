`timescale 1 ns / 1 ps
// A behavioural QSPI-flash-and-PSRAM timing model, standing in for nano_memory.v's
// zero-wait model; only mem_ready's timing differs. See the ADR recording this
// instrument's numbers for the modelled timings and its assumptions.
module nano_qspi_memory #(
  parameter int WORDS = 20480,
  parameter int PREFETCH_DEPTH = 0,
  parameter int LOOP_WINDOW = 0,
  parameter int PREAMBLE_CYCLES = 24,
  parameter int PARCEL_CYCLES = 8,
  parameter int PSRAM_CYCLES = 44
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
  output logic        reason_psram_wait
);
  localparam int WINBITS = LOOP_WINDOW <= 1 ? 1 : $clog2(LOOP_WINDOW * 2);

  logic [31:0] mem [0:WORDS-1];
  logic [31:0] word_addr;
  assign word_addr = mem_addr[31:2];
  logic [31:0] low_word, high_word;
  assign low_word = mem[word_addr];
  assign high_word = mem[word_addr + 1];
  assign mem_rdata = mem_addr[1] ? {high_word[15:0], low_word[31:16]} : low_word;

  // A parcel is a 16-bit halfword, streamed regardless of instruction boundaries.
  function automatic int unsigned parcel_at(int unsigned idx);
    logic [31:0] w;
    w = mem[idx >> 1];
    parcel_at = idx[0] ? w[31:16] : w[15:0];
  endfunction

  int unsigned target_index;
  assign target_index = mem_addr[31:1];
  logic [15:0] target_parcel;
  assign target_parcel = parcel_at(target_index);
  int unsigned target_len;
  assign target_len = target_parcel[1:0] == 2'b11 ? 32'd2 : 32'd1;
  int unsigned target_last;
  assign target_last = target_index + target_len - 1;

  logic stream_open;
  int unsigned expect_index;
  logic arrived_valid;
  int unsigned arrived_index;
  int unsigned parcel_timer;
  logic preamble_pending;
  int unsigned preamble_timer;
  int unsigned preamble_target;

  int unsigned next_to_produce;
  assign next_to_produce = arrived_valid ? arrived_index + 1 : expect_index;
  logic produce_room;
  assign produce_room = PREFETCH_DEPTH == 0 ? mem_valid
    : (next_to_produce - expect_index) < PREFETCH_DEPTH;

  logic already_aimed;
  assign already_aimed = (stream_open && expect_index == target_index) ||
    (preamble_pending && preamble_target == target_index);

  logic window_valid;
  logic [31:0] window_base;
  logic loop_hit;
  assign loop_hit = LOOP_WINDOW != 0 && window_valid &&
    mem_addr[31:WINBITS] == window_base[31:WINBITS];

  logic xfer_active;
  logic xfer_loophit;
  int unsigned psram_left;
  int unsigned psram_left_eff;
  assign psram_left_eff = xfer_active ? psram_left : PSRAM_CYCLES - 1;

  // The two *_now signals recompute fresh on a transaction's first cycle, since
  // xfer_loophit/preamble_pending only latch that decision starting the next one.
  logic redirect_now;
  assign redirect_now = mem_valid && mem_instr && !already_aimed;
  logic is_loophit_now;
  assign is_loophit_now = xfer_active ? xfer_loophit : (redirect_now && loop_hit);
  logic preamble_active_now;
  assign preamble_active_now = xfer_active ? preamble_pending : (redirect_now && !loop_hit);

  assign mem_ready = mem_valid && (
    !mem_instr ? psram_left_eff == 0 :
    is_loophit_now ? xfer_active :
    already_aimed && arrived_valid && arrived_index >= target_last);

  assign reason_redirect_preamble = mem_valid && mem_instr && !is_loophit_now && preamble_active_now;
  assign reason_parcel_wait = mem_valid && mem_instr && !reason_redirect_preamble;
  assign reason_psram_wait = mem_valid && !mem_instr;

  always_ff @(posedge clk) begin
    if (reset) begin
      stream_open <= 1'b0;
      expect_index <= 0;
      arrived_valid <= 1'b0;
      arrived_index <= 0;
      parcel_timer <= PARCEL_CYCLES;
      preamble_pending <= 1'b0;
      preamble_timer <= 0;
      preamble_target <= 0;
      window_valid <= 1'b0;
      window_base <= 0;
      xfer_active <= 1'b0;
      xfer_loophit <= 1'b0;
      psram_left <= 0;
    end else begin
      // Ordered before the redirect-trigger block: on the same cycle a redirect abandons
      // this stream, that block's arrived_valid clear must win the write race here.
      if (preamble_pending) begin
        if (preamble_timer != 0) begin
          preamble_timer <= preamble_timer - 1;
        end else begin
          preamble_pending <= 1'b0;
          stream_open <= 1'b1;
          expect_index <= preamble_target;
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
        end
      end

      if (mem_valid && !xfer_active) begin
        xfer_active <= 1'b1;
        xfer_loophit <= 1'b0;
        if (!mem_instr) begin
          stream_open <= 1'b0;
          preamble_pending <= 1'b0;
          psram_left <= PSRAM_CYCLES - 1;
        end else if (!already_aimed) begin
          if (loop_hit) begin
            xfer_loophit <= 1'b1;
          end
          stream_open <= 1'b0;
          arrived_valid <= 1'b0;
          preamble_pending <= 1'b1;
          preamble_timer <= PREAMBLE_CYCLES - 1;
          preamble_target <= loop_hit ? target_index + target_len : target_index;
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
          window_base <= mem_addr;
          window_valid <= 1'b1;
        end
        if (mem_wstrb[0]) mem[word_addr][7:0]   <= mem_wdata[7:0];
        if (mem_wstrb[1]) mem[word_addr][15:8]  <= mem_wdata[15:8];
        if (mem_wstrb[2]) mem[word_addr][23:16] <= mem_wdata[23:16];
        if (mem_wstrb[3]) mem[word_addr][31:24] <= mem_wdata[31:24];
      end
    end
  end
endmodule
