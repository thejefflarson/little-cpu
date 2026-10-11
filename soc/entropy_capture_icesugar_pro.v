`timescale 1 ns / 1 ps
`default_nettype none
// Streams the raw edge intervals of OSCG out of the UART: the capture the entropy assessment of
// rtl/trng.v reads. It has no core. An interval is the number of `clk` cycles between two rising
// edges of the oscillator after the synchroniser, which is what rtl/trng.v counts. Each frame is
// 0xA5 0x5A, 4,096 little-endian 16-bit intervals (0xFFFF saturates) and a little-endian 16-bit
// sum of those intervals, and the loop then captures a fresh run, so a frame is never stitched
// from two. The sum is what lets a reader tell a frame from bytes that merely follow a marker.
module entropy_capture_icesugar_pro #(
  parameter integer OSC_DIV  = 128,  // OSCG divider, 2..128
  parameter integer FAB_LOG2 = 8,    // fabric divide-by-2^FAB_LOG2, at least 1
  parameter integer CORE_PAD = 0     // 1 clocks the capture from the 25 MHz pad, 0 from the PLL
) (
  input  logic clk_pin,
  output logic led_r_n,
  output logic led_g_n,
  output logic uart_tx
);
  localparam integer CLOCK_HZ = CORE_PAD ? 25_000_000 : 30_000_000;
  localparam integer DIVISOR  = (CLOCK_HZ + 57_600) / 115_200;
  localparam integer SAMPLES  = 4096;

  logic clk_core, pll_locked;
  icesugar_pro_pll pll (
    .clk_pad(clk_pin),
    .clk_core(clk_core),
    .locked(pll_locked)
  );

  logic clk;
  assign clk = CORE_PAD ? clk_pin : clk_core;

  logic osc_slow;
  icesugar_pro_entropy #(.DIV(OSC_DIV), .DIV_LOG2(FAB_LOG2)) entropy (.raw(osc_slow));

  logic [2:0] sync = '0;
  always_ff @(posedge clk) sync <= {sync[1:0], osc_slow};
  logic edge_seen;
  assign edge_seen = sync[1] && !sync[2];

  logic [3:0] ready_count = '0;
  logic       running;
  assign running = &ready_count;
  always_ff @(posedge clk) begin
    if (!(CORE_PAD != 0 || pll_locked)) ready_count <= '0;
    else if (!running) ready_count <= ready_count + 1'b1;
  end

  logic [15:0] ram [SAMPLES];
  logic [15:0] ram_q;
  logic [11:0] idx = '0;
  logic [15:0] cnt = 16'd1;
  logic        have_prev = 1'b0;
  logic [15:0] sum = '0;

  typedef enum logic [2:0] {CAP, HDR0, HDR1, FETCH, LO, HI, TRL, TRH} state_t;
  state_t state = CAP;

  always_ff @(posedge clk) ram_q <= ram[idx];

  logic [9:0]  shift = '1;
  logic [3:0]  bits_left = '0;
  logic [15:0] baud = '0;
  logic        tx_busy;
  assign tx_busy = |bits_left;
  assign uart_tx = shift[0];

  always_ff @(posedge clk) begin
    if (tx_busy) begin
      if (baud == 16'(DIVISOR - 1)) begin
        baud      <= '0;
        shift     <= {1'b1, shift[9:1]};
        bits_left <= bits_left - 1'b1;
      end else begin
        baud <= baud + 1'b1;
      end
    end

    if (!running) begin
      state     <= CAP;
      idx       <= '0;
      have_prev <= 1'b0;
      cnt       <= 16'd1;
    end else begin
      if (edge_seen) cnt <= 16'd1;
      else if (cnt != 16'hFFFF) cnt <= cnt + 1'b1;

      case (state)
        CAP: if (edge_seen) begin
          have_prev <= 1'b1;
          if (have_prev) begin
            ram[idx] <= cnt;
            idx      <= idx + 1'b1;
            if (idx == 12'(SAMPLES - 1)) begin
              idx   <= '0;
              state <= HDR0;
            end
          end
        end
        HDR0: if (!tx_busy) begin
          shift <= {1'b1, 8'hA5, 1'b0}; bits_left <= 4'd10; baud <= '0;
          sum   <= '0;
          state <= HDR1;
        end
        HDR1: if (!tx_busy) begin
          shift <= {1'b1, 8'h5A, 1'b0}; bits_left <= 4'd10; baud <= '0;
          state <= FETCH;
        end
        FETCH: state <= LO;
        LO: if (!tx_busy) begin
          shift <= {1'b1, ram_q[7:0], 1'b0}; bits_left <= 4'd10; baud <= '0;
          sum   <= sum + ram_q;
          state <= HI;
        end
        HI: if (!tx_busy) begin
          shift <= {1'b1, ram_q[15:8], 1'b0}; bits_left <= 4'd10; baud <= '0;
          idx   <= idx + 1'b1;
          if (idx == 12'(SAMPLES - 1)) begin
            idx       <= '0;
            have_prev <= 1'b0;
            state     <= TRL;
          end else begin
            state <= FETCH;
          end
        end
        TRL: if (!tx_busy) begin
          shift <= {1'b1, sum[7:0], 1'b0}; bits_left <= 4'd10; baud <= '0;
          state <= TRH;
        end
        TRH: if (!tx_busy) begin
          shift <= {1'b1, sum[15:8], 1'b0}; bits_left <= 4'd10; baud <= '0;
          state <= CAP;
        end
        default: state <= CAP;
      endcase
    end
  end

  assign led_r_n = 1'b1;
  assign led_g_n = !running;
endmodule
`default_nettype wire
