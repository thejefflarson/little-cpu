`timescale 1 ns / 1 ps
`default_nettype none
// Streams the sample bit rtl/trng.v reads for every edge in 14-byte frames with a sequence number.
module entropy_bit_framer #(
  parameter integer CLOCK_HZ = 30_000_000
) (
  input  logic clk,
  input  logic running,
  input  logic edge_seen,
  output logic uart_tx
);
  localparam integer DIVISOR = (CLOCK_HZ + 57_600) / 115_200;

  logic [15:0] cnt = 16'd1;
  logic        have_prev = 1'b0;
  logic [1:0]  ticks;
  logic        sample;
  assign ticks  = cnt[1:0] - 2'd1;
  assign sample = ticks[1] ^ ticks[0];

  logic [63:0] acc = '0;
  logic [5:0]  fill = '0;
  logic [15:0] seq = '0;

  logic [79:0] payload = '0;
  logic        pending = 1'b0;

  typedef enum logic [2:0] {IDLE, HDR1, BODY, TRL, TRH} state_t;
  state_t state = IDLE;
  logic [3:0]  byte_index = '0;
  logic [15:0] sum = '0;
  logic [7:0]  body_byte;
  assign body_byte = payload[8*byte_index +: 8];

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
      cnt       <= 16'd1;
      have_prev <= 1'b0;
      fill      <= '0;
      seq       <= '0;
      pending   <= 1'b0;
      state     <= IDLE;
    end else begin
      if (edge_seen) cnt <= 16'd1;
      else if (cnt != 16'hFFFF) cnt <= cnt + 1'b1;

      if (edge_seen) begin
        have_prev <= 1'b1;
        if (have_prev) begin
          fill <= fill + 1'b1;
          acc  <= {sample, acc[63:1]};
          if (&fill) begin
            seq <= seq + 1'b1;
            if (!pending) begin
              payload <= {sample, acc[63:1], seq};
              pending <= 1'b1;
            end
          end
        end
      end

      case (state)
        IDLE: if (pending && !tx_busy) begin
          shift <= {1'b1, 8'hA5, 1'b0}; bits_left <= 4'd10; baud <= '0;
          sum   <= '0;
          state <= HDR1;
        end
        HDR1: if (!tx_busy) begin
          shift <= {1'b1, 8'h5A, 1'b0}; bits_left <= 4'd10; baud <= '0;
          byte_index <= '0;
          state <= BODY;
        end
        BODY: if (!tx_busy) begin
          shift <= {1'b1, body_byte, 1'b0}; bits_left <= 4'd10; baud <= '0;
          sum   <= sum + 16'(body_byte);
          byte_index <= byte_index + 1'b1;
          if (byte_index == 4'd9) state <= TRL;
        end
        TRL: if (!tx_busy) begin
          shift <= {1'b1, sum[7:0], 1'b0}; bits_left <= 4'd10; baud <= '0;
          state <= TRH;
        end
        TRH: if (!tx_busy) begin
          shift <= {1'b1, sum[15:8], 1'b0}; bits_left <= 4'd10; baud <= '0;
          pending <= 1'b0;
          state   <= IDLE;
        end
        default: state <= IDLE;
      endcase
    end
  end
endmodule
`default_nettype wire
