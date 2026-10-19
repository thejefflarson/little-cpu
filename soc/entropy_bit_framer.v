`timescale 1 ns / 1 ps
`default_nettype none
// Streams the sample bit rtl/trng.v reads for every edge in 15-byte frames: marker, sequence number,
// epoch, 64 sample bits and a CRC-16/CCITT-FALSE of the eleven bytes after the marker.
// With FILL the line never idles: a zero byte is sent whenever there is no frame to send.
// The epoch counts how many times `running` has dropped, so a reset of the sequence is never a wrap.
module entropy_bit_framer #(
  parameter integer CLOCK_HZ = 30_000_000,
  parameter bit     FILL     = 1'b0
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
  logic [7:0]  epoch = '0;
  logic        was_running = 1'b0;

  logic [87:0] payload = '0;
  logic        pending = 1'b0;

  typedef enum logic [2:0] {IDLE, HDR1, BODY, TRL, TRH} state_t;
  state_t state = IDLE;
  logic [3:0]  byte_index = '0;
  logic [15:0] crc = 16'hFFFF;
  logic [7:0]  body_byte;
  function automatic logic [15:0] crc_step(input logic [15:0] c, input logic [7:0] d);
    integer i;
    crc_step = c ^ {d, 8'h00};
    for (i = 0; i < 8; i = i + 1) crc_step = crc_step[15] ? {crc_step[14:0], 1'b0} ^ 16'h1021 : {crc_step[14:0], 1'b0};
  endfunction

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

    was_running <= running;
    if (!running) begin
      if (was_running) epoch <= epoch + 1'b1;
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
              payload <= {sample, acc[63:1], epoch, seq};
              pending <= 1'b1;
            end
          end
        end
      end

      case (state)
        IDLE: if (pending && !tx_busy) begin
          shift <= {1'b1, 8'hA5, 1'b0}; bits_left <= 4'd10; baud <= '0;
          crc   <= 16'hFFFF;
          state <= HDR1;
        end else if (FILL && !tx_busy) begin
          shift <= {1'b1, 8'h00, 1'b0}; bits_left <= 4'd10; baud <= '0;
        end
        HDR1: if (!tx_busy) begin
          shift <= {1'b1, 8'h5A, 1'b0}; bits_left <= 4'd10; baud <= '0;
          byte_index <= '0;
          state <= BODY;
        end
        BODY: if (!tx_busy) begin
          shift <= {1'b1, body_byte, 1'b0}; bits_left <= 4'd10; baud <= '0;
          crc   <= crc_step(crc, body_byte);
          byte_index <= byte_index + 1'b1;
          if (byte_index == 4'd10) state <= TRL;
        end
        TRL: if (!tx_busy) begin
          shift <= {1'b1, crc[7:0], 1'b0}; bits_left <= 4'd10; baud <= '0;
          state <= TRH;
        end
        TRH: if (!tx_busy) begin
          shift <= {1'b1, crc[15:8], 1'b0}; bits_left <= 4'd10; baud <= '0;
          pending <= 1'b0;
          state   <= IDLE;
        end
        default: state <= IDLE;
      endcase
    end
  end
endmodule
`default_nettype wire
