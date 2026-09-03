`timescale 1 ns / 1 ps
`default_nettype none
// A transmit-only UART, 8N1. A byte written while `busy` is set is dropped,
// not queued, so software polls the status register between bytes.
module uart #(
  parameter logic [31:0] BASE     = 32'h0002_0020,
  parameter integer      CLOCK_HZ = 12_000_000,
  parameter integer      BAUD     = 115_200
) (
  input  logic        clk,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  output logic        tx
);
  localparam int DIVISOR    = (CLOCK_HZ + BAUD / 2) / BAUD;
  localparam int DIV_BITS   = $clog2(DIVISOR);
  localparam int FRAME_BITS = 10;
  localparam int COUNT_BITS = $clog2(FRAME_BITS + 1);

  if (|BASE[2:0]) begin : l_base_aligned
    $fatal(1, "uart: BASE must be 8-byte aligned");
  end
  // $clog2(1) is zero, which would declare `baud_count` with no bits in it.
  if (DIVISOR < 2) begin : l_divisor_at_least_two
    $fatal(1, "uart: CLOCK_HZ / BAUD must be at least 2");
  end

  logic [FRAME_BITS-1:0] shift;
  logic [COUNT_BITS-1:0] bits_left;
  logic [DIV_BITS-1:0]   baud_count;

  logic busy;
  assign busy = |bits_left;
  assign tx   = shift[0];

  logic in_range, is_status, start_frame;
  assign in_range  = mem_addr[31:3] == BASE[31:3];
  assign is_status = mem_addr[2];
  assign start_frame = in_range && !is_status && mem_wstrb[0] && !busy;

  logic baud_tick;
  assign baud_tick = busy && baud_count == '0;

  logic rd_busy;
  assign mem_rdata = {31'b0, rd_busy};

  always_ff @(posedge clk) begin
    if (reset) begin
      shift      <= '1;
      bits_left  <= '0;
      baud_count <= '0;
      rd_busy    <= 1'b0;
    end else begin
      if (start_frame) begin
        shift      <= {1'b1, mem_wdata[7:0], 1'b0};
        bits_left  <= FRAME_BITS;
        baud_count <= DIVISOR - 1;
      end else if (baud_tick) begin
        shift      <= {1'b1, shift[FRAME_BITS-1:1]};
        bits_left  <= bits_left - 1'b1;
        baud_count <= DIVISOR - 1;
      end else if (busy) begin
        baud_count <= baud_count - 1'b1;
      end
      // Zero out of range: rtl/littlesoc.v ORs the read buses together.
      rd_busy <= in_range && is_status && busy;
    end
  end
endmodule
