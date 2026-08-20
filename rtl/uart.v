`timescale 1 ns / 1 ps
`default_nettype none
// A transmit-only UART on the data bus, so a flashed bitstream has something to
// say. Without it the board's whole observable output is two LEDs, and a
// program reports its verdict through `tohost` -- a simulation convention with
// no hardware behind it.
//
// NO RECEIVER, NO FIFO AND NO INTERRUPT, and none of the three is an oversight.
// A receiver needs an oversampler and a second half of the map to put it in; a
// queue needs memory this part would rather give the core; an interrupt would
// be a second source, and the machine timer is the only line rtl/csrs.v has a
// pending bit for. Software polls `busy` below, which is the whole cost of the
// device.
//
// Two registers, eight bytes:
//
//   BASE+0  data    writing byte lane 0 starts a frame; reads as zero
//   BASE+4  status  bit 0 is `busy`; the rest read zero, and a write is ignored
//
// A frame is 8N1: one low start bit, eight data bits least significant first,
// one high stop bit, ten bit times in all, and the line idles high between
// them. A write arriving while `busy` is set is DROPPED, not queued -- that is
// what having no FIFO means, and it is why software reads the status register
// before every byte.
//
// THE BAUD RATE IS THE PLATFORM'S TO STATE, the way the machine timer's tick
// period is: firmware can read neither the crystal nor this divisor, so neither
// can be derived from anything the ISA exposes. The divisor is the clock over
// the baud rate, rounded to the nearest cycle. AT 12 MHz AND 115200 BAUD IT IS
// 104, AND 12000000/104 IS 115384.6 BAUD -- 0.16% FAST. A receiver samples the
// middle of each bit, so by the stop bit, nine bit times out, that error has
// accumulated to 1.5% of a bit time against the roughly 5% an asynchronous
// receiver tolerates. 9600 baud divides exactly at 1250 and has no error at
// all; a clock or a baud rate that leaves more than about 2% belongs at a
// slower rate rather than here.
//
// Nothing in this module reaches the core's fetch loop. It answers the data bus
// only, `tx` leaves the chip, and the read-back is one bit wide by construction
// -- see `mem_rdata` below.
module uart #(
  // The eight bytes above rtl/timer.v's four words, so the four ranges on the
  // shared bus abut and the read buses can be ORed together.
  parameter logic [31:0] BASE     = 32'h0002_0010,
  // The two numbers the divisor is derived from, rather than the divisor: a
  // literal here would silently keep its old meaning the day either moves.
  // CLOCK_HZ is the board crystal rtl/littlesoc.v is placed and timed against.
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

  // The two registers are an aligned 8-byte window, so membership is an
  // equality on the bits above it and the register select is one bit of the
  // address itself -- neither needs the subtraction they replace, and both are
  // true only while BASE is 8-byte aligned. Off it the equality names a window
  // this module does not occupy and the select picks the wrong register, where
  // the subtract-and-compare was merely slow. So it is an elaboration check
  // rather than a comment.
  if (|BASE[2:0]) begin : l_base_aligned
    $fatal(1, "uart: BASE must be 8-byte aligned");
  end
  // One cycle per bit is not a serial line, it is the clock; and $clog2(1) is
  // zero, which would declare a counter with no bits in it.
  if (DIVISOR < 2) begin : l_divisor_at_least_two
    $fatal(1, "uart: CLOCK_HZ / BAUD must be at least 2");
  end

  // {stop, data, start}, sent out of bit 0. Ones are shifted in behind it, so
  // the register reaches all ones and holds the line at its idle level with no
  // mux in front of `tx`.
  logic [FRAME_BITS-1:0] shift;
  logic [COUNT_BITS-1:0] bits_left;
  logic [DIV_BITS-1:0]   baud_count;

  logic busy;
  assign busy = |bits_left;
  assign tx   = shift[0];

  logic in_range, is_status, start_frame;
  assign in_range  = mem_addr[31:3] == BASE[31:3];
  assign is_status = mem_addr[2];
  // Byte lane 0, so an `sb` to the data register works and an `sb` to any other
  // lane of it is not a transmission of whatever happened to be on the bus.
  // `busy` is what drops a write that arrives mid-frame.
  assign start_frame = in_range && !is_status && mem_wstrb[0] && !busy;

  logic baud_tick;
  assign baud_tick = busy && baud_count == '0;

  // ONE FLIP-FLOP, not a registered 32-bit word. Everything this device reports
  // is `busy`, so the other 31 bits are constants that yosys folds out of the
  // OR in rtl/littlesoc.v: the three-input OR the other memories already need
  // stays one `SB_LUT4` per bit and gains a fourth input on bit 0 alone. A
  // 32-bit register here would cost 32 flops to say the same thing.
  logic rd_busy;
  assign mem_rdata = {31'b0, rd_busy};

  always_ff @(posedge clk) begin
    if (reset) begin
      // All ones: an idle line is high, and this is the state the shifting
      // below walks back to on its own.
      shift      <= '1;
      bits_left  <= '0;
      baud_count <= '0;
      rd_busy    <= 1'b0;
    end else begin
      // `start_frame` tests `!busy` and `baud_tick` tests `busy`, so these two
      // arms are disjoint and the order between them states nothing.
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
      // An out-of-range access reads zero, so the memories on this bus join
      // with an OR rather than a mux. The data register reads zero as well: it
      // is write-only, and reporting the byte in flight would be a second thing
      // to keep true.
      rd_busy <= in_range && is_status && busy;
    end
  end
endmodule
