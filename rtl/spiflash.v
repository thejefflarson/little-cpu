`timescale 1 ns / 1 ps
`default_nettype none
// A byte-at-a-time SPI master on the data bus, so software can read the flash
// the part already configures from. Single lane, mode 0, one byte per
// transaction step -- the whole device is a shift register, a bit counter and a
// chip select.
//
// WHY A SHIFT REGISTER AND NOT A COMMAND ENGINE. A `0x03` sequential read is a
// command byte, three address bytes and then as many byte exchanges as the
// caller wants, and every one of those is the same eight clocks. A device that
// knew what `0x03` meant would have to know what the other 40 commands mean the
// first time one is needed; this one already reads the JEDEC id, the status
// register and the data array, because it does not know what any of them are.
//
// Two registers, eight bytes:
//
//   BASE+0  data     writing byte lane 0 shifts that byte out and shifts eight
//                    bits in; reading gives the byte the LAST exchange shifted
//                    in
//   BASE+4  control  writing bit 0 drives the chip select -- 1 selects the
//                    flash, 0 releases it; reading gives `busy` in bit 0
//
// A write to either register while `busy` is set is DROPPED, the way rtl/uart.v
// drops a byte written mid-frame, so software polls the control register
// between steps. Nothing here is queued and nothing is interrupted.
//
// MODE 0, WHICH IS THE ONE THE PART ITSELF USES: the clock idles low, this
// master presents a bit while it is low, and both ends sample on the rising
// edge. `sck` is the bus clock divided by two, so a byte is sixteen cycles --
// 1.33 us at 12 MHz, and 8 KB is about 11 ms.
//
// THE CHIP SELECT IS ALSO THE BOARD'S OUTPUT ENABLE. On the UPduino the four
// pins below are shared with the FTDI, which drives three of them whenever the
// host talks to the flash, so a design that drove them all the time would fight
// `iceprog` for the wire. `cs_n` is published for soc/board_upduino.v to use as
// the enable on all three outputs: this master drives the pins only while it
// holds the flash selected, and lets go the rest of the time.
//
// Nothing in this module reaches the core's fetch loop. It answers the data bus
// only, and its read-back is eight bits wide by construction.
module spiflash #(
  // The eight bytes above rtl/uart.v's two registers, so the five regions on
  // the shared bus abut and the read buses can be ORed together.
  parameter logic [31:0] BASE = 32'h0002_0028
) (
  input  logic        clk,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  // The flash's pins. `sck` and `mosi` are meaningful only while `cs_n` is low;
  // see the output-enable note above.
  output logic        sck,
  output logic        mosi,
  input  logic        miso,
  output logic        cs_n
);
  // The two registers are an aligned 8-byte window, so membership is an
  // equality on the bits above it and the register select is one bit of the
  // address. Both are true only while BASE is 8-byte aligned, so this is an
  // elaboration check and not a comment -- off the alignment the equality names
  // a window this module does not occupy.
  if (|BASE[2:0]) begin : l_base_aligned
    $fatal(1, "spiflash: BASE must be 8-byte aligned");
  end

  logic [7:0] shift_out, shift_in;
  logic [3:0] bits_left;
  logic       phase, selected;
  logic [7:0] rd_byte;

  logic busy;
  assign busy = |bits_left;

  logic in_range, is_control, start_xfer, control_write;
  assign in_range      = mem_addr[31:3] == BASE[31:3];
  assign is_control    = mem_addr[2];
  // Byte lane 0, so an `sb` works and an `sb` to any other lane is not a
  // transaction started with whatever happened to be on the bus.
  assign start_xfer    = in_range && !is_control && mem_wstrb[0] && !busy;
  assign control_write = in_range &&  is_control && mem_wstrb[0] && !busy;

  // EIGHT FLIP-FLOPS, not a registered 32-bit word: everything this device
  // reports fits in a byte, so the other 24 bits are constants yosys folds out
  // of the read-back OR in rtl/littlesoc.v.
  assign mem_rdata = {24'b0, rd_byte};

  assign mosi = shift_out[7];
  assign cs_n = !selected;

  always_ff @(posedge clk) begin
    if (reset) begin
      selected  <= 1'b0;
      sck       <= 1'b0;
      shift_out <= 8'b0;
      shift_in  <= 8'b0;
      bits_left <= 4'b0;
      phase     <= 1'b0;
      rd_byte   <= 8'b0;
    end else begin
      if (control_write) selected <= mem_wdata[0];

      // `start_xfer` tests `!busy` and the other arm tests `busy`, so the two
      // are disjoint and the order between them states nothing.
      if (start_xfer) begin
        shift_out <= mem_wdata[7:0];
        bits_left <= 4'd8;
        phase     <= 1'b0;
        sck       <= 1'b0;
      end else if (busy) begin
        if (!phase) begin
          // The rising edge. The bit this master presents has been on the wire
          // for a whole cycle by now, which is the setup the flash is owed.
          sck   <= 1'b1;
          phase <= 1'b1;
        end else begin
          // The clock is high across this cycle, so the flash's bit is the one
          // it settled on before the rise: sample it here and not on the edge
          // that produced the rise, where the flash has not yet seen anything.
          // Then fall, and present the next bit while the clock is low.
          shift_in  <= {shift_in[6:0], miso};
          sck       <= 1'b0;
          shift_out <= {shift_out[6:0], 1'b0};
          bits_left <= bits_left - 1'b1;
          phase     <= 1'b0;
        end
      end

      // An out-of-range access reads zero, so the memories on this bus join
      // with an OR rather than a mux. The data register reads the byte the last
      // exchange shifted in, which is the only thing a read-only master has to
      // report; `shift_in` is stable whenever software is allowed to read it,
      // because a read arriving mid-exchange is a read of a device the caller
      // was told to poll first.
      rd_byte <= !in_range   ? 8'b0
               :  is_control ? {7'b0, busy}
                             : shift_in;
    end
  end
endmodule
