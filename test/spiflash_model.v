`timescale 1 ns / 1 ps
`default_nettype none
// A model of the SPI flash the board configures from, for the two simulators.
// It answers two commands: `0x9F`, the JEDEC id, and `0x03`, a sequential read
// of the data array. Every other command reads back zeroes.
//
// THE DATA ARRAY IS A FUNCTION, NOT AN ARRAY. A real memory here would have to
// be filled, and the two legs fill memories differently -- iverilog runs
// `$readmemh` and the cxxrtl runner pokes `debug_items`, which is why
// rtl/imemory.v's init is guarded. A function needs neither, is identical in
// both legs by construction, and gives a program something it can predict at
// any offset without a file travelling beside it. `flash_byte` below is that
// function, and test/asm/spiflash.S recomputes it rather than quoting bytes.
//
// CLOCKED BY THE BUS CLOCK, NOT BY `sck`. `sck` is a signal the design
// produces, and a module clocked by it is a second clock domain in a tree that
// has exactly one everywhere else -- the cxxrtl leg in particular is built
// around a single `clk`. So this samples `sck` and works from its edges, which
// is what a real device's input register does anyway.
//
// TIMING IS NOT MODELLED. No `tRES`, no page boundary, no write path at all --
// this is read-only, like the controller that drives it.
module spiflash_model #(
  // The three bytes `0x9F` returns. Winbond W25Q32JV, which is what the
  // UPduino carries and what `iceprog` prints.
  parameter logic [7:0] JEDEC0 = 8'hEF,
  parameter logic [7:0] JEDEC1 = 8'h70,
  parameter logic [7:0] JEDEC2 = 8'h16
) (
  input  logic clk,
  input  logic sck,
  input  logic cs_n,
  input  logic mosi,
  output logic miso
);
  // The whole data array, as a function of the address. Both halves of the
  // address take part so that a sequential read crossing a 256-byte boundary
  // does not repeat, which is the mistake a program reading only `a[7:0]`
  // could not tell from a controller that stopped incrementing.
  function automatic logic [7:0] flash_byte(input logic [23:0] a);
    flash_byte = a[7:0] ^ a[15:8] ^ 8'h5a;
  endfunction

  logic [7:0]  rx, tx, tx_next;
  logic [2:0]  bit_count;
  logic [7:0]  command;
  logic [23:0] address;
  logic [1:0]  addr_bytes;
  logic        have_command, byte_done, sck_q;

  // The line is released between transactions; the board's pull-up is what
  // makes that a one, and this drives one for the same reason.
  assign miso = cs_n ? 1'b1 : tx[7];

  logic sck_rise, sck_fall;
  assign sck_rise = !cs_n &&  sck && !sck_q;
  assign sck_fall = !cs_n && !sck &&  sck_q;

  // The byte completing on this rising edge. Read before `rx` has taken the
  // last bit, so the incoming bit is spelled out.
  logic [7:0] incoming;
  assign incoming = {rx[6:0], mosi};

  always_ff @(posedge clk) begin
    sck_q <= sck;

    if (cs_n) begin
      // Deselecting ends the transaction outright. A flash that carried its
      // state across a chip select would let a program read the right bytes
      // for the wrong reason.
      bit_count    <= 3'b0;
      have_command <= 1'b0;
      addr_bytes   <= 2'b0;
      byte_done    <= 1'b0;
      command      <= 8'b0;
      address      <= 24'b0;
      rx           <= 8'b0;
      tx           <= 8'b0;
      tx_next      <= 8'b0;
    end else begin
      if (sck_rise) begin
        rx        <= incoming;
        bit_count <= bit_count + 3'b1;
        if (bit_count == 3'd7) begin
          byte_done <= 1'b1;
          if (!have_command) begin
            have_command <= 1'b1;
            command      <= incoming;
            // `0x9F` starts answering immediately; everything else spends the
            // next bytes on an address or on nothing.
            tx_next      <= (incoming == 8'h9f) ? JEDEC0 : 8'h00;
          end else if (command == 8'h03 && addr_bytes != 2'd3) begin
            address    <= {address[15:0], incoming};
            addr_bytes <= addr_bytes + 2'b1;
            // The third address byte completes the address, and the byte at it
            // is what the next exchange carries.
            tx_next    <= (addr_bytes == 2'd2)
                            ? flash_byte({address[15:0], incoming}) : 8'h00;
          end else if (command == 8'h03) begin
            address <= address + 24'd1;
            tx_next <= flash_byte(address + 24'd1);
          end else if (command == 8'h9f) begin
            // Three id bytes and then zeroes, which is what a part with a
            // two-byte continuation code does not do -- this one has none.
            case (address[1:0])
              2'd0:    tx_next <= JEDEC1;
              2'd1:    tx_next <= JEDEC2;
              default: tx_next <= 8'h00;
            endcase
            address <= address + 24'd1;
          end else begin
            tx_next <= 8'h00;
          end
        end
      end

      if (sck_fall) begin
        // The output changes while the clock is low, so the controller's rising
        // edge samples a bit that has been stable for half a period. A byte
        // boundary loads instead of shifting: the bit already presented at the
        // completing rising edge belongs to the byte that just ended.
        if (byte_done) begin
          tx        <= tx_next;
          byte_done <= 1'b0;
        end else begin
          tx <= {tx[6:0], 1'b0};
        end
      end
    end
  end
endmodule
