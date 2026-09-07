`timescale 1 ns / 1 ps
`default_nettype none
// A model of the SPI flash the board configures from, for the two simulators.
module spiflash_model #(
  // The three bytes `0x9F` returns.
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
  // The whole data array, as a function of the address.
  function automatic logic [7:0] flash_byte(input logic [23:0] a);
    flash_byte = a[7:0] ^ a[15:8] ^ 8'h5a;
  endfunction

  logic [7:0]  rx, tx, tx_next;
  logic [2:0]  bit_count;
  logic [7:0]  command;
  logic [23:0] address;
  logic [1:0]  addr_bytes;
  logic        have_command, byte_done, sck_q;

  assign miso = cs_n ? 1'b1 : tx[7];

  logic sck_rise, sck_fall;
  assign sck_rise = !cs_n &&  sck && !sck_q;
  assign sck_fall = !cs_n && !sck &&  sck_q;

  // The byte completing on this rising edge.
  logic [7:0] incoming;
  assign incoming = {rx[6:0], mosi};

  always_ff @(posedge clk) begin
    sck_q <= sck;

    if (cs_n) begin
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
            tx_next      <= (incoming == 8'h9f) ? JEDEC0 : 8'h00;
          end else if (command == 8'h03 && addr_bytes != 2'd3) begin
            address    <= {address[15:0], incoming};
            addr_bytes <= addr_bytes + 2'b1;
            tx_next    <= (addr_bytes == 2'd2)
                            ? flash_byte({address[15:0], incoming}) : 8'h00;
          end else if (command == 8'h03) begin
            address <= address + 24'd1;
            tx_next <= flash_byte(address + 24'd1);
          end else if (command == 8'h9f) begin
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
