`timescale 1 ns / 1 ps
`default_nettype none
// A mode-0 SPI target for an external controller, polled: a read of BASE returns
// {cs, tx_empty, rx_valid, rx byte} and clears rx_valid; a write of BASE queues the next byte out.
module spi_target #(
  parameter logic [31:0] BASE = 32'h0002_0030
) (
  input  logic        clk,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  input  logic        mem_ren,
  output logic [31:0] mem_rdata,
  input  logic        sck,
  input  logic        cs_n,
  input  logic        mosi,
  output logic        miso,
  output logic        miso_oe
);
  logic [2:0] sck_s;
  logic [1:0] cs_s, mosi_s;
  always_ff @(posedge clk) begin
    sck_s  <= {sck_s[1:0], sck};
    cs_s   <= {cs_s[0], cs_n};
    mosi_s <= {mosi_s[0], mosi};
  end

  logic selected, sck_rise, sck_fall;
  assign selected = !cs_s[1];
  assign sck_rise = selected &&  sck_s[1] && !sck_s[2];
  assign sck_fall = selected && !sck_s[1] &&  sck_s[2];

  logic [7:0] rx_shift, rx_byte, tx_shift, tx_buf, data_q;
  logic [2:0] bit_count;
  logic       rx_valid, tx_empty, tx_write, rx_take, byte_end;
  logic [10:0] rd_word;
  logic        in_range;
  assign in_range = mem_addr[31:3] == BASE[31:3];

  assign miso      = tx_shift[7];
  assign miso_oe   = selected;
  assign mem_rdata = {21'b0, rd_word};

  always_ff @(posedge clk) data_q <= mem_wdata[7:0];

  always_ff @(posedge clk) begin
    if (reset) begin
      bit_count <= 3'b0;
      rx_valid  <= 1'b0;
      tx_empty  <= 1'b1;
      tx_write  <= 1'b0;
      rx_take   <= 1'b0;
      byte_end  <= 1'b0;
      tx_shift  <= 8'b0;
      rd_word   <= 11'b0;
    end else begin
      tx_write <= in_range && mem_wstrb[0];
      rx_take  <= in_range && mem_ren;
      if (tx_write) begin
        tx_buf   <= data_q;
        tx_empty <= 1'b0;
      end

      if (rx_take) rx_valid <= 1'b0;
      if (!selected) begin
        bit_count <= 3'b0;
        byte_end  <= 1'b0;
      end else if (sck_rise) begin
        rx_shift  <= {rx_shift[6:0], mosi_s[1]};
        bit_count <= bit_count + 3'b1;
        if (bit_count == 3'd7) begin
          rx_byte  <= {rx_shift[6:0], mosi_s[1]};
          rx_valid <= 1'b1;
          byte_end <= 1'b1;
        end
      end

      // A byte boundary (or the idle bus) takes the queued byte; an empty queue sends zero. A write landing
      // on the boundary itself is not in tx_buf yet, so it loads the shift register directly.
      if ((selected && byte_end && sck_fall) || (!selected && !tx_empty && !tx_write)) begin
        tx_shift <= tx_write ? data_q : tx_empty ? 8'b0 : tx_buf;
        tx_empty <= 1'b1;
        byte_end <= 1'b0;
      end else if (sck_fall) begin
        tx_shift <= {tx_shift[6:0], 1'b0};
      end

      rd_word <= in_range ? {cs_s[1], tx_empty, rx_valid, rx_byte} : 11'b0;
    end
  end
endmodule
