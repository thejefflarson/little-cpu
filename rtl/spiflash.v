`timescale 1 ns / 1 ps
`default_nettype none
// A single-lane, mode-0 SPI shift register for the configuration flash. It
// decodes no commands, so it sends a write enable and a sector erase as
// readily as a read, and the bottom of that flash is the bitstream itself.
module spiflash #(
  parameter logic [31:0] BASE = 32'h0002_0028
) (
  input  logic        clk,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  output logic        sck,
  output logic        mosi,
  input  logic        miso,
  output logic        cs_n
);
  if (|BASE[2:0]) begin : l_base_aligned
    $fatal(1, "spiflash: BASE must be 8-byte aligned");
  end

  logic [7:0] shift_out, shift_in;
  logic [3:0] bits_left;
  logic       selected;

  logic busy;
  assign busy = |bits_left;

  logic [8:0] rd_word;
  logic in_range, is_control, start_xfer, control_write;
  assign in_range      = mem_addr[31:3] == BASE[31:3];
  assign is_control    = mem_addr[2];
  assign start_xfer    = in_range && !is_control && mem_wstrb[0] && !busy;
  assign control_write = in_range &&  is_control && mem_wstrb[0] && !busy;

  assign mem_rdata = {23'b0, rd_word};

  assign mosi = shift_out[7];
  assign cs_n = !selected;

  always_ff @(posedge clk) begin
    if (reset) begin
      selected  <= 1'b0;
      sck       <= 1'b0;
      shift_out <= 8'b0;
      shift_in  <= 8'b0;
      bits_left <= 4'b0;
      rd_word   <= 9'b0;
    end else begin
      if (control_write) selected <= mem_wdata[0];

      if (start_xfer) begin
        shift_out <= mem_wdata[7:0];
        bits_left <= 4'd8;
        sck       <= 1'b0;
      end else if (busy) begin
        if (!sck) begin
          sck <= 1'b1;
        end else begin
          // Sampled a cycle after the rise, once the flash has seen the edge;
          // the next bit is presented while the clock is low.
          shift_in  <= {shift_in[6:0], miso};
          sck       <= 1'b0;
          shift_out <= {shift_out[6:0], 1'b0};
          bits_left <= bits_left - 1'b1;
        end
      end

      // Zero out of range: rtl/littlesoc.v ORs the read buses together.
      rd_word <= (in_range && !is_control) ? {busy, shift_in} : 9'b0;
    end
  end
endmodule
