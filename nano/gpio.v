`default_nettype none
// Two memory-mapped words: an output register the CPU writes and reads back, and an
// input register reflecting the pad after a two-flop synchroniser. Reads are
// combinational, unlike littlecpu's UART status byte, because nano's bus has no
// pipeline stage to absorb a registered read's one-cycle lag.
module nano_gpio #(
  parameter logic [31:0] BASE = 32'h1080_0008
) (
  input  logic        clk,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  input  logic [7:0]  pin_in,
  output logic [6:0]  pin_out
);
  if (|BASE[2:0]) begin : l_base_aligned
    $fatal(1, "nano_gpio: BASE must be 8-byte aligned");
  end

  logic in_range, is_in_reg;
  assign in_range = mem_addr[31:3] == BASE[31:3];
  assign is_in_reg = mem_addr[2];

  logic [6:0] out_reg;
  assign pin_out = out_reg;

  logic [7:0] in_sync1, in_sync2;

  always_ff @(posedge clk) begin
    if (reset) begin
      out_reg  <= 7'b0;
      in_sync1 <= 8'b0;
      in_sync2 <= 8'b0;
    end else begin
      in_sync1 <= pin_in;
      in_sync2 <= in_sync1;
      if (in_range && !is_in_reg && mem_wstrb[0]) out_reg <= mem_wdata[6:0];
    end
  end

  assign mem_rdata = !in_range ? 32'b0 : is_in_reg ? {24'b0, in_sync2} : {25'b0, out_reg};
endmodule

`default_nettype wire
