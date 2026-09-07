`timescale 1 ns / 1 ps
`default_nettype none
module busarbiter (
    input  logic       clk,
    input  logic       reset,
    input  logic [1:0] request,
    input  logic [1:0] mem_lock,
    output logic [1:0] grant
);
  logic [1:0] winner;
  assign winner = (request == 2'b11) ? (grant[0] ? 2'b10 : 2'b01) : request;

  logic held;
  assign held = |(grant & mem_lock);

  always_ff @(posedge clk) begin
    if (reset) grant <= 2'b00;
    else grant <= held ? grant : winner;
  end
endmodule
