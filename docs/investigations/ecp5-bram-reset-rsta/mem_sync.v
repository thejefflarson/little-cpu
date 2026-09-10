`default_nettype none
// Variant 1 of 4 -- see README.md. Bad: yosys folds the zero arm into the
// block RAM's own synchronous reset.
module mem_sync (
  input  wire        clk,
  input  wire [8:0]  addr,
  input  wire        sel,
  input  wire [31:0] wdata,
  input  wire        we,
  output reg  [31:0] rdata
);
  reg [31:0] ram[0:511];
  always @(posedge clk) begin
    if (sel && we) begin
      ram[addr] <= wdata;
    end else if (!we) begin
      rdata <= sel ? ram[addr] : 32'b0;
    end
  end
endmodule
