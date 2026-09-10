`default_nettype none
// Variant 2 of 4 -- see README.md. Good: the shipped fix, a mux on the
// block's output.
module mem_mux (
  input  wire        clk,
  input  wire [8:0]  addr,
  input  wire        sel,
  input  wire [31:0] wdata,
  input  wire        we,
  output wire [31:0] rdata
);
  reg [31:0] ram[0:511];
  reg [31:0] ram_q;
  reg        sel_q;
  always @(posedge clk) begin
    if (sel && we) begin
      ram[addr] <= wdata;
    end else if (!we) begin
      ram_q <= ram[addr];
      sel_q <= sel;
    end
  end
  assign rdata = sel_q ? ram_q : 32'b0;
endmodule
