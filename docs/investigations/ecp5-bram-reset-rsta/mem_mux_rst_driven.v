`default_nettype none
// Variant 3 of 4 -- see README.md: a logic-driven synchronous clear folded
// into the block's own reset, plus the output mux variant 2 uses.
module mem_mux_rst_driven (
  input  wire        clk,
  input  wire [8:0]  addr,
  input  wire        sel,
  input  wire        extra_rst,
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
      if (extra_rst) ram_q <= 32'b0;
      else            ram_q <= ram[addr];
      sel_q <= sel;
    end
  end
  assign rdata = sel_q ? ram_q : 32'b0;
endmodule
