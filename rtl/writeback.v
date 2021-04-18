`default_nettype none
module writeback(
  input  var logic        clk,
  input  var logic        reset,
  // handshake
  input  var logic        writeback_ready,
  input  var logic        writeback_valid,
  // inputs
  input  var logic        accessor_rd,
  input  var logic [31:0] accessor_rd_data,
  // outputs
  output var logic        wen,
  output var logic        waddr,
  output var logic [31:0] wdata
);

endmodule
