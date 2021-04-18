`default_nettype none
module writeback(
  input  var logic        clk,
  input  var logic        reset,
  // handshake
  input  var logic        accessor_valid,
  // inputs
  input  var logic [4:0]  accessor_rd,
  input  var logic [31:0] accessor_rd_data,
  // outputs
  output var logic        wen,
  output var logic [4:0]  waddr,
  output var logic [31:0] wdata
);
  always_ff @(posedge clk) begin
    if(reset) begin
      wen <= 1;
      waddr <= 0;
      wdata <= 32'b0;
    end else begin
      if (accessor_valid) begin
        wen <= 1;
        waddr <= accessor_rd;
        wdata <= accessor_rd_data;
      end
    end
  end
endmodule
