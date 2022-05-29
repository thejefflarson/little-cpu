`default_nettype none
module writeback(
  input  var logic clk,
  input  var logic reset,
  // handshake
  input  var logic accessor_valid,
  output var logic writeback_ready,
  // inputs
  input  var logic [4:0] accessor_rd,
  input  var logic [31:0] accessor_rd_data,
  // outputs
  output var logic wen,
  output var logic [4:0] waddr,
  output var logic [31:0] wdata
);
  always_ff @(posedge clk) begin
    writeback_ready <= 1;
    if(reset) begin
      wen <= 0;
      waddr <= 0;
      wdata <= 32'b0;
    end else begin
      if (accessor_valid) begin
        wen <= 1;
        waddr <= accessor_rd;
        wdata <= accessor_rd_data;
      end else begin
        wen <= 0;
      end
    end
  end // always_ff @ (posedge clk)
 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always_ff @(posedge clk) if(clocked && $past(accessor_valid)) assert(wen == 1);
 `endif
endmodule
