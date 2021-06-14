`default_nettype none
module handshake(
  input  var logic clk,
  input  var logic reset,
  output var logic ready,
  output var logic valid,
  input  var logic busy,
  input  var logic next_ready,
  input  var logic prev_valid
);
  always_ff @(posedge clk) begin
    if (reset) begin
      ready <= 1;
    end else if (next_ready && !prev_valid && !busy) begin
      ready <= 1;
    end else if (busy) begin
      ready <= 0;
    end
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      valid <= 0;
    end else if (prev_valid && next_ready && !busy) begin
      valid <= 1;
    end else begin
      valid <= 0;
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked = 1;
  initial assume(reset);
  always @(*) if(!clocked) assume(reset);
  always_ff @(posedge clk) if(clocked && $past(prev_valid) && $past(next_ready) && $past(!busy) && $past(!reset)) assert(valid);
  always_ff @(posedge clk) if(clocked && $past(next_ready) && $past(!prev_valid) && $past(!busy) && $past(!reset)) assert(ready);
  always_ff @(posedge clk) if(clocked && $past(busy)) assert(!valid);
  always_ff @(posedge clk) if(clocked && $past(busy) && $past(!reset)) assert(!ready);
  always_ff @(posedge clk) if($past(reset)) assert(ready && !valid);
 `endif
endmodule
