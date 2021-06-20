`default_nettype none
module handshake(
  input  var logic clk,
  input  var logic reset,
  output var logic input_ready,
  input  var logic input_valid,
  input  var logic output_ready,
  output var logic output_valid,
  input  var logic busy
);
  always_ff @(posedge clk) begin
    if (reset) begin
      input_ready <= 1;
    end else if (output_ready && !input_valid && !busy) begin
      input_ready <= 1;
    end else if (busy) begin
      input_ready <= 0;
    end
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      output_valid <= 0;
    end else if (input_valid && output_ready && !busy) begin
      output_valid <= 1;
    end else begin
      output_valid <= 0;
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked = 1;
  initial assume(reset);
  always @(*) if(!clocked) assume(reset);
  always_ff @(posedge clk) if(clocked && $past(input_valid) && $past(output_ready) && $past(!busy) && $past(!reset)) assert(output_valid);
  always_ff @(posedge clk) if(clocked && $past(output_ready) && $past(!input_valid) && $past(!busy) && $past(!reset)) assert(input_ready);
  always_ff @(posedge clk) if(clocked && $past(busy)) assert(!output_valid);
  always_ff @(posedge clk) if(clocked && $past(busy) && $past(!reset)) assert(!input_ready);
  always_ff @(posedge clk) if(clocked && $past(reset)) assert(input_ready && !output_valid);
 `endif
endmodule
