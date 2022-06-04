`default_nettype none
module handshake(
  input  logic clk,
  input  logic reset,
  output logic unit_ready,
  input  logic input_valid,
  input  logic output_ready,
  output logic unit_valid,
  input  logic busy
);
  always_ff @(posedge clk) begin
    if (reset) begin
      unit_ready <= 1;
    end else if (output_ready && !input_valid && !busy) begin
      unit_ready <= 1;
    end else begin
      unit_ready <= 0;
    end
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      unit_valid <= 0;
    end else if (input_valid && output_ready && !busy) begin
      unit_valid <= 1;
    end else begin
      unit_valid <= 0;
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked = 1;
  initial assume(reset);
  always @(*) if(!clocked) assume(reset);
  always_ff @(posedge clk) if(clocked && $past(input_valid) && $past(output_ready) && $past(!busy) && $past(!reset)) assert(unit_valid);
  always_ff @(posedge clk) if(clocked && $past(output_ready) && $past(!input_valid) && $past(!busy) && $past(!reset)) assert(unit_ready);
  always_ff @(posedge clk) if(clocked && $past(busy)) assert(!unit_valid);
  always_ff @(posedge clk) if(clocked && $past(busy) && $past(!reset)) assert(!unit_ready);
  always_ff @(posedge clk) if(clocked && $past(reset)) assert(unit_ready && !unit_valid);
 `endif
endmodule
