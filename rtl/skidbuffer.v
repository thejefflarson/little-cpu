`default_nettype none
module skidbuffer
#(
  parameter WIDTH = 32
)
(
  input  var logic             clk,
  input  var logic             reset,
  output var logic             input_ready,
  input  var logic             input_valid,
  input  var logic [WIDTH-1:0] input_data,
  input  var logic             output_ready,
  output var logic             output_valid,
  output var logic [WIDTH-1:0] output_data
);
  logic [WIDTH-1:0] buffer;
  logic [WIDTH-1:0] out;
  logic             insert;
  logic             remove;
  always_comb begin
    insert = (input_ready && input_valid);
    remove = (output_ready && output_valid);
  end

  logic [1:0] state;
  localparam empty = 2'b00;
  localparam busy = 2'b01;
  localparam full = 2'b10;

  always_ff @(posedge clk) begin
    if (reset) begin
      input_ready <= 1;
      output_valid <= 0;
      state <= empty;
    end begin
      input_ready <= 1;
      output_valid <= output_ready;
      (* parallel_case, full_case *)
      case(state)
        empty: if (insert && !remove) begin
          state <= busy;
          output_valid <= 0;
        end
        busy: if (insert && !remove) begin
          state <= full;
        end else if (!insert && remove) begin
          state <= empty;
        end else if (insert && remove) begin
          state <= busy;
        end
        full: if (!insert && remove) begin
          state <= busy;
          input_ready <= 0;
        end
      endcase
    end
  end
endmodule
