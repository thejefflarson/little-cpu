// One real dlclkp_1 gating a counter, resolved from the cell library as a netlist's cells are; the probe forces GATE to 0 in a copy.
module nano_gl_gate_probe_fixture (
  input  wire clk,
  input  wire gate,
  output reg [7:0] count
);
  wire gclk;
  sky130_fd_sc_hd__dlclkp_1 icg (.GCLK(gclk), .GATE(gate), .CLK(clk));

  initial count = 8'b0;
  always @(posedge gclk) count <= count + 8'd1;
endmodule

module nano_gl_gate_probe_tb;
  reg clk = 0;
  wire [7:0] count;
  always #5 clk = ~clk;

  nano_gl_gate_probe_fixture dut (.clk(clk), .gate(1'b1), .count(count));

  initial begin
    repeat (40) @(posedge clk);
    if (count > 8'd0) $display("PASS count=%0d", count);
    else $display("FAIL count=%0d", count);
    $finish;
  end
endmodule
