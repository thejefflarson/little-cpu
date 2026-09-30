// A real enabled flop (dfrtp_1 behind mux2_1), clocked through buf_1 and buf_2 so two drive strengths share one base model as a routed netlist's do; the probe ties the enable low in a copy.
module nano_gl_gate_probe_fixture (
  input  wire clk,
  input  wire reset_n,
  input  wire enable,
  output wire q
);
  wire clk_b1, clk_b2, q_n, next;
  sky130_fd_sc_hd__buf_1 b1 (.X(clk_b1), .A(clk));
  sky130_fd_sc_hd__buf_2 b2 (.X(clk_b2), .A(clk_b1));
  sky130_fd_sc_hd__inv_1 flip (.Y(q_n), .A(q));
  sky130_fd_sc_hd__mux2_1 hold_or_flip (.X(next), .A0(q), .A1(q_n), .S(enable));
  sky130_fd_sc_hd__dfrtp_1 toggle (.Q(q), .D(next), .CLK(clk_b2), .RESET_B(reset_n));
endmodule

module nano_gl_gate_probe_tb;
  reg clk = 0;
  reg reset_n = 0;
  wire q;
  integer toggles = 0;
  always #5 clk = ~clk;

  nano_gl_gate_probe_fixture dut (.clk(clk), .reset_n(reset_n), .enable(1'b1), .q(q));

  always @(q) if (reset_n) toggles = toggles + 1;

  initial begin
    repeat (2) @(negedge clk);
    reset_n = 1;
    repeat (40) @(posedge clk);
    if (toggles > 0 && q !== 1'bx) $display("PASS toggles=%0d", toggles);
    else $display("FAIL toggles=%0d q=%b", toggles, q);
    $finish;
  end
endmodule
