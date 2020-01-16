module riscv (
   input logic clk,
   output reg a
);
   reg [31:0] regs[0:31];
   assign regs[0] = 0;

   always @(posedge clk) begin
      a <= ~clk;
   end
endmodule
