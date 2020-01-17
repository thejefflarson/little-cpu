module testbench (input logic clk);
   wire a;

   reg[31:0] memory[0:319]
   riscv uut(clk, a);
endmodule // testbench
