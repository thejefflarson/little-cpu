module testbench (input logic clk);
   wire a;

   riscv uut(clk, a);
endmodule // testbench
