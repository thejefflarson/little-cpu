`timescale 1 ns / 1 ps

module testbench;
  logic [31:0] memory[0:255];
  logic clk = 0;
  logic reset = 0;
  always #5 clk = ~clk;

  initial begin
    $dumpfile("testbench.vcd");
    $dumpvars(0, testbench);
    repeat (10) @(posedge clk);
    reset <= 1;
    mem_ready <= 1;

    repeat (100) @(posedge clk);
    $finish;
  end

  logic        mem_valid;
  logic        mem_instr;
  logic        mem_ready;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  logic        trap;
  logic [1:0]  trap_code;

  always_ff @(posedge clk) begin
    if (mem_valid && mem_wstrb == 4'b0) begin
              $display("mem_addr %8x", mem_addr);

      if (mem_addr < 1024) begin
        $display("hi");

        mem_rdata <= memory[mem_addr >> 2];
        mem_ready <= 1;
      end
    end
  end

  always_ff @(posedge clk) begin
    if(mem_valid && mem_wstrb != 4'b0) begin
      if (mem_addr < 1024) begin
        memory[mem_addr < 1024] <= mem_wdata;
        mem_ready <= 1;
      end
    end
  end

  riscv uut (
    .clk(clk),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap),
    .trap_code(trap_code)
  );

  initial begin
    memory[0] = 32'h 3fc00093; //       li      x1,1020
    memory[1] = 32'h 0000a023; //       sw      x0,0(x1)
    memory[2] = 32'h 0000a103; // loop: lw      x2,0(x1)
    memory[3] = 32'h 00110113; //       addi    x2,x2,1
    memory[4] = 32'h 0020a023; //       sw      x2,0(x1)
    memory[5] = 32'h ff5ff06f; //       j       <loop>
  end

  always_ff @(posedge clk) begin
    if (mem_valid && mem_wstrb == 4'b0) begin
      if (mem_instr) begin
        $display("fetch insn 0x%08x: 0x%08x", mem_addr, mem_rdata);
      end else begin
        $display("fetch data 0x%08x: 0x%08x", mem_addr, mem_rdata);
      end
    end

    if (mem_valid && mem_wstrb != 4'b0) begin
      $display("write 0x%08x: 0x%08x", mem_addr, mem_wdata);
    end

    if (trap) begin
      $display("trap!");
    end
  end
endmodule
