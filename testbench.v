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
    repeat (1000) @(posedge clk);
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

  always_ff @(posedge clk) begin
    mem_ready <= 0;
    if (mem_valid && !mem_ready) begin
      if (mem_addr < 1024) begin
        mem_rdata <= memory[mem_addr >> 2];
        if(mem_wstrb[0]) memory[mem_addr >> 2][7:0] <= mem_wdata[7:0];
        if(mem_wstrb[1]) memory[mem_addr >> 2][15:8] <= mem_wdata[15:8];
        if(mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
        if(mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];

        mem_ready <= 1;
      end
    end
  end

  riscv uut (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap)
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
    if (mem_valid && mem_ready) begin
      if (mem_instr) begin
        $display("fetch insn 0x%08x: 0x%08x", mem_addr, mem_rdata);
      end else if (mem_wstrb != 4'b0) begin
        $display("write 0x%08x: 0x%08x", mem_addr, mem_wdata);
      end else begin
        $display("fetch data 0x%08x: 0x%08x", mem_addr, mem_rdata);
      end
    end
    if (trap) begin
      $display("trap!");
    end
  end
endmodule
