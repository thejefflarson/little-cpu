`timescale 1 ns / 1 ps
//`define RISCV_FORMAL
module testbench;
  logic [31:0] memory[0:255];
  logic [31:0] rom[0:255];
  logic clk = 0;
  logic reset = 1;
  // always #5 clk = ~clk;

  // initial begin
  //   $dumpfile("testbench.vcd");
  //   $dumpvars(0, testbench);
  //   repeat (1) @(posedge clk);
  //   reset <= 0;
  //   repeat (200) @(posedge clk);
  //   $finish;
  // end

  logic [31:0] imem_addr;
  logic [31:0] imem_data;
  logic        mem_valid;
  logic        mem_instr;
  logic        mem_ready;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  logic        trap;
 `ifdef RISCV_FORMAL
  logic        rvfi_valid;
  logic [63:0] rvfi_order;
  logic [31:0] rvfi_insn;
  logic        rvfi_trap;
  logic        rvfi_halt;
  logic        rvfi_intr;
  logic [4:0]  rvfi_rs1_addr;
  logic [4:0]  rvfi_rs2_addr;
  logic [31:0] rvfi_rs1_rdata;
  logic [31:0] rvfi_rs2_rdata;
  logic [4:0]  rvfi_rd_addr;
  logic [31:0] rvfi_rd_wdata;
  logic [31:0] rvfi_pc_rdata;
  logic [31:0] rvfi_pc_wdata;
  logic [31:0] rvfi_mem_addr;
  logic [3:0]  rvfi_mem_rmask;
  logic [3:0]  rvfi_mem_wmask;
  logic [31:0] rvfi_mem_rdata;
  logic [31:0] rvfi_mem_wdata;
 `endif

  always_ff @(posedge clk) begin
    mem_valid <= 0;
    if (!mem_valid && mem_ready) begin
      if (mem_addr < 1024) begin
        mem_rdata <= memory[mem_addr >> 2];
        if(mem_wstrb[0]) memory[mem_addr >> 2][7:0] <= mem_wdata[7:0];
        if(mem_wstrb[1]) memory[mem_addr >> 2][15:8] <= mem_wdata[15:8];
        if(mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
        if(mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
        mem_valid <= 1;
      end
    end
  end // always_ff @ (posedge clk)

  always_ff @(posedge clk) begin
    imem_data <= rom[imem_addr[31:2]];
  end

  littlecpu uut (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .mem_valid(mem_valid),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap)
   `ifdef RISCV_FORMAL
    , .rvfi_valid(rvfi_valid),
    .rvfi_order(rvfi_order),
    .rvfi_insn(rvfi_insn),
    .rvfi_trap(rvfi_trap),
    .rvfi_halt(rvfi_halt),
    .rvfi_intr(rvfi_intr),
    .rvfi_rs1_addr(rvfi_rs1_addr),
    .rvfi_rs2_addr(rvfi_rs2_addr),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_rd_addr(rvfi_rd_addr),
    .rvfi_rd_wdata(rvfi_rd_wdata),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_pc_wdata(rvfi_pc_wdata),
    .rvfi_mem_addr(rvfi_mem_addr),
    .rvfi_mem_rmask(rvfi_mem_rmask),
    .rvfi_mem_wmask(rvfi_mem_wmask),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .rvfi_mem_wdata(rvfi_mem_wdata)
   `endif
  );
 `ifdef RISCV_FORMAL
  monitor monitor (
    .clock(clk),
    .reset(reset),
    .rvfi_valid(rvfi_valid),
    .rvfi_order(rvfi_order),
    .rvfi_insn(rvfi_insn),
    .rvfi_trap(rvfi_trap),
    .rvfi_halt(rvfi_halt),
    .rvfi_intr(rvfi_intr),
    .rvfi_rs1_addr(rvfi_rs1_addr),
    .rvfi_rs2_addr(rvfi_rs2_addr),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_rd_addr(rvfi_rd_addr),
    .rvfi_rd_wdata(rvfi_rd_wdata),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_pc_wdata(rvfi_pc_wdata),
    .rvfi_mem_addr(rvfi_mem_addr),
    .rvfi_mem_rmask(rvfi_mem_rmask),
    .rvfi_mem_wmask(rvfi_mem_wmask),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .rvfi_mem_wdata(rvfi_mem_wdata)
  );
 `endif
  initial begin
    rom[0] = 32'h 3fc00093; //       li      x1,1020
    rom[1] = 32'h 0000a023; //       sw      x0,0(x1)
    rom[2] = 32'h 0000a103; // loop: lw      x2,0(x1)
    rom[3] = 32'h 00110113; //       addi    x2,x2,1
    rom[4] = 32'h 0020a023; //       sw      x2,0(x1)
    rom[5] = 32'h ff5ff06f; //       j       <loop>
  end

  logic [31:0] past_addr;
  initial past_addr = 32'b1;
  always_ff @(posedge clk) begin
    if (past_addr != imem_addr) begin
      $display("ifetch 0x%08x: 0x%08x", imem_addr, imem_data);
      past_addr <= imem_addr;
    end
  end

  always_ff @(posedge clk) begin
    if (mem_valid && mem_ready) begin
      if (mem_wstrb) begin
        $display("write  0x%08x: 0x%08x (wstrb=%b)", mem_addr, mem_wdata, mem_wstrb);
      end else begin
        $display("read   0x%08x: 0x%08x", mem_addr, mem_rdata);
      end
    end
    if (trap) begin
      $display("trap!");
    end
  end
endmodule
