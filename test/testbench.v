`timescale 1 ns / 1 ps
//`define RISCV_FORMAL
module testbench(
`ifndef ICARUS
	input clk,
	input reset
`endif
);
  // Sized to test/asm/sections.lds (ADR-0008): rom holds >=16K of .text,
  // memory (RAM) holds >=4K of .data/.rodata/.bss based at RAM_BASE, which
  // matches the ram region's ORIGIN there. RAM_BASE is non-zero so a wild
  // store through an uninitialized/zero pointer lands outside the mapped
  // region instead of silently aliasing real test data. The cxxrtl runner
  // (test/cxxrtl.cc) subtracts RAM_BASE back out of the `--ram` image's
  // word addresses before poking `memory` via debug_items. rom grew from
  // 8K/2048 words to 16K/4096 when test/asm/rvc.S landed (ADR-0003/
  // ADR-0021) -- see sections.lds for why.
  localparam logic [31:0] RAM_BASE  = 32'h0001_0000;
  localparam int          ROM_WORDS = 4096;
  localparam int          RAM_WORDS = 1024;
  logic [31:0] memory[0:RAM_WORDS-1];
  logic [31:0] rom[0:ROM_WORDS-1];
  logic [31:0] imem_addr;
  logic [31:0] imem_data = 32'b0;
  // ADR-0003: the second word of the dual-word combinational fetch window
  // (rtl/fetcher.v drives imem_addr2 = imem_addr + 4), read from the same
  // `rom` array at a second, independent index.
  logic [31:0] imem_addr2;
  logic [31:0] imem_data2 = 32'b0;
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
  // ADR-0006: the monitor's own per-retire error code (0 = no error this
  // cycle). test/cxxrtl.cc reads this by hierarchical debug-item name
  // ("monitor errcode") to turn a mismatch into a distinct process exit
  // code; under iverilog the monitor's own $display diagnostics (which
  // this triggers) are the loud failure.
  logic [15:0] rvfi_monitor_errcode;
 `endif //  `ifdef RISCV_FORMAL
 `ifdef ICARUS
  logic clk = 0;
  logic reset = 1;
  always #5 clk = ~clk;

  initial begin
    $dumpfile("testbench.vcd");
    $dumpvars(0, testbench);
    repeat (1) @(posedge clk);
    reset <= 0;
    repeat (200) @(posedge clk);
    $finish;
  end
 `endif
  always_ff @(posedge clk) begin
    if (mem_addr >= RAM_BASE && mem_addr < RAM_BASE + RAM_WORDS * 4) begin
      mem_rdata <= memory[(mem_addr - RAM_BASE) >> 2];
      if(mem_wstrb[0]) memory[(mem_addr - RAM_BASE) >> 2][7:0] <= mem_wdata[7:0];
      if(mem_wstrb[1]) memory[(mem_addr - RAM_BASE) >> 2][15:8] <= mem_wdata[15:8];
      if(mem_wstrb[2]) memory[(mem_addr - RAM_BASE) >> 2][23:16] <= mem_wdata[23:16];
      if(mem_wstrb[3]) memory[(mem_addr - RAM_BASE) >> 2][31:24] <= mem_wdata[31:24];
    end
  end // always_ff @ (posedge clk)

  // Selected out of the always_comb for the same reason as decoder.v's
  // register-index fields: a constant part-select inside an always_* block
  // defeats iverilog's sensitivity analysis and draws a `sorry:` note.
  logic [11:0] rom_index, rom_index2;
  assign rom_index  = imem_addr[13:2];
  assign rom_index2 = imem_addr2[13:2];

  always_comb //@(posedge clk)
    if(reset) begin
      imem_data = 32'b0;
    end else begin
      imem_data = rom[rom_index];
    end

  // ADR-0003: second, independent combinational read of the same `rom`
  // array for the dual-word fetch window's other word.
  always_comb
    if(reset) begin
      imem_data2 = 32'b0;
    end else begin
      imem_data2 = rom[rom_index2];
    end

  littlecpu uut (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
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
    .rvfi_mem_wdata(rvfi_mem_wdata),
    .errcode(rvfi_monitor_errcode)
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
  initial past_addr = 32'b0;
  always @(posedge clk) begin
    if (past_addr != imem_addr) begin
      $display("ifetch 0x%08x: 0x%08x", imem_addr, imem_data);
      past_addr <= imem_addr;
    end
  end

  always @(posedge clk) begin
    if (mem_wstrb != 4'b0000) begin
      $display("write  0x%08x: 0x%08x (wstrb=%b)", mem_addr, mem_wdata, mem_wstrb);
    end else begin
      $display("read   0x%08x: 0x%08x", mem_addr, mem_rdata);
    end
    if (trap) begin
      $display("trap!");
    end
  end

  // ADR-0009: for every store, mem_wstrb is high for
  // exactly one cycle. Direct regression for the divide-replay defect, where
  // the accessor kept re-issuing the same store every cycle the executor sat
  // busy in `divide` because nothing upstream defaulted back to zero in
  // between. Two DIFFERENT consecutive real stores legitimately produce two
  // adjacent high cycles, so this checks for the same request repeating
  // (identical address, data, and strobe on back-to-back high cycles), not
  // merely back-to-back nonzero cycles.
  //
  // $fatal is Icarus-only below: yosys's read_verilog (the write_cxxrtl leg
  // this file also feeds, per the Makefile) doesn't implement it at all, so
  // this stays inside `ifdef ICARUS` the same way the clock/reset generation
  // above does. The $display fires unconditionally, so the violation is
  // still visible under cxxrtl if it were ever encountered there.
  logic [31:0] prev_wstrb_mem_addr, prev_wstrb_mem_wdata;
  logic [3:0]  prev_mem_wstrb;
  initial prev_mem_wstrb = 4'b0000;
  always @(posedge clk) begin
    if (mem_wstrb != 4'b0000 && prev_mem_wstrb != 4'b0000 &&
        mem_addr == prev_wstrb_mem_addr && mem_wdata == prev_wstrb_mem_wdata &&
        mem_wstrb == prev_mem_wstrb) begin
      $display("ASSERTION FAILED: mem_wstrb held high for >1 cycle on the same store (addr=0x%08x)",
                mem_addr);
     `ifdef ICARUS
      $fatal(1);
     `endif
    end
    prev_wstrb_mem_addr <= mem_addr;
    prev_wstrb_mem_wdata <= mem_wdata;
    prev_mem_wstrb <= mem_wstrb;
  end
endmodule
