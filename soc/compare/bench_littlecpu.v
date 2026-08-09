`timescale 1 ns / 1 ps
`default_nettype none
// This core, in the cross-core comparison harness. Its twin is
// soc/compare/bench_vexriscv.v, and the two are held to one geometry: the same
// ROM depth, the same data RAM module at the same base, the same three pads,
// the same program image, the same part and the same seeds.
//
// It is NOT rtl/littlesoc.v and its number is not `make soc-timing`'s. Four
// things differ, all of them so that both cores fit one part:
//   - 4 KB of ROM instead of 8, and 2 KB of data RAM instead of 64;
//   - the data RAM is block RAM, because the comparison runs on an hx8k, which
//     has no SPRAM at all;
//   - no rtl/timer.v, because with it this design is 7829 logic cells against
//     the part's 7680 and does not place. `irq_timer` is driven from a counter
//     instead of tied off, so the core's interrupt path is still real logic and
//     the 324 cells left out are the peripheral's, not the core's;
//   - no reset button, so the pinout is three pads rather than four.
// Nothing about the core changes. rtl/ is untouched by this whole directory.
module bench_littlecpu #(
  parameter integer ROM_WORDS = 1024,
  parameter integer RAM_WORDS = 512,
  parameter INIT_EVEN = "soc/compare/rom_even.hex",
  parameter INIT_ODD  = "soc/compare/rom_odd.hex"
) (
  input  logic clk,
  output logic led0_n,
  output logic led1_n
);
  // Power-on reset only. rtl/littlesoc.v also debounces a button; there is no
  // button here because VexRiscv's harness would then need one too, and its
  // core has no equivalent input to hang it off.
  logic [3:0] por_count = 4'b0;
  logic       por_done  = 1'b0;
  logic       reset     = 1'b1;
  always_ff @(posedge clk) begin
    if (!por_done) begin
      por_count <= por_count + 4'd1;
      if (por_count == 4'hf) por_done <= 1'b1;
    end
    reset <= !por_done;
  end

  logic        trap;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [31:0] imem_mem_rdata, dmem_mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren, fetch_stall, irq_timer;

  // Stands in for rtl/timer.v's `mtip` line, which there is no room on the part
  // for. Tying it to zero instead would let yosys constant-fold `mip.MTIP`, the
  // interrupt-taking condition and its arm of the trap cause -- real core logic,
  // deleted from a measurement of the core. The program never sets `mie.MTIE`,
  // so nothing is ever taken.
  logic [15:0] irq_count = 16'b0;
  always_ff @(posedge clk) irq_count <= irq_count + 16'd1;
  assign irq_timer = irq_count[15];
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic        imem_ren;
  logic [31:0] imem_data, imem_data2;

  littlecpu riscv (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .imem_ren(imem_ren),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    .irq_timer(irq_timer),
    .trap(trap)
  );

  imemory #(
    .ROM_WORDS(ROM_WORDS),
    .INIT_EVEN(INIT_EVEN),
    .INIT_ODD(INIT_ODD)
  ) imem (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_ren(imem_ren),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(imem_mem_rdata),
    .fetch_stall(fetch_stall)
  );

  // The same module soc/compare/bench_vexriscv.v instantiates, at the same base
  // and the same depth, so the data RAM is not a variable between the two.
  memory #(.RAM_WORDS(RAM_WORDS)) dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dmem_mem_rdata)
  );

  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata;

  // Both harnesses publish the same two bits of the data bus, so neither core
  // is given an output the other does not have. `trap` is the one asymmetry and
  // it is this core's: VexRiscv's configuration here implements no traps, so
  // there is nothing on that side to fold it into.
  logic store_bit, load_bit;
  always_ff @(posedge clk) begin
    if (reset) begin
      store_bit <= 1'b0;
      load_bit  <= 1'b0;
    end else begin
      if (|mem_wstrb) store_bit <= mem_wdata[0];
      if (mem_ren)    load_bit  <= mem_rdata[0];
      if (trap)       load_bit  <= 1'b1;
    end
  end
  assign led0_n = !store_bit;
  assign led1_n = !load_bit;
endmodule
