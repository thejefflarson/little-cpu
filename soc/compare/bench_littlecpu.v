`timescale 1 ns / 1 ps
`default_nettype none
// This core, in the cross-core comparison harness.
module bench_littlecpu #(
  parameter integer ROM_WORDS = 1024,
  parameter integer RAM_WORDS = 16384,
  parameter INIT_EVEN = "soc/compare/rom_even.hex",
  parameter INIT_ODD  = "soc/compare/rom_odd.hex"
) (
  input  logic clk,
  output logic led0_n,
  output logic led1_n
);
  // Power-on reset only. rtl/littlesoc.v also debounces a button; there is no button here
  // because VexRiscv's harness would then need one too, and its core has no equivalent
  // input to hang it off.
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
  logic        mem_ren, fetch_stall, irq_timer, imem_fault, mem_reservable;
  logic        atomic_supported, mem_lock, bus_request;
  logic [31:0] atomic_addr;

  // Stands in for rtl/timer.v's `mtip` line, which there is no room on the part for.
  logic [15:0] irq_count = 16'b0;
  always_ff @(posedge clk) irq_count <= irq_count + 16'd1;
  assign irq_timer = irq_count[15];
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic [31:0] imem_data, imem_data2;

  littlecpu riscv (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    .mem_reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    .bus_request(bus_request),
    .irq_timer(irq_timer),
    .imem_fault(imem_fault),
    .trap(trap)
  );

  imemory #(
    .ROM_WORDS(ROM_WORDS),
    .INIT_EVEN(INIT_EVEN),
    .INIT_ODD(INIT_ODD)
  ) imem (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(imem_mem_rdata),
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault)
  );

  memory #(.RAM_WORDS(RAM_WORDS)) dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dmem_mem_rdata),
    .reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported)
  );

  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata;

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
