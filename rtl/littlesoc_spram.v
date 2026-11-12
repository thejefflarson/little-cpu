`timescale 1 ns / 1 ps
`default_nettype none
// The SPRAM-text SoC: 64 KB of program text in the part's four SPRAMs, 8 KB of data RAM in block RAM,
// and a 1 KB block-RAM boot ROM at the top of the text window (soc/boot_loader.c) that the core resets into.
// Its memory map is its own: rtl/memory.v's RAM_WORDS is overridden here, which test/memmap_test.sh grades.
module littlesoc_spram #(
  // The board's clock, so rtl/uart.v can derive its divisor.
  parameter integer CLOCK_HZ = 12_000_000
) (
  input  logic clk,
  input  logic btn_n,
  // A slow free-running oscillator, asynchronous to `clk`; low reads as DEAD.
  input  logic entropy_raw,
  output logic ledr_n,
  output logic ledg_n,
  output logic uart_tx,
  // The configuration flash. Its pins are the programmer's, so a board file must not
  // drive these three unconditionally.
  output logic spi_sck,
  output logic spi_mosi,
  input  logic spi_miso,
  output logic spi_cs_n,
  // The SPI target's four wires, a peripheral port for an external controller.
  input  logic spit_sck,
  input  logic spit_cs_n,
  input  logic spit_mosi,
  output logic spit_miso,
  output logic spit_miso_oe
);
  localparam integer ROM_WORDS  = 16384;
  localparam integer BOOT_WORDS = 256;
  localparam integer RAM_WORDS  = 2048;

  // KEEP `reset` REGISTERED: unregistered, the button pin headed the design's longest path.
  logic [3:0] por_count = 4'b0;
  logic       por_done  = 1'b0;
  logic [1:0] btn_sync  = 2'b0;
  logic       reset     = 1'b1;
  always_ff @(posedge clk) begin
    if (!por_done) begin
      por_count <= por_count + 4'd1;
      if (por_count == 4'hf) por_done <= 1'b1;
    end
    btn_sync <= {btn_sync[0], btn_n};
    reset    <= !por_done || !btn_sync[1];
  end

  // verilator lint_off UNUSED
  logic        trap;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [31:0] imem_mem_rdata, dmem_mem_rdata, timer_mem_rdata, uart_mem_rdata;
  logic [31:0] flash_mem_rdata, spit_mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren, fetch_stall, irq_timer, irq_software, imem_fault, mem_reservable;
  logic        mem_lock, bus_request;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic [31:0] imem_data, imem_data2;

  littlecpu #(.RESET_PC(32'h0000_FC00), .LS_TEXT_WORDS(ROM_WORDS), .LS_RAM_WORDS(RAM_WORDS), .LS_HAS_SPI(1)) riscv (
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
    .imem_fault(imem_fault),
    .mem_reservable(mem_reservable),
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    .bus_request(bus_request),
    .irq_timer(irq_timer),
    .irq_software(irq_software),
    .entropy_raw(entropy_raw),
    .trap(trap)
  );

  imemory_spram #(
    .ROM_WORDS(ROM_WORDS),
    .BOOT_WORDS(BOOT_WORDS),
    .BOOT_INIT_EVEN("soc/bootrom_even.hex"),
    .BOOT_INIT_ODD("soc/bootrom_odd.hex")
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
    .reservable(mem_reservable)
  );

  timer mtimer (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(timer_mem_rdata),
    .mtip(irq_timer),
    .msip(irq_software)
  );

  uart #(.CLOCK_HZ(CLOCK_HZ)) tty (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(uart_mem_rdata),
    .tx(uart_tx)
  );

  spiflash flash (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(flash_mem_rdata),
    .sck(spi_sck),
    .mosi(spi_mosi),
    .miso(spi_miso),
    .cs_n(spi_cs_n)
  );

  spi_target target (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(spit_mem_rdata),
    .sck(spit_sck),
    .cs_n(spit_cs_n),
    .mosi(spit_mosi),
    .miso(spit_miso),
    .miso_oe(spit_miso_oe)
  );

  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata | timer_mem_rdata | uart_mem_rdata
                   | flash_mem_rdata | spit_mem_rdata;

  logic led_green, led_red;
  always_ff @(posedge clk) begin
    if (reset) begin
      led_green <= 1'b0;
      led_red   <= 1'b0;
    end else if (|mem_wstrb) begin
      led_green <= mem_wdata[0];
      led_red   <= mem_wdata[1];
    end
  end
  assign ledg_n = !led_green;
  assign ledr_n = !led_red;
endmodule // littlesoc_spram
