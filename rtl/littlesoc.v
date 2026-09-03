`timescale 1 ns / 1 ps
`default_nettype none
// The ROM is initialised from the bitstream and the SPRAM cannot be, so a
// program's `.data` is copied out of ROM by its startup code.
module littlesoc (
  input  logic clk,
  input  logic btn_n,
  // Without an output something can see, yosys deletes the whole design.
  output logic ledr_n,
  output logic ledg_n,
  output logic uart_tx,
  // The configuration flash. Its pins are the programmer's too, so a board file
  // must not drive these three outputs unconditionally.
  output logic spi_sck,
  output logic spi_mosi,
  input  logic spi_miso,
  output logic spi_cs_n
);
  // The FPGA comes out of configuration with no reset of its own. Keep `reset`
  // registered: unregistered, the button pin headed the design's longest path.
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

  /* verilator lint_off UNUSED */
  logic        trap;
  /* verilator lint_on UNUSED */
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [31:0] imem_mem_rdata, dmem_mem_rdata, timer_mem_rdata, uart_mem_rdata;
  logic [31:0] flash_mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren, fetch_stall, irq_timer, imem_fault, mem_reservable;
  logic        atomic_supported, mem_lock, bus_request;
  logic [31:0] atomic_addr;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic [31:0] imem_data, imem_data2;

  // Must equal the `imemory` below's ROM_WORDS.
  littlecpu #(.LS_TEXT_WORDS(2048)) riscv (
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
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    // One bus initiator: nothing takes the bus away and nothing else writes.
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    .bus_request(bus_request),
    .irq_timer(irq_timer),
    .trap(trap)
  );

  // `imem_addr` and `imem_addr2` are unconnected on purpose: the ROM takes its
  // address a cycle early on `imem_addr_next`.
  imemory #(
    .ROM_WORDS(2048),
    .INIT_EVEN("soc/rom_even.hex"),
    .INIT_ODD("soc/rom_odd.hex")
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

  memory dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dmem_mem_rdata),
    .reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported)
  );

  timer mtimer (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(timer_mem_rdata),
    .mtip(irq_timer)
  );

  uart tty (
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

  // Every source but the one addressed answers zero, so the OR never mixes two.
  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata | timer_mem_rdata | uart_mem_rdata
                   | flash_mem_rdata;

  // The low two bits of the last store to any address, one per LED.
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
endmodule // littlesoc
