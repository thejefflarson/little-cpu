`default_nettype none
// The chip top: `riscv` plus the QSPI, UART and GPIO peripherals nano/bus.v routes to.
module tt_um_thejefflarson_nanocpu (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire        ena,
    input  wire        clk,
    input  wire        rst_n
);
  localparam int CLOCK_HZ = 64_000_000;

  localparam logic [31:0] PSRAM_BASE  = 32'h1000_0000;
  localparam logic [31:0] PSRAM_BYTES = 32'h0080_0000;
  localparam logic [31:0] MAP_TOP     = PSRAM_BASE + PSRAM_BYTES + 32'd8 + 32'd8 + 32'd16;
  localparam int          RAM_WORDS   = (MAP_TOP - PSRAM_BASE) / 4;

  logic reset;
  assign reset = !rst_n;

  logic        mem_valid, mem_instr, mem_ready, trap;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [3:0]  mem_wstrb;

  riscv #(.RAM_BASE(PSRAM_BASE), .RAM_WORDS(RAM_WORDS)) core (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .irq_meip(ui_in[7]),
    .trap(trap)
  );

  logic       sck, flash_cs_n, psram_cs_n, spare_cs_n, sio_oe;
  logic [3:0] sio_out;
  logic       uart_tx;
  logic [6:0] gpio_out;

  nano_bus #(.PSRAM_BASE(PSRAM_BASE), .PSRAM_BYTES(PSRAM_BYTES), .CLOCK_HZ(CLOCK_HZ)) bus (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .sck(sck),
    .flash_cs_n(flash_cs_n),
    .psram_cs_n(psram_cs_n),
    .spare_cs_n(spare_cs_n),
    .sio_out(sio_out),
    .sio_oe(sio_oe),
    .sio_in(uio_in[7:4]),
    .uart_tx(uart_tx),
    .gpio_out(gpio_out),
    .gpio_in(ui_in)
  );

  assign uio_out = {sio_out, spare_cs_n, psram_cs_n, flash_cs_n, sck};
  // sio turns around with the controller's own state; sck and the three selects are always output.
  assign uio_oe  = {{4{sio_oe}}, 4'b1111};

  assign uo_out = {gpio_out, uart_tx};

  logic _unused;
  assign _unused = &{ena, trap, 1'b0};
endmodule

`default_nettype wire
