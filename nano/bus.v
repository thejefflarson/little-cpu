`default_nettype none
// The core's own region check refuses anything outside this window before it arrives, so
// what is left is which sub-region answers. The reserved span reads zero and drops a
// write, the way an unimplemented CSR does.
module nano_bus #(
  parameter logic [31:0] PSRAM_BASE  = 32'h1000_0000,
  parameter logic [31:0] PSRAM_BYTES = 32'h0080_0000,
  parameter integer      CLOCK_HZ    = 64_000_000,
  parameter integer      BAUD        = 115_200
) (
  input  logic        clk,
  input  logic        reset,

  input  logic        mem_valid,
  input  logic        mem_instr,
  output logic        mem_ready,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,

  output logic       sck,
  output logic       flash_cs_n,
  output logic       psram_cs_n,
  output logic       spare_cs_n,
  output logic [3:0] sio_out,
  output logic       sio_oe,
  input  logic [3:0] sio_in,

  output logic       uart_tx,
  output logic [6:0] gpio_out,
  input  logic [7:0] gpio_in
);
  localparam logic [31:0] UART_BASE = PSRAM_BASE + PSRAM_BYTES;
  localparam logic [31:0] GPIO_BASE = UART_BASE + 32'd8;

  if ($clog2(PSRAM_BYTES) < 1 || (32'd1 << $clog2(PSRAM_BYTES)) != PSRAM_BYTES) begin : l_psram_bytes_pow2
    $fatal(1, "nano_bus: PSRAM_BYTES must be a power of two");
  end
  if ((PSRAM_BASE & (PSRAM_BYTES - 1)) != 0) begin : l_psram_base_aligned
    $fatal(1, "nano_bus: PSRAM_BASE must be aligned to PSRAM_BYTES");
  end

  logic psram_sel;
  assign psram_sel = !mem_instr &&
    mem_addr[31:$clog2(PSRAM_BYTES)] == PSRAM_BASE[31:$clog2(PSRAM_BYTES)];

  logic        ctrl_mem_valid, ctrl_mem_ready;
  logic [31:0] ctrl_mem_addr, ctrl_mem_rdata;
  assign ctrl_mem_valid = mem_instr ? mem_valid : (mem_valid && psram_sel);
  assign ctrl_mem_addr  = mem_instr ? mem_addr : (mem_addr - PSRAM_BASE);

  nano_qspi_ctrl ctrl (
    .clk(clk),
    .reset(reset),
    .mem_valid(ctrl_mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(ctrl_mem_ready),
    .mem_addr(ctrl_mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(ctrl_mem_rdata),
    .sck(sck),
    .flash_cs_n(flash_cs_n),
    .psram_cs_n(psram_cs_n),
    .spare_cs_n(spare_cs_n),
    .sio_out(sio_out),
    .sio_oe(sio_oe),
    .sio_in(sio_in)
  );

  logic [31:0] uart_rdata;
  nano_uart #(.BASE(UART_BASE), .CLOCK_HZ(CLOCK_HZ), .BAUD(BAUD)) uart (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(uart_rdata),
    .tx(uart_tx)
  );

  logic [31:0] gpio_rdata;
  nano_gpio #(.BASE(GPIO_BASE)) gpio (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(gpio_rdata),
    .pin_in(gpio_in),
    .pin_out(gpio_out)
  );

  logic uart_sel, gpio_sel;
  assign uart_sel = !mem_instr && mem_addr[31:3] == UART_BASE[31:3];
  assign gpio_sel = !mem_instr && mem_addr[31:3] == GPIO_BASE[31:3];

  assign mem_ready = (mem_instr || psram_sel) ? ctrl_mem_ready : mem_valid;
  assign mem_rdata = mem_instr ? ctrl_mem_rdata :
                      psram_sel ? ctrl_mem_rdata :
                      uart_sel  ? uart_rdata :
                      gpio_sel  ? gpio_rdata : 32'b0;
endmodule

`default_nettype wire
