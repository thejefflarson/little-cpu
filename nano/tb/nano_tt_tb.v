`timescale 1ns/1ps
// Drives the tt_um_thejefflarson_nanocpu top by its own pins -- QSPI on uio, GPIO on
// ui_in/uo_out[7:1], UART tx on uo_out[0] -- never the core's internal bus, so a wiring
// mistake in nano/bus.v, nano/uart.v or nano/gpio.v is visible here even when the
// core-only suite cannot see it. Single clock throughout: cxxrtl cannot fire an
// `always @(posedge <a derived clock>)`, so every wait below counts real `clk` edges.
module nano_tt_tb;
  localparam int CLOCK_HZ = 64_000_000;
  localparam int BAUD     = 115_200;
  localparam int DIVISOR  = (CLOCK_HZ + BAUD / 2) / BAUD;
  localparam int unsigned TOHOST_INDEX = 0;
  localparam [7:0] EXPECT_UART_BYTE = 8'ha5;
  localparam [6:0] GPIO_PATTERN     = 7'h5a;

  logic clk = 0;
  always #5 clk = ~clk;
  logic rst_n = 0;

  logic [7:0] ui_in;
  logic [7:0] uo_out;
  logic [7:0] uio_out, uio_oe;
  logic [3:0] sio_bus;
  wire  [7:0] uio_in = {sio_bus, 4'b0};

  assign ui_in = {1'b0, GPIO_PATTERN};

  tt_um_thejefflarson_nanocpu dut (
    .ui_in(ui_in),
    .uo_out(uo_out),
    .uio_in(uio_in),
    .uio_out(uio_out),
    .uio_oe(uio_oe),
    .ena(1'b1),
    .clk(clk),
    .rst_n(rst_n)
  );

  logic [3:0] flash_out, psram_out;
  logic       flash_oe, psram_oe;
  assign sio_bus = uio_oe[4] ? uio_out[7:4] : (flash_oe ? flash_out : (psram_oe ? psram_out : 4'bz));

  nano_qspi_flash_model #(.WORDS(4096), .DUMMY_SCK(4)) flash (
    .clk(clk), .reset(!rst_n), .sck(uio_out[0]), .cs_n(uio_out[1]),
    .sio_in(uio_out[7:4]), .sio_out(flash_out), .sio_oe(flash_oe)
  );

  nano_qspi_psram_model #(.WORDS(4096), .DUMMY_SCK(4)) psram (
    .clk(clk), .reset(!rst_n), .sck(uio_out[0]), .cs_n(uio_out[2]),
    .sio_in(uio_out[7:4]), .sio_out(psram_out), .sio_oe(psram_oe)
  );

  task automatic uart_rx(output [7:0] byte_out, output logic timed_out);
    integer i, guard;
    byte_out = 8'b0;
    timed_out = 1'b0;
    guard = 0;
    while (uo_out[0] !== 1'b0) begin
      @(posedge clk);
      guard = guard + 1;
      if (guard > 100_000) begin
        timed_out = 1'b1;
        return;
      end
    end
    repeat (DIVISOR / 2) @(posedge clk);
    for (i = 0; i < 8; i = i + 1) begin
      repeat (DIVISOR) @(posedge clk);
      byte_out[i] = uo_out[0];
    end
  endtask

  logic [7:0] rx_byte;
  logic       rx_timeout;
  initial uart_rx(rx_byte, rx_timeout);

  string icarus_rom_path, icarus_ram_path;
  initial begin
    if ($value$plusargs("ROM=%s", icarus_rom_path)) $readmemh(icarus_rom_path, flash.mem);
    if ($value$plusargs("RAM=%s", icarus_ram_path)) $readmemh(icarus_ram_path, psram.mem);
  end

  // iverilog is the only leg that carries an X, so it is the only leg that can catch an
  // undriven or mid-transition uio_oe -- two-state tools drive a free input arbitrarily
  // and stay green either way.
  always @(posedge clk) begin
    if (rst_n && $isunknown(uio_oe)) begin
      $display("FAIL: uio_oe is X: %b", uio_oe);
      $finish;
    end
  end

  int cycles;
  initial begin
    cycles = 0;
    repeat (4) @(posedge clk);
    rst_n = 1;
    forever begin
      @(posedge clk);
      cycles = cycles + 1;
      if (psram.mem[TOHOST_INDEX] != 32'b0) begin
        if (psram.mem[TOHOST_INDEX] != 32'b1) begin
          $display("FAIL: tohost verdict %0d", psram.mem[TOHOST_INDEX] >> 1);
          $finish;
        end
        break;
      end
      if (cycles > 200_000) begin
        $display("TIMEOUT waiting for tohost");
        $display("FAIL");
        $finish;
      end
    end

    if (rx_timeout) begin
      $display("FAIL: UART start bit never arrived");
      $finish;
    end
    if (rx_byte !== EXPECT_UART_BYTE) begin
      $display("FAIL: UART tx byte %h, expected %h", rx_byte, EXPECT_UART_BYTE);
      $finish;
    end
    if (uo_out[7:1] !== GPIO_PATTERN) begin
      $display("FAIL: GPIO out %h, expected %h", uo_out[7:1], GPIO_PATTERN);
      $finish;
    end

    $display("PASS");
    $finish;
  end
endmodule
