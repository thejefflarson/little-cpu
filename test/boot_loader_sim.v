`timescale 1 ns / 1 ps
`default_nettype none
// rtl/littlesoc.v running soc/boot_loader.c (its ROM read from soc/rom_*.hex in the working directory) against
// test/spiflash_model.v, whose image window holds the file named by +image=. The UART line is decoded here and
// echoed; the run ends TAIL_CLOCKS after the +lines= th line. test/boot_loader_test.sh grades what was echoed.
module boot_loader_sim;
  parameter integer FLASH_OFFSET = 0;
  parameter integer IMAGE_BYTES  = 0;
  parameter integer BIT_CLOCKS   = 104;
  parameter integer IDLE_LIMIT   = 128;
  parameter integer TAIL_CLOCKS  = 40_000;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  integer gap = 0, max_gap = 0;
  logic uart_tx, spi_sck, spi_mosi, spi_miso, spi_cs_n;

  littlesoc soc (
    .clk(clk),
    .btn_n(1'b1),
    .entropy_raw(1'b0),
    .ledr_n(),
    .ledg_n(),
    .uart_tx(uart_tx),
    .spi_sck(spi_sck),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_cs_n(spi_cs_n)
  );

  spiflash_model #(
    .IMAGE_BYTES(IMAGE_BYTES),
    .IMAGE_BASE(24'(FLASH_OFFSET))
  ) flash_part (
    .clk(clk),
    .sck(spi_sck),
    .cs_n(spi_cs_n),
    .mosi(spi_mosi),
    .miso(spi_miso)
  );

  string image_file;
  integer lines_wanted, lines_seen;
  initial begin
    if (!$value$plusargs("lines=%d", lines_wanted)) lines_wanted = 1;
    if ($value$plusargs("image=%s", image_file)) $readmemh(image_file, flash_part.image);
    lines_seen = 0;
  end

  logic [7:0] rx;
  always begin
    @(negedge uart_tx);
    repeat (BIT_CLOCKS + BIT_CLOCKS / 2) @(posedge clk);
    for (int i = 0; i < 8; i++) begin
      rx = {uart_tx, rx[7:1]};
      repeat (BIT_CLOCKS) @(posedge clk);
    end
    $write("%c", rx);
    if (rx == 8'h0a) begin
      lines_seen = lines_seen + 1;
    end
  end

  // Past the last wanted line the UART keeps being echoed for TAIL_CLOCKS, so a loader that carried on into
  // an image it should have refused is heard.
  initial begin
    wait (lines_seen == lines_wanted);
    repeat (TAIL_CLOCKS) @(posedge clk);
    $display("MAXGAP %0d", max_gap);
    $finish;
  end

  // The longest run of core cycles the clock sat low with the flash selected. Past IDLE_LIMIT the board's
  // pin lockout gives the pins back mid-read.
  always @(posedge clk) begin
    if (!spi_cs_n && !spi_sck) gap <= gap + 1;
    else gap <= 0;
    if (gap > max_gap) max_gap <= gap;
    if (gap >= IDLE_LIMIT) begin
      $display("IDLE GAP: the clock sat low %0d cycles with the flash selected", gap);
      $fatal(1);
    end
  end

  initial begin
    #(30_000_000);
    $display("TIMEOUT after %0d lines of %0d", lines_seen, lines_wanted);
    $fatal(1);
  end
endmodule
