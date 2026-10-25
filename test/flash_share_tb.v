`timescale 1 ns / 1 ps
`default_nettype none

// soc/flash_share.v between rtl/spiflash.v and test/spiflash_model.v, with the board's pads
// modelled as pulled-up wires: pin 14 carries the UART's line and the flash's data input, 15 its
// clock, 16 its select, and pin 17 only its data output. A host (iceprog on the FT232H) can drive
// 14, 15 and 16; any interval in which it and the FPGA drive the same pad counts as contention.
module flash_share_tb;
  localparam real PERIOD = 10.0;
  logic clk = 0;
  always #(PERIOD / 2) clk = ~clk;

  logic reset;
  logic [31:0] mem_addr, mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  logic        ctl_sck, ctl_mosi, ctl_cs_n;

  localparam logic [31:0] BASE = 32'h0002_0028;
  localparam logic [31:0] SPI_DATA    = BASE + 32'd0;
  localparam logic [31:0] SPI_CONTROL = BASE + 32'd4;
  localparam logic [31:0] SPI_BUSY    = 32'h0000_0100;

  tri1 txd_pad, sck_pad, ssn_pad, miso_pad;
  logic txd_oe, txd_d, sck_oe, ssn_oe, ssn_d;
  logic uart_tx = 1'b1;
  logic host_cs_oe = 1'b0, host_sck_oe = 1'b0, host_do_oe = 1'b0;
  logic host_cs_d = 1'b1, host_do_d = 1'b0;

  assign txd_pad = txd_oe      ? txd_d     : 1'bz;
  assign txd_pad = host_do_oe  ? host_do_d : 1'bz;
  assign sck_pad = sck_oe      ? ctl_sck   : 1'bz;
  assign sck_pad = host_sck_oe ? 1'b0      : 1'bz;
  assign ssn_pad = ssn_oe      ? ssn_d     : 1'bz;
  assign ssn_pad = host_cs_oe  ? host_cs_d : 1'bz;

  logic model_miso, flash_drives;
  assign #5 flash_drives = (ssn_pad === 1'b0);
  assign miso_pad = flash_drives ? model_miso : 1'bz;

  spiflash #(.BASE(BASE)) ctl (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .sck(ctl_sck),
    .mosi(ctl_mosi),
    .miso(miso_pad),
    .cs_n(ctl_cs_n)
  );

  flash_share dut (
    .clk(clk),
    .cs_n(ctl_cs_n),
    .sck(ctl_sck),
    .mosi(ctl_mosi),
    .uart_tx(uart_tx),
    .ssn_in(ssn_pad),
    .txd_oe(txd_oe),
    .txd_d(txd_d),
    .sck_oe(sck_oe),
    .ssn_oe(ssn_oe),
    .ssn_d(ssn_d)
  );

  spiflash_model flash (
    .clk(clk),
    .sck(sck_pad),
    .cs_n(ssn_pad),
    .mosi(txd_pad),
    .miso(model_miso)
  );

  int errors = 0;
  int contention = 0;

  task automatic check_bit(input string what, input logic got, input logic expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%b expected=%b", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_int(input string what, input int got, input int expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%0d expected=%0d", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_hex(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%02x expected=%02x", what, got, expected);
        errors++;
      end
    end
  endtask

  // The FPGA and the host on one pad at once, for any time at all.
  always @(txd_oe or host_do_oe)
    if (txd_oe === 1'b1 && host_do_oe === 1'b1) contention++;
  always @(sck_oe or host_sck_oe)
    if (sck_oe === 1'b1 && host_sck_oe === 1'b1) contention++;
  always @(ssn_oe or host_cs_oe)
    if (ssn_oe === 1'b1 && host_cs_oe === 1'b1) contention++;

  // While the chip select is held low by the FPGA, pin 14 carries the controller's data and not the UART's.
  int uart_leaks = 0;
  always @(posedge clk) begin
    #2;
    if (ssn_oe === 1'b1 && ssn_d === 1'b0 && !(txd_oe === 1'b1 && txd_d === ctl_mosi)) uart_leaks++;
  end

  time t_cs_high_drive, t_txd_on;
  always @(posedge ssn_d) if (ssn_oe) t_cs_high_drive = $time;
  always @(posedge txd_oe) t_txd_on = $time;

  task automatic step();
    begin
      @(posedge clk);
      #1;
    end
  endtask

  task automatic idle();
    begin
      mem_addr  = 32'h0;
      mem_wstrb = 4'b0000;
      mem_wdata = 32'h0;
      step();
    end
  endtask

  task automatic store(input logic [31:0] a, input logic [31:0] d);
    begin
      mem_addr  = a;
      mem_wdata = d;
      mem_wstrb = 4'b0001;
      step();
      mem_wstrb = 4'b0000;
    end
  endtask

  task automatic load(input logic [31:0] a);
    begin
      mem_addr  = a;
      mem_wstrb = 4'b0000;
      step();
    end
  endtask

  task automatic wait_idle();
    begin
      load(SPI_DATA);
      while (|(mem_rdata & SPI_BUSY)) load(SPI_DATA);
    end
  endtask

  task automatic xfer(input logic [7:0] send, output logic [7:0] got);
    begin
      store(SPI_DATA, {24'b0, send});
      wait_idle();
      got = mem_rdata[7:0];
    end
  endtask

  logic [7:0] id0, id1, id2, ignored;

  task automatic read_id();
    begin
      store(SPI_CONTROL, 32'd1);
      xfer(8'h9f, ignored);
      xfer(8'h00, id0);
      xfer(8'h00, id1);
      xfer(8'h00, id2);
      store(SPI_CONTROL, 32'd0);
      idle();
    end
  endtask

  int ssn_oe_rises, ssn_high_drive_cycles;
  always @(posedge clk) begin
    if (ssn_oe && ssn_d) ssn_high_drive_cycles++;
  end
  always @(posedge ssn_oe) ssn_oe_rises++;

  initial begin
    reset = 1'b1;
    idle();
    idle();
    reset = 1'b0;
    repeat (6) idle();

    check_bit("idle: the UART owns pin 14", txd_oe, 1'b1);
    check_bit("idle: the UART's level reaches the pad", txd_d, uart_tx);
    check_bit("idle: nothing drives the chip select", ssn_oe, 1'b0);
    check_bit("idle: nothing drives SCK", sck_oe, 1'b0);

    // The request, with the UART mid-character driving a low.
    uart_tx = 1'b0;
    store(SPI_CONTROL, 32'd1);
    idle();
    check_bit("a request: the chip select is driven low", ssn_pad, 1'b0);
    check_bit("a request: SCK is driven", sck_oe, 1'b1);
    check_bit("a request: pin 14 is still driven, now by the controller", txd_oe, 1'b1);

    xfer(8'h9f, ignored);
    xfer(8'h00, id0);
    xfer(8'h00, id1);
    xfer(8'h00, id2);
    check_hex("JEDEC manufacturer through the shared pins, UART driving low at the request", id0, 32'hEF);
    check_hex("JEDEC type", id1, 32'h70);
    check_hex("JEDEC capacity", id2, 32'h16);
    check_int("the UART never reaches pin 14 while the chip select is ours", uart_leaks, 0);

    ssn_high_drive_cycles = 0;
    store(SPI_CONTROL, 32'd0);
    repeat (6) idle();
    check_int("deselect: the chip select is driven high for exactly one cycle", ssn_high_drive_cycles, 1);
    check_bit("deselect: the chip select reads high afterwards", ssn_pad, 1'b1);
    check_bit("deselect: the pad is released", ssn_oe, 1'b0);
    check_bit("deselect: SCK is released", sck_oe, 1'b0);
    if (t_txd_on - t_cs_high_drive < 2 * PERIOD - 1.0)
      check_int("the UART takes pin 14 back at least two cycles after the chip select goes high (ns)",
                int'(t_txd_on - t_cs_high_drive), int'(2 * PERIOD));
    check_bit("the UART drives pin 14 again", txd_oe, 1'b1);
    check_bit("...with its own level", txd_d, uart_tx);
    uart_tx = 1'b1;

    read_id();
    check_hex("second read, JEDEC manufacturer", id0, 32'hEF);
    check_hex("second read, JEDEC capacity", id2, 32'h16);
    repeat (6) idle();

    // The gap compiled firmware leaves between bytes (about 22 cycles) must not end the transaction.
    ssn_oe_rises = 0;
    store(SPI_CONTROL, 32'd1);
    xfer(8'h9f, ignored);
    repeat (30) idle();
    xfer(8'h00, id0);
    repeat (30) idle();
    xfer(8'h00, id1);
    repeat (30) idle();
    xfer(8'h00, id2);
    store(SPI_CONTROL, 32'd0);
    repeat (6) idle();
    check_hex("30 idle cycles between bytes: JEDEC manufacturer", id0, 32'hEF);
    check_hex("30 idle cycles between bytes: JEDEC type", id1, 32'h70);
    check_hex("30 idle cycles between bytes: JEDEC capacity", id2, 32'h16);
    check_int("30 idle cycles between bytes: the grant is never dropped", ssn_oe_rises, 1);

    // A host takes the bus: the UART lets go of pin 14 before the host's data output is live.
    uart_tx = 1'b0;
    host_cs_oe = 1'b1; host_cs_d = 1'b0;
    #2;
    host_do_oe = 1'b1; host_sck_oe = 1'b1;
    repeat (4) idle();
    check_bit("a host select: the UART is off pin 14", txd_oe, 1'b0);
    ssn_oe_rises = 0;
    store(SPI_CONTROL, 32'd1);
    repeat (24) begin
      idle();
      check_bit("a host holds the select: the controller never drives the chip select", ssn_oe, 1'b0);
      check_bit("a host holds the select: the controller never drives SCK", sck_oe, 1'b0);
      check_bit("a host holds the select: nothing of ours drives pin 14", txd_oe, 1'b0);
    end
    store(SPI_CONTROL, 32'd0);
    host_do_oe = 1'b0; host_sck_oe = 1'b0;
    #2;
    host_cs_oe = 1'b0;
    repeat (6) idle();
    check_int("the host never saw the controller's select", ssn_oe_rises, 0);
    check_bit("the host let go: the UART returns", txd_oe, 1'b1);
    check_int("no pad is ever driven by the host and the FPGA together", contention, 0);
    uart_tx = 1'b1;

    // A request that never shifts: the grant is bounded.
    ssn_oe_rises = 0;
    store(SPI_CONTROL, 32'd1);
    repeat (6) idle();
    check_bit("a held request is granted", ssn_oe, 1'b1);
    repeat (160) idle();
    if (ssn_oe_rises < 2) check_int("an idle grant is dropped and regranted inside 160 cycles", ssn_oe_rises, 2);
    store(SPI_CONTROL, 32'd0);
    repeat (6) idle();
    check_bit("after the idle request the UART returns", txd_oe, 1'b1);
    check_int("no pad is ever driven by the host and the FPGA together, end of run", contention, 0);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: flash_share (controller owns pin 14 while it holds the select, one cycle of driven-high deselect, UART back after two, host select withheld, idle grant bounded)");
      $finish;
    end
  end
endmodule
