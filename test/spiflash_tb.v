`timescale 1 ns / 1 ps
`default_nettype none

// rtl/spiflash.v's bus port driven directly, against test/spiflash_model.v on the other
// end of four wires.
module spiflash_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  logic [31:0] mem_addr, mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  logic        sck, mosi, miso, cs_n;

  localparam logic [31:0] BASE = 32'h0002_0028;
  localparam logic [31:0] SPI_DATA    = BASE + 32'd0;
  localparam logic [31:0] SPI_CONTROL = BASE + 32'd4;
  localparam logic [31:0] SPI_BUSY    = 32'h0000_0100;

  // The line the device reads.
  logic model_miso;
  logic force_line = 1'b0;
  logic forced_value = 1'b1;
  assign miso = force_line ? forced_value : model_miso;

  spiflash #(.BASE(BASE)) dut (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .sck(sck),
    .mosi(mosi),
    .miso(miso),
    .cs_n(cs_n)
  );

  spiflash_model flash (
    .clk(clk),
    .sck(sck),
    .cs_n(cs_n),
    .mosi(mosi),
    .miso(model_miso)
  );

  // The model's data array, restated.
  function automatic logic [7:0] flash_byte(input logic [23:0] a);
    flash_byte = a[7:0] ^ a[15:8] ^ 8'h5a;
  endfunction

  int errors = 0;
  int reds_forced = 0;

  task automatic check_hex(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

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

  // The other direction: a comparison that MUST differ.
  task automatic force_red(input string what, input logic [31:0] got, input logic [31:0] wrong);
    begin
      if (got === wrong) begin
        $display("MISMATCH %s: the forced red direction agreed anyway (%08x)", what, got);
        errors++;
      end else begin
        reds_forced++;
      end
    end
  endtask

  // The bus. Idle unless a task is driving it.

  task automatic idle();
    begin
      mem_addr  = 32'h0;
      mem_wstrb = 4'b0000;
      mem_wdata = 32'h0;
      @(posedge clk);
      #1;
    end
  endtask

  task automatic store(input logic [31:0] a, input logic [31:0] d, input logic [3:0] strb);
    begin
      mem_addr  = a;
      mem_wdata = d;
      mem_wstrb = strb;
      @(posedge clk);
      #1;
      mem_wstrb = 4'b0000;
    end
  endtask

  // The read port is registered, so the answer belongs to the address presented across
  // the previous edge -- the same one-cycle turnaround rtl/accessor.v gives every load.
  task automatic load(input logic [31:0] a);
    begin
      mem_addr  = a;
      mem_wstrb = 4'b0000;
      @(posedge clk);
      #1;
    end
  endtask

  // Counting the wire, because a byte that came back right off eight clocks and a byte
  // that came back right off nine are the same byte.

  logic sck_prev = 1'b0;
  int   sck_rises = 0;
  always @(posedge clk) begin
    if (sck && !sck_prev) sck_rises++;
    sck_prev <= sck;
  end

  task automatic select_flash(input logic on);
    begin
      store(SPI_CONTROL, {31'b0, on}, 4'b0001);
      idle();
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
      store(SPI_DATA, {24'b0, send}, 4'b0001);
      wait_idle();
      got = mem_rdata[7:0];
    end
  endtask

  // A whole id read, down to the first byte of the answer.
  task automatic read_id_first(output logic [7:0] got);
    logic [7:0] cmd_ignored;
    begin
      select_flash(1'b1);
      xfer(8'h9f, cmd_ignored);
      xfer(8'h00, got);
      select_flash(1'b0);
    end
  endtask

  task automatic read_at(input logic [23:0] addr, input int count, output logic [7:0] out[]);
    logic [7:0] got;
    begin
      out = new[count];
      select_flash(1'b1);
      xfer(8'h03, got);
      xfer(addr[23:16], got);
      xfer(addr[15:8],  got);
      xfer(addr[7:0],   got);
      for (int i = 0; i < count; i++) begin
        xfer(8'h00, got);
        out[i] = got;
      end
      select_flash(1'b0);
    end
  endtask

  logic [7:0] id0, id1, id2;
  logic [7:0] data[];
  int         rises_before;

  initial begin
    reset = 1'b1;
    idle();
    idle();
    reset = 1'b0;
    idle();

    check_bit("cs_n is released out of reset", cs_n, 1'b1);
    check_bit("sck idles low",                 sck,  1'b0);
    load(SPI_DATA);
    check_hex("nothing busy and nothing received out of reset", mem_rdata, 32'h0);
    load(SPI_CONTROL);
    check_hex("the control register is write-only", mem_rdata, 32'h0);

    load(BASE + 32'd8);
    check_hex("an address above the window reads zero", mem_rdata, 32'h0);
    load(BASE - 32'd8);
    check_hex("an address below the window reads zero", mem_rdata, 32'h0);

    store(SPI_DATA, 32'hffff_ff00, 4'b1110);
    idle();
    load(SPI_DATA);
    check_hex("a write outside byte lane 0 starts nothing", mem_rdata, 32'h0);

    // The chip select, which is also soc/board_upduino.v's output enable.
    select_flash(1'b1);
    check_bit("writing 1 to the control register selects the flash", cs_n, 1'b0);
    select_flash(1'b0);
    check_bit("writing 0 releases it",                               cs_n, 1'b1);

    select_flash(1'b1);
    store(SPI_DATA, 32'h0000_009f, 4'b0001);
    load(SPI_DATA);
    check_hex("an exchange takes the device busy", mem_rdata & SPI_BUSY, SPI_BUSY);
    wait_idle();
    rises_before = sck_rises;
    xfer(8'h00, id0);
    check_int("one exchange is exactly eight clocks", sck_rises - rises_before, 8);
    xfer(8'h00, id1);
    xfer(8'h00, id2);
    select_flash(1'b0);
    check_hex("JEDEC manufacturer", {24'b0, id0}, 32'h0000_00ef);
    check_hex("JEDEC type",         {24'b0, id1}, 32'h0000_0070);
    check_hex("JEDEC capacity",     {24'b0, id2}, 32'h0000_0016);

    read_id_first(id0);
    check_hex("the id again after a chip select", {24'b0, id0}, 32'h0000_00ef);

    read_at(24'h01_00fe, 4, data);
    for (int i = 0; i < 4; i++)
      check_hex($sformatf("sequential byte %0d", i),
                {24'b0, data[i]}, {24'b0, flash_byte(24'h01_00fe + 24'(i))});

    read_at(24'h00_1234, 2, data);
    check_hex("a second offset, byte 0", {24'b0, data[0]}, {24'b0, flash_byte(24'h00_1234)});
    check_hex("a second offset, byte 1", {24'b0, data[1]}, {24'b0, flash_byte(24'h00_1235)});

    select_flash(1'b1);
    rises_before = sck_rises;
    store(SPI_DATA, 32'h0000_009f, 4'b0001);
    store(SPI_DATA, 32'h0000_00ff, 4'b0001);   // arrives while busy
    wait_idle();
    check_int("a write while busy is dropped, not queued", sck_rises - rises_before, 8);
    xfer(8'h00, id0);
    check_hex("...and the exchange that ran was the first one",
              {24'b0, id0}, 32'h0000_00ef);
    select_flash(1'b0);

    select_flash(1'b1);
    store(SPI_DATA, 32'h0000_009f, 4'b0001);
    store(SPI_CONTROL, 32'h0000_0000, 4'b0001);
    check_bit("a control write while busy is dropped", cs_n, 1'b0);
    wait_idle();
    select_flash(1'b0);

    force_red("an expectation off by one still compares",
              {24'b0, id0}, 32'h0000_00f0);

    force_line = 1'b1; forced_value = 1'b1;
    read_id_first(id0);
    force_red("a line held high cannot produce the id", {24'b0, id0}, 32'h0000_00ef);
    check_hex("...it produces all ones",                {24'b0, id0}, 32'h0000_00ff);

    forced_value = 1'b0;
    read_id_first(id0);
    force_red("a line held low cannot produce the id",  {24'b0, id0}, 32'h0000_00ef);
    check_hex("...it produces all zeroes",              {24'b0, id0}, 32'h0000_0000);
    force_line = 1'b0;

    read_at(24'h00_1234, 1, data);
    force_red("a sequential byte compared against the wrong address",
              {24'b0, data[0]}, {24'b0, flash_byte(24'h00_1235)});

    if (reds_forced != 4) begin
      $display("MISMATCH the forced failures did not all run: %0d of 4", reds_forced);
      errors++;
    end

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: spiflash (map, chip select, eight clocks a byte, mode 0, 0x9f and 0x03)");
      $finish;
    end
  end
endmodule
