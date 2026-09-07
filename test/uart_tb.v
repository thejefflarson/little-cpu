`timescale 1 ns / 1 ps
`default_nettype none

// rtl/uart.v's bus port driven directly, and its serial line decoded bit by bit at the
// divisor the device was elaborated with.
module uart_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  logic [31:0] mem_addr, mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata;
  logic        tx;

  localparam logic [31:0] BASE     = 32'h0002_0020;
  localparam integer      CLOCK_HZ = 12_000_000;
  localparam integer      BAUD     = 115_200;

  localparam logic [31:0] UART_DATA   = BASE + 32'd0;
  localparam logic [31:0] UART_STATUS = BASE + 32'd4;

  // The divisor the device derives, recomputed from the same two numbers rather than
  // written down: a literal here would go on decoding at the old rate the day either
  // parameter moves, which is a bench agreeing with itself.
  localparam int DIVISOR    = (CLOCK_HZ + BAUD / 2) / BAUD;
  localparam int FRAME_BITS = 10;

  uart #(.BASE(BASE), .CLOCK_HZ(CLOCK_HZ), .BAUD(BAUD)) dut (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .tx(tx)
  );

  int errors = 0;

  task automatic check_hex(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
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

  task automatic check_bit(input string what, input logic got, input logic expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%b expected=%b", what, got, expected);
        errors++;
      end
    end
  endtask

  // The bus. Idle unless a task is driving it, so nothing here holds a write strobe over
  // an edge it did not mean to.

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

  // The read port is registered, so the answer belongs to the address that was presented
  // across the previous edge -- the same one-cycle turnaround rtl/accessor.v gives every
  // load.
  task automatic load(input logic [31:0] a);
    begin
      mem_addr  = a;
      mem_wstrb = 4'b0000;
      @(posedge clk);
      #1;
    end
  endtask

  // The line recorder and the decoder.

  localparam int TRACE_LEN = 8192;
  localparam int MAX_BYTES = 32;

  logic trace [0:TRACE_LEN-1];
  int   trace_n = 0;
  logic capture = 1'b0;

  // A plain `always`: iverilog warns about a $display inside an `always_ff`, and the
  // decoder below reports through one.
  always @(posedge clk) begin
    if (capture && trace_n < TRACE_LEN) begin
      trace[trace_n] <= tx;
      trace_n        <= trace_n + 1;
    end
  end

  // The decoder's verdict. Statuses, in the order a receiver meets them:
  localparam int DEC_OK       = 0;  // every frame in the trace decoded cleanly
  localparam int DEC_ABSENT   = 1;  // the line never left its idle level
  localparam int DEC_FRAMING  = 2;  // a frame's stop bit was not high
  localparam int DEC_TRUNCATED = 3; // a frame ran off the end of the trace

  int   decode_status;
  int   decoded_n;
  logic [7:0] decoded [0:MAX_BYTES-1];

  task automatic decode_stream(input int div);
    int i, mid, k, last;
    logic [7:0] value;
    begin
      decode_status = DEC_ABSENT;
      decoded_n     = 0;
      i             = 0;
      while (i < trace_n && decoded_n < MAX_BYTES) begin
        // Idle is high; a frame opens on the first low sample.
        while (i < trace_n && trace[i] !== 1'b0) i++;
        if (i >= trace_n) begin
          // Nothing more on the line.
          if (decoded_n > 0 && decode_status == DEC_ABSENT) decode_status = DEC_OK;
          return;
        end
        mid  = i + div / 2;
        last = mid + (FRAME_BITS - 1) * div;
        if (last >= trace_n) begin
          decode_status = DEC_TRUNCATED;
          return;
        end
        for (k = 0; k < 8; k++) value[k] = trace[mid + (k + 1) * div];
        if (trace[last] !== 1'b1) begin
          decode_status = DEC_FRAMING;
          return;
        end
        decoded[decoded_n] = value;
        decoded_n++;
        decode_status = DEC_OK;
        i = last + div / 2;
      end
    end
  endtask

  task automatic paint_frame(input int at, input logic [7:0] value, input logic stop);
    int k, b;
    begin
      for (k = 0; k < DIVISOR; k++) trace[at + k] = 1'b0;
      for (b = 0; b < 8; b++)
        for (k = 0; k < DIVISOR; k++) trace[at + (b + 1) * DIVISOR + k] = value[b];
      for (k = 0; k < DIVISOR; k++) trace[at + 9 * DIVISOR + k] = stop;
    end
  endtask

  task automatic paint_idle();
    int k;
    begin
      for (k = 0; k < TRACE_LEN; k++) trace[k] = 1'b1;
    end
  endtask

  task automatic send_string(input string s);
    int c;
    begin
      trace_n = 0;
      capture = 1'b1;
      idle();
      for (c = 0; c < s.len(); c++) begin
        store(UART_DATA, {24'b0, s[c]}, 4'b0001);
        load(UART_STATUS);
        while (mem_rdata[0] === 1'b1) load(UART_STATUS);
      end
      repeat (DIVISOR) idle();
      capture = 1'b0;
    end
  endtask

  task automatic check_decoded(input string what, input string expected);
    int c;
    begin
      if (decode_status != DEC_OK) begin
        $display("MISMATCH %s: decoder status %0d, expected %0d", what, decode_status, DEC_OK);
        errors++;
      end else if (decoded_n != expected.len()) begin
        $display("MISMATCH %s: decoded %0d bytes, expected %0d", what, decoded_n, expected.len());
        errors++;
      end else begin
        for (c = 0; c < expected.len(); c++) begin
          if (decoded[c] !== expected[c]) begin
            $display("MISMATCH %s byte %0d: got=%02x expected=%02x",
                     what, c, decoded[c], expected[c]);
            errors++;
          end
        end
      end
    end
  endtask

  int reds_forced = 0;

  task automatic require_disagreement(input string what, input string expected);
    int c;
    logic agrees;
    begin
      reds_forced++;
      agrees = (decode_status == DEC_OK) && (decoded_n == expected.len());
      if (agrees)
        for (c = 0; c < expected.len(); c++)
          if (decoded[c] !== expected[c]) agrees = 1'b0;
      if (agrees) begin
        $display("MISMATCH %s: the decoder AGREED, so it is not reading the line", what);
        errors++;
      end
    end
  endtask

  task automatic require_status(input string what, input int want);
    begin
      reds_forced++;
      if (decode_status != want) begin
        $display("MISMATCH %s: decoder status %0d, expected %0d", what, decode_status, want);
        errors++;
      end
    end
  endtask

  int busy_cycles;

  initial begin
    reset     = 1'b1;
    mem_addr  = 32'h0;
    mem_wdata = 32'h0;
    mem_wstrb = 4'b0000;
    repeat (2) @(posedge clk);
    #1;
    reset = 1'b0;

    // 12 MHz over 115200 baud is 104.17, and the device rounds.
    check_int("the divisor is the clock over the baud rate, rounded", DIVISOR, 104);

    check_bit("the line idles high out of reset", tx, 1'b1);

    load(UART_STATUS);
    check_hex("nothing is in flight, so status reads zero", mem_rdata, 32'h0);
    load(UART_DATA);
    check_hex("the data register is write-only and reads zero", mem_rdata, 32'h0);

    load(BASE - 32'd4);
    check_hex("below the range reads zero", mem_rdata, 32'h0);
    load(BASE + 32'd8);
    check_hex("just past the range reads zero, not the status word", mem_rdata, 32'h0);
    load(32'h0002_0000);
    check_hex("the machine timer's base reads zero here", mem_rdata, 32'h0);

    load(UART_STATUS + 32'h0010_0000);
    check_hex("the range test reads the WHOLE address, not the low bits",
              mem_rdata, 32'h0);
    store(UART_DATA + 32'h0010_0000, 32'h0000_0041, 4'b1111);
    idle();
    load(UART_STATUS);
    check_hex("...and a store a megabyte away starts nothing", mem_rdata, 32'h0);

    store(UART_DATA, 32'h0000_0041, 4'b1111);
    load(UART_DATA);
    check_hex("the data register reads zero while a frame is in flight",
              mem_rdata, 32'h0);
    load(UART_STATUS);
    while (mem_rdata[0] === 1'b1) load(UART_STATUS);

    store(UART_DATA, 32'h0000_0041, 4'b1111);
    load(UART_STATUS);
    check_hex("a write makes it busy", mem_rdata, 32'h1);
    check_bit("...and pulls the line down for the start bit", tx, 1'b0);

    busy_cycles = 0;
    while (mem_rdata[0] === 1'b1) begin
      load(UART_STATUS);
      busy_cycles++;
    end
    check_int("a frame is ten bit times on the wire",
              busy_cycles, FRAME_BITS * DIVISOR);
    check_bit("...and the line is back at its idle level", tx, 1'b1);

    store(UART_DATA, 32'h0000_5a00, 4'b0010);
    idle();
    load(UART_STATUS);
    check_hex("a byte store to lane 1 of the data register starts nothing", mem_rdata, 32'h0);

    store(BASE + 32'd8, 32'h0000_0041, 4'b1111);
    idle();
    load(UART_STATUS);
    check_hex("a store just past the range starts nothing", mem_rdata, 32'h0);

    store(UART_STATUS, 32'h0000_0041, 4'b1111);
    idle();
    load(UART_STATUS);
    check_hex("the status register is read-only, so writing it starts nothing",
              mem_rdata, 32'h0);

    send_string("Hi\n");
    decode_stream(DIVISOR);
    check_decoded("the bytes on the line are the bytes software wrote", "Hi\n");

    check_hex("...least significant bit first", {24'b0, decoded[0]}, 32'h48);

    decode_stream(DIVISOR + DIVISOR / 4);
    require_disagreement("a divisor a quarter too slow", "Hi\n");
    decode_stream(DIVISOR / 2);
    require_disagreement("a divisor half the transmitter's", "Hi\n");

    decode_stream(DIVISOR);
    check_decoded("the transmitter's own divisor still agrees", "Hi\n");

    paint_idle();
    trace_n = TRACE_LEN;
    paint_frame(16, 8'h5a, 1'b1);
    decode_stream(DIVISOR);
    check_decoded("a painted frame decodes like a real one", "Z");

    paint_idle();
    paint_frame(16, 8'h5a, 1'b0);
    decode_stream(DIVISOR);
    require_status("a frame with no stop bit is a framing error", DEC_FRAMING);

    paint_idle();
    decode_stream(DIVISOR);
    require_status("an idle line decodes as no stream, not as zero bytes", DEC_ABSENT);
    check_int("...and it yields no bytes either", decoded_n, 0);

    paint_idle();
    trace_n = 3 * DIVISOR;
    paint_frame(16, 8'h5a, 1'b1);
    decode_stream(DIVISOR);
    require_status("a frame running off the end of the trace is truncated", DEC_TRUNCATED);

    trace_n = 0;
    capture = 1'b1;
    idle();
    store(UART_DATA, 32'h0000_0041, 4'b0001);   // 'A'
    idle();
    load(UART_STATUS);
    check_hex("busy, so the next write has nowhere to go", mem_rdata, 32'h1);
    store(UART_DATA, 32'h0000_0042, 4'b0001);   // 'B', dropped
    load(UART_STATUS);
    while (mem_rdata[0] === 1'b1) load(UART_STATUS);
    repeat (DIVISOR) idle();
    capture = 1'b0;

    decode_stream(DIVISOR);
    check_decoded("a write while busy is dropped, not queued and not spliced", "A");

    trace_n = 0;
    capture = 1'b1;
    idle();
    store(UART_DATA, 32'h0000_0042, 4'b0001);
    load(UART_STATUS);
    while (mem_rdata[0] === 1'b1) load(UART_STATUS);
    repeat (DIVISOR) idle();
    capture = 1'b0;

    decode_stream(DIVISOR);
    check_decoded("...and the byte after it goes out", "B");

    if (reds_forced != 5) begin
      $display("MISMATCH the forced failures did not all run: %0d of 5", reds_forced);
      errors++;
    end

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: uart (map, frame length, lsb first, no FIFO, decoded at %0d cycles a bit)",
               DIVISOR);
      $finish;
    end
  end
endmodule
