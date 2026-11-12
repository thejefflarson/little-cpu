`timescale 1 ns / 1 ps
`default_nettype none

// rtl/spi_target.v driven as a core and an external mode-0 controller would: bus reads and writes on one side,
// sck/cs_n/mosi/miso on the other. The controller clocks one chip-select frame of bytes while the core queues
// a byte at a swept cycle offset, so one offset lands on the target's byte boundary.
module spi_target_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset = 1, sck = 0, cs_n = 1, mosi = 0, miso, miso_oe, ren = 0;
  logic [31:0] addr = 0, wdata = 0, rdata;
  logic [3:0]  wstrb = 0;

  localparam logic [31:0] BASE = 32'h0002_0030;
  // The controller's half period in core clocks, slower than the target's three-stage synchronizer.
  localparam int HALF = 8;
  localparam int FRAME_BYTES = 4;

  spi_target #(.BASE(BASE)) dut (
    .clk(clk), .reset(reset), .mem_addr(addr), .mem_wdata(wdata), .mem_wstrb(wstrb), .mem_ren(ren),
    .mem_rdata(rdata), .sck(sck), .cs_n(cs_n), .mosi(mosi), .miso(miso), .miso_oe(miso_oe)
  );

  int errors = 0;
  logic [7:0] got [FRAME_BYTES];

  task automatic bus_write(input logic [7:0] b);
    begin
      @(posedge clk); #1; addr = BASE; wdata = {24'b0, b}; wstrb = 4'h1;
      @(posedge clk); #1; wstrb = 0; addr = 0; wdata = 0;
    end
  endtask

  task automatic bus_read(output logic [31:0] w);
    begin
      @(posedge clk); #1; addr = BASE; ren = 1;
      @(posedge clk); #1; ren = 0;
      @(posedge clk); #1; w = rdata; addr = 0;
    end
  endtask

  task automatic clock_frame;
    logic [7:0] g;
    begin
      cs_n = 0; repeat (HALF) @(posedge clk);
      if (miso_oe !== 1'b1) begin
        $display("MISMATCH miso_oe: low while selected");
        errors++;
      end
      for (int i = 0; i < FRAME_BYTES; i++) begin
        for (int k = 7; k >= 0; k--) begin
          mosi = (8'h5A + 8'(i)) >> k;
          repeat (HALF) @(posedge clk); sck = 1;
          g = {g[6:0], miso};
          repeat (HALF) @(posedge clk); sck = 0;
        end
        got[i] = g;
      end
      repeat (HALF) @(posedge clk); cs_n = 1;
    end
  endtask

  // One frame with the core queueing `queued` write_at clocks after the frame starts; negative queues it first.
  task automatic frame(input int write_at, input logic [7:0] queued);
    begin
      reset = 1; repeat (3) @(posedge clk); #1; reset = 0; repeat (3) @(posedge clk);
      if (write_at < 0) bus_write(queued);
      repeat (4) @(posedge clk);
      fork
        clock_frame;
        begin
          if (write_at >= 0) begin
            repeat (write_at) @(posedge clk);
            bus_write(queued);
          end
        end
      join
      repeat (6) @(posedge clk);
      if (miso_oe !== 1'b0) begin
        $display("MISMATCH miso_oe: still driving after the select released");
        errors++;
      end
    end
  endtask

  // A queued byte must leave whole and exactly once: every received byte is zero or the byte, never a
  // mixture, and the byte appears once.
  task automatic grade(input string what, input logic [7:0] queued);
    int seen;
    begin
      seen = 0;
      for (int i = 0; i < FRAME_BYTES; i++) begin
        if (got[i] == queued) seen++;
        else if (got[i] !== 8'h00) begin
          $display("MISMATCH %s: byte %0d came out %02x, neither 00 nor %02x", what, i, got[i], queued);
          errors++;
        end
      end
      if (seen != 1) begin
        $display("MISMATCH %s: %02x appeared %0d times in %02x %02x %02x %02x", what, queued, seen,
                 got[0], got[1], got[2], got[3]);
        errors++;
      end
    end
  endtask

  logic [31:0] w;
  initial begin
    repeat (3) @(posedge clk); #1; reset = 0;

    frame(-1, 8'hA5);
    grade("queued before the frame", 8'hA5);

    // Sweep the write across more than a byte time (16 * HALF clocks) so one offset is the byte boundary.
    for (int off = 0; off < 40 * HALF; off++) begin
      frame(off, 8'hC3);
      grade($sformatf("write %0d clocks into the frame", off), 8'hC3);
    end

    frame(-1, 8'h00);
    bus_read(w);
    if (!w[8] || w[7:0] !== (8'h5A + 8'(FRAME_BYTES - 1))) begin
      $display("MISMATCH rx: read %03x, wanted valid with %02x", w[10:0], 8'h5A + 8'(FRAME_BYTES - 1));
      errors++;
    end
    bus_read(w);
    if (w[8]) begin
      $display("MISMATCH rx: rx_valid still set after a read (%03x)", w[10:0]);
      errors++;
    end
    if (!w[10]) begin
      $display("MISMATCH status: cs_n high must read as bit 10 set (%03x)", w[10:0]);
      errors++;
    end

    if (errors != 0) begin
      $display("FAILED: spi_target, %0d mismatches", errors);
      $fatal(1);
    end
    $display("PASSED: spi_target");
    $finish;
  end
endmodule
