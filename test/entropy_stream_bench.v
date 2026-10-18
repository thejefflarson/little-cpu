`timescale 1 ns / 1 ps
`default_nettype none

// Drives the framer with edges and prints the expected sample bits and the decoded bytes.
module entropy_stream_bench;
  parameter integer EDGES = 1800;
`ifdef FAST_EDGES
  localparam integer MIN_GAP = 150;
  localparam integer SPREAD  = 4;
`else
  localparam integer MIN_GAP = 3140;
  localparam integer SPREAD  = 30;
`endif
  localparam integer CLOCK_HZ = 30_000_000;
  localparam integer DIVISOR  = (CLOCK_HZ + 57_600) / 115_200;

  logic clk = 0;
  always #5 clk = ~clk;

  logic running = 0;
  logic edge_seen = 0;
  logic uart_tx;

  entropy_bit_framer #(.CLOCK_HZ(CLOCK_HZ)) dut (
    .clk(clk), .running(running), .edge_seen(edge_seen), .uart_tx(uart_tx)
  );

  integer gap, ticks, i, b;
  logic [7:0] byte_in;

  initial begin
    repeat (20) @(posedge clk);
    running <= 1;
    repeat (7) @(posedge clk);
    edge_seen <= 1; @(posedge clk); edge_seen <= 0;
    for (i = 0; i < EDGES; i = i + 1) begin
      gap = MIN_GAP + ($urandom % SPREAD);
      repeat (gap - 1) @(posedge clk);
      edge_seen <= 1; @(posedge clk); edge_seen <= 0;
      ticks = (gap - 1) & 3;
      $display("S %0d", ticks[1] ^ ticks[0]);
    end
    repeat (60000) @(posedge clk);
    $finish;
  end

  initial forever begin
    @(negedge uart_tx);
    repeat (DIVISOR + DIVISOR / 2) @(posedge clk);
    for (b = 0; b < 8; b = b + 1) begin
      byte_in[b] = uart_tx;
      repeat (DIVISOR) @(posedge clk);
    end
    $display("B %02x", byte_in);
  end
endmodule
`default_nettype wire
