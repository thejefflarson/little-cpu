`timescale 1 ns / 1 ps
module nano_memory #(
  parameter int WORDS = 20480,
  parameter int WAIT_STATES = 0
) (
  input  logic        clk,
  input  logic        reset,
  input  logic        mem_valid,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [ 3:0] mem_wstrb,
  output logic        mem_ready,
  output logic [31:0] mem_rdata
);
  // A fetch address may be halfword- rather than word-aligned (a compressed successor);
  // this combines the two neighbouring words rather than only answering word-aligned.
  logic [31:0] mem [0:WORDS-1];
  logic [31:0] word_addr;
  assign word_addr = mem_addr[31:2];
  logic [31:0] low_word, high_word;
  assign low_word = mem[word_addr];
  assign high_word = mem[word_addr + 1];
  assign mem_rdata = mem_addr[1] ? {high_word[15:0], low_word[31:16]} : low_word;

  logic waiting;
  int unsigned wait_left;

  always_ff @(posedge clk) begin
    if (reset) begin
      waiting <= 1'b0;
      wait_left <= 0;
    end else if (mem_valid && !mem_ready) begin
      if (!waiting) begin
        waiting <= (WAIT_STATES != 0);
        wait_left <= WAIT_STATES == 0 ? 0 : WAIT_STATES - 1;
      end else if (wait_left != 0) begin
        wait_left <= wait_left - 1;
      end
    end else begin
      waiting <= 1'b0;
    end
  end

  assign mem_ready = mem_valid && (WAIT_STATES == 0 || (waiting && wait_left == 0));

  always_ff @(posedge clk) begin
    if (mem_valid && mem_ready) begin
      if (mem_wstrb[0]) mem[word_addr][7:0]   <= mem_wdata[7:0];
      if (mem_wstrb[1]) mem[word_addr][15:8]  <= mem_wdata[15:8];
      if (mem_wstrb[2]) mem[word_addr][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) mem[word_addr][31:24] <= mem_wdata[31:24];
    end
  end
endmodule
