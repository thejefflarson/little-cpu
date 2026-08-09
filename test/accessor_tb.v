`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// The read enable rtl/imemory.v arbitrates on.
//
// The idle bus presents address 0, which is inside the text range, so the
// memory cannot tell a real load from an idle cycle by address alone. This
// bench is what says `mem_ren` draws that line: it is high for the one cycle a
// load's request is on the bus and low for every other cycle, including the
// response cycle, a store, an ALU op, a bubble and reset.
//
// Stuck high is caught elsewhere -- the ladder's environment would steal every
// cycle and `hang` would go red -- but stuck low is invisible to everything
// else in the tree until a program does a text-region load, and none does yet.
module accessor_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  executor_output in;
  logic [31:0] mem_addr, mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata = 32'hcafef00d;
  logic        mem_ren, stalled, pending_valid;
  logic [4:0]  pending_rd;
  accessor_output out;

  accessor dut (
    .clk(clk),
    .reset(reset),
    .in(in),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_wdata(mem_wdata),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .stalled(stalled),
    .pending_valid(pending_valid),
    .pending_rd(pending_rd),
    .out(out)
  );

  int errors = 0;

  task automatic check_bit(input string what, input logic got, input logic expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%b expected=%b", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_hex(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

  // A payload with no execution flag set. Everything else is zeroed, so a
  // vector cannot inherit a flag from the one before it.
  task automatic present(input logic valid, input logic [31:0] addr);
    begin
      in = '0;
      in.valid = valid;
      in.rd = 5'd7;
      in.mem_addr = addr;
      in.mem_data = 32'h1234_5678;
    end
  endtask

  // One load of the given width, presented and then drained through its
  // response cycle. The enable is one predicate over five flags, so each width
  // gets its own vector rather than `lw` standing in for all of them.
  task automatic load_raises(input string what, input logic [4:0] widths);
    begin
      present(1'b1, 32'h0000_0500);
      {in.is_lw, in.is_lhu, in.is_lh, in.is_lbu, in.is_lb} = widths;
      #1;
      check_bit(what, mem_ren, 1'b1);
      @(posedge clk);
      #1;
      present(1'b0, 32'b0);
      @(posedge clk);
      #1;
    end
  endtask

  // One load, driven through its response cycle with `data` on the bus, and the
  // register value it unpacks to.
  task automatic load_unpacks(input string what, input logic [4:0] widths,
                              input logic [31:0] addr, input logic [31:0] data,
                              input logic [31:0] expected);
    begin
      present(1'b1, addr);
      {in.is_lw, in.is_lhu, in.is_lh, in.is_lbu, in.is_lb} = widths;
      @(posedge clk);
      #1;
      present(1'b0, 32'b0);
      mem_rdata = data;
      @(posedge clk);
      #1;
      check_bit({what, " completes"}, out.valid, 1'b1);
      check_hex(what, out.rd_data, expected);
      @(posedge clk);
      #1;
    end
  endtask

  initial begin
    reset = 1;
    present(1'b1, 32'h0000_0100);
    in.is_lw = 1'b1;
    repeat (2) @(posedge clk);
    #1;
    // Reset outranks the payload: the request block drives no address, and a
    // read enable that ignored reset would steal a fetch out of the reset
    // pulse itself.
    check_bit("no read enable while reset is high", mem_ren, 1'b0);
    check_hex("...and no address either", mem_addr, 32'b0);
    reset = 0;
    #1;

    check_bit("a load's request cycle raises the read enable", mem_ren, 1'b1);
    check_hex("...at the word-aligned load address", mem_addr, 32'h0000_0100);
    check_bit("...which is also the load-response stall", stalled, 1'b1);
    check_bit("...and is not a write", |mem_wstrb, 1'b0);

    // The response cycle. rtl/decoder.v froze upstream and rtl/executor.v
    // bubbled, so `in` is a bubble here -- and the enable must fall, or the
    // idle turnaround would take a fetch window of its own.
    @(posedge clk);
    #1;
    present(1'b0, 32'b0);
    #1;
    check_bit("the load's own response cycle drops it", mem_ren, 1'b0);
    check_bit("...with the response really in flight", pending_valid, 1'b1);
    @(posedge clk);
    #1;
    check_bit("the load completed", out.valid, 1'b1);

    // Every other bus cycle. A store holds the write port instead, and
    // rtl/imemory.v steals on the strobe for that -- not on this.
    present(1'b1, 32'h0000_0200);
    in.is_sw = 1'b1;
    #1;
    check_bit("a store does not raise the read enable", mem_ren, 1'b0);
    check_hex("...but it does drive the bus", {28'b0, mem_wstrb}, 32'hf);
    @(posedge clk);
    #1;

    present(1'b1, 32'h0000_0300);
    #1;
    check_bit("an instruction that touches no memory does not raise it", mem_ren, 1'b0);
    @(posedge clk);
    #1;

    present(1'b0, 32'h0000_0400);
    in.is_lb = 1'b1;
    #1;
    check_bit("a bubble carrying a load flag does not raise it", mem_ren, 1'b0);

    load_raises("lb raises it",  5'b00001);
    load_raises("lbu raises it", 5'b00010);
    load_raises("lh raises it",  5'b00100);
    load_raises("lhu raises it", 5'b01000);
    load_raises("lw raises it",  5'b10000);

    // The unpack. One shift down to the addressed byte and one extension serve
    // every width, so a lane the shift gets wrong and a sign taken from the
    // wrong bit both look right at offset zero and only at offset zero. Each
    // width is driven at every offset it can legally take -- a word and a
    // halfword are aligned because decode traps them otherwise -- against a
    // word whose four bytes differ and whose signs differ too.
    load_unpacks("lb  @+0", 5'b00001, 32'h0001_0000, 32'h817293f4, 32'hfffffff4);
    load_unpacks("lb  @+1", 5'b00001, 32'h0001_0001, 32'h817293f4, 32'hffffff93);
    load_unpacks("lb  @+2", 5'b00001, 32'h0001_0002, 32'h817293f4, 32'h00000072);
    load_unpacks("lb  @+3", 5'b00001, 32'h0001_0003, 32'h817293f4, 32'hffffff81);
    load_unpacks("lbu @+0", 5'b00010, 32'h0001_0000, 32'h817293f4, 32'h000000f4);
    load_unpacks("lbu @+1", 5'b00010, 32'h0001_0001, 32'h817293f4, 32'h00000093);
    load_unpacks("lbu @+2", 5'b00010, 32'h0001_0002, 32'h817293f4, 32'h00000072);
    load_unpacks("lbu @+3", 5'b00010, 32'h0001_0003, 32'h817293f4, 32'h00000081);
    load_unpacks("lh  @+0", 5'b00100, 32'h0001_0000, 32'h817293f4, 32'hffff93f4);
    load_unpacks("lh  @+2", 5'b00100, 32'h0001_0002, 32'h817293f4, 32'hffff8172);
    load_unpacks("lh  @+0", 5'b00100, 32'h0001_0000, 32'h81720074, 32'h00000074);
    load_unpacks("lhu @+0", 5'b01000, 32'h0001_0000, 32'h817293f4, 32'h000093f4);
    load_unpacks("lhu @+2", 5'b01000, 32'h0001_0002, 32'h817293f4, 32'h00008172);
    load_unpacks("lw  @+0", 5'b10000, 32'h0001_0000, 32'h817293f4, 32'h817293f4);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: accessor read enable and load unpack");
      $finish;
    end
  end
endmodule
