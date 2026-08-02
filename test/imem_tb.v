`timescale 1 ns / 1 ps
`default_nettype none

// Standalone bench for rtl/imemory.v -- the interleaved-bank instruction ROM
// (ADR-0054).
//
// Two things here are not reachable from the `.S` suite, which is why this
// bench exists rather than the suite being treated as coverage:
//
//   * THE BANK SELECT AT EVERY ALIGNMENT. The suite runs real programs, so it
//     exercises whatever alignments those programs happen to produce. This
//     walks every word index in a small ROM, both parities, and checks BOTH
//     window words against a flat reference array -- so a bank swap, an
//     off-by-one on the `+ W[0]` index bump, or a `imem_data`/`imem_data2`
//     transposition is caught at the one address where it differs rather than
//     wherever a program first noticed.
//
//   * THE RANGE DECODE. The last word of ROM is in range while the word after
//     it is not, so a correct fetch there returns a real `imem_data` and a zero
//     `imem_data2`. Getting that wrong aliases the top of ROM back to the
//     bottom, which reads as a plausible instruction rather than as a fault.
//
// The reference is a flat, independently-filled array indexed by word number.
// It is not built from the bank images: the bank split is the thing under test,
// so a reference derived from it would agree with any split at all.
module imem_tb;
  localparam int ROM_WORDS = 16;

  logic clk = 0;
  always #5 clk = ~clk;

  logic [31:0] imem_addr_next;
  logic [31:0] imem_data, imem_data2;

  imemory #(.ROM_WORDS(ROM_WORDS)) dut (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2)
  );

  // The flat reference. Word i holds a value that is unique in both halves, so
  // a transposed window or a swapped bank cannot coincide with the right answer.
  logic [31:0] ref_rom[0:ROM_WORDS-1];

  int errors = 0;

  task automatic check(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

  // Present a word address and take the edge that latches it; the outputs are
  // valid for the whole of the cycle after, which is where they are read.
  // That IS the contract (ADR-0054): the address is presented one cycle before
  // the data is needed.
  task automatic fetch(input logic [31:0] addr);
    begin
      imem_addr_next = addr;
      @(posedge clk);
      #1;
    end
  endtask

  int i;
  initial begin
    for (i = 0; i < ROM_WORDS; i++) begin
      ref_rom[i] = 32'hc0de_0000 + 32'(i) * 32'h0001_0101;
      // The banks, filled from the reference by the same rule rtl/imemory.v's
      // read side has to invert. soc/rom_banks.py writes the same split for
      // the synthesis flow.
      if (i % 2 == 0) dut.rom_even[i/2] = ref_rom[i];
      else            dut.rom_odd[i/2]  = ref_rom[i];
    end

    imem_addr_next = 32'b0;
    @(posedge clk);
    #1;

    // Every word index, both parities, both window words.
    for (i = 0; i < ROM_WORDS - 1; i++) begin
      fetch(32'(i) * 4);
      check($sformatf("word %0d: imem_data", i), imem_data, ref_rom[i]);
      check($sformatf("word %0d: imem_data2", i), imem_data2, ref_rom[i+1]);
    end

    // The last word: in range, and its successor is not.
    fetch(32'(ROM_WORDS - 1) * 4);
    check("last word: imem_data", imem_data, ref_rom[ROM_WORDS-1]);
    check("last word: imem_data2 is out of range", imem_data2, 32'b0);

    // One past the end, and far past it. Both words zero; neither aliases
    // ref_rom[0], which bit truncation alone would produce.
    fetch(32'(ROM_WORDS) * 4);
    check("one past the end: imem_data", imem_data, 32'b0);
    check("one past the end: imem_data2", imem_data2, 32'b0);
    fetch(32'h0000_1000);
    check("far out of range: imem_data", imem_data, 32'b0);
    check("far out of range: imem_data2", imem_data2, 32'b0);

    // The top of the address space, where a `word + 1` range test would wrap to
    // 0 and answer the second word of the pair out of real ROM. rtl/imemory.v
    // tests `word < ROM_WORDS - 1` instead -- an incrementer there would sit in
    // series with the one that produced the address, which `make soc-timing`
    // measured on the critical path -- so both words fault here.
    fetch(32'hffff_fffc);
    check("top of the address space: imem_data faults", imem_data, 32'b0);
    check("top of the address space: imem_data2 does not wrap to word 0",
          imem_data2, 32'b0);

    // Back in range on the very next cycle, with no state carried over: the
    // window is stateless (ADR-0003) and the range flags are per-cycle.
    fetch(32'h0000_0008);
    check("back in range after an out-of-range fetch", imem_data, ref_rom[2]);
    check("...and its second word", imem_data2, ref_rom[3]);

    // The address is latched, so holding it re-presents the same words -- which
    // is what makes a stalled cycle free (rtl/decoder.v holds `next_pc = pc`).
    fetch(32'h0000_0008);
    check("a held address re-presents the same word", imem_data, ref_rom[2]);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: imemory.v bank select, dual-word window, range decode");
      $finish;
    end
  end
endmodule
