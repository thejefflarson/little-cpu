`timescale 1 ns / 1 ps
`default_nettype none

// rtl/imemory.v, the instruction ROM split across two banks.
//
// The .S suite only hits whatever alignments its programs happen to produce,
// and it reaches the data port only through the few programs that store into
// text or load from it. So the bank select, the range decode, the write port's
// byte lanes and the steal are checked exhaustively here or nowhere.
//
// Keep the reference a flat array filled on its own. Build it from the bank
// images and it will agree with any split, which is the thing being tested.
module imem_tb;
  localparam int ROM_WORDS = 16;

  logic clk = 0;
  always #5 clk = ~clk;

  logic [31:0] imem_addr_next;
  logic [31:0] imem_data, imem_data2;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren, fetch_stall, imem_fault;

  imemory #(.ROM_WORDS(ROM_WORDS)) dut (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault)
  );

  // A second instance with two fetch windows, on the same data bus. What the
  // two windows do to EACH OTHER is only visible here: the shipping SoC has one,
  // and the vectors above cannot tell a per-window signal from a shared one.
  // That the two windows read one storage rather than two is a property of the
  // MAPPED netlist and is graded by test/imem_share_test.sh -- here there is one
  // array by construction and the question cannot be asked.
  logic [63:0] d_addr_next, d_data, d_data2;
  logic [31:0] d_mem_rdata;
  logic [1:0]  d_fetch_stall, d_imem_fault;

  imemory #(.ROM_WORDS(ROM_WORDS), .NHARTS(2)) dut2 (
    .clk(clk),
    .imem_addr_next(d_addr_next),
    .imem_data(d_data),
    .imem_data2(d_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(d_mem_rdata),
    .fetch_stall(d_fetch_stall),
    .imem_fault(d_imem_fault)
  );

  // Unique in both halves per word, so a transposed window or a swapped bank
  // cannot coincide with the right answer.
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

  // The memory latches the address on the edge. Its outputs are then valid for
  // the whole of the next cycle, which is where they are read.
  task automatic step(input logic [31:0] fetch_addr,
                      input logic [31:0] data_addr,
                      input logic        ren,
                      input logic [3:0]  strb,
                      input logic [31:0] wdata);
    begin
      imem_addr_next = fetch_addr;
      mem_addr       = data_addr;
      mem_ren        = ren;
      mem_wstrb      = strb;
      mem_wdata      = wdata;
      @(posedge clk);
      #1;
    end
  endtask

  // The idle bus presents address 0, which is inside the text range, so
  // `mem_ren` low is the only thing keeping it from stealing every cycle.
  task automatic fetch(input logic [31:0] addr);
    step(addr, 32'b0, 1'b0, 4'b0000, 32'b0);
  endtask

  // The fetch address parks at word 0, so an answer that came off the fetch
  // side rather than the data side reads as word 0.
  task automatic dread(input logic [31:0] addr);
    step(32'b0, addr, 1'b1, 4'b0000, 32'b0);
  endtask

  task automatic check_stall(input string what, input logic expected);
    check(what, {31'b0, fetch_stall}, {31'b0, expected});
  endtask

  // The bit decode raises as an instruction access fault. It is checked beside
  // every out-of-range window below rather than on its own, because zero data
  // and a raised fault are one answer: the data alone is an illegal instruction
  // and reports the wrong cause, and the fault alone would be a cause with a
  // real word behind it.
  task automatic check_fault(input string what, input logic expected);
    check(what, {31'b0, imem_fault}, {31'b0, expected});
  endtask

  // One task, so the memory and the reference always get the same arguments.
  // Out of range the reference does nothing, which is how check_all tells that
  // a dropped write really was dropped.
  task automatic write_word(input logic [31:0] addr, input logic [3:0] strb,
                            input logic [31:0] data);
    int word;
    begin
      step(32'b0, addr, 1'b0, strb, data);
      word = int'(addr >> 2);
      if (word < ROM_WORDS) begin
        if (strb[0]) ref_rom[word][7:0]   = data[7:0];
        if (strb[1]) ref_rom[word][15:8]  = data[15:8];
        if (strb[2]) ref_rom[word][23:16] = data[23:16];
        if (strb[3]) ref_rom[word][31:24] = data[31:24];
      end
    end
  endtask

  // A write that landed in the wrong bank, at the wrong index or on the wrong
  // bytes shows up here even when the word it was aimed at reads correctly.
  task automatic check_all(input string what);
    int j;
    for (j = 0; j < ROM_WORDS; j++) begin
      fetch(32'(j) * 4);
      check($sformatf("%s: word %0d", what, j), imem_data, ref_rom[j]);
    end
  endtask

  // The two-window instance's own step, so a case can put a different address in
  // each window on one cycle. The data bus is the same one the tasks above
  // drive.
  task automatic dstep(input logic [31:0] fetch0, input logic [31:0] fetch1,
                       input logic [31:0] data_addr, input logic ren,
                       input logic [3:0] strb, input logic [31:0] wdata);
    begin
      d_addr_next    = {fetch1, fetch0};
      imem_addr_next = 32'b0;
      mem_addr       = data_addr;
      mem_ren        = ren;
      mem_wstrb      = strb;
      mem_wdata      = wdata;
      @(posedge clk);
      #1;
    end
  endtask

  task automatic dfetch(input logic [31:0] a0, input logic [31:0] a1);
    dstep(a0, a1, 32'b0, 1'b0, 4'b0000, 32'b0);
  endtask

  task automatic check_dstall(input string what, input logic [1:0] expected);
    check(what, {30'b0, d_fetch_stall}, {30'b0, expected});
  endtask

  int i;
  initial begin
    for (i = 0; i < ROM_WORDS; i++) begin
      ref_rom[i] = 32'hc0de_0000 + 32'(i) * 32'h0001_0101;
      // The split soc/rom_banks.py writes for synthesis, which the read side
      // has to invert.
      if (i % 2 == 0) dut.rom_even[i/2] = ref_rom[i];
      else            dut.rom_odd[i/2]  = ref_rom[i];
      if (i % 2 == 0) dut2.rom_even[i/2] = ref_rom[i];
      else            dut2.rom_odd[i/2]  = ref_rom[i];
    end
    d_addr_next = 64'b0;

    fetch(32'b0);
    check_fault("word 0 is in the ROM", 1'b0);

    for (i = 0; i < ROM_WORDS - 1; i++) begin
      fetch(32'(i) * 4);
      check($sformatf("word %0d: imem_data", i), imem_data, ref_rom[i]);
      check($sformatf("word %0d: imem_data2", i), imem_data2, ref_rom[i+1]);
    end

    fetch(32'(ROM_WORDS - 1) * 4);
    check("last word: imem_data", imem_data, ref_rom[ROM_WORDS-1]);
    check("last word: imem_data2 is out of range", imem_data2, 32'b0);
    check_fault("the last word is still in the ROM", 1'b0);

    // Neither aliases ref_rom[0], which bit truncation alone would produce.
    fetch(32'(ROM_WORDS) * 4);
    check("one past the end: imem_data", imem_data, 32'b0);
    check("one past the end: imem_data2", imem_data2, 32'b0);
    check_fault("one past the end faults", 1'b1);
    fetch(32'h0000_1000);
    check("far out of range: imem_data", imem_data, 32'b0);
    check("far out of range: imem_data2", imem_data2, 32'b0);
    check_fault("far out of range faults", 1'b1);

    // A `word + 1` range test would wrap to 0 here and answer the second word
    // out of real ROM; rtl/imemory.v tests `word < ROM_WORDS - 1` instead.
    fetch(32'hffff_fffc);
    check("top of the address space: imem_data faults", imem_data, 32'b0);
    check("top of the address space: imem_data2 does not wrap to word 0",
          imem_data2, 32'b0);
    check_fault("the top of the address space faults", 1'b1);

    fetch(32'h0000_0008);
    check("back in range after an out-of-range fetch", imem_data, ref_rom[2]);
    check_fault("...and the fault clears with it", 1'b0);
    check("...and its second word", imem_data2, ref_rom[3]);

    // Holding the address re-presents the same words, which is what makes a
    // stalled cycle free (rtl/decoder.v holds `next_pc = pc`).
    fetch(32'h0000_0008);
    check("a held address re-presents the same word", imem_data, ref_rom[2]);
    check_stall("an idle data bus does not steal a fetch", 1'b0);

    for (i = 0; i < ROM_WORDS; i++) begin
      dread(32'(i) * 4);
      check($sformatf("data read of word %0d", i), mem_rdata, ref_rom[i]);
      check_stall($sformatf("word %0d: a data read steals the fetch", i), 1'b1);
    end

    // No steal out of range: the fetch keeps the port, the access was never ours.
    dread(32'(ROM_WORDS) * 4);
    check("data read one past the end", mem_rdata, 32'b0);
    check_stall("an out-of-range read steals nothing", 1'b0);
    dread(32'h0000_1004);
    check("far out-of-range data read does not alias", mem_rdata, 32'b0);

    // The fetch that follows a steal is unaffected by having lost the port.
    dread(32'h0000_0010);
    check("read word 4 (even bank)", mem_rdata, ref_rom[4]);
    fetch(32'h0000_0010);
    check("fetch word 4 the cycle after reading it", imem_data, ref_rom[4]);
    check("...and its second word", imem_data2, ref_rom[5]);
    check_stall("the fetch after a steal is not itself stolen", 1'b0);

    dread(32'h0000_0014);
    check("read word 5 (odd bank)", mem_rdata, ref_rom[5]);
    fetch(32'h0000_0014);
    check("fetch word 5 the cycle after reading it", imem_data, ref_rom[5]);
    check("...and its second word", imem_data2, ref_rom[6]);

    fetch(32'h0000_0018);
    check("fetch word 6", imem_data, ref_rom[6]);
    dread(32'h0000_0018);
    check("read word 6 the cycle after fetching it", mem_rdata, ref_rom[6]);

    write_word(32'h0000_0008, 4'b1111, 32'h1122_3344);
    check_stall("a write steals the fetch", 1'b1);
    fetch(32'h0000_0008);
    check("a written even-bank word fetches back", imem_data, ref_rom[2]);
    dread(32'h0000_0008);
    check("...and reads back", mem_rdata, ref_rom[2]);

    write_word(32'h0000_000c, 4'b1111, 32'h5566_7788);
    fetch(32'h0000_000c);
    check("a written odd-bank word fetches back", imem_data, ref_rom[3]);
    check_all("after two word writes");

    // A strobe applied to the wrong lane, or a write that put the whole word
    // down regardless of it, changes bytes the reference keeps.
    for (i = 0; i < 4; i++) begin
      write_word(32'h0000_0010, 4'b0001 << i, 32'hdead_beef);
      check_all($sformatf("after sb into byte %0d of word 4", i));
      write_word(32'h0000_0014, 4'b0001 << i, 32'h0f1e_2d3c);
      check_all($sformatf("after sb into byte %0d of word 5", i));
    end

    write_word(32'h0000_0018, 4'b0011, 32'ha5a5_5a5a);
    write_word(32'h0000_0018, 4'b1100, 32'h1234_5678);
    write_word(32'h0000_001c, 4'b0011, 32'hbeef_cafe);
    write_word(32'h0000_001c, 4'b1100, 32'h9876_5432);
    check_all("after four sh writes");

    write_word(32'(ROM_WORDS - 1) * 4, 4'b1111, 32'h0bad_f00d);
    check_all("after writing the last word");

    // Addresses whose low bits index a real bank word. `check_all` is what says
    // these were dropped rather than landing somewhere else.
    write_word(32'(ROM_WORDS) * 4, 4'b1111, 32'hffff_ffff);
    check_stall("a write past the end steals nothing", 1'b0);
    write_word(32'h0000_1004, 4'b1111, 32'hffff_ffff);
    write_word(32'hffff_fffc, 4'b1111, 32'hffff_ffff);
    check_all("after three out-of-range writes");

    // ---- two fetch windows ------------------------------------------------

    // Each window answers about its own address on the same cycle, which is
    // what says the second one is a read port of its own rather than a copy of
    // the first's answer.
    dfetch(32'h0000_0008, 32'h0000_0014);
    check("window 0 fetches its own word", d_data[31:0], ref_rom[2]);
    check("window 1 fetches its own word", d_data[63:32], ref_rom[5]);
    check("window 0's second word", d_data2[31:0], ref_rom[3]);
    check("window 1's second word", d_data2[63:32], ref_rom[6]);
    check_dstall("an idle data bus steals neither window", 2'b00);

    // The fault is per window: one hart running off the end of text must not
    // trap the other.
    dfetch(32'h0000_0004, 32'(ROM_WORDS) * 4);
    check("window 0 is in the ROM", {31'b0, d_imem_fault[0]}, 32'b0);
    check("window 1 past the end faults", {31'b0, d_imem_fault[1]}, 32'b1);
    check("window 1 past the end reads zero", d_data[63:32], 32'b0);
    check("window 0 is unaffected by it", d_data[31:0], ref_rom[1]);

    // A LOAD is answered out of window 0's copy, so it takes that window's read
    // port and no other.
    dstep(32'h0000_0008, 32'h0000_0014, 32'h0000_0010, 1'b1, 4'b0000, 32'b0);
    check("a text read is answered", d_mem_rdata, ref_rom[4]);
    check_dstall("a text read steals window 0 alone", 2'b01);
    // Both banks, because a read stolen from one of window 1's two ports shows
    // up in only one of the two words it publishes.
    check("window 1 kept fetching through it", d_data[63:32], ref_rom[5]);
    check("...including its second word", d_data2[63:32], ref_rom[6]);

    // A WRITE lands in every window's copy on one edge, and both parts define a
    // same-address write-during-read as returning invalid data on the reading
    // port -- so both windows have to be told, and this is the vector that says
    // the second one is.
    dstep(32'h0000_0008, 32'h0000_0014, 32'h0000_0014, 1'b0, 4'b1111, 32'h4a4a_5b5b);
    check_dstall("a text write steals BOTH windows", 2'b11);
    ref_rom[5] = 32'h4a4a_5b5b;

    // ...and the word it wrote is what both windows fetch afterwards, which is
    // the same array in this simulation and two block RAMs on the part.
    dfetch(32'h0000_0014, 32'h0000_0014);
    check("window 0 fetches the written word", d_data[31:0], ref_rom[5]);
    check("window 1 fetches the written word", d_data[63:32], ref_rom[5]);

    // Out of the text range the fetch keeps both ports: the access was never
    // this memory's.
    dstep(32'h0000_0008, 32'h0000_0014, 32'h0000_1000, 1'b1, 4'b0000, 32'b0);
    check_dstall("an out-of-range access steals neither window", 2'b00);
    check("...and window 0 fetched through it", d_data[31:0], ref_rom[2]);
    check("...and so did window 1", d_data[63:32], ref_rom[5]);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: imemory.v bank select, dual-word window, range decode, data port, two windows");
      $finish;
    end
  end
endmodule
