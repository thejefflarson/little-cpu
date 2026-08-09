`timescale 1 ns / 1 ps
`default_nettype none

// rtl/imemory.v, the instruction ROM split across two banks.
//
// The .S suite only hits whatever alignments its programs happen to produce,
// and it never touches the data port: no program stores into the text region,
// and both runners hold mem_ren low. So the bank select, the range decode, the
// write port and the steal are checked here or nowhere.
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
  logic        mem_ren, fetch_stall, imem_ren;

  imemory #(.ROM_WORDS(ROM_WORDS)) dut (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_ren(imem_ren),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall)
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
      imem_ren       = 1'b1;
      imem_addr_next = fetch_addr;
      mem_addr       = data_addr;
      mem_ren        = ren;
      mem_wstrb      = strb;
      mem_wdata      = wdata;
      @(posedge clk);
      #1;
    end
  endtask

  // A cycle the core wants no new window. The address is driven to something
  // else on purpose: a memory that answered it instead of holding would pass
  // every vector that re-presents the same address, which is what the shipping
  // core did before it had this pin.
  task automatic hold(input logic [31:0] decoy_addr,
                      input logic        ren,
                      input logic [3:0]  strb,
                      input logic [31:0] data_addr,
                      input logic [31:0] wdata);
    begin
      imem_ren       = 1'b0;
      imem_addr_next = decoy_addr;
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

  int i;
  initial begin
    for (i = 0; i < ROM_WORDS; i++) begin
      ref_rom[i] = 32'hc0de_0000 + 32'(i) * 32'h0001_0101;
      // The split soc/rom_banks.py writes for synthesis, which the read side
      // has to invert.
      if (i % 2 == 0) dut.rom_even[i/2] = ref_rom[i];
      else            dut.rom_odd[i/2]  = ref_rom[i];
    end

    fetch(32'b0);

    for (i = 0; i < ROM_WORDS - 1; i++) begin
      fetch(32'(i) * 4);
      check($sformatf("word %0d: imem_data", i), imem_data, ref_rom[i]);
      check($sformatf("word %0d: imem_data2", i), imem_data2, ref_rom[i+1]);
    end

    fetch(32'(ROM_WORDS - 1) * 4);
    check("last word: imem_data", imem_data, ref_rom[ROM_WORDS-1]);
    check("last word: imem_data2 is out of range", imem_data2, 32'b0);

    // Neither aliases ref_rom[0], which bit truncation alone would produce.
    fetch(32'(ROM_WORDS) * 4);
    check("one past the end: imem_data", imem_data, 32'b0);
    check("one past the end: imem_data2", imem_data2, 32'b0);
    fetch(32'h0000_1000);
    check("far out of range: imem_data", imem_data, 32'b0);
    check("far out of range: imem_data2", imem_data2, 32'b0);

    // A `word + 1` range test would wrap to 0 here and answer the second word
    // out of real ROM; rtl/imemory.v tests `word < ROM_WORDS - 1` instead.
    fetch(32'hffff_fffc);
    check("top of the address space: imem_data faults", imem_data, 32'b0);
    check("top of the address space: imem_data2 does not wrap to word 0",
          imem_data2, 32'b0);

    fetch(32'h0000_0008);
    check("back in range after an out-of-range fetch", imem_data, ref_rom[2]);
    check("...and its second word", imem_data2, ref_rom[3]);

    // Holding the address re-presents the same words, which is what makes a
    // stalled cycle free.
    fetch(32'h0000_0008);
    check("a held address re-presents the same word", imem_data, ref_rom[2]);
    check_stall("an idle data bus does not steal a fetch", 1'b0);

    // ...and so does holding the read, which is the port the core uses instead:
    // the address moves under it and the window does not follow.
    hold(32'h0000_0000, 1'b0, 4'b0000, 32'b0, 32'b0);
    check("a held read keeps the window (imem_data)", imem_data, ref_rom[2]);
    check("...and its second word", imem_data2, ref_rom[3]);
    hold(32'h0000_0020, 1'b0, 4'b0000, 32'b0, 32'b0);
    hold(32'h0000_0004, 1'b0, 4'b0000, 32'b0, 32'b0);
    check("a window held for three cycles is still the same one", imem_data, ref_rom[2]);
    check("...and its second word", imem_data2, ref_rom[3]);
    check_stall("holding the read steals nothing", 1'b0);
    fetch(32'h0000_0004);
    check("...and the read that follows lands on the address it asked for",
          imem_data, ref_rom[1]);

    // The range decode belongs to the window, not to the address being
    // presented, so it has to be held with it. Otherwise a held cycle whose
    // decoy address is in range turns an out-of-range window into real ROM.
    fetch(32'h0000_1000);
    check("a fetch out of range reads zero", imem_data, 32'b0);
    hold(32'h0000_0008, 1'b0, 4'b0000, 32'b0, 32'b0);
    check("...and a held cycle does not bring it back in range", imem_data, 32'b0);
    check("...on either word", imem_data2, 32'b0);

    // A data access takes the read port whether the fetch wanted one or not.
    // The load still has to be answered, so the window is lost -- and
    // `fetch_stall` is what says so, on a cycle the core was already holding.
    fetch(32'h0000_0008);
    check("back in range", imem_data, ref_rom[2]);
    hold(32'h0000_0000, 1'b1, 4'b0000, 32'h0000_0018, 32'b0);
    check("a text read during a held fetch is still answered", mem_rdata, ref_rom[6]);
    check_stall("...and reports the steal", 1'b1);
    fetch(32'h0000_0008);
    check("...so the window is fetched again after it", imem_data, ref_rom[2]);

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

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: imemory.v bank select, dual-word window, range decode, read enable, data port");
      $finish;
    end
  end
endmodule
