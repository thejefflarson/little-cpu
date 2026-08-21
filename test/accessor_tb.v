`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// The read enable rtl/imemory.v arbitrates on, the byte unpack, and the guard
// that keeps one memory instruction to one bus transaction.
//
// The idle bus presents address 0, which is inside the text range, so the
// memory cannot tell a real load from an idle cycle by address alone. This
// bench is what says `mem_ren` draws that line: it is high for the one cycle a
// load's request is on the bus and low for every other cycle, including the
// response cycle, a store, an ALU op, a bubble and reset.
//
// Stuck high is caught elsewhere -- the generated checks' environment would
// steal every cycle and `hang` would go red -- but stuck low is invisible to
// everything else in the tree until a program does a text-region load, and none
// does yet.
//
// The transaction count is the other half. Decode holds `launch` unchanged for
// every cycle of a divide, so a request block that read it without
// `launch_taken` would present the same store on all thirty-three of them. Two
// writes are one write for RAM and are not one for a device, and no program can
// see the difference in a value -- only in a count, which is what the divide
// vector below takes.
module accessor_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  decoder_output launch;
  logic launch_taken;
  executor_output in;
  logic [31:0] mem_addr, mem_wdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] mem_rdata = 32'hcafef00d;
  logic        mem_ren;
  // The platform's answer to "may a reservation be held at this address". High
  // for every vector except the one that asks what happens when it is not.
  logic        mem_reservable = 1'b1;
  // Another bus master's write. Low except in the vectors that drive it: this
  // core has no second master, so nothing else in the tree can say whether a
  // foreign write ends a reservation.
  logic        snoop_write = 1'b0;
  logic [31:0] snoop_addr = 32'b0;
  // The cycle an arbiter has to keep the bus here. Watched rather than driven.
  logic        mem_lock;
  accessor_output out;

  accessor dut (
    .clk(clk),
    .reset(reset),
    .launch(launch),
    .launch_taken(launch_taken),
    .in(in),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_wdata(mem_wdata),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .mem_reservable(mem_reservable),
    .snoop_write(snoop_write),
    .snoop_addr(snoop_addr),
    .mem_lock(mem_lock),
    .out(out)
  );

  int errors = 0;

  // Every cycle the bus is driven at all, counted on the same edge the memory
  // would latch it. This is the only thing in the tree that can tell one
  // transaction from thirty-three of the same one.
  int transactions = 0;
  always @(posedge clk)
    if (!reset && (mem_ren || |mem_wstrb)) transactions++;

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

  task automatic check_int(input string what, input int got, input int expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%0d expected=%0d", what, got, expected);
        errors++;
      end
    end
  endtask

  // A request payload with no execution flag set. Everything else is zeroed, so
  // a vector cannot inherit a flag from the one before it.
  task automatic present(input logic valid, input logic [31:0] addr);
    begin
      launch = '0;
      launch.valid = valid;
      launch.rd = 5'd7;
      launch.mem_addr = addr;
      launch.rs2 = 32'h1234_5678;
      launch_taken = 1'b1;
    end
  endtask

  // The struct the executor hands on a cycle later, carrying the same
  // instruction. Only `valid` and `rd` reach the accessor now; a load's data
  // comes off the bus and an ALU result rides `rd_data`.
  task automatic arrive(input logic valid, input logic [31:0] rd_data);
    begin
      in = '0;
      in.valid = valid;
      in.rd = 5'd7;
      in.rd_data = rd_data;
    end
  endtask

  // One load of the given width, presented for its request cycle. The enable is
  // one predicate over five flags, so each width gets its own vector rather
  // than `lw` standing in for all of them.
  task automatic load_raises(input string what, input logic [4:0] widths);
    begin
      present(1'b1, 32'h0000_0500);
      {launch.is_lw, launch.is_lhu, launch.is_lh, launch.is_lbu, launch.is_lb} = widths;
      #1;
      check_bit(what, mem_ren, 1'b1);
      @(posedge clk);
      #1;
      present(1'b0, 32'b0);
      @(posedge clk);
      #1;
    end
  endtask

  // One load, launched and then completed on the next cycle with `data` on the
  // bus, and the register value it unpacks to. The load takes exactly the two
  // cycles an add takes: no turnaround anywhere.
  task automatic load_unpacks(input string what, input logic [4:0] widths,
                              input logic [31:0] addr, input logic [31:0] data,
                              input logic [31:0] expected);
    begin
      present(1'b1, addr);
      {launch.is_lw, launch.is_lhu, launch.is_lh, launch.is_lbu, launch.is_lb} = widths;
      arrive(1'b0, 32'b0);
      @(posedge clk);
      #1;
      present(1'b0, 32'b0);
      arrive(1'b1, 32'hdead_beef);
      mem_rdata = data;
      @(posedge clk);
      #1;
      check_bit({what, " completes"}, out.valid, 1'b1);
      check_hex(what, out.rd_data, expected);
      arrive(1'b0, 32'b0);
      @(posedge clk);
      #1;
    end
  endtask

  // One AMO, from its read to its write to the value it hands back. Bit 0 of
  // `ops` is amoswap and the rest follow the order the result mux lists them.
  // The transaction count is taken across the whole thing: an AMO is one read
  // and one write, and two writes are one write for RAM and are not one for a
  // device.
  task automatic amo_does(input string what, input logic [8:0] ops,
                          input logic [31:0] addr, input logic [31:0] old,
                          input logic [31:0] arg, input logic [31:0] expected);
    begin
      transactions = 0;
      present(1'b1, addr);
      launch.rs2 = arg;
      {launch.is_amomaxu, launch.is_amominu, launch.is_amomax, launch.is_amomin,
       launch.is_amoor, launch.is_amoand, launch.is_amoxor, launch.is_amoadd,
       launch.is_amoswap} = ops;
      // Decode publishes the AMO bit beside the nine functions rather than
      // leaving each reader to OR them, so a launch that named a function
      // without it is a struct no decoder produces.
      launch.is_amo = |ops;
      arrive(1'b0, 32'b0);
      mem_rdata = old;
      #1;
      check_bit({what, " reads first"}, mem_ren, 1'b1);
      check_hex({what, " reads the named word"}, mem_addr, addr);
      check_bit({what, " writes nothing on its read cycle"}, |mem_wstrb, 1'b0);
      // The lock is raised with the READ, because an arbiter that registers its
      // grant has to be told a cycle before the one it covers.
      check_bit({what, " locks the bus on its read cycle"}, mem_lock, 1'b1);
      @(posedge clk);
      #1;
      // Decode spends this cycle, so nothing is launched into the accessor
      // while it owns the bus.
      present(1'b0, 32'b0);
      arrive(1'b1, 32'hdead_beef);
      #1;
      check_hex({what, " writes back on the next cycle"}, {28'b0, mem_wstrb}, 32'hf);
      check_hex({what, " writes at the same word"}, mem_addr, addr);
      check_hex(what, mem_wdata, expected);
      check_bit({what, " does not read twice"}, mem_ren, 1'b0);
      // ...and dropped on the cycle it bought. A lock still up here would be a
      // second cycle of the bus this instruction has no use for, and a hart
      // able to hold a shared bus for as long as it liked.
      check_bit({what, " holds the lock for that one cycle only"}, mem_lock, 1'b0);
      @(posedge clk);
      #1;
      check_bit({what, " retires"}, out.valid, 1'b1);
      check_hex({what, " returns the word it found"}, out.rd_data, old);
      arrive(1'b0, 32'b0);
      @(posedge clk);
      #1;
      check_int({what, " is one read and one write"}, transactions, 2);
    end
  endtask

  // One store-conditional, presented for its launch cycle. `expected_fail` is
  // the bit it writes to rd: 0 for the store that went out, 1 for the one that
  // did not.
  task automatic sc_does(input string what, input logic [31:0] addr,
                         input logic [31:0] arg, input logic expected_fail);
    begin
      transactions = 0;
      present(1'b1, addr);
      launch.rs2 = arg;
      launch.is_sc = 1'b1;
      arrive(1'b0, 32'b0);
      #1;
      check_bit({what, " never reads"}, mem_ren, 1'b0);
      // One transaction, so there is no second cycle to keep the bus for.
      check_bit({what, " locks nothing"}, mem_lock, 1'b0);
      if (expected_fail) begin
        check_hex({what, " puts nothing on the bus"}, {28'b0, mem_wstrb}, 32'b0);
        check_hex({what, " drives no address either"}, mem_addr, 32'b0);
      end else begin
        check_hex({what, " stores a whole word"}, {28'b0, mem_wstrb}, 32'hf);
        check_hex({what, " at the named address"}, mem_addr, addr);
        check_hex({what, " of rs2"}, mem_wdata, arg);
      end
      @(posedge clk);
      #1;
      present(1'b0, 32'b0);
      arrive(1'b1, 32'hdead_beef);
      @(posedge clk);
      #1;
      check_bit({what, " retires"}, out.valid, 1'b1);
      check_hex({what, " reports it in rd"}, out.rd_data, {31'b0, expected_fail});
      check_int({what, " costs the bus what it says it does"},
                transactions, expected_fail ? 0 : 1);
      arrive(1'b0, 32'b0);
      @(posedge clk);
      #1;
    end
  endtask

  // One load-reserved, which is a word load that leaves a reservation behind.
  task automatic lr_takes(input logic [31:0] addr, input logic [31:0] word);
    begin
      present(1'b1, addr);
      launch.is_lr = 1'b1;
      arrive(1'b0, 32'b0);
      mem_rdata = word;
      @(posedge clk);
      #1;
      present(1'b0, 32'b0);
      arrive(1'b1, 32'hdead_beef);
      @(posedge clk);
      #1;
      arrive(1'b0, 32'b0);
      @(posedge clk);
      #1;
    end
  endtask

  // Another bus master's write, for one cycle, with this core launching
  // nothing. That is the shape a snoop really has: the other hart owns the bus
  // on the cycle it writes, so this one is not requesting on it.
  task automatic snoop_writes(input logic [31:0] addr);
    begin
      present(1'b0, 32'b0);
      snoop_write = 1'b1;
      snoop_addr = addr;
      transactions = 0;
      @(posedge clk);
      #1;
      check_int("a foreign write puts nothing of this core's on the bus",
                transactions, 0);
      snoop_write = 1'b0;
      snoop_addr = 32'b0;
      @(posedge clk);
      #1;
    end
  endtask

  // One plain store, used only to invalidate a reservation.
  task automatic store_word(input logic [31:0] addr);
    begin
      present(1'b1, addr);
      launch.is_sw = 1'b1;
      @(posedge clk);
      #1;
      present(1'b0, 32'b0);
      @(posedge clk);
      #1;
    end
  endtask

  initial begin
    reset = 1;
    present(1'b1, 32'h0000_0100);
    launch.is_lw = 1'b1;
    arrive(1'b0, 32'b0);
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
    check_bit("...and is not a write", |mem_wstrb, 1'b0);

    // The response cycle. The request has moved on, so the enable must fall or
    // the answer's own cycle would take a fetch window of its own.
    @(posedge clk);
    #1;
    present(1'b0, 32'b0);
    arrive(1'b1, 32'hdead_beef);
    #1;
    check_bit("the load's own response cycle drops it", mem_ren, 1'b0);
    @(posedge clk);
    #1;
    check_bit("the load completed one cycle after its request", out.valid, 1'b1);
    check_hex("...with the bus answer, not the executor's rd_data",
              out.rd_data, 32'hcafef00d);
    arrive(1'b0, 32'b0);

    // Every other bus cycle. A store holds the write port instead, and
    // rtl/imemory.v steals on the strobe for that -- not on this.
    present(1'b1, 32'h0000_0200);
    launch.is_sw = 1'b1;
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
    launch.is_lb = 1'b1;
    #1;
    check_bit("a bubble carrying a load flag does not raise it", mem_ren, 1'b0);
    @(posedge clk);
    #1;

    // An ALU result reaches `out` unchanged: only a load takes the bus answer.
    present(1'b0, 32'b0);
    arrive(1'b1, 32'h0bad_f00d);
    @(posedge clk);
    #1;
    check_bit("a non-load retires", out.valid, 1'b1);
    check_hex("...carrying the executor's result", out.rd_data, 32'h0bad_f00d);
    arrive(1'b0, 32'b0);
    @(posedge clk);
    #1;

    // The issued-once guard. Decode holds `launch` for every cycle of a divide
    // and the executor takes it on exactly one, so a store to a device is
    // written once. Without `launch_taken` in the request block this counts
    // eight instead of one, and no value anywhere in the machine differs.
    transactions = 0;
    present(1'b1, 32'h0002_0010);   // mtimecmp low
    launch.is_sw = 1'b1;
    launch_taken = 1'b0;
    repeat (7) begin
      @(posedge clk);
      #1;
      check_bit("a held store presents nothing while the divide runs", |mem_wstrb, 1'b0);
      check_hex("...and drives no address either", mem_addr, 32'b0);
    end
    launch_taken = 1'b1;
    #1;
    check_hex("the cycle the executor takes it, the store goes out",
              {28'b0, mem_wstrb}, 32'hf);
    check_hex("...at the word-aligned device address", mem_addr, 32'h0002_0010);
    @(posedge clk);
    #1;
    present(1'b0, 32'b0);
    @(posedge clk);
    #1;
    check_int("one store to mtimecmp under a divide is one bus transaction",
              transactions, 1);

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

    //-----------------------------------------------------------------------
    // The nine AMO functions. Each is driven at operands where getting the
    // shared adder/subtractor wrong is visible: the min/max family straddles
    // zero so a signed compare read as unsigned lands on the other operand,
    // and the unsigned pair uses the same two words so the two families
    // disagree about which one is larger.
    //-----------------------------------------------------------------------
    amo_does("amoswap.w", 9'b000000001, 32'h0001_0020, 32'h1111_2222, 32'h3333_4444,
             32'h3333_4444);
    amo_does("amoadd.w",  9'b000000010, 32'h0001_0020, 32'hffff_ffff, 32'h0000_0002,
             32'h0000_0001);
    amo_does("amoxor.w",  9'b000000100, 32'h0001_0020, 32'hf0f0_ff00, 32'h0ff0_00ff,
             32'hff00_ffff);
    amo_does("amoand.w",  9'b000001000, 32'h0001_0020, 32'hf0f0_ff00, 32'h0ff0_00ff,
             32'h00f0_0000);
    amo_does("amoor.w",   9'b000010000, 32'h0001_0020, 32'hf0f0_ff00, 32'h0ff0_00ff,
             32'hfff0_ffff);
    // -1 against 1. Signed, memory is the smaller; unsigned, it is the larger.
    amo_does("amomin.w",  9'b000100000, 32'h0001_0020, 32'hffff_ffff, 32'h0000_0001,
             32'hffff_ffff);
    amo_does("amomax.w",  9'b001000000, 32'h0001_0020, 32'hffff_ffff, 32'h0000_0001,
             32'h0000_0001);
    amo_does("amominu.w", 9'b010000000, 32'h0001_0020, 32'hffff_ffff, 32'h0000_0001,
             32'h0000_0001);
    amo_does("amomaxu.w", 9'b100000000, 32'h0001_0020, 32'hffff_ffff, 32'h0000_0001,
             32'hffff_ffff);
    // The other direction of each compare, so none of the four passes by
    // always picking the same operand.
    amo_does("amomin.w  the other way",  9'b000100000, 32'h0001_0020, 32'h0000_0007,
             32'h0000_0009, 32'h0000_0007);
    amo_does("amomax.w  the other way",  9'b001000000, 32'h0001_0020, 32'h0000_0007,
             32'h0000_0009, 32'h0000_0009);
    amo_does("amominu.w the other way",  9'b010000000, 32'h0001_0020, 32'h0000_0009,
             32'h0000_0007, 32'h0000_0007);
    amo_does("amomaxu.w the other way",  9'b100000000, 32'h0001_0020, 32'h0000_0009,
             32'h0000_0007, 32'h0000_0009);
    // Equal operands: min and max must both leave the word alone rather than
    // depending on which side a strict comparison falls.
    amo_does("amomin.w  on equal operands", 9'b000100000, 32'h0001_0020, 32'h1234_5678,
             32'h1234_5678, 32'h1234_5678);
    amo_does("amomax.w  on equal operands", 9'b001000000, 32'h0001_0020, 32'h1234_5678,
             32'h1234_5678, 32'h1234_5678);

    // Every function that is one bit function of a memory bit and an rs2 bit
    // shares one four-entry table, so each of these drives all four combinations
    // of that pair at eight bits apiece and no entry is graded by fewer. Both
    // operands are negative, which is why min takes rs2 and max the memory word.
    amo_does("amoswap.w over all four operand-bit pairs", 9'b000000001, 32'h0001_0020,
             32'hcccc_cccc, 32'haaaa_aaaa, 32'haaaa_aaaa);
    amo_does("amoxor.w  over all four operand-bit pairs", 9'b000000100, 32'h0001_0020,
             32'hcccc_cccc, 32'haaaa_aaaa, 32'h6666_6666);
    amo_does("amoand.w  over all four operand-bit pairs", 9'b000001000, 32'h0001_0020,
             32'hcccc_cccc, 32'haaaa_aaaa, 32'h8888_8888);
    amo_does("amoor.w   over all four operand-bit pairs", 9'b000010000, 32'h0001_0020,
             32'hcccc_cccc, 32'haaaa_aaaa, 32'heeee_eeee);
    amo_does("amomin.w  over all four operand-bit pairs", 9'b000100000, 32'h0001_0020,
             32'hcccc_cccc, 32'haaaa_aaaa, 32'haaaa_aaaa);
    amo_does("amomax.w  over all four operand-bit pairs", 9'b001000000, 32'h0001_0020,
             32'hcccc_cccc, 32'haaaa_aaaa, 32'hcccc_cccc);

    // The issued-once guard for an AMO. Decode holds `launch` for every cycle
    // of a divide, and a request block that read it without `launch_taken`
    // would read the same word thirty-three times and write it back as many.
    transactions = 0;
    present(1'b1, 32'h0001_0030);
    launch.is_amoadd = 1'b1;
    launch.is_amo = 1'b1;
    launch.rs2 = 32'd1;
    launch_taken = 1'b0;
    mem_rdata = 32'd41;
    repeat (5) begin
      @(posedge clk);
      #1;
      check_bit("a held AMO presents nothing while the divide runs", mem_ren, 1'b0);
      check_hex("...and drives no address either", mem_addr, 32'b0);
    end
    launch_taken = 1'b1;
    #1;
    check_bit("the cycle the executor takes it, the read goes out", mem_ren, 1'b1);
    @(posedge clk);
    #1;
    present(1'b0, 32'b0);
    arrive(1'b1, 32'hdead_beef);
    #1;
    check_hex("...and the write follows on the next cycle", mem_wdata, 32'd42);
    @(posedge clk);
    #1;
    arrive(1'b0, 32'b0);
    @(posedge clk);
    #1;
    check_int("one AMO under a divide is one read and one write", transactions, 2);

    //-----------------------------------------------------------------------
    // The reservation. Every line here is a store-conditional's result, which
    // is the only thing about a reservation a program can observe.
    //-----------------------------------------------------------------------
    lr_takes(32'h0001_0040, 32'h0000_0001);
    sc_does("sc.w behind its own lr.w", 32'h0001_0040, 32'h5555_6666, 1'b0);
    // ...and the reservation is gone afterwards, whether the store happened or
    // not. Without this a lock could be taken twice.
    sc_does("a second sc.w with nothing reserved", 32'h0001_0040, 32'h5555_6666, 1'b1);

    lr_takes(32'h0001_0040, 32'h0000_0001);
    sc_does("sc.w one word up from the reservation", 32'h0001_0044, 32'h5555_6666, 1'b1);

    lr_takes(32'h0001_0040, 32'h0000_0001);
    store_word(32'h0001_0040);
    sc_does("sc.w after a store to the reserved word", 32'h0001_0040, 32'h5555_6666, 1'b1);

    // A store somewhere else leaves it alone. Without this the vector above
    // passes on a design that clears the reservation on every store.
    lr_takes(32'h0001_0040, 32'h0000_0001);
    store_word(32'h0001_0048);
    sc_does("...but a store elsewhere does not disturb it",
            32'h0001_0040, 32'h5555_6666, 1'b0);

    // An AMO is a write too, so it invalidates a reservation on its own word.
    lr_takes(32'h0001_0040, 32'h0000_0001);
    amo_does("amoadd.w over the reserved word", 9'b000000010, 32'h0001_0040,
             32'h0000_0001, 32'h0000_0001, 32'h0000_0002);
    sc_does("...and the reservation does not survive it",
            32'h0001_0040, 32'h5555_6666, 1'b1);

    // Another master's write to the reserved word ends the reservation, which
    // is the whole of what a second hart needs from this module. The failing
    // store-conditional puts nothing on the bus, and `sc_does` counts that: a
    // reservation cleared is a transaction not made, not just a bit in rd.
    lr_takes(32'h0001_0040, 32'h0000_0001);
    snoop_writes(32'h0001_0040);
    sc_does("sc.w after another master wrote the reserved word",
            32'h0001_0040, 32'h5555_6666, 1'b1);

    // ...and a foreign write elsewhere leaves it alone. Without this the vector
    // above passes on a design that clears on any snoop at all, which would
    // fail every store-conditional the moment a second hart touched memory.
    lr_takes(32'h0001_0040, 32'h0000_0001);
    snoop_writes(32'h0001_0048);
    sc_does("...but another master's write elsewhere does not disturb it",
            32'h0001_0040, 32'h5555_6666, 1'b0);

    // The region attribute. A platform that will not answer an address will
    // not hold a reservation at it either, so the store-conditional that would
    // otherwise report success for a write going nowhere fails instead.
    mem_reservable = 1'b0;
    lr_takes(32'h0009_0000, 32'h0000_0001);
    mem_reservable = 1'b1;
    sc_does("sc.w after an lr.w outside reservable memory",
            32'h0009_0000, 32'h5555_6666, 1'b1);

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: accessor read enable, load unpack, the AMO datapath, the reservation under this core's writes and another master's, the bus lock and the one-transaction guard");
      $finish;
    end
  end
endmodule
