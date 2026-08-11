`timescale 1 ns / 1 ps
`default_nettype none
// What followed each instruction last time it issued, in one block RAM.
//
// Decode presents a register-number pair a cycle before it needs the operands
// and guesses that pair from the fetch window's successor word. That word is
// the instruction physically after the one issuing, which is the wrong one
// every time the flow is redirected. This answers the same question from
// history instead: the entry for `pc` holds the pair the instruction that
// really ran next read, so a branch that goes the same way twice costs no
// operand-fetch cycle on the second pass.
//
// Nothing here can be wrong in a way that matters. Decode compares what it
// presented against what the issuing instruction reads and bubbles a cycle when
// they differ, so a miss costs the cycle that was being paid anyway and a hit
// saves it. That is why there is no flush, no invalidation and no clearing on a
// self-modifying store.
//
// The read is addressed off `next_pc` on the same edge `imem_addr_next` is
// latched, so the entry arrives with the instruction it describes and the
// select downstream is a comparison of two registers.
module pairtable #(
  // Both are widths of `pc`, so together they say how much text this
  // disambiguates: 2**INDEX_BITS halfwords indexed, 2**(INDEX_BITS+TAG_BITS)
  // halfwords distinct. At 8 and 6 that is 512 bytes of index and 32 KB of
  // distinct text, which is four times the SoC's whole ROM -- so no two
  // addresses a program can execute share an entry, and the tag turns a
  // collision into a miss rather than into another instruction's pair.
  parameter integer INDEX_BITS = 8,
  parameter integer TAG_BITS   = 6
) (
  input  logic        clk,
  input  logic        reset,
  // The address the fetch is about to be taken from. Latched here on the same
  // edge the instruction memory latches it.
  input  logic [31:0] next_pc,
  // The instruction being issued this cycle: its own address, and the pair it
  // reads. The pair is stored against the PREVIOUS issue's address, because
  // that is the question the entry answers.
  input  logic        issuing,
  input  logic [31:0] pc,
  input  logic [4:0]  rs1,
  input  logic [4:0]  rs2,
  // The entry for the instruction issuing this cycle, and whether it is one.
  output logic        hit,
  output logic [4:0]  guess_rs1,
  output logic [4:0]  guess_rs2
);
  localparam int ENTRIES = 1 << INDEX_BITS;
  localparam int WIDTH   = TAG_BITS + 10;

  // A block RAM this wide and this deep is exactly one SB_RAM40_4K. Widening
  // the entry past 16 bits, or deepening it past 256, costs a second one and
  // buys nothing measured.
  if (WIDTH > 16) begin : l_entry_fits_one_ebr
    $fatal(1, "pairtable: TAG_BITS + 10 must fit a 256-deep block RAM's 16 bits");
  end

  logic [WIDTH-1:0] entries[ENTRIES-1:0];

  // An EBR with no INIT comes out of the bitstream as zeros, and a simulation
  // whose array starts undefined is not a model of one: the X would reach the
  // register file's address port through the guess below and turn the whole
  // pipeline X. Only the two-state runner would stay green. Zeroed, an unwritten
  // entry reads as tag 0 with the pair x0/x0, which is a guess like any other.
  initial begin
    for (int i = 0; i < ENTRIES; i++) entries[i] = '0;
  end

  logic [INDEX_BITS-1:0] read_index, write_index;
  logic [TAG_BITS-1:0]   read_tag, write_tag;
  // pc[0] is always zero -- every instruction is two-byte aligned -- so the
  // index starts at bit 1 and the table is one entry per halfword rather than
  // one per four bytes, which would alias every compressed pair together.
  assign read_index = next_pc[INDEX_BITS:1];
  assign read_tag   = next_pc[INDEX_BITS+TAG_BITS:INDEX_BITS+1];

  logic [WIDTH-1:0]    read_entry;
  logic [TAG_BITS-1:0] held_tag;
  logic                held_valid;

  // The address of the last instruction to issue, which is the entry the next
  // issue's pair belongs in. `primed` keeps the first issue out of reset from
  // writing a pair against address zero, which no instruction has issued from.
  logic [INDEX_BITS-1:0] prev_index;
  logic [TAG_BITS-1:0]   prev_tag;
  logic                  primed;

  always_ff @(posedge clk) begin
    read_entry <= entries[read_index];
    held_tag   <= read_tag;
    held_valid <= !reset;
    if (reset) begin
      primed     <= 1'b0;
      prev_index <= '0;
      prev_tag   <= '0;
    end else if (issuing) begin
      if (primed) entries[prev_index] <= {prev_tag, rs1, rs2};
      primed     <= 1'b1;
      prev_index <= pc[INDEX_BITS:1];
      prev_tag   <= pc[INDEX_BITS+TAG_BITS:INDEX_BITS+1];
    end
  end

  assign hit       = held_valid && read_entry[WIDTH-1:10] == held_tag;
  assign guess_rs1 = read_entry[9:5];
  assign guess_rs2 = read_entry[4:0];
endmodule
