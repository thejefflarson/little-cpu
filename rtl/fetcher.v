`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// The ROM answers a word address a cycle after `imem_addr_next` presents it, and that
// address is built from registers alone: decode reads the window at `pc` out of the
// ROM's own output register, or out of `skid`, the one window fetch has moved past.
// A guess formed a cycle early steers the ROM to a jump's target; a wrong guess or an
// unguessed redirect is a miss, and a miss costs one cycle and un-commits nothing.
module fetcher(
  input  logic clk,
  input  logic reset,
  input  logic [31:0] pc,
  input  logic [31:0] next_pc,
  input  logic        issuing,
  input  logic        redirect,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  output logic [31:0] imem_addr_next,
  // The read `imem_addr_next` asked for last cycle went to a load or store instead.
  input  logic        imem_stall,
  input  logic        imem_fault,
  output logic        fetch_stall,
  output logic        fault,
  output fetcher_output out
);
  logic [29:0] word, rom_addr, fetch_word;
  logic        rom_hit, hit, pop, capture, skid_valid, skid_fault;
  logic [31:0] skid_lo, skid_hi;
  logic        guess_valid;
  logic [31:0] guess_target;

  assign word     = pc[31:2];
  assign rom_hit  = !imem_stall && rom_addr == word;
  assign hit      = skid_valid || rom_hit;
  assign pop      = next_pc[31:2] != word;
  assign capture  = rom_hit && !skid_valid && !pop;
  assign fetch_stall = !hit;

  // A miss re-reads decode's own word; a hit reads the word after it, which is also
  // the word the skid's successor needs, unless a guess says where decode goes instead.
  assign fetch_word = reset ? 30'd0 :
                      hit && guess_valid ? guess_target[31:2] : word + {29'b0, hit};
  assign imem_addr_next = {fetch_word, 2'b00};
  assign imem_addr      = {rom_addr, 2'b00};
  assign imem_addr2     = imem_addr + 32'd4;

  always_ff @(posedge clk) begin
    rom_addr <= fetch_word;
    if (reset) skid_valid <= 1'b0;
    else       skid_valid <= capture || (skid_valid && !pop);
    if (capture) begin
      skid_lo    <= imem_data;
      skid_hi    <= imem_data2;
      skid_fault <= imem_fault;
    end
  end

  logic [31:0] win_lo, win_hi;
  assign win_lo = skid_valid ? skid_lo : imem_data;
  assign win_hi = skid_valid ? skid_hi : imem_data2;
  assign fault  = skid_valid ? skid_fault : imem_fault;

  logic [63:0] fetch_pair;
  assign fetch_pair = {win_hi, win_lo} >> (pc[1] ? 16 : 0);
  logic [31:0] windowed_instr, next_word;
  logic        uncompressed;
  assign windowed_instr = fetch_pair[31:0];
  assign uncompressed   = windowed_instr[1:0] == 2'b11;
  assign next_word = uncompressed ? fetch_pair[63:32] : fetch_pair[47:16];

  always_comb begin
    if (reset) begin
      out.valid = 1'b0;
      out.pc = 32'b0;
      out.instr = 32'b0;
      out.next_instr = 32'b0;
    end else begin
      out.valid = 1'b1;
      out.instr = windowed_instr;
      out.next_instr = next_word;
      out.pc = pc;
    end
  end

  // A jal or a backward branch next in line is guessed taken, off its own immediate.
  // A word-straddling instruction leaves `next_word`'s upper half empty, so no guess.
  logic n_jal, n_branch, n_cj, n_cb, candidate, next_whole;
  assign n_jal    = next_word[1:0] == 2'b11 && next_word[6:2] == 5'b11011;
  assign n_branch = next_word[1:0] == 2'b11 && next_word[6:2] == 5'b11000;
  assign n_cj     = next_word[1:0] == 2'b01 && next_word[14:13] == 2'b01;
  assign n_cb     = next_word[1:0] == 2'b01 && next_word[15:14] == 2'b11;
  assign candidate  = n_jal || (n_branch && next_word[31]) || n_cj || (n_cb && next_word[12]);
  assign next_whole = !(uncompressed && pc[1]);

  logic [31:0] n_imm, j_imm, b_imm, cj_imm, cb_imm;
  assign j_imm  = {{12{next_word[31]}}, next_word[19:12], next_word[20], next_word[30:21],
                   1'b0};
  assign b_imm  = {{20{next_word[31]}}, next_word[7], next_word[30:25], next_word[11:8],
                   1'b0};
  assign cj_imm = {{20{next_word[12]}}, next_word[12], next_word[8], next_word[10],
                   next_word[9], next_word[6], next_word[7], next_word[2], next_word[11],
                   next_word[5], next_word[4], next_word[3], 1'b0};
  assign cb_imm = {{23{next_word[12]}}, next_word[12], next_word[6:5], next_word[2],
                   next_word[11:10], next_word[4:3], 1'b0};
  assign n_imm  = n_jal ? j_imm : n_branch ? b_imm : n_cj ? cj_imm : cb_imm;

  logic [31:0] seq_pc;
  assign seq_pc = pc + (uncompressed ? 32'd4 : 32'd2);

  always_ff @(posedge clk) begin
    if (reset)        guess_valid <= 1'b0;
    else if (issuing) guess_valid <= candidate && next_whole && !redirect;
    if (issuing) guess_target <= seq_pc + n_imm;
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;

  logic [29:0] skid_word, past_fetch_word;
  always_ff @(posedge clk) begin
    if (capture) skid_word <= rom_addr;
    past_fetch_word <= fetch_word;
  end
  always_comb if (clocked) assert(rom_addr == past_fetch_word);
  always_comb if (clocked && skid_valid) assert(skid_word == word);
  always_comb if (clocked && !fetch_stall)
    assert((skid_valid ? skid_word : rom_addr) == word);
 `endif
endmodule
