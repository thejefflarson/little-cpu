`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// The fetch address is built from registers alone. Decode reads the window at `pc` off
// the ROM's output register or off `skid`, the one window fetch has moved past.
// `predicted_taken`/`predicted_target_low` are D's own guess, riding `dx_out`.
module fetcher (
  input  logic clk,
  input  logic reset,
  input  logic [31:0] pc,
  input  logic [31:0] next_pc,
  input  logic        issuing,
  input  logic        predicted_taken,
  input  logic [7:0] predicted_target_low,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
  output logic [31:0] imem_addr_next,
  input  logic        imem_stall,
  input  logic        imem_fault,
  output logic        fetch_stall,
  output logic        fault,
  output fetcher_output out
);
  logic [29:0] word, rom_addr, fetch_word;
  logic        rom_hit, hit, pop, skid_load, capture, skid_valid, skid_fault;
  logic [31:0] skid_lo, skid_hi;

  assign word     = pc[31:2];
  assign rom_hit  = !imem_stall && rom_addr == word;
  assign hit      = skid_valid || rom_hit;
  assign pop      = next_pc[31:2] != word;
  // The valid bit alone reads decode's answer, so `next_pc` reaches one flop, not two.
  assign skid_load = rom_hit && !skid_valid;
  assign capture   = skid_load && !pop;
  assign fetch_stall = !hit;

  assign fetch_word = reset          ? 30'd0 :
                      predicted_taken ? {24'b0, predicted_target_low[7:2]} :
                                        word + {29'b0, hit};
  assign imem_addr_next = {fetch_word, 2'b00};
  assign imem_addr      = {rom_addr, 2'b00};
  assign imem_addr2     = imem_addr + 32'd4;

  always_ff @(posedge clk) begin
    rom_addr <= fetch_word;
    if (reset) skid_valid <= 1'b0;
    else       skid_valid <= capture || (skid_valid && !pop);
    if (skid_load) begin
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
  logic [31:0] windowed_instr;
  assign windowed_instr = fetch_pair[31:0];

  always_comb begin
    if (reset) begin
      out.valid = 1'b0;
      out.pc = 32'b0;
      out.instr = 32'b0;
    end else begin
      out.valid = 1'b1;
      out.instr = windowed_instr;
      out.pc = pc;
    end
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

  logic prev_fetch_stall, prev_predicted_taken;
  always_ff @(posedge clk) prev_fetch_stall     <= fetch_stall;
  always_ff @(posedge clk) prev_predicted_taken <= predicted_taken;
  always_ff @(posedge clk) if (clocked && !reset) begin
    skid_read:   cover (skid_valid && issuing);
    miss_then_hit: cover (prev_fetch_stall && !fetch_stall);
    guess_served: cover (prev_predicted_taken && !fetch_stall);
  end
 `endif
endmodule
