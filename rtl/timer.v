`timescale 1 ns / 1 ps
`default_nettype none
// The machine timer, on the data bus rather than in the CSR file. `mtime` is a
// free-running 64-bit counter and `mtimecmp` is the value it is compared
// against, unsigned; `mtip` is high for as long as `mtime >= mtimecmp`. That is
// a LEVEL, not a pulse: it stays posted until `mtimecmp` becomes greater than
// `mtime`, and taking the trap does not lower it. Moving `mtimecmp` forward is
// the only thing that does, which is why a handler that returns without doing
// so is re-entered at once.
//
// THE TICK PERIOD IS ONE CLOCK CYCLE, so on the shipping SoC's 12 MHz crystal
// an `mtime` tick is 83.33 ns. The privileged spec requires only that `mtime`
// advance at a constant frequency and that the platform publish the period; it
// names a fixed-frequency system with no frequency scaling as the case where
// driving it from the cycle counter is the right answer, which is this board.
// Firmware cannot work the period out from anywhere else, so it is written
// here.
//
// Four words, low half first:
//
//   BASE+0  mtime      BASE+4  mtimeh
//   BASE+8  mtimecmp   BASE+c  mtimecmph
//
// A 32-bit store touches one half, so an update to `mtimecmp` is a sequence and
// its order matters. The spec's own RV32 sample writes the low half all ones,
// then the high half, then the low half; test/timer_tb.v fires a spurious
// interrupt with the naive order to show the difference is real.
//
// `mtip` is REGISTERED, and that is what keeps this cheap. It reaches the core
// as one input of the decoder's trap term, which is on the fetch loop; an
// unregistered comparison would put a 64-bit carry chain there instead. Any
// other platform driving `irq_timer` owes the core the same.
//
// A write beats that cycle's increment of `mtime`, for the reason rtl/csrs.v
// gives about `mcycle`: the two halves are one register, so a write to either
// suppresses the carry as well as the half it names.
module timer #(
  // Above rtl/memory.v's 64 KB at 0x0001_0000, so the three ranges on the
  // shared bus do not overlap and the read buses can be ORed together.
  parameter logic [31:0] BASE = 32'h0002_0000
) (
  input  logic        clk,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  output logic        mtip
);
  logic [63:0] mtime, mtimecmp;

  // The four words are an aligned 16-byte window, so membership is an equality
  // on the bits above it and the register select is two bits of the address
  // itself -- neither needs the subtraction they replace. Both are true only
  // while BASE is 16-byte aligned: off it the equality names a window the timer
  // does not occupy and `word` selects the wrong register, where the
  // subtract-and-compare was merely slow. So it is an elaboration check rather
  // than a comment.
  if (|BASE[3:0]) begin : l_base_aligned
    $fatal(1, "timer: BASE must be 16-byte aligned");
  end

  logic in_range;
  assign in_range = mem_addr[31:4] == BASE[31:4];
  logic [1:0] word;
  assign word = mem_addr[3:2];

  logic writing;
  assign writing = in_range && |mem_wstrb;

  logic wr_time_lo, wr_time_hi, wr_cmp_lo, wr_cmp_hi;
  assign wr_time_lo = writing && word == 2'd0;
  assign wr_time_hi = writing && word == 2'd1;
  assign wr_cmp_lo  = writing && word == 2'd2;
  assign wr_cmp_hi  = writing && word == 2'd3;

  // Selected out here rather than inside the always blocks below: a constant
  // part-select taken inside one defeats iverilog's sensitivity analysis and
  // draws a `sorry:` note. Use a named continuous assign for any added later.
  logic [31:0] mtime_lo, mtime_hi, mtimecmp_lo, mtimecmp_hi;
  assign mtime_lo    = mtime[31:0];
  assign mtime_hi    = mtime[63:32];
  assign mtimecmp_lo = mtimecmp[31:0];
  assign mtimecmp_hi = mtimecmp[63:32];

  logic [31:0] read_word;
  always_comb begin
    (* parallel_case *)
    case (word)
      2'd0:    read_word = mtime_lo;
      2'd1:    read_word = mtime_hi;
      2'd2:    read_word = mtimecmp_lo;
      default: read_word = mtimecmp_hi;
    endcase
  end

  logic [63:0] mtime_next;
  assign mtime_next = (wr_time_lo || wr_time_hi) ? mtime : mtime + 64'd1;

  always_ff @(posedge clk) begin
    if (reset) begin
      mtime    <= 64'b0;
      // Zero, which means `mtip` is asserted from the first cycle. That is what
      // the comparison says, and it is harmless: mstatus.MIE and mie.MTIE both
      // reset to zero in rtl/csrs.v, so nothing is taken until software has
      // enabled it -- and software must set mtimecmp before it does, which is
      // the ordering every RISC-V platform requires anyway.
      //
      // Every reset value in this file is zero for a second reason: the cxxrtl
      // runner deasserts reset before the first rising edge, so the whole `.S`
      // suite runs on cxxrtl's zero-initialised state rather than on this
      // branch. A non-zero reset value here would be invisible on the primary
      // simulator and present on the board.
      mtimecmp  <= 64'b0;
      mtip      <= 1'b0;
      mem_rdata <= 32'b0;
    end else begin
      mtime <= mtime_next;
      if (wr_time_lo) begin
        if (mem_wstrb[0]) mtime[7:0]    <= mem_wdata[7:0];
        if (mem_wstrb[1]) mtime[15:8]   <= mem_wdata[15:8];
        if (mem_wstrb[2]) mtime[23:16]  <= mem_wdata[23:16];
        if (mem_wstrb[3]) mtime[31:24]  <= mem_wdata[31:24];
      end
      if (wr_time_hi) begin
        if (mem_wstrb[0]) mtime[39:32]  <= mem_wdata[7:0];
        if (mem_wstrb[1]) mtime[47:40]  <= mem_wdata[15:8];
        if (mem_wstrb[2]) mtime[55:48]  <= mem_wdata[23:16];
        if (mem_wstrb[3]) mtime[63:56]  <= mem_wdata[31:24];
      end
      if (wr_cmp_lo) begin
        if (mem_wstrb[0]) mtimecmp[7:0]   <= mem_wdata[7:0];
        if (mem_wstrb[1]) mtimecmp[15:8]  <= mem_wdata[15:8];
        if (mem_wstrb[2]) mtimecmp[23:16] <= mem_wdata[23:16];
        if (mem_wstrb[3]) mtimecmp[31:24] <= mem_wdata[31:24];
      end
      if (wr_cmp_hi) begin
        if (mem_wstrb[0]) mtimecmp[39:32] <= mem_wdata[7:0];
        if (mem_wstrb[1]) mtimecmp[47:40] <= mem_wdata[15:8];
        if (mem_wstrb[2]) mtimecmp[55:48] <= mem_wdata[23:16];
        if (mem_wstrb[3]) mtimecmp[63:56] <= mem_wdata[31:24];
      end
      // Both sides come out of flip-flops, so a write to `mtimecmp` is visible
      // here one cycle later. Comparing against the value the write is
      // installing would put `mem_wdata` in front of a 64-bit carry chain, and
      // the cycle buys nothing: an `mret` cannot commit until the pipeline is
      // empty, which is several cycles after the handler's store lands.
      mtip <= mtime_next >= mtimecmp;
      // An out-of-range access reads zero, so the three memories on this bus
      // join with an OR rather than a mux.
      mem_rdata <= in_range ? read_word : 32'b0;
    end
  end
endmodule
