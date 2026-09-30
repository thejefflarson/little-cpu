`timescale 1 ns / 1 ps
`default_nettype none
// The machine timer. `mtip` is a LEVEL, held while `mtime >= mtimecmp`, and the layout --
// `mtime`, then one `mtimecmp` per hart -- is deliberately not a CLINT's.
module timer #(
  parameter logic [31:0] BASE = 32'h0002_0000,
  parameter integer      NHARTS = 1
) (
  input  logic              clk,
  input  logic              reset,
  input  logic [31:0]       mem_addr,
  input  logic [31:0]       mem_wdata,
  input  logic [3:0]        mem_wstrb,
  output logic [31:0]       mem_rdata,
  output logic [NHARTS-1:0] mtip
);
  localparam int WINDOW_WORDS = 1 << $clog2(2 + 2 * NHARTS);
  localparam int SEL_BITS     = $clog2(WINDOW_WORDS);
  localparam int ADDR_LSB     = SEL_BITS + 2;

  logic [63:0] mtime, mtimecmp;

  if (|BASE[ADDR_LSB-1:0]) begin : l_base_aligned
    $fatal(1, "timer: BASE must be aligned to the whole NHARTS-sized window");
  end

  logic in_range;
  assign in_range = mem_addr[31:ADDR_LSB] == BASE[31:ADDR_LSB];
  logic [SEL_BITS-1:0] word;
  assign word = mem_addr[ADDR_LSB-1:2];

  // Stores are latched and land a cycle later; `mtip` only gets later, and a load reads them via `bypassed`.
  logic        writing_q;
  logic [SEL_BITS-1:0] word_q;
  logic [3:0]  wstrb_q;
  logic [31:0] wdata_q;
  always_ff @(posedge clk) begin
    writing_q <= !reset && in_range && |mem_wstrb;
    word_q    <= word;
    wstrb_q   <= mem_wstrb;
    wdata_q   <= mem_wdata;
  end

  logic wr_time_lo, wr_time_hi, wr_cmp_lo, wr_cmp_hi;
  assign wr_time_lo = writing_q && word_q == 2'd0;
  assign wr_time_hi = writing_q && word_q == 2'd1;
  assign wr_cmp_lo  = writing_q && word_q == 2'd2;
  assign wr_cmp_hi  = writing_q && word_q == 2'd3;

  logic [31:0] mtime_lo, mtime_hi, mtimecmp_lo, mtimecmp_hi;
  assign mtime_lo    = mtime[31:0];
  assign mtime_hi    = mtime[63:32];
  assign mtimecmp_lo = mtimecmp[31:0];
  assign mtimecmp_hi = mtimecmp[63:32];

  logic [63:0] mtime_next;
  assign mtime_next = (wr_time_lo || wr_time_hi) ? mtime : mtime + 64'd1;

  // Hart 0 has an arm of its own: folding it into the general mux maps to a different
  // netlist. Change one arm, change both.
  logic [31:0] read_word;
  generate if (NHARTS == 1) begin : l_read_one
    always_comb begin
      (* parallel_case *)
      case (word)
        2'd0:    read_word = mtime_lo;
        2'd1:    read_word = mtime_hi;
        2'd2:    read_word = mtimecmp_lo;
        default: read_word = mtimecmp_hi;
      endcase
    end
  end else begin : l_harts
    logic [32*NHARTS-1:0] read_hart;
    logic [31:0]          read_above;
    assign read_hart[31:0] = 32'b0;
    assign read_above = read_hart[32*(NHARTS-1) +: 32];

    for (genvar h = 1; h < NHARTS; h++) begin : l_hart
      logic [63:0] cmp;
      logic        sel_lo, sel_hi, rsel_lo, rsel_hi;
      assign rsel_lo = word == (2 + 2*h);
      assign rsel_hi = word == (3 + 2*h);
      assign sel_lo = word_q == (2 + 2*h);
      assign sel_hi = word_q == (3 + 2*h);
      assign read_hart[32*h +: 32] = read_hart[32*(h-1) +: 32]
                                   | (rsel_lo ? cmp[31:0]  : 32'b0)
                                   | (rsel_hi ? cmp[63:32] : 32'b0);

      always_ff @(posedge clk) begin
        if (reset) begin
          cmp     <= 64'b0;
          mtip[h] <= 1'b0;
        end else begin
          if (writing_q && sel_lo) begin
            if (wstrb_q[0]) cmp[7:0]   <= wdata_q[7:0];
            if (wstrb_q[1]) cmp[15:8]  <= wdata_q[15:8];
            if (wstrb_q[2]) cmp[23:16] <= wdata_q[23:16];
            if (wstrb_q[3]) cmp[31:24] <= wdata_q[31:24];
          end
          if (writing_q && sel_hi) begin
            if (wstrb_q[0]) cmp[39:32] <= wdata_q[7:0];
            if (wstrb_q[1]) cmp[47:40] <= wdata_q[15:8];
            if (wstrb_q[2]) cmp[55:48] <= wdata_q[23:16];
            if (wstrb_q[3]) cmp[63:56] <= wdata_q[31:24];
          end
          mtip[h] <= mtime >= cmp;
        end
      end
    end

    always_comb begin
      case (word)
        2'd0:    read_word = mtime_lo;
        2'd1:    read_word = mtime_hi;
        2'd2:    read_word = mtimecmp_lo;
        2'd3:    read_word = mtimecmp_hi;
        default: read_word = read_above;
      endcase
    end
  end endgenerate

  logic [31:0] bypassed, wmask;
  assign wmask = {{8{wstrb_q[3]}}, {8{wstrb_q[2]}}, {8{wstrb_q[1]}}, {8{wstrb_q[0]}}};
  assign bypassed = (writing_q && word_q == word) ? ((wdata_q & wmask) | (read_word & ~wmask)) : read_word;

  always_ff @(posedge clk) begin
    if (reset) begin
      mtime    <= 64'b0;
      // Zero puts `mtip` up from reset; csrs.v resets both enables to zero, so it is harmless.
      mtimecmp  <= 64'b0;
      mtip[0]   <= 1'b0;
      mem_rdata <= 32'b0;
    end else begin
      mtime <= mtime_next;
      if (wr_time_lo) begin
        if (wstrb_q[0]) mtime[7:0]    <= wdata_q[7:0];
        if (wstrb_q[1]) mtime[15:8]   <= wdata_q[15:8];
        if (wstrb_q[2]) mtime[23:16]  <= wdata_q[23:16];
        if (wstrb_q[3]) mtime[31:24]  <= wdata_q[31:24];
      end
      if (wr_time_hi) begin
        if (wstrb_q[0]) mtime[39:32]  <= wdata_q[7:0];
        if (wstrb_q[1]) mtime[47:40]  <= wdata_q[15:8];
        if (wstrb_q[2]) mtime[55:48]  <= wdata_q[23:16];
        if (wstrb_q[3]) mtime[63:56]  <= wdata_q[31:24];
      end
      if (wr_cmp_lo) begin
        if (wstrb_q[0]) mtimecmp[7:0]   <= wdata_q[7:0];
        if (wstrb_q[1]) mtimecmp[15:8]  <= wdata_q[15:8];
        if (wstrb_q[2]) mtimecmp[23:16] <= wdata_q[23:16];
        if (wstrb_q[3]) mtimecmp[31:24] <= wdata_q[31:24];
      end
      if (wr_cmp_hi) begin
        if (wstrb_q[0]) mtimecmp[39:32] <= wdata_q[7:0];
        if (wstrb_q[1]) mtimecmp[47:40] <= wdata_q[15:8];
        if (wstrb_q[2]) mtimecmp[55:48] <= wdata_q[23:16];
        if (wstrb_q[3]) mtimecmp[63:56] <= wdata_q[31:24];
      end
      // The plain register, not `mtime_next`: `mtip` is registered, so a cycle-stale compare is legal.
      mtip[0] <= mtime >= mtimecmp;
      mem_rdata <= in_range ? bypassed : 32'b0;
    end
  end
endmodule
