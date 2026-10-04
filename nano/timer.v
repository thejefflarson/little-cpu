`default_nettype none
// `mtip` is a level, held while `mtime >= mtimecmp`, and registered: it posts a cycle late and never early.
// An RV32 `mtimecmp` update is three stores, low all-ones, high, low; any other order posts a spurious interrupt.
module nano_timer #(
  parameter logic [31:0] BASE = 32'h1080_0010
) (
  input  logic        clk,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  output logic        mtip
);
  if (|BASE[3:0]) begin : l_base_aligned
    $fatal(1, "nano_timer: BASE must be 16-byte aligned");
  end

  logic        in_range;
  logic [1:0]  word;
  assign in_range = mem_addr[31:4] == BASE[31:4];
  assign word     = mem_addr[3:2];

  logic [31:0] time_lo, time_hi, cmp_lo, cmp_hi;

  logic writing, wr_time_lo, wr_time_hi, wr_cmp_lo, wr_cmp_hi;
  assign writing    = in_range && |mem_wstrb;
  assign wr_time_lo = writing && word == 2'd0;
  assign wr_time_hi = writing && word == 2'd1;
  assign wr_cmp_lo  = writing && word == 2'd2;
  assign wr_cmp_hi  = writing && word == 2'd3;

  logic [31:0] wmask;
  assign wmask = {{8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}}};

  logic [63:0] time_inc;
  assign time_inc = {time_hi, time_lo} + 64'd1;

  always_ff @(posedge clk) begin
    if (reset) begin
      time_lo <= 32'b0;
      time_hi <= 32'b0;
      // Zero posts `mtip` out of reset; harmless because mie.MTIE and mstatus.MIE both reset to zero.
      cmp_lo  <= 32'b0;
      cmp_hi  <= 32'b0;
      mtip    <= 1'b0;
    end else begin
      time_lo <= wr_time_lo ? (mem_wdata & wmask) | (time_lo & ~wmask) :
                 wr_time_hi ? time_lo : time_inc[31:0];
      time_hi <= wr_time_hi ? (mem_wdata & wmask) | (time_hi & ~wmask) :
                 wr_time_lo ? time_hi : time_inc[63:32];
      if (wr_cmp_lo) cmp_lo <= (mem_wdata & wmask) | (cmp_lo & ~wmask);
      if (wr_cmp_hi) cmp_hi <= (mem_wdata & wmask) | (cmp_hi & ~wmask);
      mtip <= {time_hi, time_lo} >= {cmp_hi, cmp_lo};
    end
  end

  always_comb begin
    mem_rdata = 32'b0;
    if (in_range) begin
      case (word)
        2'd0:    mem_rdata = time_lo;
        2'd1:    mem_rdata = time_hi;
        2'd2:    mem_rdata = cmp_lo;
        default: mem_rdata = cmp_hi;
      endcase
    end
  end
endmodule

`default_nettype wire
