`default_nettype none
// `mtime` is the core's `mcycle` (stores reach it through `mtime_wr`); `mtip` is a registered level: late, never early.
// An RV32 `mtimecmp` update is three stores, low all-ones, high, low; any other order posts a spurious interrupt.
module nano_timer #(
  parameter logic [31:0] BASE = 32'h1080_0010
) (
  input  logic        clk,
  input  logic        reset,
  input  logic [63:0] mtime,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,
  output logic        mtime_wr,
  output logic        mtip
);
  if (|BASE[3:0]) begin : l_base_aligned
    $fatal(1, "nano_timer: BASE must be 16-byte aligned");
  end

  logic        in_range;
  logic [1:0]  word;
  assign in_range = mem_addr[31:4] == BASE[31:4];
  assign word     = mem_addr[3:2];

  logic [31:0] cmp_lo, cmp_hi, time_lo, time_hi;
  assign time_lo = mtime[31:0];
  assign time_hi = mtime[63:32];

  logic writing;
  assign writing  = in_range && |mem_wstrb;
  assign mtime_wr = writing && !word[1];

  logic [31:0] wmask;
  assign wmask = {{8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}}};

  always_ff @(posedge clk) begin
    if (reset) begin
      cmp_lo <= 32'b0;
      cmp_hi <= 32'b0;
      mtip   <= 1'b0;
    end else begin
      if (writing && word == 2'd2) cmp_lo <= (mem_wdata & wmask) | (cmp_lo & ~wmask);
      if (writing && word == 2'd3) cmp_hi <= (mem_wdata & wmask) | (cmp_hi & ~wmask);
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
