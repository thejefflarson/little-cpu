// Wraps `riscv` (nano/nano.v) for Tiny Tapeout: every core input is a pin or a register,
// never a tied constant, and every output reaches uo_out through a register.
`default_nettype none

module tt_um_thejefflarson_nanocpu (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire        ena,
    input  wire        clk,
    input  wire        rst_n
);

  logic reset;
  assign reset = !rst_n;

  logic [31:0] mem_rdata_shift;
  always_ff @(posedge clk) begin
    if (reset) mem_rdata_shift <= 32'b0;
    else if (ena) mem_rdata_shift <= {mem_rdata_shift[23:0], uio_in};
  end

  logic mem_ready;
  always_ff @(posedge clk) begin
    if (reset) mem_ready <= 1'b0;
    else mem_ready <= ui_in[7];
  end

  logic        mem_valid, mem_instr, trap;
  logic [31:0] mem_addr, mem_wdata;
  logic [3:0]  mem_wstrb;

  riscv core (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata_shift),
    .trap(trap)
  );

  logic [31:0] addr_r, wdata_r;
  logic [3:0]  wstrb_r;
  logic        valid_r, instr_r, trap_r;
  always_ff @(posedge clk) begin
    addr_r  <= mem_addr;
    wdata_r <= mem_wdata;
    wstrb_r <= mem_wstrb;
    valid_r <= mem_valid;
    instr_r <= mem_instr;
    trap_r  <= trap;
  end

  logic [71:0] observed;
  assign observed = {1'b0, trap_r, instr_r, valid_r, wstrb_r, addr_r, wdata_r};

  logic [7:0] observed_byte;
  always_comb begin
    case (ui_in[3:0])
      4'd0: observed_byte = observed[7:0];
      4'd1: observed_byte = observed[15:8];
      4'd2: observed_byte = observed[23:16];
      4'd3: observed_byte = observed[31:24];
      4'd4: observed_byte = observed[39:32];
      4'd5: observed_byte = observed[47:40];
      4'd6: observed_byte = observed[55:48];
      4'd7: observed_byte = observed[63:56];
      4'd8: observed_byte = observed[71:64];
      default: observed_byte = 8'b0;
    endcase
  end

  assign uo_out  = observed_byte;
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  logic _unused;
  assign _unused = &{ui_in[6:4], 1'b0};

endmodule

`default_nettype wire
