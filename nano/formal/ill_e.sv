// RV32E's register-naming restriction, read off the real core's own RVFI report rather than a hand-written reference: nano.v already zeroes a register field that is not decode-significant, so bit 4 of a reported address alone says E-illegal.

module rvfi_testbench (
  input var clk,
  output logic        mem_valid,
  output logic        mem_instr,
  input  logic        mem_ready,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
);
  logic reset = 1;
  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES
  logic trap;

  riscv wrapper (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap),
    `RVFI_CONN
  );

  wire live = !reset && rvfi_valid;
  wire e_illegal = rvfi_rs1_addr[4] || rvfi_rs2_addr[4] || rvfi_rd_addr[4];

  always @* if (live && e_illegal) assert(rvfi_trap);

  // a LOAD naming x16 (rd) and x17 (rs1) at once, correctly flagged illegal
  cover property (live && rvfi_insn[6:0] == 7'b0000011 && rvfi_rd_addr == 5'd16 && rvfi_rs1_addr == 5'd17 && rvfi_trap);
endmodule
