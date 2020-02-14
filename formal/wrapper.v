module rvfi_wrapper (
  input var clock, reset,
  `RVFI_OUTPUTS
);
  `RVFI_WIRES
  (* keep *) `rvformal_rand_reg mem_ready;
  (* keep *) `rvformal_rand_reg [31:0] mem_rdata;

  (* keep *) logic        mem_valid;
  (* keep *) logic        mem_instr;
  (* keep *) logic [31:0] mem_addr;
  (* keep *) logic [31:0] mem_wdata;
  (* keep *) logic [3:0]  mem_wstrb;
  (* keep *) logic        trap;

  riscv wrapper (
    .clk(clock),
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

 `ifdef RISCV_FAIRNESS
  // from picorv32
  logic [2:0] mem_wait = 0;
  always_ff @(posedge clock) begin
    mem_wait <= {mem_wait, mem_valid && ! mem_ready};
    assume(~mem_wait || trap);
  end
 `endif
endmodule
