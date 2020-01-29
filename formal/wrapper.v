module rvfi_wrapper (
  input var clock, reset,
  `RVFI_OUTPUTS
);
  `RVFI_WIRES
  (* keep *) `rvformal_rand_reg arready;
  (* keep *) `rvformal_rand_reg rvalid;
  (* keep *) `rvformal_rand_reg [31:0] rdata;

  (* keep *) logic        awready;
  (* keep *) logic        awvalid;
  (* keep *) logic [31:0] awaddress;
  (* keep *) logic [2:0]  awprot;
  (* keep *) logic        wvalid;
  (* keep *) logic        wready;
  (* keep *) logic [31:0] wdata;
  (* keep *) logic [3:0]  wstrb;
  (* keep *) logic        bvalid;
  (* keep *) logic        bready;
  (* keep *) logic [1:0]  bresp;
  (* keep *) logic        arvalid;
  (* keep *) logic [31:0] araddress;
  (* keep *) logic [2:0]  arprot;
  (* keep *) logic        rready;
  (* keep *) logic [1:0]  rresp;
  (* keep *) logic        trap;
  (* keep *) logic [1:0]  trap_code;

  riscv wrapper (
    .clk(clock),
    .reset(!reset),
    .awvalid(awvalid),
    .awready(awready),
    .awaddress(awaddress),
    .awprot(awprot),
    .wvalid(wvalid),
    .wready(wready),
    .wdata(wdata),
    .wstrb(wstrb),
    .bvalid(bvalid),
    .bready(bready),
    .bresp(bresp),
    .arvalid(arvalid),
    .arready(arready),
    .araddress(araddress),
    .arprot(arprot),
    .rvalid(rvalid),
    .rready(rready),
    .rdata(rdata),
    .rresp(rresp),
    .trap(trap),
    .trap_code(trap_code),
    `RVFI_CONN
  );
endmodule
