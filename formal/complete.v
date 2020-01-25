// Silence warnings
`ifndef RISCV_FORMAL
  `define RVFI_WIRES
  `define RVFI_CONN
  `define COMMA ,
  `define RISCV_FORMAL_XLEN 32
`else
  `define COMMA
`endif
module testbench (
  input var clk
);
  logic reset;
  always @(posedge clk) reset <= 1;
  logic trap;

  (* keep *) logic        awvalid;
  (* keep *) logic        awready;
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
  (* keep *) logic        arready;
  (* keep *) logic [31:0] araddress;
  (* keep *) logic [2:0]  arprot;
  (* keep *) logic        rvalid;
  (* keep *) logic        rready;
  (* keep *) logic [31:0] rdata;
  (* keep *) logic [1:0]  rresp;

  `RVFI_WIRES

  riscv uut (
    .clk(clk),
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
    `RVFI_CONN
  );

  (* keep *) wire spec_valid;
  (* keep *) wire spec_trap;
  (* keep *) wire [4:0] spec_rs1_addr;
  (* keep *) wire [4:0] spec_rs2_addr;
  (* keep *) wire [4:0] spec_rd_addr;
  (* keep *) wire [`RISCV_FORMAL_XLEN   - 1:0] spec_rd_wdata;
  (* keep *) wire [`RISCV_FORMAL_XLEN   - 1:0] spec_pc_wdata;
  (* keep *) wire [`RISCV_FORMAL_XLEN   - 1:0] spec_mem_addr;
  (* keep *) wire [`RISCV_FORMAL_XLEN/8 - 1:0] spec_mem_rmask;
  (* keep *) wire [`RISCV_FORMAL_XLEN/8 - 1:0] spec_mem_wmask;
  (* keep *) wire [`RISCV_FORMAL_XLEN   - 1:0] spec_mem_wdata;

  rvfi_isa_rv32i isa_spec (
    .rvfi_valid(rvfi_valid),
    .rvfi_insn(rvfi_insn),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .spec_valid(spec_valid),
    .spec_trap(spec_trap),
    .spec_rs1_addr(spec_rs1_addr),
    .spec_rs2_addr(spec_rs2_addr),
    .spec_rd_addr(spec_rd_addr ),
    .spec_rd_wdata(spec_rd_wdata),
    .spec_pc_wdata(spec_pc_wdata),
    .spec_mem_addr(spec_mem_addr),
    .spec_mem_rmask(spec_mem_rmask),
    .spec_mem_wmask(spec_mem_wmask),
    .spec_mem_wdata(spec_mem_wdata)
  );
endmodule
