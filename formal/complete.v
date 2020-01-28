module rvfi_testbench (
  input var clk,

  output var        arvalid,
  input  var        arready,
  output var [31:0] araddress,
  output var [2:0]  arprot,

  input  var        rvalid,
  output var        rready,
  input  var [31:0] rdata,
  input  var [1:0]  rresp,
  output var        trap,
  output var [1:0]  trap_code
);
  logic reset = 0;
  always @(posedge clk) reset <= 1;
  logic trap;

  `RVFI_WIRES

  (* keep *) logic        awready;
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

  riscv wrapper (
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
    .trap_code(trap_code),
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

  // do the instruction check
  always_comb begin
    if (rvfi_insn[6:0] != 7'b1110011) begin
       assert(spec_valid == rvfi_valid);
    end
  end
endmodule
