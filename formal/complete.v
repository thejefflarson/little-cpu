module testbench (
  input var clk,
  output var        awvalid, // we wrote the address
  input  var        awready, // address is ready for write
  output var [31:0] awaddress, // address to write
  output var [2:0]  awprot, // permissions

  output var        wvalid, // we wrote the value
  input  var        wready, // value is ready
  output var [31:0] wdata, // value to write
  output var [3:0]  wstrb, // what bytes we wrote

  output var        bvalid, // we received the response
  input  var        bready, // status is ready
  input  var [1:0]  bresp, // status of our write request

  output var        arvalid, // we put something in the read addreas
  input  var        arready, // they are reading the value
  output var [31:0] araddress, // address to read
  output var [2:0]  arprot, // permissions

  input  var        rvalid, // Data is valid and can be read by us
  output var        rready, // we are ready to read
  input  var [31:0] rdata, // value to read
  input  var [1:0]  rresp, // status of our read request

  // outputs
  output var        trap,
  output var [1:0]  trap_code

);
  logic reset;
  always @(posedge clk) reset <= 1;
  logic trap;

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
    .trap(trap_code),
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
