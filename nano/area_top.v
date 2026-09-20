// A synthesis-only top that wires nano.v's core to nano_qspi_ctrl the way the shipped
// design does, so `make nano-area` measures the pair together rather than letting
// yosys's auto-top synthesize one and drop the other as unconnected.
module nano_area_top (
  input  logic        clk,
  input  logic        reset,
  output logic        sck,
  output logic        flash_cs_n,
  output logic        psram_cs_n,
  output logic        spare_cs_n,
  output logic [3:0]  sio_out,
  output logic        sio_oe,
  input  logic [3:0]  sio_in
);
  logic        mem_valid, mem_instr, mem_ready;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        trap;

  riscv core (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .trap(trap)
  );

  nano_qspi_ctrl ctrl (
    .clk(clk),
    .reset(reset),
    .mem_valid(mem_valid),
    .mem_instr(mem_instr),
    .mem_ready(mem_ready),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .sck(sck),
    .flash_cs_n(flash_cs_n),
    .psram_cs_n(psram_cs_n),
    .spare_cs_n(spare_cs_n),
    .sio_out(sio_out),
    .sio_oe(sio_oe),
    .sio_in(sio_in)
  );
endmodule
