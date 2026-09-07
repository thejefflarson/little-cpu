`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_ALIGNED_MEM
`include "rvfi_macros.vh"
`include "rvfi_channel.sv"

module testbench (
  input clk
);
  logic reset = 1;
  logic trap;

  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES

  logic [31:0] uut_imem_addr;
  logic [31:0] uut_imem_data;
  logic [31:0] uut_imem_addr2;
  logic [31:0] uut_imem_data2;
  // The fetch address one cycle early.
  logic [31:0] uut_imem_addr_next;
  // The address the core publishes for the platform to decode.
  logic [31:0] atomic_addr;
  // The lock an arbiter would read.
  logic mem_lock;
  logic bus_request;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren;
  logic [31:0] mem_rdata;
  logic        fetch_stall;
  logic        text_write;

  imem_arbiter arbiter (
    .clock(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .fetch_stall(fetch_stall),
    .text_write(text_write)
  );

  // The one halfword this task watches.
  `rvformal_rand_const_reg [31:0] shadow_addr;
  logic [15:0] shadow_data;
  logic        shadow_stored = 1'b0;

  logic        shadow_hit;
  logic [1:0]  shadow_wstrb;
  logic [15:0] shadow_wdata;
  assign shadow_hit   = text_write && mem_addr[31:2] == shadow_addr[31:2];
  assign shadow_wstrb = shadow_addr[1] ? mem_wstrb[3:2]   : mem_wstrb[1:0];
  assign shadow_wdata = shadow_addr[1] ? mem_wdata[31:16] : mem_wdata[15:0];

  always_ff @(posedge clk) begin
    if (shadow_hit) begin
      if (shadow_wstrb[0]) shadow_data[ 7:0] <= shadow_wdata[ 7:0];
      if (shadow_wstrb[1]) shadow_data[15:8] <= shadow_wdata[15:8];
      if (|shadow_wstrb)   shadow_stored     <= 1'b1;
    end
  end

  always_comb begin
    if (!reset && !fetch_stall) begin
      if (uut_imem_addr      == shadow_addr) assume(uut_imem_data [15: 0] == shadow_data);
      if (uut_imem_addr + 2  == shadow_addr) assume(uut_imem_data [31:16] == shadow_data);
      if (uut_imem_addr2     == shadow_addr) assume(uut_imem_data2[15: 0] == shadow_data);
      if (uut_imem_addr2 + 2 == shadow_addr) assume(uut_imem_data2[31:16] == shadow_data);
    end
  end

  // Every retire at the watched halfword reports it.
  always_ff @(posedge clk) begin
    if (!reset && rvfi_valid && !shadow_stored) begin
      if (rvfi_pc_rdata == shadow_addr)
        assert(rvfi_insn[15:0] == shadow_data);
      if (rvfi_insn[1:0] == 2'b11 && rvfi_pc_rdata + 2 == shadow_addr)
        assert(rvfi_insn[31:16] == shadow_data);
    end
  end

  littlecpu uut (
    .clk(clk),
    .reset(reset),
    .imem_addr(uut_imem_addr),
    .imem_data(uut_imem_data),
    .imem_addr2(uut_imem_addr2),
    .imem_data2(uut_imem_data2),
    .imem_addr_next(uut_imem_addr_next),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    .imem_fault(1'b0),
    .mem_reservable(1'b1),
    .atomic_addr(atomic_addr),
    .atomic_supported(1'b1),
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    .bus_request(bus_request),
    .irq_timer(1'b0),
    .trap(trap),
    `RVFI_CONN
  );
endmodule
