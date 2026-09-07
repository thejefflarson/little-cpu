module rvfi_testbench (
  input var clk,
  output logic [31:0] imem_addr,
  input  logic [31:0] imem_data,
  // The fetch window's second word, free every cycle for the same reason imem_data is.
  output logic [31:0] imem_addr2,
  input  logic [31:0] imem_data2,
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

  // A one-address write-through shadow of the data bus: the most recent store, not an array.
  logic [31:0] dmem_shadow;
  logic [31:0] dmem_shadow_addr;
  logic        dmem_shadow_valid = 0;
  always_ff @(posedge clk) begin
    if (!reset && mem_wstrb) begin
      dmem_shadow_addr <= mem_addr;
      if (mem_wstrb[0]) dmem_shadow[ 7: 0] <= mem_wdata[ 7: 0];
      if (mem_wstrb[1]) dmem_shadow[15: 8] <= mem_wdata[15: 8];
      if (mem_wstrb[2]) dmem_shadow[23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) dmem_shadow[31:24] <= mem_wdata[31:24];
      dmem_shadow_valid <= 1'b1;
    end
  end
  always_ff @(posedge clk) begin
    if (!reset && dmem_shadow_valid && !$past(mem_wstrb) &&
        $past(mem_addr) == dmem_shadow_addr)
      assume(mem_rdata == dmem_shadow);
  end

  // Unread here -- this environment answers imem_data in the same cycle -- but connected.
  logic [31:0] imem_addr_next;
  // The address the core publishes for the platform to decode.
  logic [31:0] atomic_addr;
  logic mem_lock;
  logic bus_request;
  logic        mem_ren;
  logic        fetch_stall;

  imem_arbiter arbiter (
    .clock(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .fetch_stall(fetch_stall),
    .text_write()
  );

  littlecpu wrapper (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
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

  rvfi_isa_rv32imc isa_spec (
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

  wire        insn_uncompressed = rvfi_insn[1:0] == 2'b11;
  wire [6:0]  insn_opcode       = rvfi_insn[6:0];

  wire [2:0]  insn_funct3       = rvfi_insn[14:12];
  wire [6:0]  insn_funct7       = rvfi_insn[31:25];

  // EXCLUDE MISC-MEM 0001111 fence fence.i
  // No spec model at the pin; both retire non-trapping, so !rvfi_trap does not excuse them.
  wire exclude_misc_mem = insn_uncompressed && insn_opcode == 7'b0001111;

  // EXCLUDE SYSTEM 1110011 ecall ebreak mret wfi csrrw csrrs csrrc csrrwi csrrsi csrrci
  // No spec model at the pin for any of the ten; ecall and ebreak are also excused by !rvfi_trap.
  wire exclude_system = insn_uncompressed && insn_opcode == 7'b1110011;

  // EXCLUDE AMO 0101111 amoadd.w amoswap.w amoxor.w amoand.w amoor.w amomin.w amomax.w amominu.w amomaxu.w lr.w sc.w
  // No spec model at the pin: insns/generate.py's insn_amo generator is wholly commented out.
  wire exclude_amo = insn_uncompressed && insn_opcode == 7'b0101111;

  wire insn_excluded = exclude_misc_mem || exclude_system || exclude_amo;

  always_comb begin
    if (!reset && rvfi_valid && !rvfi_trap && !insn_excluded) begin
      assert(spec_valid && !spec_trap);
    end
  end

  wire complete_live = !reset && rvfi_valid && !rvfi_trap && !insn_excluded;
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0000011); // LOAD
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0010011); // OP-IMM
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0010111); // AUIPC
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0100011); // STORE
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0110011); // OP
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b0110111); // LUI
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b1100011); // BRANCH
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b1100111); // JALR
  cover property (complete_live && insn_uncompressed && insn_opcode == 7'b1101111); // JAL
  cover property (complete_live && rvfi_insn[1:0] == 2'b00);                        // RVC quadrant 0
  cover property (complete_live && rvfi_insn[1:0] == 2'b01);                        // RVC quadrant 1
  cover property (complete_live && rvfi_insn[1:0] == 2'b10);                        // RVC quadrant 2

  cover property (!reset && rvfi_valid && !rvfi_trap && exclude_amo);
endmodule
