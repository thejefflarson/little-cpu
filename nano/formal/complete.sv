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

  always_comb begin
    if (!reset && rvfi_valid && !rvfi_trap) begin
      if (rvfi_insn[6:0] != 7'b1110011) begin
        assert(spec_valid && !spec_trap);
      end
    end
  end

  wire       insn_uncompressed = rvfi_insn[1:0] == 2'b11;
  wire [6:0] insn_opcode       = rvfi_insn[6:0];

  wire complete_live = !reset && rvfi_valid && !rvfi_trap;
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
endmodule
