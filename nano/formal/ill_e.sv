// RV32E's register-naming restriction, checked against the reference model below
// (`ill_e_top`) rather than `nano.v`, which has no 16-register limit or CSR layer yet.
// Every (class, field) membership is its own named, isolated term with its own cover goal.

module ill_e_top (
  input logic        clk,
  input logic [31:0] instr
);
  logic reset = 1;
  always_ff @(posedge clk)
    reset <= 0;

  wire       is32   = instr[1:0] == 2'b11;
  wire [6:0] opcode = instr[6:0];
  wire [2:0] funct3 = instr[14:12];
  wire [4:0] rd     = instr[11:7];
  wire [4:0] rs1    = instr[19:15];
  wire [4:0] rs2    = instr[24:20];

  wire class_load    = is32 && opcode == 7'b0000011;
  wire class_opimm   = is32 && opcode == 7'b0010011;
  wire class_auipc   = is32 && opcode == 7'b0010111;
  wire class_store   = is32 && opcode == 7'b0100011;
  wire class_op      = is32 && opcode == 7'b0110011;
  wire class_lui     = is32 && opcode == 7'b0110111;
  wire class_branch  = is32 && opcode == 7'b1100011;
  wire class_jalr    = is32 && opcode == 7'b1100111;
  wire class_jal     = is32 && opcode == 7'b1101111;
  wire class_miscmem = is32 && opcode == 7'b0001111;
  // funct3[2] tells csrrw/csrrs/csrrc (0, rs1 is a register) from csrrwi/csrrsi/csrrci (1, rs1 is a 5-bit uimm).
  wire class_sys_reg = is32 && opcode == 7'b1110011 && funct3 != 3'b000 && !funct3[2];
  wire class_sys_imm = is32 && opcode == 7'b1110011 && funct3 != 3'b100 &&  funct3[2];

  // CR (rd/rs1 and rs2), CI (rd/rs1 only), CSS (rs2 only), each matching the pinned insn_c_*.v models field-for-field.
  wire       is16    = !is32;
  wire [1:0] c_quad  = instr[1:0];
  wire [2:0] c_f3    = instr[15:13];
  wire [3:0] c_f4    = instr[15:12];
  wire [4:0] c_rdrs1 = instr[11:7];
  wire [4:0] c_rs2   = instr[6:2];

  wire c_jr   = is16 && c_quad == 2'b10 && c_f4 == 4'b1000 && c_rdrs1 != 5'd0 && c_rs2 == 5'd0;
  wire c_jalr = is16 && c_quad == 2'b10 && c_f4 == 4'b1001 && c_rdrs1 != 5'd0 && c_rs2 == 5'd0;
  wire c_mv   = is16 && c_quad == 2'b10 && c_f4 == 4'b1000 && c_rs2 != 5'd0;
  wire c_add  = is16 && c_quad == 2'b10 && c_f4 == 4'b1001 && c_rs2 != 5'd0;
  wire class_c_cr = c_jr || c_jalr || c_mv || c_add;

  wire c_addi = is16 && c_quad == 2'b01 && c_f3 == 3'b000;
  wire c_li   = is16 && c_quad == 2'b01 && c_f3 == 3'b010;
  wire c_lui  = is16 && c_quad == 2'b01 && c_f3 == 3'b011 && c_rdrs1 != 5'd2;
  wire c_slli = is16 && c_quad == 2'b10 && c_f3 == 3'b000;
  wire c_lwsp = is16 && c_quad == 2'b10 && c_f3 == 3'b010 && c_rdrs1 != 5'd0;
  wire class_c_ci = c_addi || c_li || c_lui || c_slli || c_lwsp;

  wire class_c_css = is16 && c_quad == 2'b10 && c_f3 == 3'b110;

  wire ill_load_rd    = class_load    && rd[4]      && !rs1[4];
  wire ill_load_rs1   = class_load    && rs1[4]     && !rd[4];
  wire ill_opimm_rd   = class_opimm   && rd[4]      && !rs1[4];
  wire ill_opimm_rs1  = class_opimm   && rs1[4]     && !rd[4];
  wire ill_auipc_rd   = class_auipc   && rd[4];
  wire ill_store_rs1  = class_store   && rs1[4]     && !rs2[4];
  wire ill_store_rs2  = class_store   && rs2[4]     && !rs1[4];
  wire ill_op_rd      = class_op      && rd[4]      && !rs1[4] && !rs2[4];
  wire ill_op_rs1     = class_op      && rs1[4]     && !rd[4]  && !rs2[4];
  wire ill_op_rs2     = class_op      && rs2[4]     && !rd[4]  && !rs1[4];
  wire ill_lui_rd     = class_lui     && rd[4];
  wire ill_branch_rs1 = class_branch  && rs1[4]     && !rs2[4];
  wire ill_branch_rs2 = class_branch  && rs2[4]     && !rs1[4];
  wire ill_jalr_rd    = class_jalr    && rd[4]      && !rs1[4];
  wire ill_jalr_rs1   = class_jalr    && rs1[4]     && !rd[4];
  wire ill_jal_rd     = class_jal     && rd[4];
  wire ill_sysreg_rd  = class_sys_reg && rd[4]      && !rs1[4];
  wire ill_sysreg_rs1 = class_sys_reg && rs1[4]     && !rd[4];
  wire ill_sysimm_rd  = class_sys_imm && rd[4];
  wire ill_ccr_rdrs1  = class_c_cr    && c_rdrs1[4] && !c_rs2[4];
  wire ill_ccr_rs2    = class_c_cr    && c_rs2[4]   && !c_rdrs1[4];
  wire ill_cci_rdrs1  = class_c_ci    && c_rdrs1[4];
  wire ill_ccss_rs2   = class_c_css   && c_rs2[4];

  wire e_illegal =
    ill_load_rd   || ill_load_rs1  || ill_opimm_rd  || ill_opimm_rs1 || ill_auipc_rd  ||
    ill_store_rs1 || ill_store_rs2 || ill_op_rd      || ill_op_rs1    || ill_op_rs2    ||
    ill_lui_rd    || ill_branch_rs1|| ill_branch_rs2 || ill_jalr_rd   || ill_jalr_rs1  ||
    ill_jal_rd    || ill_sysreg_rd || ill_sysreg_rs1 || ill_sysimm_rd ||
    ill_ccr_rdrs1 || ill_ccr_rs2   || ill_cci_rdrs1  || ill_ccss_rs2;

  logic       trap;
  logic [4:0] rd_addr;
  logic       mem_write;
  assign trap      = e_illegal;
  assign rd_addr   = e_illegal ? 5'd0 : rd;
  assign mem_write = e_illegal ? 1'b0 : class_store;

  always_comb begin
    if (!reset && e_illegal) begin
      assert (trap);
      assert (rd_addr == 5'd0);
      assert (!mem_write);
    end
  end

  wire live = !reset;
  cover property (live && e_illegal && ill_load_rd);
  cover property (live && e_illegal && ill_load_rs1);
  cover property (live && e_illegal && ill_opimm_rd);
  cover property (live && e_illegal && ill_opimm_rs1);
  cover property (live && e_illegal && ill_auipc_rd);
  cover property (live && e_illegal && ill_store_rs1);
  cover property (live && e_illegal && ill_store_rs2);
  cover property (live && e_illegal && ill_op_rd);
  cover property (live && e_illegal && ill_op_rs1);
  cover property (live && e_illegal && ill_op_rs2);
  cover property (live && e_illegal && ill_lui_rd);
  cover property (live && e_illegal && ill_branch_rs1);
  cover property (live && e_illegal && ill_branch_rs2);
  cover property (live && e_illegal && ill_jalr_rd);
  cover property (live && e_illegal && ill_jalr_rs1);
  cover property (live && e_illegal && ill_jal_rd);
  cover property (live && e_illegal && ill_sysreg_rd);
  cover property (live && e_illegal && ill_sysreg_rs1);
  cover property (live && e_illegal && ill_sysimm_rd);
  cover property (live && e_illegal && ill_ccr_rdrs1);
  cover property (live && e_illegal && ill_ccr_rs2);
  cover property (live && e_illegal && ill_cci_rdrs1);
  cover property (live && e_illegal && ill_ccss_rs2);
  cover property (live && class_miscmem);
endmodule
