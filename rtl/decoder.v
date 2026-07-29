`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module decoder (
  input  logic clk,
  input  logic reset,
  // inputs
  input  fetcher_output in,
  input  logic [31:0] reg_rs1,
  input  logic [31:0] reg_rs2,
  // ADR-0004 hazard scoreboard: the other two producer stages (write-through
  // in the regfile already covers writeback, CLAUDE.md invariant 6).
  input  executor_output executor_out,
  // ADR-0009: the executor's multi-cycle-divide busy signal. The divider
  // latches everything it needs internally at issue (rtl/executor.v), so
  // decode is free to bubble its own output for the whole divide — and must:
  // holding it unchanged instead would have the executor misread the same
  // stale instruction as freshly issued the moment it returns to `init`
  // (worked through in detail on rtl/executor.v).
  input  logic divider_stall,
  // ADR-0009: the accessor's one-cycle load-response stall. Unlike the
  // divider, nothing downstream has captured *this* cycle's decoder_out yet
  // when this fires — the executor freezes for that one cycle too (see
  // rtl/executor.v), so bubbling here would silently drop the very
  // instruction sitting in decoder_out. Decode must hold it unchanged
  // instead, for exactly the one cycle this is asserted.
  input  logic accessor_stall,
  // ADR-0004 hazard scoreboard, third producer: a load in the accessor's
  // one-cycle turnaround (see accessor_stall and rtl/accessor.v) is a live
  // producer for that one extra cycle neither decoder_out nor executor_out
  // can see it in. Narrow, not a general widening of the scoreboard to a
  // third pipeline stage.
  input  logic       accessor_pending_valid,
  input  logic [4:0] accessor_pending_rd,
  // outputs
  output logic [31:0] pc,
  // rs1 and rs2 are synchronous outputs
  output logic [4:0] rs1,
  output logic [4:0] rs2,
  // forwards
  output decoder_output out
);
  logic [31:0] instr;
  assign instr = in.instr;
  logic [31:0] fetcher_pc;
  assign fetcher_pc = in.pc;
  // instruction decoder (figure 2.3)
  logic [4:0] opcode;
  assign opcode = instr[6:2];
  logic [1:0] quadrant, cfunct2, cmath_funct2;
  assign quadrant = instr[1:0];
  logic uncompressed;
  assign uncompressed = quadrant == 2'b11;
  logic [2:0] funct3, cfunct3;
  logic [3:0] cfunct4;
  assign funct3 = instr[14:12];
  assign cfunct3 = instr[15:13];
  assign cfunct2 = instr[11:10];
  assign cmath_funct2 = instr[6:5];
  assign cfunct4 = instr[15:12];
  logic [5:0] cfunct6;
  assign cfunct6 = instr[15:10];
  logic [6:0] funct7;
  assign funct7 = instr[31:25];

  // Forward declarations: the immediate-select and rd-select blocks below reference
  // these before their own assign groups (further down) come into scope. iverilog
  // (unlike yosys) requires every identifier declared before its first use, so the
  // bare declarations live here; each signal's driving `assign` stays with its group.
  logic instr_lui_op, instr_jal_op, instr_jalr_op, instr_cj, instr_cjal, instr_cjr, instr_cjalr,
    instr_clui;
  logic instr_branch_op, instr_cbeqz, instr_cbnez;
  logic instr_load_op, instr_clwsp, instr_clw;
  logic instr_store_op, instr_cswsp, instr_csw;
  logic instr_math_immediate, instr_math_immediate_op, instr_cli, instr_caddi, instr_caddi16sp,
    instr_caddi4spn, instr_cslli, instr_csrli, instr_csrai, instr_candi, instr_addi, instr_slti,
    instr_sltiu, instr_xori, instr_ori, instr_andi, instr_slli, instr_srli, instr_srai;
  logic [4:0] rd;

  // all instructions
  logic instr_auipc, instr_jal, instr_jalr, instr_beq, instr_bne, instr_blt, instr_bltu, instr_bge,
        instr_bgeu, instr_add, instr_sub, instr_mul, instr_mulh, instr_mulhu, instr_mulhsu,
        instr_div, instr_divu, instr_rem, instr_remu, instr_xor, instr_or, instr_and, instr_sll,
        instr_slt, instr_sltu, instr_srl, instr_sra, instr_lui, instr_lb, instr_lbu, instr_lhu,
        instr_lh, instr_lw, instr_sb, instr_sh, instr_sw, instr_ecall, instr_ebreak, instr_csrrw,
        instr_csrrs, instr_csrrc;

  // immediate decoder (figure 2.4 & table 16.1)
  logic [31:0] immediate, i_immediate, s_immediate, b_immediate, u_immediate, j_immediate;
  assign i_immediate = {{20{instr[31]}}, instr[31:20]};
  assign s_immediate = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  assign b_immediate = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
  assign u_immediate = {instr[31], instr[30:20], instr[19:12], 12'b0};
  assign j_immediate = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

  // compressed instructions
  logic [31:0] cl_immediate, clwsp_immediate, cli_immediate, css_immediate, cj_immediate,
    cb_immediate, clui_immediate, caddi_immediate, caddi16sp_immediate, caddi4spn_immediate;
  assign cl_immediate = {25'b0, instr[5], instr[12:10], instr[6], 2'b00};
  assign clwsp_immediate = {24'b0, instr[3:2], instr[12], instr[6:4], 2'b00};
  assign cli_immediate = {{26{instr[12]}}, instr[12], instr[6:2]};
  assign css_immediate = {24'b0, instr[8:7], instr[12:9], 2'b00};
  assign cj_immediate = {{20{instr[12]}}, instr[12], instr[8], instr[10], instr[9], instr[6],
                          instr[7], instr[2], instr[11], instr[5], instr[4], instr[3], 1'b0};
  assign cb_immediate = {{23{instr[12]}}, instr[12], instr[6:5], instr[2], instr[11:10], instr[4:3], 1'b0};
  assign clui_immediate = {{14{instr[12]}}, instr[12], instr[6:2], 12'b0};
  assign caddi_immediate = {{26{instr[12]}}, instr[12], instr[6:2]};
  assign caddi16sp_immediate = {{22{instr[12]}}, instr[12], instr[4:3], instr[5], instr[2], instr[6], 4'b0};
  assign caddi4spn_immediate = {22'b0, instr[10:7], instr[12:11], instr[5], instr[6], 2'b00};

  always_comb begin
    (* parallel_case *)
    case (1'b1)
      instr_load_op || instr_jalr: immediate = i_immediate;
      instr_store_op: immediate = s_immediate;
      instr_lui_op || instr_auipc: immediate = u_immediate;
      instr_jal_op: immediate = j_immediate;
      instr_branch_op: immediate = b_immediate;
      instr_math_immediate_op: immediate = i_immediate;
      instr_clwsp: immediate = clwsp_immediate;
      instr_cswsp: immediate = css_immediate;
      instr_csw: immediate = cl_immediate;
      instr_clw: immediate = cl_immediate;
      instr_cj || instr_cjal: immediate = cj_immediate;
      instr_cbeqz || instr_cbnez: immediate = cb_immediate;
      instr_cli: immediate = cli_immediate;
      instr_clui: immediate = clui_immediate;
      instr_caddi: immediate = caddi_immediate;
      instr_caddi16sp: immediate = caddi16sp_immediate;
      instr_caddi4spn: immediate = caddi4spn_immediate;
      instr_candi: immediate = caddi_immediate;
      default: immediate = 32'b0;
    endcase
  end

  // Table 24.2 RV32I and Table 16.5-7
  assign instr_lui_op = opcode == 5'b01101 && uncompressed;
  assign instr_lui = instr_lui_op || instr_clui;
  assign instr_clui = quadrant == 2'b01 && cfunct3 == 3'b011 && clui_immediate != 0 &&
    instr[11:7] != 2;
  assign instr_auipc = opcode == 5'b00101 && uncompressed;
  assign instr_jal_op = opcode == 5'b11011 && uncompressed;
  assign instr_jal = instr_jal_op || instr_cj || instr_cjal;
  assign instr_jalr_op = opcode == 5'b11001 && uncompressed && funct3 == 3'b000;
  assign instr_jalr = instr_jalr_op || instr_cjr || instr_cjalr;
  assign instr_cj = quadrant == 2'b01 && cfunct3 == 3'b101;
  assign instr_cjal = quadrant == 2'b01 && cfunct3 == 3'b001;
  assign instr_cjr = quadrant == 2'b10 && cfunct3 == 3'b100 && instr[12] == 0 && instr[6:2] == 0 &&
    instr[11:7] != 0;
  assign instr_cjalr = quadrant == 2'b10 && cfunct3 == 3'b100 && instr[12] == 1 && instr[6:2] == 0 &&
    instr[11:7] != 0;

  assign instr_branch_op = opcode == 5'b11000 && uncompressed;
  assign instr_beq = (instr_branch_op && funct3 == 3'b000) || instr_cbeqz;
  assign instr_bne = (instr_branch_op && funct3 == 3'b001) || instr_cbnez;
  assign instr_blt = instr_branch_op && funct3 == 3'b100;
  assign instr_bge = instr_branch_op && funct3 == 3'b101;
  assign instr_bltu = instr_branch_op && funct3 == 3'b110;
  assign instr_bgeu = instr_branch_op && funct3 == 3'b111;
  assign instr_cbeqz = quadrant == 2'b01 && cfunct3 == 3'b110;
  assign instr_cbnez = quadrant == 2'b01 && cfunct3 == 3'b111;

  assign instr_load_op = opcode == 5'b00000 && uncompressed;
  assign instr_lb = instr_load_op && funct3 == 3'b000;
  assign instr_lh = instr_load_op && funct3 == 3'b001;
  assign instr_lw = (instr_load_op && funct3 == 3'b010) || instr_clwsp || instr_clw;
  assign instr_lbu = instr_load_op && funct3 == 3'b100;
  assign instr_lhu = instr_load_op && funct3 == 3'b101;
  assign instr_clwsp = quadrant == 2'b10 && cfunct3 == 3'b010 && instr[11:7] != 5'b0;
  assign instr_clw = quadrant == 2'b00 && cfunct3 == 3'b010;

  assign instr_store_op = opcode == 5'b01000 && uncompressed;
  assign instr_sb = instr_store_op && funct3 == 3'b000;
  assign instr_sh = instr_store_op && funct3 == 3'b001;
  assign instr_sw = (instr_store_op && funct3 == 3'b010) || instr_cswsp || instr_csw;
  assign instr_cswsp = quadrant == 2'b10 && cfunct3 == 3'b110;
  assign instr_csw = quadrant == 2'b00 && cfunct3 == 3'b110;

  logic math_low;
  assign math_low = funct7 == 7'b0000000;
  logic math_high;
  assign math_high = funct7 == 7'b0100000;
  assign instr_math_immediate_op = opcode == 5'b00100 && uncompressed;
  assign instr_addi = (instr_math_immediate_op && funct3 == 3'b000) || instr_cli || instr_caddi ||
    instr_caddi16sp || instr_caddi4spn;
  assign instr_caddi = quadrant == 2'b01 && cfunct3 == 3'b000;
  assign instr_caddi16sp = quadrant == 2'b01 && cfunct3 == 3'b011 && instr[11:7] == 2 &&
    caddi16sp_immediate != 0;
  assign instr_caddi4spn = quadrant == 2'b00 && cfunct3 == 3'b000 && caddi4spn_immediate != 0;
  // c.li is addi in disguise
  assign instr_cli = quadrant == 2'b01 && cfunct3 == 3'b010;
  assign instr_slti = instr_math_immediate_op && funct3 == 3'b010;
  assign instr_sltiu = instr_math_immediate_op && funct3 == 3'b011;
  assign instr_xori = instr_math_immediate_op && funct3 == 3'b100;
  assign instr_ori = instr_math_immediate_op && funct3 == 3'b110;
  assign instr_andi = (instr_math_immediate_op && funct3 == 3'b111) || instr_candi;
  assign instr_candi = quadrant == 2'b01 && cfunct3 == 3'b100 && cfunct2 == 2'b10;
  assign instr_slli = (instr_math_immediate_op && math_low && funct3 == 3'b001) || instr_cslli;
  assign instr_srli = (instr_math_immediate_op && math_low && funct3 == 3'b101) || instr_csrli;
  assign instr_srai = (instr_math_immediate_op && math_high && funct3 == 3'b101) || instr_csrai;
  assign instr_cslli = quadrant == 2'b10 && cfunct4 == 4'b0000;
  assign instr_csrli = quadrant == 2'b01 && cfunct4 == 4'b1000 && cfunct2 == 2'b00;
  assign instr_csrai = quadrant == 2'b01 && cfunct4 == 4'b1000 && cfunct2 == 2'b01;
  assign instr_math_immediate = instr_addi || instr_slti || instr_sltiu || instr_xori || instr_ori || instr_andi ||
    instr_slli || instr_srli || instr_srai;

  logic instr_math_op, instr_cmv, instr_cadd, instr_cand, instr_cor, instr_cxor, instr_csub;
  assign instr_math_op = opcode == 5'b01100 && uncompressed;
  assign instr_add = (instr_math_op && math_low && funct3 == 3'b000) || instr_cmv || instr_cadd || instr_addi;
  assign instr_cmv = quadrant == 2'b10 && cfunct4 == 4'b1000 && instr[6:2] != 0;
  assign instr_cadd = quadrant == 2'b10 && cfunct4 == 4'b1001 && instr[6:2] != 0;
  assign instr_sub = (instr_math_op && math_high && funct3 == 3'b000) || instr_csub;
  assign instr_csub = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b00;
  assign instr_sll = instr_math_op && math_low && funct3 == 3'b001 || instr_slli;
  assign instr_slt = instr_math_op && math_low && funct3 == 3'b010 || instr_slti;
  assign instr_sltu = instr_math_op && math_low && funct3 == 3'b011 || instr_sltiu;
  assign instr_xor = (instr_math_op && math_low && funct3 == 3'b100) || instr_cxor || instr_xori;
  assign instr_cxor = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b01;
  assign instr_srl = instr_math_op && math_low && funct3 == 3'b101 || instr_srli;
  assign instr_sra = instr_math_op && math_high && funct3 == 3'b101 || instr_srai;
  assign instr_or = (instr_math_op && math_low && funct3 == 3'b110) || instr_cor || instr_ori;
  assign instr_cor = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b10;
  assign instr_and = (instr_math_op && math_low && funct3 == 3'b111) || instr_cand || instr_andi;
  assign instr_cand = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b11;

  logic instr_m;
  assign instr_m = instr_math_op && funct7 == 7'b0000001;
  assign instr_mul = instr_m && funct3 == 3'b000;
  assign instr_mulh = instr_m && funct3 == 3'b001;
  assign instr_mulhu = instr_m && funct3 == 3'b011;
  assign instr_mulhsu = instr_m && funct3 == 3'b010;
  assign instr_div = instr_m && funct3 == 3'b100;
  assign instr_divu = instr_m && funct3 == 3'b101;
  assign instr_rem = instr_m && funct3 == 3'b110;
  assign instr_remu = instr_m && funct3 == 3'b111;

  logic instr_csr, instr_csrrwi, instr_csrrsi, instr_csrrci;
  assign instr_csr = opcode == 5'b11100 && uncompressed;
  assign instr_csrrw = instr_csr && funct3 == 3'b001 || instr_csrrwi;
  assign instr_csrrs = instr_csr && funct3 == 3'b010 || instr_csrrsi;
  assign instr_csrrc = instr_csr && funct3 == 3'b011 || instr_csrrci;
  assign instr_csrrwi = instr_csr && funct3 == 3'b101;
  assign instr_csrrsi = instr_csr && funct3 == 3'b110;
  assign instr_csrrci = instr_csr && funct3 == 3'b111;

  logic instr_error;
  assign instr_error = opcode == 5'b11100 && uncompressed && funct3 == 0 && rs1 == 0 && rd == 0;
  assign instr_ecall = instr_error && instr[31:20] == 12'h0;
  // funct12 must be exactly 1 (ebreak's SYSTEM encoding); any other nonzero funct12
  // (e.g. mret = 0x302, wfi = 0x105) is a different SYSTEM instruction, not ebreak.
  assign instr_ebreak = instr_error && instr[31:20] == 12'h1;
  logic instr_valid;

  assign instr_valid = instr_auipc || instr_jal || instr_jalr || instr_beq || instr_bne || instr_blt
    || instr_bltu || instr_bge || instr_bgeu || instr_add || instr_sub || instr_xor || instr_or ||
    instr_and || instr_mul || instr_mulh || instr_mulhu || instr_mulhsu || instr_div || instr_divu
    || instr_rem || instr_remu || instr_sll || instr_slt || instr_sltu || instr_srl || instr_sra ||
    instr_lui || instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw || instr_sb || instr_sh
    || instr_sw || instr_ecall || instr_ebreak;

  always_comb begin
    (* parallel_case, full_case *)
    case (1'b1)
      instr_beq || instr_bne || instr_blt || instr_bge || instr_bltu || instr_bgeu ||
        instr_sb || instr_sh || instr_sw || instr_cj || instr_cjr: rd = 0;
      instr_cjal || instr_cjalr: rd = 1;
      instr_clw || instr_caddi4spn: rd = {2'b01, instr[4:2]};
      instr_csrai || instr_csrli || instr_candi || instr_cand ||
        instr_cor || instr_cxor || instr_csub: rd = {2'b01, instr[9:7]};
      default: rd = instr[11:7];
    endcase
  end // always_comb

  always_comb begin
    (* parallel_case, full_case *)
    case (1'b1)
      instr_clwsp || instr_cswsp || instr_caddi4spn: rs1 = 2;
      instr_clw || instr_csw || instr_cbeqz || instr_cbnez ||
        instr_csrai || instr_csrli || instr_candi || instr_cand ||
        instr_cor || instr_cxor || instr_csub: rs1 = {2'b01, instr[9:7]};
      instr_cjr || instr_cjalr || instr_cslli: rs1 = instr[11:7];
      instr_cli || instr_cmv: rs1 = 0;
      instr_caddi || instr_caddi16sp || instr_cadd: rs1 = instr[11:7];
      default: rs1 = instr[19:15];
    endcase // case (1'b1)
  end

  always_comb begin
    (* parallel_case, full_case *)
    case(1'b1)
      instr_cswsp || instr_cslli || instr_csrai || instr_csrli || instr_cmv || instr_cadd: rs2 = instr[6:2];
      instr_csw || instr_cand || instr_cor || instr_cxor || instr_csub: rs2 = {2'b01, instr[4:2]};
      instr_cbeqz || instr_cbnez: rs2 = 0;
      default: rs2 = instr[24:20];
    endcase
  end
  // ALU handling
  logic instr_math, instr_shift;
  assign instr_math = instr_add || instr_sub || instr_sll || instr_slt || instr_sltu || instr_xor || instr_srl ||
    instr_sra || instr_or || instr_and || instr_mul || instr_mulh || instr_mulhu || instr_mulhsu || instr_div ||
    instr_divu || instr_rem || instr_remu;
  assign instr_shift = instr_slli || instr_srli || instr_srai;

  logic [31:0] math_arg;
  always_comb
    if (instr_math_immediate) math_arg = instr_shift ? {27'b0, rs2} : immediate;
    else math_arg = reg_rs2;

  logic [31:0] pc_inc;
  assign pc_inc = uncompressed ? 4 : 2;

  // ADR-0004 stall-only hazard scoreboard. Stall only when THIS instruction
  // actually consumes rs1/rs2 as a register operand — not, say, a shift's
  // shamt or a U-type/J-type immediate's overlapping bit position — and that
  // register has a live (non-x0) producer still in flight at decoder_out (the
  // module's own `out`, i.e. the instruction decode issued last cycle),
  // executor_out, or (JEF-607) a load still in the accessor's one-cycle
  // memory turnaround. Write-through in the regfile covers the writeback
  // stage itself; the accessor check exists only because a load specifically
  // needs one more cycle there than every other instruction (see
  // accessor_stall/rtl/accessor.v) — not a general widening to a third stage.
  logic uses_rs1, uses_rs2;
  assign uses_rs1 = !(instr_lui || instr_jal || instr_auipc);
  assign uses_rs2 = (instr_math && !instr_math_immediate) || instr_sb || instr_sh || instr_sw ||
    instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu;

  // Shared by hazard_rs1/hazard_rs2 below: is register `r` the destination
  // of a live (not-yet-retired) producer at any of the three checked points?
  function automatic logic live_producer(input logic [4:0] r);
    live_producer = (out.valid && out.rd == r) || (executor_out.valid && executor_out.rd == r) ||
      (accessor_pending_valid && accessor_pending_rd == r);
  endfunction

  logic hazard_rs1, hazard_rs2, hazard, stall;
  assign hazard_rs1 = uses_rs1 && rs1 != 0 && live_producer(rs1);
  assign hazard_rs2 = uses_rs2 && rs2 != 0 && live_producer(rs2);
  assign hazard = hazard_rs1 || hazard_rs2;

 `ifdef RISCV_FORMAL
  // ADR-0006 / JEF-628: rs1_valid/rs2_valid decide whether rvfi_rs{1,2}_addr
  // and _rdata report the real register or are masked to 0, per RVFI
  // convention (an instruction with no rsN field must report addr/rdata as
  // 0, not whatever bits of the encoding happen to alias a register index).
  // rvfi_rs1_valid is the serialized core's green-run formula, ported as-is
  // (`git show 1709433^:rtl/riscv.v`): LUI/JAL/AUIPC have no rs1 field.
  //
  // rvfi_rs2_valid is `uses_rs2` above, not the serialized core's broader
  // formula (which additionally excludes only JALR/loads, leaving it true
  // for e.g. ADDI) — that broader version looked safe the same way the
  // over-reported case above is (the monitor's channel-0 check only
  // compares rs2_addr against spec when spec_rs2_addr != 0, so a garbage
  // address for an instruction with no real rs2 field looked harmless) but
  // is not: the monitor's *reordered*-channel shadow-register check
  // (errors 131/132) compares whatever rs2_addr/rs2_rdata is reported
  // against its own last-write model for that address unconditionally,
  // with no such exemption. ADDI's I-type encoding has no rs2 field, but
  // decode's default rs2 selection (`rs2 = instr[24:20]`) reads immediate
  // bits there regardless — for `addi x2, x0, 1` those bits are 1, so the
  // broader formula reports rs2_addr=x1 with rs2_addr's *current* value,
  // which the shadow model (tracking x1's last real write) can legitimately
  // disagree with. Caught empirically: JEF-628's `make test` run failed
  // error 132 ("mismatch with shadow rs2") on a correct core. `uses_rs2` is
  // exactly "does this instruction have a real rs2 operand", so it doesn't
  // have this hole.
  logic rvfi_rs1_valid, rvfi_rs2_valid;
  assign rvfi_rs1_valid = !instr_lui && !instr_jal && !instr_auipc;
  assign rvfi_rs2_valid = uses_rs2;
 `endif
  // ADR-0009: a single global stall, combining the local RAW hazard with
  // whatever's busy downstream (the divider, or the accessor's one-cycle
  // load turnaround). Any reason freezes the PC; see below for why the two
  // downstream reasons freeze decoder_out differently.
  assign stall = hazard || divider_stall || accessor_stall;

  // publish the decoded results
  always_ff @(posedge clk) begin
    if (reset) begin
      // zero out the pc and bubble the output
      pc <= 0;
      out <= '0;
    end else if (divider_stall || accessor_stall) begin
      // ADR-0009: PC holds like any other freeze, but decoder_out must hold
      // too, unchanged — not bubble. Decode gets exactly one free cycle to
      // issue the instruction *after* the one that made the executor or
      // accessor busy before either of these stalls exists (they both fire
      // one cycle behind their cause); that next instruction is sitting in
      // decoder_out, genuinely not yet consumed by anything downstream, and
      // bubbling it here would silently drop it. It's safe to hold rather
      // than bubble specifically because nothing downstream can reprocess it
      // while held: the executor's own `case (state)` is mutually exclusive
      // (it only ever reads `in` from its `init` branch, never while
      // `state == divide`) and, for the accessor case, the executor is
      // itself frozen the same cycle (see rtl/executor.v's accessor_stall).
      // Takes priority over a same-cycle hazard for the same reason — that
      // hazard is about the *next* instruction, which isn't decode's problem
      // yet since decoder_out hasn't advanced.
      pc <= pc;
    end else if (hazard) begin
      // ADR-0009: upstream of the stalling stage freezes — the PC (and so
      // the fetch window) holds, so the stalled instruction re-presents next
      // cycle. Bubbles rather than holds: unlike divider_stall/accessor_stall
      // above, nothing stops the executor from reading `in` every cycle here,
      // so holding decoder_out unchanged would have it reprocess the same
      // instruction repeatedly instead of retrying the *hazarded* one.
      pc <= pc;
      out <= '0;
    end else begin
      // branches handled below
      pc <= fetcher_pc + pc_inc;
      out.valid <= 1'b1;
      out.mem_addr <= $signed(immediate) + $signed(reg_rs1);
      // forwards
      out.rs1 <= instr_lui ? immediate : reg_rs1;
      out.rs2 <= instr_math ? math_arg : reg_rs2;
      out.rd <= rd;
     `ifdef RISCV_FORMAL
      // ADR-0006 shadow capture: everything RVFI needs that only decode
      // knows. rvfi.pc_wdata defaults to the same fall-through target `pc`
      // gets above and is overridden below in lockstep with every place
      // that overrides `pc` itself (jal/jalr, branches) — decode owns the
      // PC (CLAUDE.md invariant 1), so this is the one place either value
      // is ever computed; duplicating the expression here (rather than
      // reading `pc` back) is required because a non-blocking assignment
      // to `pc` above isn't visible until next cycle.
      out.rvfi.pc_wdata <= fetcher_pc + pc_inc;
      out.rvfi.insn <= uncompressed ? instr : {16'b0, instr[15:0]};
      out.rvfi.pc_rdata <= fetcher_pc;
      out.rvfi.rs1_addr <= rvfi_rs1_valid ? rs1 : 5'b0;
      out.rvfi.rs2_addr <= rvfi_rs2_valid ? rs2 : 5'b0;
      out.rvfi.rs1_rdata <= rvfi_rs1_valid ? reg_rs1 : 32'b0;
      out.rvfi.rs2_rdata <= rvfi_rs2_valid ? reg_rs2 : 32'b0;
     `endif
      // outputs
      out.is_add <= instr_add;
      out.is_sub <= instr_sub;
      out.is_xor <= instr_xor;
      out.is_or <= instr_or;
      out.is_and <= instr_and;
      out.is_mul <= instr_mul;
      out.is_mulh <= instr_mulh;
      out.is_mulhu <= instr_mulhu;
      out.is_mulhsu <= instr_mulhsu;
      out.is_div <= instr_div;
      out.is_divu <= instr_divu;
      out.is_rem <= instr_rem;
      out.is_remu <= instr_remu;
      out.is_sll <= instr_sll;
      out.is_slt <= instr_slt;
      out.is_sltu <= instr_sltu;
      out.is_srl <= instr_srl;
      out.is_sra <= instr_sra;
      out.is_lui <= instr_lui;
      out.is_lb <= instr_lb;
      out.is_lbu <= instr_lbu;
      out.is_lhu <= instr_lhu;
      out.is_lh <= instr_lh;
      out.is_lw <= instr_lw;
      out.is_sb <= instr_sb;
      out.is_sh <= instr_sh;
      out.is_sw <= instr_sw;
      out.is_ecall <= instr_ecall;
      out.is_ebreak <= instr_ebreak;
      out.is_csrrw <= instr_csrrw;
      out.is_csrrs <= instr_csrrs;
      out.is_csrrc <= instr_csrrc;
      out.is_valid_instr <= instr_valid;
      // calculate branch
      (* parallel_case *)
      case(1'b1)
        default: ;
        instr_auipc: begin
          // AUIPC has no rs1/rs2 fields at all (U-type: imm[31:12] | rd |
          // opcode) — the bit positions decode.v's default rs1/rs2 selection
          // reads are part of the immediate here, not a register index. The
          // result is pc + immediate, computed the same way jal/jalr compute
          // their return address: through is_add with the operands supplied
          // directly rather than through reg_rs1/reg_rs2.
          out.rd <= rd;
          out.rs1 <= fetcher_pc;
          out.rs2 <= immediate;
          out.is_add <= 1;
        end

        instr_jal || instr_jalr: begin
          pc <= instr_jalr ?
            ($signed(immediate) + $signed(reg_rs1)) & 32'hfffffffe :
            $signed(fetcher_pc) + $signed(immediate);
         `ifdef RISCV_FORMAL
          out.rvfi.pc_wdata <= instr_jalr ?
            ($signed(immediate) + $signed(reg_rs1)) & 32'hfffffffe :
            $signed(fetcher_pc) + $signed(immediate);
         `endif
          out.rs1 <= fetcher_pc;
          out.rs2 <= pc_inc;
          out.rd <= rd;
          out.is_add <= 1;
        end

        instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu: begin
          (* parallel_case, full_case *)
          case(1'b1)
            instr_beq: pc <= reg_rs1 == reg_rs2 ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_bne: pc <= reg_rs1 != reg_rs2 ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_blt: pc <= $signed(reg_rs1) < $signed(reg_rs2) ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_bltu: pc <= reg_rs1 < reg_rs2 ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_bge: pc <= $signed(reg_rs1) >= $signed(reg_rs2) ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_bgeu: pc <= reg_rs1 >= reg_rs2 ? fetcher_pc + immediate : fetcher_pc + pc_inc;
          endcase // case (1'b1)
         `ifdef RISCV_FORMAL
          (* parallel_case, full_case *)
          case(1'b1)
            instr_beq: out.rvfi.pc_wdata <= reg_rs1 == reg_rs2 ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_bne: out.rvfi.pc_wdata <= reg_rs1 != reg_rs2 ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_blt: out.rvfi.pc_wdata <= $signed(reg_rs1) < $signed(reg_rs2) ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_bltu: out.rvfi.pc_wdata <= reg_rs1 < reg_rs2 ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_bge: out.rvfi.pc_wdata <= $signed(reg_rs1) >= $signed(reg_rs2) ? fetcher_pc + immediate : fetcher_pc + pc_inc;
            instr_bgeu: out.rvfi.pc_wdata <= reg_rs1 >= reg_rs2 ? fetcher_pc + immediate : fetcher_pc + pc_inc;
          endcase // case (1'b1)
         `endif
          out.rs1 <= 0;
          out.rs2 <= 0;
          out.rd <= 0;
        end
      endcase
      // Suppress all execution flags for unrecognized instructions (covers illegal opcodes,
      // reserved encodings, and unimplemented CSR — prevents silent pipeline corruption)
      if (!instr_valid) begin
        out.is_add <= 0; out.is_sub <= 0; out.is_xor <= 0; out.is_or <= 0; out.is_and <= 0;
        out.is_mul <= 0; out.is_mulh <= 0; out.is_mulhu <= 0; out.is_mulhsu <= 0;
        out.is_div <= 0; out.is_divu <= 0; out.is_rem <= 0; out.is_remu <= 0;
        out.is_sll <= 0; out.is_slt <= 0; out.is_sltu <= 0; out.is_srl <= 0; out.is_sra <= 0;
        out.is_lui <= 0; out.is_lb <= 0; out.is_lbu <= 0; out.is_lhu <= 0; out.is_lh <= 0; out.is_lw <= 0;
        out.is_sb <= 0; out.is_sh <= 0; out.is_sw <= 0;
        out.is_ecall <= 0; out.is_ebreak <= 0;
        out.is_csrrw <= 0; out.is_csrrs <= 0; out.is_csrrc <= 0;
        out.rd <= 0; // prevent writeback to arbitrary register
      end
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // Reset is a once-at-the-start pulse everywhere in this design (matches
  // rtl/executor.v's identical assumption); without this, the solver can
  // reassert it on an arbitrary later cycle and the pc-increment assertions
  // below — which reason about "pc advanced by pc_inc since last cycle" —
  // would have to separately account for every point reset could jump pc
  // back to 0.
  always_comb if (clocked) assume(!reset);

  // This component proof stands alone (no real fetcher instantiated), so
  // `in` is otherwise a free input. In the real pipeline the decoder owns
  // pc and the fetcher only echoes it straight back combinationally
  // (rtl/fetcher.v's `out.pc = pc;`, wired pc->pc in littlecpu.v) — without
  // pinning that down here, the solver can present a `fetcher_pc` that
  // bears no relation to what this decoder itself last drove, and the
  // pc-increment assertions below are unprovable against a fetcher that
  // isn't behaving like this design's actual one.
  always_comb assume(in.pc == pc);

  // pc increment logic. Skipped for a cycle whose *previous* cycle was
  // stalled (the PC held instead of advancing by pc_inc that edge — checked
  // separately below) or reset (pc is forced to 0 on reset, not advanced by
  // pc_inc from wherever it was). Every "previous cycle" signal here
  // (branch_jump, past_pc, prev_stall, prev_reset, prev_uncompressed) is
  // deliberately its own directly-registered copy of a real (reset-gated)
  // signal rather than routed through $past() on a free input (in.pc/instr
  // are unconstrained beyond the in.pc == pc assumption above) — $past()
  // chained across another register adds an extra cycle of history the
  // solver can fill with a pre-reset garbage value that this component
  // proof, standing alone, has no way to rule out.
  logic branch_jump;
  always_ff @(posedge clk) if (reset) branch_jump <= 1'b0;
    else branch_jump <= instr_jal || instr_jalr || instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu;
  logic [31:0] past_pc;
  logic prev_reset, prev_stall, prev_uncompressed;
  always_ff @(posedge clk) begin
    past_pc <= pc;
    prev_reset <= reset;
    prev_stall <= stall;
    prev_uncompressed <= uncompressed;
  end
  always_ff @(posedge clk) if(clocked && !branch_jump && !prev_stall && !prev_reset && prev_uncompressed) assert(past_pc + 4 == pc);
  always_ff @(posedge clk) if(clocked && !branch_jump && !prev_stall && !prev_reset && !prev_uncompressed) assert(past_pc + 2 == pc);

  // JEF-607 / ADR-0009 criterion 4: a stalled cycle never advances pc.
  always_ff @(posedge clk) if (clocked && prev_stall && !prev_reset) assert(pc == past_pc);

  // JEF-607 criterion 4: valid == 0 implies out.rd == 0 (a bubble is fully
  // zeroed, never a partial one that could sneak a spurious rd through).
  // Gated on `clocked`: before the first clock edge applies `reset`, `out`
  // is a free, uninitialized register as far as the solver is concerned, so
  // the property only holds once the design has actually been reset.
  always_comb if (clocked && !out.valid) assert(out.rd == 0);

  // JEF-607 criterion 4: rs1 == 0 / rs2 == 0 never cause a stall — x0 has no
  // producer to wait on (write-through already special-cases it to 0).
  always_comb if (rs1 == 0) assert(!hazard_rs1);
  always_comb if (rs2 == 0) assert(!hazard_rs2);

  logic one_of;
  // Use $onehot() so pairwise overlaps (even count of flags) are detected rather than
  // cancelling out as they would with XOR.
  assign one_of = $onehot({instr_auipc, instr_jal, instr_jalr, instr_beq, instr_bne, instr_blt,
    instr_bltu, instr_bge, instr_bgeu, instr_add, instr_sub, instr_xor, instr_or, instr_and,
    instr_mul, instr_mulh, instr_mulhu, instr_mulhsu, instr_div, instr_divu, instr_rem,
    instr_remu, instr_sll, instr_slt, instr_sltu, instr_srl, instr_sra, instr_lui, instr_lb,
    instr_lbu, instr_lh, instr_lhu, instr_lw, instr_sb, instr_sh, instr_sw, instr_ecall,
    instr_ebreak});

  // we should only get one type of instruction
  always_comb if (instr_valid) assert(one_of);
 `endif
endmodule
