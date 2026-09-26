`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// D decodes the buffered word and presents the register file its own pair, never a
// guess, so X reads the right answer next cycle. Everything needing a register value
// commits in X; fetch-address ownership lives in rtl/littlecpu.v, since it spans F and X.
module decoder (
  input  logic clk,
  input  logic reset,
  input  fetcher_output in,
  input  logic x_busy,  // X still working `out` (the divider, or the region test's wait)
  input  executor_output executor_out,
  input  logic fetch_stall,  // the fetch port went to a load/store; `in.instr` is data
  input  logic bus_wait,
  // Decode's request for the bus; the platform ANDs it against its own grant.
  output logic bus_request,
  input  logic imem_fault,
  input  logic accessor_out_valid,
  output logic issuing,
  // The sequential guess `+2`/`+4`, never F's word-granular ROM address.
  output logic [31:0] predicted_pc,
  output logic [4:0] read_rs1,
  output logic [4:0] read_rs2,
  input  logic interrupt_pending,
  // `in` was fetched down the wrong path: discard it unconditionally, no counter or list.
  input  logic x_redirect,
  output dx_output out
);
  logic [31:0] instr;
  assign instr = (in.instr[1:0] == 2'b11) ? in.instr : {16'b0, in.instr[15:0]};

  logic [4:0] rd_field, rs1_field;
  assign rd_field  = instr[11:7];
  assign rs1_field = instr[19:15];

  logic [2:0] c_rd_rs1_prime, c_rs2_prime;
  assign c_rd_rs1_prime = instr[9:7];
  assign c_rs2_prime    = instr[4:2];
  logic [31:0] fetcher_pc;
  assign fetcher_pc = in.pc;
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

  logic instr_lui_op, instr_jal_op, instr_jalr_op, instr_cj, instr_cjal, instr_cjr, instr_cjalr,
    instr_clui;
  logic instr_branch_op, instr_cbeqz, instr_cbnez;
  logic instr_load_op, instr_clwsp, instr_clw;
  logic instr_store_op, instr_cswsp, instr_csw;
  logic instr_math_immediate, instr_math_immediate_op, instr_cli, instr_caddi, instr_caddi16sp,
    instr_caddi4spn, instr_cslli, instr_csrli, instr_csrai, instr_candi, instr_addi, instr_slti,
    instr_sltiu, instr_xori, instr_ori, instr_andi, instr_slli, instr_srli, instr_srai;
  logic [4:0] rd;

  logic instr_auipc, instr_jal, instr_jalr, instr_beq, instr_bne, instr_blt, instr_bltu, instr_bge,
        instr_bgeu, instr_add, instr_sub, instr_mul, instr_mulh, instr_mulhu, instr_mulhsu,
        instr_div, instr_divu, instr_rem, instr_remu, instr_xor, instr_or, instr_and, instr_sll,
        instr_slt, instr_sltu, instr_srl, instr_sra, instr_lui, instr_lb, instr_lbu, instr_lhu,
        instr_lh, instr_lw, instr_sb, instr_sh, instr_sw, instr_ecall, instr_ebreak, instr_csrrw,
        instr_csrrs, instr_csrrc;

  logic instr_amoswap, instr_amoadd, instr_amoxor, instr_amoand, instr_amoor, instr_amomin,
        instr_amomax, instr_amominu, instr_amomaxu, instr_lr, instr_sc;
  logic instr_amo_op, instr_amo, instr_atomic;

  logic [31:0] immediate, i_immediate, s_immediate, b_immediate, u_immediate, j_immediate;
  assign i_immediate = {{20{instr[31]}}, instr[31:20]};
  assign s_immediate = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  assign b_immediate = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
  assign u_immediate = {instr[31], instr[30:20], instr[19:12], 12'b0};
  assign j_immediate = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

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
      instr_load_op || instr_jalr_op: immediate = i_immediate;
      instr_store_op: immediate = s_immediate;
      instr_lui_op || instr_auipc: immediate = u_immediate;
      instr_jal_op: immediate = j_immediate;
      instr_branch_op: immediate = b_immediate;
      instr_math_immediate_op: immediate = i_immediate;
      instr_amo_op: immediate = 32'b0;
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

  logic [4:0] funct5;
  assign funct5 = instr[31:27];
  assign instr_amo_op = opcode == 5'b01011 && uncompressed && funct3 == 3'b010;
  assign instr_amoadd  = instr_amo_op && funct5 == 5'b00000;
  assign instr_amoswap = instr_amo_op && funct5 == 5'b00001;
  assign instr_amoxor  = instr_amo_op && funct5 == 5'b00100;
  assign instr_amoor   = instr_amo_op && funct5 == 5'b01000;
  assign instr_amoand  = instr_amo_op && funct5 == 5'b01100;
  assign instr_amomin  = instr_amo_op && funct5 == 5'b10000;
  assign instr_amomax  = instr_amo_op && funct5 == 5'b10100;
  assign instr_amominu = instr_amo_op && funct5 == 5'b11000;
  assign instr_amomaxu = instr_amo_op && funct5 == 5'b11100;
  assign instr_lr = instr_amo_op && funct5 == 5'b00010 && instr[24:20] == 5'b0;
  assign instr_sc = instr_amo_op && funct5 == 5'b00011;
  assign instr_amo = instr_amoswap || instr_amoadd || instr_amoxor || instr_amoand ||
    instr_amoor || instr_amomin || instr_amomax || instr_amominu || instr_amomaxu;
  assign instr_atomic = instr_amo || instr_lr || instr_sc;

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
  logic instr_csr_access, is_csr_imm;
  assign instr_csr_access = instr_csrrw || instr_csrrs || instr_csrrc;
  assign is_csr_imm = instr_csrrwi || instr_csrrsi || instr_csrrci;

  // Raw instruction fields, not the muxed `rs1`/`rd`: those would put the compressed
  // register-select decode in a trap arm.
  logic instr_error, instr_mret, instr_wfi, instr_cebreak;
  assign instr_error = opcode == 5'b11100 && uncompressed && funct3 == 0 &&
    rs1_field == 5'b0 && rd_field == 5'b0;
  assign instr_ecall = instr_error && instr[31:20] == 12'h0;
  assign instr_cebreak = quadrant == 2'b10 && cfunct4 == 4'b1001 &&
    instr[11:7] == 5'b0 && instr[6:2] == 5'b0;
  assign instr_ebreak = (instr_error && instr[31:20] == 12'h1) || instr_cebreak;
  assign instr_mret = instr_error && instr[31:20] == 12'h302;
  assign instr_wfi = instr_error && instr[31:20] == 12'h105;

  logic instr_miscmem, instr_fence, instr_fencei;
  assign instr_miscmem = opcode == 5'b00011 && uncompressed;
  assign instr_fence  = instr_miscmem && funct3 == 3'b000;
  assign instr_fencei = instr_miscmem && funct3 == 3'b001;

  always_comb begin
    (* parallel_case, full_case *)
    case (1'b1)
      instr_beq || instr_bne || instr_blt || instr_bge || instr_bltu || instr_bgeu ||
        instr_sb || instr_sh || instr_sw || instr_cj || instr_cjr: rd = 0;
      instr_cjal || instr_cjalr: rd = 1;
      instr_clw || instr_caddi4spn: rd = {2'b01, c_rs2_prime};
      instr_csrai || instr_csrli || instr_candi || instr_cand ||
        instr_cor || instr_cxor || instr_csub: rd = {2'b01, c_rd_rs1_prime};
      default: rd = rd_field;
    endcase
  end // always_comb

  logic [4:0] rs1, rs2;
  regsel current_regs (.word(in.instr), .rs1(rs1), .rs2(rs2));

  logic uses_rs1, uses_rs2;
  assign uses_rs1 = !(instr_lui || instr_jal || instr_auipc || is_csr_imm);
  assign uses_rs2 = ((instr_add || instr_sub || instr_sll || instr_slt || instr_sltu ||
    instr_xor || instr_srl || instr_sra || instr_or || instr_and || instr_mul || instr_mulh ||
    instr_mulhu || instr_mulhsu || instr_div || instr_divu || instr_rem || instr_remu) &&
    !instr_math_immediate) || instr_sb || instr_sh || instr_sw ||
    instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu ||
    instr_amo || instr_sc;

  // `out`'s producer lands in executor_out exactly when this instruction reaches X, so a
  // match there forwards (below); `executor_out`'s own producer is already reachable
  // through the regfile's write-through bypass by then.
  logic dx_match_rs1, dx_match_rs2, ex_match_rs1, ex_match_rs2;
  assign dx_match_rs1 = out.valid && out.rd == rs1;
  assign dx_match_rs2 = out.valid && out.rd == rs2;
  assign ex_match_rs1 = executor_out.valid && executor_out.rd == rs1;
  assign ex_match_rs2 = executor_out.valid && executor_out.rd == rs2;

  // Mirrors executor.v's `in_has_result`: same-cycle ops, never a load/store/AMO/LR/SC/div/rem.
  logic out_has_result;
  assign out_has_result = out.is_add || out.is_sub || out.is_xor || out.is_or || out.is_and ||
    out.is_sll || out.is_slt || out.is_sltu || out.is_srl || out.is_sra ||
    out.is_mul || out.is_mulh || out.is_mulhu || out.is_mulhsu ||
    out.is_auipc || out.is_lui || out.is_jal || out.is_jalr || out.is_csr_access;

  // A CSR access's own rs1 feeds `csr_arg` in X, which reads `reg_rs1` verbatim -- it is
  // never a forwarding consumer, so a dx_match against it still stalls even when `out`
  // would otherwise be forwardable. rs2 has no such use (a CSR access never reads rs2).
  // x0 is excluded the same way the hazard check excludes it below: a producer that
  // targeted x0 must never forward, since x0 reads zero regardless of what `out.rd_data`
  // holds.
  logic fwd_rs1, fwd_rs2;
  assign fwd_rs1 = uses_rs1 && rs1 != 0 && dx_match_rs1 && out_has_result && !instr_csr_access;
  assign fwd_rs2 = uses_rs2 && rs2 != 0 && dx_match_rs2 && out_has_result;

  // hzA: dx_match without a forward select -- the producer will not publish a ready
  // result next cycle (a load/AMO/LR/SC, a div/rem just starting, or a CSR access's own
  // excluded rs1). hzB: ex_match whose producer is in the executor but not yet unpacked.
  // A ready ex_match (the old hzC) needs no stall at all: the regfile's write-through
  // bypass reaches it on its own, so that population is exactly the cycles this split
  // over hazard_rs1_dx/hazard_rs1_ex no longer counts.
  logic hazard_rs1_dx, hazard_rs1_ex, hazard_rs2_dx, hazard_rs2_ex;
  assign hazard_rs1_dx = uses_rs1 && rs1 != 0 && dx_match_rs1 && !fwd_rs1;
  assign hazard_rs1_ex = uses_rs1 && rs1 != 0 && ex_match_rs1 && !executor_out.rd_ready;
  assign hazard_rs2_dx = uses_rs2 && rs2 != 0 && dx_match_rs2 && !fwd_rs2;
  assign hazard_rs2_ex = uses_rs2 && rs2 != 0 && ex_match_rs2 && !executor_out.rd_ready;

  logic hazard_rs1, hazard_rs2, hazard;
  assign hazard_rs1 = hazard_rs1_dx || hazard_rs1_ex;
  assign hazard_rs2 = hazard_rs2_dx || hazard_rs2_ex;
  assign hazard = hazard_rs1 || hazard_rs2;

  // A CSR access/`mret`/`fence.i` must not interleave with older instructions.
  logic pipe_drained, serialize;
  assign pipe_drained = !out.valid && !executor_out.valid && !accessor_out_valid;
  assign serialize = (instr_csr_access || instr_mret || instr_fencei) && !pipe_drained;

  // X already consumed the AMO in `out`; re-presenting it would retire it twice.
  logic out_is_amo, atomic_stall;
  assign out_is_amo = out.is_amoswap || out.is_amoadd || out.is_amoxor || out.is_amoand ||
    out.is_amoor || out.is_amomin || out.is_amomax || out.is_amominu || out.is_amomaxu;
  assign atomic_stall = out.valid && out_is_amo && !x_busy;

  logic stall_own, stall;
  // X still working `out` holds the whole pipeline, presented pair included.
  assign stall_own = hazard || serialize || fetch_stall || atomic_stall || x_busy;
  assign stall = stall_own || bus_wait;

  // Over-asking is deliberate (a store-conditional with no reservation makes no
  // transaction, and X may yet find this instruction traps); under-asking is not.
  assign bus_request = !reset && !stall_own &&
    (instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw ||
     instr_sb || instr_sh || instr_sw || instr_atomic);

  // While X works `out`, keep presenting `out`'s own pair; the regfile answers late.
  assign read_rs1 = x_busy ? out.rs1 : rs1;
  assign read_rs2 = x_busy ? out.rs2 : rs2;

  assign issuing = !reset && !stall;
  assign predicted_pc = fetcher_pc + (uncompressed ? 32'd4 : 32'd2);

  always_ff @(posedge clk) begin
    if (reset) begin
      out <= '0;
    end else if (x_busy) begin
      out <= out;
    end else if (x_redirect) begin
      out <= '0;
    end else if (stall) begin
      out <= '0;
    end else if (interrupt_pending) begin
      out <= '0;
      out.valid <= 1'b1;
      out.is_interrupt <= 1'b1;
      out.pc <= fetcher_pc;
    end else begin
      out.valid <= 1'b1;
      out.is_interrupt <= 1'b0;
      out.imem_fault <= imem_fault;
      out.pc <= fetcher_pc;
      out.instr <= instr;
      out.immediate <= immediate;
      out.rd <= rd;
      out.rs1 <= rs1;
      out.rs2 <= rs2;
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
      out.is_lb <= instr_lb;
      out.is_lbu <= instr_lbu;
      out.is_lhu <= instr_lhu;
      out.is_lh <= instr_lh;
      out.is_lw <= instr_lw;
      out.is_sb <= instr_sb;
      out.is_sh <= instr_sh;
      out.is_sw <= instr_sw;
      out.is_amoswap <= instr_amoswap;
      out.is_amoadd <= instr_amoadd;
      out.is_amoxor <= instr_amoxor;
      out.is_amoand <= instr_amoand;
      out.is_amoor <= instr_amoor;
      out.is_amomin <= instr_amomin;
      out.is_amomax <= instr_amomax;
      out.is_amominu <= instr_amominu;
      out.is_amomaxu <= instr_amomaxu;
      out.is_lr <= instr_lr;
      out.is_sc <= instr_sc;
      out.is_auipc <= instr_auipc;
      out.is_lui <= instr_lui;
      out.is_jal <= instr_jal;
      out.is_jalr <= instr_jalr;
      out.is_beq <= instr_beq;
      out.is_bne <= instr_bne;
      out.is_blt <= instr_blt;
      out.is_bltu <= instr_bltu;
      out.is_bge <= instr_bge;
      out.is_bgeu <= instr_bgeu;
      out.is_ecall <= instr_ecall;
      out.is_ebreak <= instr_ebreak;
      out.is_mret <= instr_mret;
      out.is_wfi <= instr_wfi;
      out.is_fence <= instr_fence;
      out.is_fencei <= instr_fencei;
      out.is_csrrw <= instr_csrrw;
      out.is_csrrs <= instr_csrrs;
      out.is_csrrc <= instr_csrrc;
      out.is_csr_imm <= is_csr_imm;
      out.is_csr_access <= instr_csr_access;
      out.is_math_imm <= instr_math_immediate;
      out.fwd_rs1 <= fwd_rs1;
      out.fwd_rs2 <= fwd_rs2;
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  // Named continuous assigns, not part-selects inside the always_* blocks below: iverilog
  // cannot build a precise sensitivity entry for those (ADR-0037's class of defect).
  logic out_valid, out_is_interrupt;
  logic [4:0] out_rd;
  logic [31:0] out_instr, out_immediate;
  assign out_instr = out.instr;
  assign out_immediate = out.immediate;
  logic out_uncompressed;
  assign out_uncompressed = out_instr[1:0] == 2'b11;
  logic out_is_amoswap, out_is_amoadd, out_is_amoxor, out_is_amoand, out_is_amoor,
    out_is_amomin, out_is_amomax, out_is_amominu, out_is_amomaxu;
  assign out_valid = out.valid;
  assign out_is_interrupt = out.is_interrupt;
  assign out_rd = out.rd;
  assign out_is_amoswap = out.is_amoswap;
  assign out_is_amoadd = out.is_amoadd;
  assign out_is_amoxor = out.is_amoxor;
  assign out_is_amoand = out.is_amoand;
  assign out_is_amoor = out.is_amoor;
  assign out_is_amomin = out.is_amomin;
  assign out_is_amomax = out.is_amomax;
  assign out_is_amominu = out.is_amominu;
  assign out_is_amomaxu = out.is_amomaxu;

  // formal/traps.sv composes this module `-formal -noassume`, dropping executor.v's own
  // standalone-only assumes about `in` and everything its reference model re-derives
  // from `dx_instr`'s bits instead of trusting D's decode; the asserts below restate
  // each as a fact about `out` k-induction can use.
  logic out_is_auipc, out_is_jal, out_is_jalr, out_is_beq, out_is_bne, out_is_blt,
    out_is_bltu, out_is_bge, out_is_bgeu, out_is_add, out_is_sub, out_is_xor, out_is_or,
    out_is_and, out_is_sll, out_is_slt, out_is_sltu, out_is_srl, out_is_sra, out_is_mul,
    out_is_mulh, out_is_mulhu, out_is_mulhsu, out_is_div, out_is_divu, out_is_rem,
    out_is_remu, out_is_lui, out_is_lb, out_is_lbu, out_is_lh, out_is_lhu, out_is_lw,
    out_is_sb, out_is_sh, out_is_sw, out_is_ecall, out_is_ebreak, out_is_csrrw,
    out_is_csrrs, out_is_csrrc, out_is_mret, out_is_wfi, out_is_fence, out_is_fencei,
    out_is_lr, out_is_sc, out_is_csr_access;
  logic out_fwd_rs1;
  assign out_fwd_rs1 = out.fwd_rs1;
  assign out_is_auipc = out.is_auipc;
  assign out_is_jal = out.is_jal;
  assign out_is_jalr = out.is_jalr;
  assign out_is_beq = out.is_beq;
  assign out_is_bne = out.is_bne;
  assign out_is_blt = out.is_blt;
  assign out_is_bltu = out.is_bltu;
  assign out_is_bge = out.is_bge;
  assign out_is_bgeu = out.is_bgeu;
  assign out_is_add = out.is_add;
  assign out_is_sub = out.is_sub;
  assign out_is_xor = out.is_xor;
  assign out_is_or = out.is_or;
  assign out_is_and = out.is_and;
  assign out_is_sll = out.is_sll;
  assign out_is_slt = out.is_slt;
  assign out_is_sltu = out.is_sltu;
  assign out_is_srl = out.is_srl;
  assign out_is_sra = out.is_sra;
  assign out_is_mul = out.is_mul;
  assign out_is_mulh = out.is_mulh;
  assign out_is_mulhu = out.is_mulhu;
  assign out_is_mulhsu = out.is_mulhsu;
  assign out_is_div = out.is_div;
  assign out_is_divu = out.is_divu;
  assign out_is_rem = out.is_rem;
  assign out_is_remu = out.is_remu;
  assign out_is_lui = out.is_lui;
  assign out_is_lb = out.is_lb;
  assign out_is_lbu = out.is_lbu;
  assign out_is_lh = out.is_lh;
  assign out_is_lhu = out.is_lhu;
  assign out_is_lw = out.is_lw;
  assign out_is_sb = out.is_sb;
  assign out_is_sh = out.is_sh;
  assign out_is_sw = out.is_sw;
  assign out_is_ecall = out.is_ecall;
  assign out_is_ebreak = out.is_ebreak;
  assign out_is_csrrw = out.is_csrrw;
  assign out_is_csrrs = out.is_csrrs;
  assign out_is_csrrc = out.is_csrrc;
  assign out_is_mret = out.is_mret;
  assign out_is_wfi = out.is_wfi;
  assign out_is_fence = out.is_fence;
  assign out_is_fencei = out.is_fencei;
  assign out_is_lr = out.is_lr;
  assign out_is_sc = out.is_sc;
  assign out_is_csr_access = out.is_csr_access;

  // A bubble is the whole struct zeroed, never just `valid`.
  always_comb if (clocked && !out_valid) assert(out == '0);
  always_comb if (clocked && out_is_interrupt) assert(out_rd == 0);

  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(x_busy)) assert(out == $past(out));

  always_comb if (rs1 == 0) assert(!hazard_rs1);
  always_comb if (rs2 == 0) assert(!hazard_rs2);

  logic one_of;
  assign one_of = $onehot({instr_auipc, instr_jal, instr_jalr, instr_beq, instr_bne, instr_blt,
    instr_bltu, instr_bge, instr_bgeu, instr_add, instr_sub, instr_xor, instr_or, instr_and,
    instr_mul, instr_mulh, instr_mulhu, instr_mulhsu, instr_div, instr_divu, instr_rem,
    instr_remu, instr_sll, instr_slt, instr_sltu, instr_srl, instr_sra, instr_lui, instr_lb,
    instr_lbu, instr_lh, instr_lhu, instr_lw, instr_sb, instr_sh, instr_sw, instr_ecall,
    instr_ebreak, instr_csrrw, instr_csrrs, instr_csrrc,
    instr_mret, instr_wfi, instr_fence, instr_fencei,
    instr_amoswap, instr_amoadd, instr_amoxor, instr_amoand, instr_amoor, instr_amomin,
    instr_amomax, instr_amominu, instr_amomaxu, instr_lr, instr_sc});

  logic instr_valid_d;
  assign instr_valid_d = instr_auipc || instr_jal || instr_jalr || instr_beq || instr_bne ||
    instr_blt || instr_bltu || instr_bge || instr_bgeu || instr_add || instr_sub || instr_xor ||
    instr_or || instr_and || instr_mul || instr_mulh || instr_mulhu || instr_mulhsu || instr_div ||
    instr_divu || instr_rem || instr_remu || instr_sll || instr_slt || instr_sltu || instr_srl ||
    instr_sra || instr_lui || instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw ||
    instr_sb || instr_sh || instr_sw || instr_ecall || instr_ebreak || instr_mret || instr_wfi ||
    instr_fence || instr_fencei || instr_atomic || instr_csr_access;
  always_comb if (instr_valid_d) assert(one_of);

  always_comb assert($onehot0({instr_load_op || instr_jalr_op, instr_store_op,
    instr_lui_op || instr_auipc, instr_jal_op, instr_branch_op, instr_math_immediate_op,
    instr_amo_op, instr_clwsp, instr_cswsp, instr_csw, instr_clw, instr_cj || instr_cjal,
    instr_cbeqz || instr_cbnez, instr_cli, instr_clui, instr_caddi, instr_caddi16sp,
    instr_caddi4spn, instr_candi}));
  always_comb assert($onehot0({
    instr_beq || instr_bne || instr_blt || instr_bge || instr_bltu || instr_bgeu ||
      instr_sb || instr_sh || instr_sw || instr_cj || instr_cjr,
    instr_cjal || instr_cjalr,
    instr_clw || instr_caddi4spn,
    instr_csrai || instr_csrli || instr_candi || instr_cand ||
      instr_cor || instr_cxor || instr_csub}));
  always_comb assert($onehot0({instr_auipc, instr_csr_access, instr_lui,
    instr_jal || instr_jalr,
    instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu}));

  // Not `&& !out_is_interrupt`: that bubble zeroes every class flag too.
  always_comb if (clocked && out_valid)
    assert($onehot0({out_is_auipc, out_is_jal, out_is_jalr,
      out_is_beq, out_is_bne, out_is_blt, out_is_bltu, out_is_bge, out_is_bgeu,
      out_is_add, out_is_sub, out_is_xor, out_is_or, out_is_and,
      out_is_sll, out_is_slt, out_is_sltu, out_is_srl, out_is_sra,
      out_is_mul, out_is_mulh, out_is_mulhu, out_is_mulhsu,
      out_is_div, out_is_divu, out_is_rem, out_is_remu,
      out_is_lui,
      out_is_lb, out_is_lbu, out_is_lh, out_is_lhu, out_is_lw,
      out_is_sb, out_is_sh, out_is_sw,
      out_is_ecall, out_is_ebreak,
      out_is_csrrw, out_is_csrrs, out_is_csrrc,
      out_is_mret, out_is_wfi, out_is_fence, out_is_fencei,
      out_is_amoswap, out_is_amoadd, out_is_amoxor, out_is_amoand, out_is_amoor,
      out_is_amomin, out_is_amomax, out_is_amominu, out_is_amomaxu,
      out_is_lr, out_is_sc}));

  // The converse of the onehot0 above: no flag survives a reserved opcode or a zero word.
  logic out_any_class;
  assign out_any_class =
    out_is_auipc || out_is_jal || out_is_jalr ||
    out_is_beq || out_is_bne || out_is_blt || out_is_bltu || out_is_bge || out_is_bgeu ||
    out_is_add || out_is_sub || out_is_xor || out_is_or || out_is_and ||
    out_is_sll || out_is_slt || out_is_sltu || out_is_srl || out_is_sra ||
    out_is_mul || out_is_mulh || out_is_mulhu || out_is_mulhsu ||
    out_is_div || out_is_divu || out_is_rem || out_is_remu ||
    out_is_lui ||
    out_is_lb || out_is_lbu || out_is_lh || out_is_lhu || out_is_lw ||
    out_is_sb || out_is_sh || out_is_sw ||
    out_is_ecall || out_is_ebreak ||
    out_is_csrrw || out_is_csrrs || out_is_csrrc ||
    out_is_mret || out_is_wfi || out_is_fence || out_is_fencei ||
    out_is_amoswap || out_is_amoadd || out_is_amoxor || out_is_amoand || out_is_amoor ||
    out_is_amomin || out_is_amomax || out_is_amominu || out_is_amomaxu ||
    out_is_lr || out_is_sc;
  always_comb if (clocked && out_valid && out_uncompressed && out_instr[6:2] == 5'b11111)
    assert(!out_any_class);
  always_comb if (clocked && out_valid && out_instr == 32'b0)
    assert(!out_any_class);

  always_comb if (clocked && out_valid)
    assert(out_is_csr_access == (out_is_csrrw || out_is_csrrs || out_is_csrrc));

  // The Zkt-adjacent forwarding claim: a CSR access's own rs1 never forwards, since
  // `csr_arg` in X reads `reg_rs1` verbatim.
  always_comb if (clocked && out_is_csr_access) assert(!out_fwd_rs1);

  always_comb if (clocked && out_valid)
    assert(out_is_ebreak == (out_instr == 32'h0010_0073 || out_instr == 32'h0000_9002));
  always_comb if (clocked && out_valid)
    assert(out_is_ecall == (out_instr == 32'h0000_0073));

  // Gated on out_uncompressed throughout: is_lw/is_sw also cover a compressed form the
  // reference does not check, ruled out here by the quadrant bits.
  always_comb if (clocked && out_valid && out_uncompressed) begin
    assert(out_is_lb == (out_instr[6:2] == 5'b00000 && out_instr[14:12] == 3'b000));
    assert(out_is_lbu == (out_instr[6:2] == 5'b00000 && out_instr[14:12] == 3'b100));
    assert(out_is_lh == (out_instr[6:2] == 5'b00000 && out_instr[14:12] == 3'b001));
    assert(out_is_lhu == (out_instr[6:2] == 5'b00000 && out_instr[14:12] == 3'b101));
    assert(out_is_lw == (out_instr[6:2] == 5'b00000 && out_instr[14:12] == 3'b010));
    assert(out_is_sb == (out_instr[6:2] == 5'b01000 && out_instr[14:12] == 3'b000));
    assert(out_is_sh == (out_instr[6:2] == 5'b01000 && out_instr[14:12] == 3'b001));
    assert(out_is_sw == (out_instr[6:2] == 5'b01000 && out_instr[14:12] == 3'b010));
    if (out_is_lb || out_is_lbu || out_is_lh || out_is_lhu || out_is_lw)
      assert(out_immediate == {{20{out_instr[31]}}, out_instr[31:20]});
    if (out_is_sb || out_is_sh || out_is_sw)
      assert(out_immediate == {{20{out_instr[31]}}, out_instr[31:25], out_instr[11:7]});

    // The eleven A encodings, zero immediate included: X's atomic address check trusts
    // rs1 verbatim, true only because D hands an atomic a zero immediate.
    if (out_instr[6:2] == 5'b01011 && out_instr[14:12] == 3'b010) begin
      assert(out_is_amoswap == (out_instr[31:27] == 5'b00001));
      assert(out_is_amoadd == (out_instr[31:27] == 5'b00000));
      assert(out_is_amoxor == (out_instr[31:27] == 5'b00100));
      assert(out_is_amoand == (out_instr[31:27] == 5'b01100));
      assert(out_is_amoor == (out_instr[31:27] == 5'b01000));
      assert(out_is_amomin == (out_instr[31:27] == 5'b10000));
      assert(out_is_amomax == (out_instr[31:27] == 5'b10100));
      assert(out_is_amominu == (out_instr[31:27] == 5'b11000));
      assert(out_is_amomaxu == (out_instr[31:27] == 5'b11100));
      assert(out_is_lr == (out_instr[31:27] == 5'b00010 && out_instr[24:20] == 5'b0));
      assert(out_is_sc == (out_instr[31:27] == 5'b00011));
    end
    if (out_is_lr || out_is_sc || out_is_amoswap || out_is_amoadd || out_is_amoxor ||
        out_is_amoand || out_is_amoor || out_is_amomin || out_is_amomax ||
        out_is_amominu || out_is_amomaxu)
      assert(out_immediate == 32'b0);

    // A plain `add` must never fault. One direction only: out_is_add also covers
    // addi/c.add/c.mv, which the reference does not check.
    if (out_instr[6:2] == 5'b01100 && out_instr[14:12] == 3'b000 && out_instr[31:25] == 7'b0)
      assert(out_is_add);
  end
 `endif
endmodule
