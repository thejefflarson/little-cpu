`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// D decodes the buffered word and presents the register file a pair -- its own, never a
// guess -- so the answer X reads next cycle is exactly the pair this cycle's instruction
// needs. No register value is read here; everything that needs one (branch compare,
// address arithmetic, the region test, CSR access, every trap but the timer interrupt)
// commits in X. Fetch-address ownership (`fetch_pc`, and the redirect/predict mux that
// feeds it) lives in rtl/littlecpu.v, since it spans F and X and no longer belongs to one
// stage the way the fused decoder owned it.
module decoder (
  input  logic clk,
  input  logic reset,
  input  fetcher_output in,
  // X is still working the instruction it holds (the divider, or the region test's
  // deferred answer): re-present the same word next cycle, and do not overwrite `out`.
  input  logic x_busy,
  input  executor_output executor_out,
  // The fetch port went to a load or store this cycle, so `in.instr` holds a data word
  // rather than an instruction.
  input  logic fetch_stall,
  input  logic bus_wait,
  // Decode's request for the data bus, a cycle before X launches the transaction. The
  // platform ANDs it against its own grant; a grant term here would close the loop
  // through the arbiter.
  output logic bus_request,
  input  logic imem_fault,
  input  logic accessor_out_valid,
  output logic issuing,
  output logic [4:0] read_rs1,
  output logic [4:0] read_rs2,
  input  logic interrupt_pending,
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

  // Read off the raw instruction fields, not the muxed `rs1`/`rd`: those would put the
  // compressed register-select decode in a trap arm.
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

  // No forwarding in this stage (B2 adds it): a RAW match against the instruction X is
  // currently resolving, or the one it just resolved and has not yet unpacked, simply
  // stalls. A match against `executor_out` covers the load/AMO case, where the retired
  // value is still a cycle away through `accessor_out`.
  logic dx_match_rs1, dx_match_rs2, ex_match_rs1, ex_match_rs2;
  assign dx_match_rs1 = out.valid && out.rd == rs1;
  assign dx_match_rs2 = out.valid && out.rd == rs2;
  assign ex_match_rs1 = executor_out.valid && executor_out.rd == rs1;
  assign ex_match_rs2 = executor_out.valid && executor_out.rd == rs2;

  logic hazard_rs1, hazard_rs2, hazard;
  assign hazard_rs1 = uses_rs1 && rs1 != 0 && (dx_match_rs1 || ex_match_rs1);
  assign hazard_rs2 = uses_rs2 && rs2 != 0 && (dx_match_rs2 || ex_match_rs2);
  assign hazard = hazard_rs1 || hazard_rs2;

  // Two reasons share one wait, and narrowing it to suit one breaks the other: a CSR
  // access or `mret` must not interleave with older instructions, and `fence.i` waits
  // because text is writable and the fetch address goes out a cycle early.
  logic pipe_drained, serialize;
  assign pipe_drained = !out.valid && !executor_out.valid && !accessor_out_valid;
  assign serialize = (instr_csr_access || instr_mret || instr_fencei) && !pipe_drained;

  // X has already consumed the AMO in `out`; re-presenting it would retire it twice, so
  // this bubbles rather than holds.
  logic out_is_amo, atomic_stall;
  assign out_is_amo = out.is_amoswap || out.is_amoadd || out.is_amoxor || out.is_amoand ||
    out.is_amoor || out.is_amomin || out.is_amomax || out.is_amominu || out.is_amomaxu;
  assign atomic_stall = out.valid && out_is_amo && !x_busy;

  logic stall_own, stall;
  assign stall_own = hazard || serialize || fetch_stall || atomic_stall;
  assign stall = stall_own || bus_wait;

  // Over-asking is deliberate: a store-conditional with no reservation makes no
  // transaction, and X may yet find this instruction traps. Under-asking would put two
  // initiators on the bus at once.
  assign bus_request = !reset && !stall_own &&
    (instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw ||
     instr_sb || instr_sh || instr_sw || instr_atomic);

  assign read_rs1 = rs1;
  assign read_rs2 = rs2;

  assign issuing = !reset && !stall;

  always_ff @(posedge clk) begin
    if (reset) begin
      out <= '0;
    end else if (x_busy) begin
      out <= out;
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
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  always_comb if (clocked && !out.valid) assert(out.rd == 0);
  always_comb if (clocked && out.is_interrupt) assert(out.rd == 0);

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

  always_comb if (out.valid && !out.is_interrupt)
    assert($onehot0({out.is_amoswap, out.is_amoadd, out.is_amoxor, out.is_amoand, out.is_amoor,
      out.is_amomin, out.is_amomax, out.is_amominu, out.is_amomaxu}));
 `endif
endmodule
