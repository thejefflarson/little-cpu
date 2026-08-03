`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module decoder (
  input  logic clk,
  input  logic reset,
  input  fetcher_output in,
  input  logic [31:0] reg_rs1,
  input  logic [31:0] reg_rs2,
  input  executor_output executor_out,
  // Decode bubbles its own output for the whole divide rather than holding it:
  // the divider latches its operands at issue, and a held decoder_out would be
  // misread as freshly issued the moment the executor returns to `init`.
  input  logic divider_stall,
  // Held, not bubbled, for exactly the cycle this is asserted. Nothing
  // downstream has consumed this cycle's decoder_out when it fires, so bubbling
  // would silently drop that instruction.
  input  logic accessor_stall,
  // The instruction memory took its read port for a data access, so the window
  // presented this cycle is a data word rather than the instruction at `pc`
  // (ADR-0059). A divider or accessor freeze on the same cycle outranks it.
  input  logic fetch_stall,
  // A load in the accessor's turnaround is a live producer for one cycle that
  // neither decoder_out nor executor_out can see it in. That gap only.
  input  logic       accessor_pending_valid,
  input  logic [4:0] accessor_pending_rd,
  // The drain predicate's fourth slot: a *store* in the accessor appears in none
  // of the other three, and without it a CSR instruction issues early and
  // `minstret` is wrong exactly when a store is in flight (ADR-0026).
  input  logic       accessor_out_valid,
  output logic [31:0] pc,
  // The value `pc` will hold next cycle, published combinationally this one so a
  // synchronous instruction memory can latch it on the same edge `pc` takes it.
  // That is how fetch stays combinational from decode's point of view on a part
  // with no combinational-read memory (invariant 1, ADR-0054).
  output logic [31:0] next_pc,
  output logic [4:0] rs1,
  output logic [4:0] rs2,
  // rtl/csrs.v is a sibling module, not a stage: it answers combinationally in
  // time for the publish block, and commits on the edge the accessing
  // instruction issues, so nothing downstream carries CSR state (ADR-0005).
  output logic [11:0] csr_addr,
  output logic        csr_ren,
  output logic        csr_wen,
  output logic [31:0] csr_wdata,
  input  logic [31:0] csr_rdata,
  input  logic        csr_implemented,
  output logic        instret,
  // A trap is a branch: it redirects through the same `next_pc` chain every
  // other branch uses, which is why nothing downstream needs a kill signal.
  output logic        trap_entry,
  output logic [31:0] trap_cause,
  output logic [31:0] trap_epc,
  output logic        mret_entry,
  input  logic [31:0] mtvec,
  input  logic [31:0] mepc,
 `ifdef RISCV_FORMAL
  input  rvfi_csr64   csr_rvfi_mcycle,
  input  rvfi_csr64   csr_rvfi_minstret,
  input  rvfi_csr32   csr_rvfi_mscratch,
 `endif
  output decoder_output out
);
  // Zero-extending a compressed instruction leaves no neighbouring-instruction
  // garbage in the upper half for a decode path that forgets to gate on
  // `uncompressed` to read (ADR-0021).
  logic [31:0] instr;
  assign instr = (in.instr[1:0] == 2'b11) ? in.instr : {16'b0, in.instr[15:0]};

  // Base encodings (RISC-V unprivileged spec, Ch. 2.2). Hoisted out of the
  // always_comb blocks below because iverilog cannot build a precise sensitivity
  // entry for a constant part-select taken inside one; prefer a continuous
  // assign for any field read added later (ADR-0034).
  logic [4:0] rd_field, rs1_field, rs2_field;
  assign rd_field  = instr[11:7];
  assign rs1_field = instr[19:15];
  assign rs2_field = instr[24:20];

  // Compressed encodings (Ch. 16). instr[11:7] doubles as rd/rs1 in CI/CR
  // formats, so `rd_field` is reused there rather than aliased.
  logic [2:0] c_rd_rs1_prime, c_rs2_prime;
  logic [4:0] c_rs2_field;
  assign c_rd_rs1_prime = instr[9:7];
  assign c_rs2_prime    = instr[4:2];
  assign c_rs2_field    = instr[6:2];
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

  // Forward declarations: the blocks below read these before their own assign
  // groups come into scope, and iverilog (unlike yosys) requires every
  // identifier declared before its first use.
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
      instr_load_op || instr_jalr_op: immediate = i_immediate;
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

  // The Zicsr immediate forms stay named apart from the register forms: their
  // rs1 field is a zimm, so `uses_rs1` must not interlock on it and `csr_arg`
  // must read the encoding rather than the register file.
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

  assign csr_addr = instr[31:20];
  logic [31:0] csr_arg;
  assign csr_arg = is_csr_imm ? {27'b0, rs1_field} : reg_rs1;

  // Zicsr's suppression rules. Suppressing the write when the source is zero is
  // what makes `csrr` legal against a read-only CSR rather than an
  // illegal-instruction trap. The read-suppression rule already falls out of
  // `rd` gating rtl/writeback.v's `wen`, and is named here only for RVFI.
  logic csr_src_zero, csr_write_op, csr_read_op;
  assign csr_src_zero = rs1_field == 5'b0;
  assign csr_write_op = instr_csr_access && !((instr_csrrs || instr_csrrc) && csr_src_zero);
  assign csr_read_op  = instr_csr_access && !(instr_csrrw && rd_field == 5'b0);
  assign csr_wdata = instr_csrrw ? csr_arg :
                     instr_csrrs ? (csr_rdata | csr_arg) :
                                   (csr_rdata & ~csr_arg);

  // SYSTEM with funct3 == 0. An encoding missing from this group decodes to
  // nothing, which now means an illegal-instruction trap rather than a silent
  // NOP.
  logic instr_error, instr_mret, instr_wfi, instr_cebreak;
  assign instr_error = opcode == 5'b11100 && uncompressed && funct3 == 0 && rs1 == 0 && rd == 0;
  assign instr_ecall = instr_error && instr[31:20] == 12'h0;
  // C.EBREAK needs its own line because it is the rd == 0, rs2 == 0 corner of
  // quadrant 2's funct4 == 4'b1001 row that `instr_cjalr` and `instr_cadd` each
  // exclude by a different field. Without it the encoding decodes to nothing and
  // traps as illegal rather than as a breakpoint.
  assign instr_cebreak = quadrant == 2'b10 && cfunct4 == 4'b1001 &&
    instr[11:7] == 5'b0 && instr[6:2] == 5'b0;
  assign instr_ebreak = (instr_error && instr[31:20] == 12'h1) || instr_cebreak;
  assign instr_mret = instr_error && instr[31:20] == 12'h302;
  // `wfi` is a NOP, which is spec-legal: `mie`/`mip` are read-only zero, so no
  // interrupt could ever resume it.
  assign instr_wfi = instr_error && instr[31:20] == 12'h105;

  // Both MISC-MEM forms are NOPs here -- one hart and no icache -- so only
  // funct3 is tested; their other fields are legal-value fields with nothing to
  // act on. `fence.i`'s ordering half is the serialization term below.
  logic instr_miscmem, instr_fence, instr_fencei;
  assign instr_miscmem = opcode == 5'b00011 && uncompressed;
  assign instr_fence  = instr_miscmem && funct3 == 3'b000;
  assign instr_fencei = instr_miscmem && funct3 == 3'b001;

  logic instr_valid;

  assign instr_valid = instr_auipc || instr_jal || instr_jalr || instr_beq || instr_bne || instr_blt
    || instr_bltu || instr_bge || instr_bgeu || instr_add || instr_sub || instr_xor || instr_or ||
    instr_and || instr_mul || instr_mulh || instr_mulhu || instr_mulhsu || instr_div || instr_divu
    || instr_rem || instr_remu || instr_sll || instr_slt || instr_sltu || instr_srl || instr_sra ||
    instr_lui || instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw || instr_sb || instr_sh
    || instr_sw || instr_ecall || instr_ebreak || instr_mret || instr_wfi || instr_fence ||
    instr_fencei || (instr_csr_access && csr_implemented);

  // The effective address, used here to decide misalignment and again in the
  // publish block as `out.mem_addr`. That it is available in decode for free is
  // what makes invariant 2 affordable.
  logic [31:0] mem_addr_calc;
  assign mem_addr_calc = $signed(immediate) + $signed(reg_rs1);

  logic load_misaligned, store_misaligned;
  assign load_misaligned  = (instr_lw && mem_addr_calc[1:0] != 2'b00) ||
                            ((instr_lh || instr_lhu) && mem_addr_calc[0] != 1'b0);
  assign store_misaligned = (instr_sw && mem_addr_calc[1:0] != 2'b00) ||
                            (instr_sh && mem_addr_calc[0] != 1'b0);

  // `csr_write_op` already has Zicsr's suppression rules applied, so `csrr misa`
  // stays legal while `csrw misa` does not.
  logic csr_readonly_write, instr_illegal;
  assign csr_readonly_write = instr_csr_access && csr_write_op && csr_addr[11:10] == 2'b11;
  assign instr_illegal = !instr_valid || csr_readonly_write;

  // Instruction-address-misaligned (0) is absent because C makes 2-byte targets
  // legal, so it is unreachable.
  localparam logic [31:0] CAUSE_ILLEGAL_INSTRUCTION = 32'd2;
  localparam logic [31:0] CAUSE_BREAKPOINT          = 32'd3;
  localparam logic [31:0] CAUSE_LOAD_MISALIGNED     = 32'd4;
  localparam logic [31:0] CAUSE_STORE_MISALIGNED    = 32'd6;
  localparam logic [31:0] CAUSE_ECALL_M             = 32'd11;

  logic trap_pending;
  assign trap_pending = instr_illegal || instr_ebreak || instr_ecall ||
                        load_misaligned || store_misaligned;

  // No `(* parallel_case *)`. The five causes cannot co-occur today, so this
  // encoder is vacuous -- but claiming one-hot would turn that disjointness from
  // something the `ifdef FORMAL` block asserts into something the RTL assumes,
  // and a later cause that broke it would be a lie to synthesis rather than a
  // failed proof (ADR-0030).
  always_comb begin
    case (1'b1)
      instr_illegal:    trap_cause = CAUSE_ILLEGAL_INSTRUCTION;
      instr_ebreak:     trap_cause = CAUSE_BREAKPOINT;
      instr_ecall:      trap_cause = CAUSE_ECALL_M;
      load_misaligned:  trap_cause = CAUSE_LOAD_MISALIGNED;
      store_misaligned: trap_cause = CAUSE_STORE_MISALIGNED;
      default:          trap_cause = 32'b0;
    endcase
  end

  // The faulting instruction's address, not the next one's -- that is what lets
  // a handler fix up and resume.
  assign trap_epc = fetcher_pc;

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

  always_comb begin
    (* parallel_case, full_case *)
    case (1'b1)
      instr_clwsp || instr_cswsp || instr_caddi4spn: rs1 = 2;
      instr_clw || instr_csw || instr_cbeqz || instr_cbnez ||
        instr_csrai || instr_csrli || instr_candi || instr_cand ||
        instr_cor || instr_cxor || instr_csub: rs1 = {2'b01, c_rd_rs1_prime};
      instr_cjr || instr_cjalr || instr_cslli: rs1 = rd_field;
      instr_cli || instr_cmv: rs1 = 0;
      instr_caddi || instr_caddi16sp || instr_cadd: rs1 = rd_field;
      default: rs1 = rs1_field;
    endcase // case (1'b1)
  end

  always_comb begin
    (* parallel_case, full_case *)
    case(1'b1)
      instr_cswsp || instr_cslli || instr_csrai || instr_csrli || instr_cmv || instr_cadd: rs2 = c_rs2_field;
      instr_csw || instr_cand || instr_cor || instr_cxor || instr_csub: rs2 = {2'b01, c_rs2_prime};
      instr_cbeqz || instr_cbnez: rs2 = 0;
      default: rs2 = rs2_field;
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

  // "Does this instruction really read rsN?" -- not a shift's shamt, not a
  // U-type immediate's overlapping bit positions, not a zimm. Both the
  // scoreboard and the operand-fetch stall key off these, so widening either
  // predicate stalls on registers nothing reads and narrowing it reads stale
  // ones (ADR-0004).
  logic uses_rs1, uses_rs2;
  assign uses_rs1 = !(instr_lui || instr_jal || instr_auipc || is_csr_imm);
  assign uses_rs2 = (instr_math && !instr_math_immediate) || instr_sb || instr_sh || instr_sw ||
    instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu;

  // Do not factor these two into a shared `function automatic live_producer(r)`.
  // iverilog builds a continuous assign's sensitivity list from the call's
  // arguments, so a body that also reads `out`/`executor_out`/`accessor_pending_*`
  // never re-evaluates when those change -- silent under-sensitivity, with no
  // diagnostic, which froze the iverilog leg at the first RAW hazard of every
  // program while cxxrtl and the ladder stayed green (ADR-0037).
  logic live_rs1, live_rs2;
  assign live_rs1 = (out.valid && out.rd == rs1) ||
    (executor_out.valid && executor_out.rd == rs1) ||
    (accessor_pending_valid && accessor_pending_rd == rs1);
  assign live_rs2 = (out.valid && out.rd == rs2) ||
    (executor_out.valid && executor_out.rd == rs2) ||
    (accessor_pending_valid && accessor_pending_rd == rs2);

  // Serialization holds these in decode until all four in-flight slots are empty
  // (invariant 5). Two distinct reasons, so do not narrow it to suit one: a CSR
  // instruction and `mret` are here for `minstret` exactness and not for hazard
  // safety (ADR-0027), while `fence.i` needs the older store's write edge to have
  // passed before the fetch a cycle ahead of it latches stale text (ADR-0061).
  logic pipe_drained, serialize;
  assign pipe_drained = !out.valid && !executor_out.valid && !accessor_out_valid &&
    !accessor_pending_valid;
  assign serialize = (instr_csr_access || instr_mret || instr_fencei) && !pipe_drained;

  logic hazard_rs1, hazard_rs2, hazard, stall;
  assign hazard_rs1 = uses_rs1 && rs1 != 0 && live_rs1;
  assign hazard_rs2 = uses_rs2 && rs2 != 0 && live_rs2;
  assign hazard = hazard_rs1 || hazard_rs2 || serialize;

 `ifdef RISCV_FORMAL
  // These must stay exactly `uses_rs1`/`uses_rs2` and not a looser formula. The
  // monitor's reordered-channel shadow-register check compares any reported
  // address against its own last-write model unconditionally, so over-reporting
  // one the instruction never read fails error 132 on a correct core.
  logic rvfi_rs1_valid, rvfi_rs2_valid;
  assign rvfi_rs1_valid = !instr_lui && !instr_jal && !instr_auipc && !is_csr_imm;
  assign rvfi_rs2_valid = uses_rs2;
 `endif
  // Register reads take one cycle, so an instruction presents its address pair,
  // bubbles, and issues on the next cycle (invariant 9). The predicate states
  // that rule directly rather than counting cycles: valid for exactly the pair
  // presented last cycle, and only for the ports this instruction reads. The
  // instructions that skip a fetch cycle all override out.rs1/out.rs2 in the
  // publish block, so skipping cannot leak a stale operand.
  logic [4:0] prev_rs1, prev_rs2;
  logic       read_taken, operand_stall;
  always_ff @(posedge clk) begin
    if (reset) begin
      prev_rs1   <= 5'd0;
      prev_rs2   <= 5'd0;
      read_taken <= 1'b0;
    end else begin
      prev_rs1   <= rs1;
      prev_rs2   <= rs2;
      read_taken <= 1'b1;
    end
  end
  assign operand_stall = !read_taken || (uses_rs1 && prev_rs1 != rs1) ||
                                        (uses_rs2 && prev_rs2 != rs2);

  // All six stall reasons, one broadcast. They freeze the PC alike; the publish
  // block below is where they differ (ADR-0009).
  assign stall = hazard || operand_stall || divider_stall || accessor_stall || fetch_stall;

  // Kept out of the `next_pc` chain below so each comparison is a
  // self-determined expression with both operands explicitly `$signed`. Signed
  // arithmetic buried in an arm of a conditional has silently gone unsigned in
  // this repo twice.
  logic branch_taken;
  always_comb begin
    (* parallel_case *)
    case (1'b1)
      instr_beq:  branch_taken = reg_rs1 == reg_rs2;
      instr_bne:  branch_taken = reg_rs1 != reg_rs2;
      instr_blt:  branch_taken = $signed(reg_rs1) <  $signed(reg_rs2);
      instr_bge:  branch_taken = $signed(reg_rs1) >= $signed(reg_rs2);
      instr_bltu: branch_taken = reg_rs1 <  reg_rs2;
      instr_bgeu: branch_taken = reg_rs1 >= reg_rs2;
      default:    branch_taken = 1'b0;
    endcase
  end

  // Every redirect this core takes, as one priority chain (ADR-0054). No
  // `(* parallel_case *)`: `stall` and `trap_pending` do co-occur, so claiming
  // one-hot would be a lie to synthesis.
  always_comb begin
    case (1'b1)
      reset:                     next_pc = 32'b0;
      stall:                     next_pc = pc;
      trap_pending:              next_pc = mtvec;
      instr_mret:                next_pc = mepc;
      instr_jalr:                next_pc = ($signed(immediate) + $signed(reg_rs1)) & 32'hfffffffe;
      instr_jal:                 next_pc = $signed(fetcher_pc) + $signed(immediate);
      branch_taken:              next_pc = fetcher_pc + immediate;
      // Sequential: +4 for a 32-bit instruction, +2 for a compressed one.
      default:                   next_pc = fetcher_pc + pc_inc;
    endcase
  end

  // Must stay term-for-term identical to the publish block's `else` guard, and
  // in particular must not gain `&& in.valid`: that arm does not test it either.
  // The two disagreeing would apply a CSR write once per stall cycle rather than
  // once per instruction, silently.
  logic issuing;
  assign issuing = !reset && !stall;

  logic committing;
  assign committing = issuing && !trap_pending;
  assign csr_ren = committing && csr_read_op;
  assign csr_wen = committing && csr_write_op;
  // A trapping instruction issues and retires in RVFI, but it did not retire
  // architecturally, so it must not be counted (ADR-0027).
  assign instret = committing;

  assign trap_entry = issuing && trap_pending;
  assign mret_entry = committing && instr_mret;

  always_ff @(posedge clk) pc <= next_pc;

  always_ff @(posedge clk) begin
    if (reset) begin
      out <= '0;
    end else if (divider_stall || accessor_stall) begin
      // Both fire a cycle behind their cause, so decoder_out holds an instruction
      // nothing downstream has consumed and bubbling would drop it. This arm
      // outranking a same-cycle `fetch_stall` is a ruling rather than a
      // consequence of statement order (ADR-0060), and the `ifdef FORMAL` block
      // asserts both directions because reordering these two arms is silent.
      out <= out;
    end else if (hazard || operand_stall || fetch_stall) begin
      // These bubble rather than hold: nothing stops the executor reading `in`
      // every cycle here, so a held decoder_out would be reprocessed instead of
      // the stalled instruction being retried.
      out <= '0;
    end else begin
      // Trap commit lives at the bottom of this arm, which is what buys -- with
      // no guard of its own -- no trap on a stalled cycle, none from a bubble,
      // and no double commit. It also means the scoreboard has already cleared
      // rs1, so the misalignment test reads its architectural value.
      out.valid <= 1'b1;
      out.mem_addr <= mem_addr_calc;
      out.rs1 <= instr_lui ? immediate : reg_rs1;
      out.rs2 <= instr_math ? math_arg : reg_rs2;
      out.rd <= rd;
     `ifdef RISCV_FORMAL
      // `next_pc` rather than `pc`: the non-blocking assignment above is not
      // visible until next cycle. This arm runs only when `stall` is low, so it
      // is the real target and never a held PC.
      out.rvfi.pc_wdata <= next_pc;
      out.rvfi.insn <= instr;
      out.rvfi.pc_rdata <= fetcher_pc;
      out.rvfi.trap <= trap_pending;
      out.rvfi.rs1_addr <= rvfi_rs1_valid ? rs1 : 5'b0;
      out.rvfi.rs2_addr <= rvfi_rs2_valid ? rs2 : 5'b0;
      out.rvfi.rs1_rdata <= rvfi_rs1_valid ? reg_rs1 : 32'b0;
      out.rvfi.rs2_rdata <= rvfi_rs2_valid ? reg_rs2 : 32'b0;
      // A non-CSR instruction gets all-zero masks for free, because
      // `csr_ren`/`csr_wen` are low for it.
      out.rvfi.csr_mcycle   <= csr_rvfi_mcycle;
      out.rvfi.csr_minstret <= csr_rvfi_minstret;
      out.rvfi.csr_mscratch <= csr_rvfi_mscratch;
     `endif
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
      out.is_valid_instr <= instr_valid;
      (* parallel_case *)
      case(1'b1)
        default: ;
        instr_auipc: begin
          // AUIPC has no rs1/rs2 fields -- the bits the default selection reads
          // are part of the U-type immediate -- so its operands are supplied
          // directly here and added through the is_add pass-through.
          out.rd <= rd;
          out.rs1 <= fetcher_pc;
          out.rs2 <= immediate;
          out.is_add <= 1;
        end

        instr_csr_access: begin
          // Only the read result needs the datapath; rtl/csrs.v committed the
          // write on this same edge.
          out.rs1 <= csr_rdata;
          out.rs2 <= 32'b0;
          out.is_add <= 1;
        end

        instr_jal || instr_jalr: begin
          // The target is `next_pc`; what is left is the return address.
          out.rs1 <= fetcher_pc;
          out.rs2 <= pc_inc;
          out.rd <= rd;
          out.is_add <= 1;
        end

        instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu: begin
          // A branch is resolved entirely in `branch_taken`/`next_pc` and carries
          // nothing past decode.
          out.rs1 <= 0;
          out.rs2 <= 0;
          out.rd <= 0;
        end
      endcase

      // Last in the block so non-blocking last-write-wins beats every flag the
      // arms above set, and no arm has to know traps exist. The instruction
      // still retires with `out.valid` high, having architecturally done nothing
      // but redirect (ADR-0028) -- clearing these flags is the whole mechanism,
      // since rtl/accessor.v issues no bus request without one and `out.rd` of 0
      // gates rtl/writeback.v's `wen`.
      if (trap_pending) begin
        out.is_add <= 0; out.is_sub <= 0; out.is_xor <= 0; out.is_or <= 0; out.is_and <= 0;
        out.is_mul <= 0; out.is_mulh <= 0; out.is_mulhu <= 0; out.is_mulhsu <= 0;
        out.is_div <= 0; out.is_divu <= 0; out.is_rem <= 0; out.is_remu <= 0;
        out.is_sll <= 0; out.is_slt <= 0; out.is_sltu <= 0; out.is_srl <= 0; out.is_sra <= 0;
        out.is_lui <= 0; out.is_lb <= 0; out.is_lbu <= 0; out.is_lhu <= 0; out.is_lh <= 0; out.is_lw <= 0;
        out.is_sb <= 0; out.is_sh <= 0; out.is_sw <= 0;
        out.is_ecall <= 0; out.is_ebreak <= 0;
        out.rd <= 0; // prevent writeback to arbitrary register
      end
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // Assumes reset before the first edge. Discharged nowhere -- it is structural,
  // true of every harness in the tree -- and in force over the whole task,
  // because before that edge `out` and the history registers below have no
  // defined value.
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // Assumes reset never returns. Discharged nowhere, same reason. Written for
  // the pc-increment assertions below, which would otherwise have to account for
  // every point reset could jump pc back to 0; being unguarded it also covers
  // the trap-arm assertions, harmlessly, since those carry `!prev_reset`.
  always_comb if (clocked) assume(!reset);

  // Assumes the fetcher echoes this module's own pc, which it does
  // combinationally in rtl/fetcher.v. Discharged by formal/pcloop.sv, which
  // instantiates the real fetcher and asserts the echo instead; that task is on
  // CI. In force over the whole task, wider than the pc-increment pair it was
  // written for -- inert only because nothing else below reads `in.pc`.
  always_comb assume(in.pc == pc);

  // Every "previous cycle" signal below is its own registered copy rather than a
  // $past() on a free input: $past() chained across another register adds a cycle
  // of history the solver can fill with a pre-reset value this standalone proof
  // cannot rule out. `trap_pending` and `instr_mret` belong in `branch_jump`
  // because both override `pc` -- forgetting either is how "a trap is a branch"
  // stops being true in the proof while staying true in the RTL.
  logic branch_jump;
  always_ff @(posedge clk) if (reset) branch_jump <= 1'b0;
    else branch_jump <= instr_jal || instr_jalr || instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu || trap_pending || instr_mret;
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

  always_ff @(posedge clk) if (clocked && prev_stall && !prev_reset) assert(pc == past_pc);

  // Both directions of ADR-0060's ruling, because the publish block's arm order
  // is all that decides which happens and each failure is silent: bubbling on
  // the coincidence drops an unconsumed instruction, holding on a plain steal
  // republishes one the executor reads every cycle.
  decoder_output past_out;
  logic prev_hold_and_steal, prev_steal_only;
  always_ff @(posedge clk) begin
    past_out            <= out;
    prev_hold_and_steal <= fetch_stall && (divider_stall || accessor_stall);
    prev_steal_only     <= fetch_stall && !divider_stall && !accessor_stall;
  end
  always_comb if (clocked && !prev_reset && prev_hold_and_steal) assert(out == past_out);
  always_comb if (clocked && !prev_reset && prev_steal_only)     assert(out == '0);

  // A tautology one flip-flop wide, which is the point: it is what fails if a
  // later edit gives `pc` a second driver, and that would put the instruction
  // memory a cycle out of step with decode with no symptom but wrong
  // instructions (ADR-0054).
  logic [31:0] past_next_pc;
  always_ff @(posedge clk) past_next_pc <= next_pc;
  always_comb if (clocked) assert(pc == past_next_pc);

  // A bubble is fully zeroed, never a partial one that could sneak an rd
  // through.
  always_comb if (clocked && !out.valid) assert(out.rd == 0);

  always_comb if (rs1 == 0) assert(!hazard_rs1);
  always_comb if (rs2 == 0) assert(!hazard_rs2);

  logic one_of;
  // $onehot() rather than XOR, so a pairwise overlap is detected instead of
  // cancelling out.
  assign one_of = $onehot({instr_auipc, instr_jal, instr_jalr, instr_beq, instr_bne, instr_blt,
    instr_bltu, instr_bge, instr_bgeu, instr_add, instr_sub, instr_xor, instr_or, instr_and,
    instr_mul, instr_mulh, instr_mulhu, instr_mulhsu, instr_div, instr_divu, instr_rem,
    instr_remu, instr_sll, instr_slt, instr_sltu, instr_srl, instr_sra, instr_lui, instr_lb,
    instr_lbu, instr_lh, instr_lhu, instr_lw, instr_sb, instr_sh, instr_sw, instr_ecall,
    // Every encoding `instr_valid` admits has to appear here, and the forms that
    // differ only in funct3 or funct12 stay listed one per term, so a decode
    // change that made two of them true at once is still caught.
    instr_ebreak, instr_csrrw, instr_csrrs, instr_csrrc,
    instr_mret, instr_wfi, instr_fence, instr_fencei});

  always_comb if (instr_valid) assert(one_of);

  // What makes the trap-cause encoder above safe to leave un-parallel_case'd.
  // Add a sixth cause that overlaps an existing one and this fails, rather than
  // the behaviour quietly becoming whatever the encoder produced.
  always_comb assert($onehot0({instr_illegal, instr_ebreak, instr_ecall,
                               load_misaligned, store_misaligned}));

  // Written per cause rather than as a restatement of the case statement, so a
  // reordering of the arms is caught rather than mirrored.
  always_comb if (instr_illegal)    assert(trap_cause == CAUSE_ILLEGAL_INSTRUCTION);
  always_comb if (instr_ebreak)     assert(trap_cause == CAUSE_BREAKPOINT);
  always_comb if (instr_ecall)      assert(trap_cause == CAUSE_ECALL_M);
  always_comb if (load_misaligned)  assert(trap_cause == CAUSE_LOAD_MISALIGNED);
  always_comb if (store_misaligned) assert(trap_cause == CAUSE_STORE_MISALIGNED);
  always_comb if (!trap_pending)    assert(trap_cause == 32'b0);

  // `mtvec`/`mepc` are free inputs here, so these are registered copies rather
  // than reads of the live input.
  logic        prev_trap_entry, prev_mret_entry;
  logic [31:0] prev_mtvec, prev_mepc;
  always_ff @(posedge clk) begin
    prev_trap_entry <= trap_entry;
    prev_mret_entry <= mret_entry;
    prev_mtvec      <= mtvec;
    prev_mepc       <= mepc;
  end

  // That the redirect went to mtvec is something no ladder check can see:
  // `rvfi_insn_check` drops every value assertion under `spec_trap`, and
  // pc_fwd/pc_bwd accept any target that is honestly reported.
  always_comb if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_comb if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);

  always_comb if (clocked && !prev_reset && prev_trap_entry) begin
    assert(out.rd == 5'b0);
    assert(!out.is_lb && !out.is_lbu && !out.is_lh && !out.is_lhu && !out.is_lw);
    assert(!out.is_sb && !out.is_sh && !out.is_sw);
  end

  // Asserted so that an edit re-gating either on `instr_valid` -- the weaker
  // pre-M3 spelling -- fails (ADR-0027).
  always_comb if (trap_pending) assert(!instret && !csr_wen && !csr_ren);
  always_comb assert(!(trap_entry && mret_entry));
 `endif
endmodule
