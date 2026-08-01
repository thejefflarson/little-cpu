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
  // ADR-0026: the fourth stall reason (CSR serialization, CLAUDE.md
  // invariant 5) needs a genuine drain predicate, and
  // `accessor_pending_valid` above covers LOADS ONLY. A store sitting in the
  // accessor appears in none of the other three producer slots, so "the pipe
  // is drained" and "no load is pending" are different questions and this
  // input is the difference. Missing it produces a `minstret` that is wrong
  // only when a store happens to be in flight.
  input  logic       accessor_out_valid,
  // outputs
  output logic [31:0] pc,
  // rs1 and rs2 are synchronous outputs
  output logic [4:0] rs1,
  output logic [4:0] rs2,
  // ADR-0005: the CSR file (rtl/csrs.v) is a sibling module, read and
  // written here in decode. `csr_rdata`/`csr_implemented` answer `csr_addr`
  // combinationally, in time for the publish block below to register the
  // read value; `csr_wen`/`csr_wdata` commit on the same edge the accessing
  // instruction issues, so the whole access is one cycle wide and nothing
  // downstream carries CSR state.
  output logic [11:0] csr_addr,
  output logic        csr_ren,
  output logic        csr_wen,
  output logic [31:0] csr_wdata,
  input  logic [31:0] csr_rdata,
  input  logic        csr_implemented,
  // ADR-0027: minstret counts non-trapping issues, and issue happens here.
  output logic        instret,
  // ---- trap entry (CLAUDE.md invariant 2 / ADR-0005 / ADR-0030) ----------
  // Every trap this core takes is detected AND committed here, in decode, on
  // the edge the trapping instruction issues. `trap_entry` is high for exactly
  // that cycle; rtl/csrs.v commits mepc/mcause/mstatus off it and rtl/
  // littlecpu.v exports it as the top-level `trap` pulse (ADR-0028).
  // `mtvec`/`mepc` come back the other way because decode owns the PC
  // (invariant 1) -- a trap is a branch, and it is taken by the same
  // mechanism every other branch here uses. There is no flush and there is
  // nothing to flush: nothing downstream of this stage can fault.
  output logic        trap_entry,
  output logic [31:0] trap_cause,
  output logic [31:0] trap_epc,
  output logic        mret_entry,
  input  logic [31:0] mtvec,
  input  logic [31:0] mepc,
 `ifdef RISCV_FORMAL
  // ADR-0006: the CSR file's shadow payload for the access being presented
  // this cycle, latched below into the issuing instruction's shadow.
  input  rvfi_csr64   csr_rvfi_mcycle,
  input  rvfi_csr64   csr_rvfi_minstret,
  input  rvfi_csr32   csr_rvfi_mscratch,
 `endif
  // forwards
  output decoder_output out
);
  // ADR-0003/ADR-0021: instr[31:16] is only ever meaningful for a genuinely
  // 32-bit instruction (quadrant == 2'b11). Masking it here is the defence
  // behind the immediate mux's own fix (instr_jalr_op, not instr_jalr, at
  // the immediate-select case below): even if some future decode path
  // forgets to gate on `uncompressed`, there is no neighbouring-instruction
  // garbage left in the upper half to read. quadrant is derived from
  // instr[1:0] below, which this mask never touches.
  logic [31:0] instr;
  assign instr = (in.instr[1:0] == 2'b11) ? in.instr : {16'b0, in.instr[15:0]};

  // Register-index fields, named and hoisted out of the always_comb blocks
  // below. Two reasons. First, `rd_field` reads better than `instr[11:7]` at
  // every use site, which is the point of this core. Second, iverilog cannot
  // build a precise sensitivity entry for a constant part-select taken inside
  // an always_* block -- it reports `sorry: constant selects in always_*
  // processes are not fully supported` and conservatively makes the process
  // sensitive to all 32 bits. That is safe (over-sensitivity re-evaluates
  // more than necessary; it can never produce a stale value) but it is noise,
  // and CLAUDE.md says warnings are errors. Selecting out here, where a
  // continuous assign has an exact sensitivity, silences it honestly.
  //
  // Base encodings (RISC-V unprivileged spec, Ch. 2.2).
  logic [4:0] rd_field, rs1_field, rs2_field;
  assign rd_field  = instr[11:7];
  assign rs1_field = instr[19:15];
  assign rs2_field = instr[24:20];

  // Compressed encodings (Ch. 16). The 3-bit "prime" fields index x8-x15;
  // instr[11:7] doubles as rd/rs1 in CI/CR formats, so `rd_field` is reused
  // there rather than aliased under a second name.
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

  // Zicsr (ADR-0005). The immediate forms stay separately named -- they used
  // to be folded straight into the register forms, which lost the
  // zimm-vs-rs1 distinction entirely. That distinction is load-bearing
  // twice: `uses_rs1` below must not interlock on a zimm (it is not a
  // register index), and `csr_arg` must read the encoding rather than the
  // register file.
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
  // The operand: a zero-extended 5-bit zimm out of the rs1 field for the
  // immediate forms, the register itself otherwise.
  logic [31:0] csr_arg;
  assign csr_arg = is_csr_imm ? {27'b0, rs1_field} : reg_rs1;

  // Zicsr's two suppression rules (ADR-0005). Both are tests on the
  // *encoding*, not on a value: CSRRS/CSRRC suppress the write when rs1 is
  // x0 -- which is what makes `csrr` (a CSRRS with rs1 == x0) legal against
  // a read-only CSR rather than an illegal-instruction trap -- and the
  // immediate forms suppress it when zimm is 0. Those are the same five bits
  // either way, so one test covers both forms. CSRRW/CSRRWI suppress the
  // read when rd is x0; that falls out of `rd` below being 0, which already
  // gates rtl/writeback.v's `wen`, so it only has to be said here for the
  // RVFI rmask report.
  logic csr_src_zero, csr_write_op, csr_read_op;
  assign csr_src_zero = rs1_field == 5'b0;
  assign csr_write_op = instr_csr_access && !((instr_csrrs || instr_csrrc) && csr_src_zero);
  assign csr_read_op  = instr_csr_access && !(instr_csrrw && rd_field == 5'b0);
  assign csr_wdata = instr_csrrw ? csr_arg :
                     instr_csrrs ? (csr_rdata | csr_arg) :
                                   (csr_rdata & ~csr_arg);

  // The SYSTEM instructions with funct3 == 0, distinguished from each other by
  // funct12 alone -- which is why each one has to be spelled out exactly.
  // Before trap entry existed, `mret` and `wfi` fell through to
  // "unrecognised", which was harmless because unrecognised meant "execution
  // flags suppressed, no trap". It stops being harmless the moment
  // unrecognised means illegal-instruction.
  logic instr_error, instr_mret, instr_wfi, instr_cebreak;
  assign instr_error = opcode == 5'b11100 && uncompressed && funct3 == 0 && rs1 == 0 && rd == 0;
  assign instr_ecall = instr_error && instr[31:20] == 12'h0;
  // C.EBREAK (16'h9002) expands to EBREAK, so it raises a BREAKPOINT (cause 3)
  // and not an illegal instruction. It is the rd == 0, rs2 == 0 corner of
  // quadrant 2's funct4 == 4'b1001 row, which its two neighbours each exclude
  // by a different field -- `instr_cjalr` needs instr[11:7] != 0 and
  // `instr_cadd` needs instr[6:2] != 0 -- so without this line the encoding
  // decodes to NOTHING, `instr_valid` is low, and the trap comes out cause 2:
  // a wrong-but-plausible answer that still traps, still records an mepc and
  // still resumes. test/asm/cebreak.S is what catches it, at both alignments.
  assign instr_cebreak = quadrant == 2'b10 && cfunct4 == 4'b1001 &&
    instr[11:7] == 5'b0 && instr[6:2] == 5'b0;
  assign instr_ebreak = (instr_error && instr[31:20] == 12'h1) || instr_cebreak;
  // ADR-0005: `mret` restores MIE <- MPIE, sets MPIE, and jumps to mepc.
  // Committed in the publish block below, alongside trap entry, and serialized
  // on the same drain predicate CSR instructions use (CLAUDE.md invariant 5).
  assign instr_mret = instr_error && instr[31:20] == 12'h302;
  // `wfi` executes as a NOP, which is spec-legal. With `mie`/`mip` read-only
  // zero there is no interrupt that could ever resume it, so "wait" and
  // "continue" are the same instruction on this core.
  assign instr_wfi = instr_error && instr[31:20] == 12'h105;

  // MISC-MEM: `fence` and `fence.i` are NOPs (ADR-0005). One hart, no caches,
  // and a Harvard ROM no store can reach (ADR-0008), so there is nothing for
  // either to order or to invalidate. Same story as `wfi` above: merely
  // unrecognised until trap entry landed, at which point a legal `fence` would
  // have faulted. Only funct3 is tested -- `fence`'s fm/pred/succ fields and
  // `fence.i`'s reserved immediate are legal-value fields with nothing to act
  // on here.
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

  // ---- trap detection (CLAUDE.md invariant 2, ADR-0005, ADR-0030) ---------
  //
  // The effective address, computed once and used twice: here, to decide
  // whether the access is misaligned, and in the publish block below as
  // `out.mem_addr`. Misalignment was detected in rtl/accessor.v until this
  // change -- post-decode, contradicting invariant 2. ADR-0011 scoped the move
  // to M3 and this is it. That the address is already available here for free
  // is the whole reason invariant 2 is affordable.
  logic [31:0] mem_addr_calc;
  assign mem_addr_calc = $signed(immediate) + $signed(reg_rs1);

  // Word accesses need 4-byte alignment, halfword accesses 2-byte; byte
  // accesses are always aligned. That is why `lb`/`lbu`/`sb` appear nowhere
  // here -- and why every one of the nine misalignment checks that used to sit
  // in formal/EXPECTED_FAIL was a word or halfword access while every
  // byte-granularity one already passed.
  logic load_misaligned, store_misaligned;
  assign load_misaligned  = (instr_lw && mem_addr_calc[1:0] != 2'b00) ||
                            ((instr_lh || instr_lhu) && mem_addr_calc[0] != 1'b0);
  assign store_misaligned = (instr_sw && mem_addr_calc[1:0] != 2'b00) ||
                            (instr_sh && mem_addr_calc[0] != 1'b0);

  // ADR-0005's two illegal-CSR rules. Both are tests on the ENCODING plus the
  // CSR file's own address decode, never on a value:
  //
  //   * an access to a CSR rtl/csrs.v does not implement, which reaches this
  //     through `instr_valid` above exactly as it did before traps existed;
  //   * a WRITE to a read-only CSR, which the privileged spec encodes in the
  //     address itself (addr[11:10] == 2'b11). `csr_write_op` already has
  //     Zicsr's suppression rules applied, so `csrr misa` -- a CSRRS with
  //     rs1 == x0, write suppressed -- stays legal while `csrw misa` does not.
  //     That rule was architecturally unobservable until this change, which is
  //     recorded next to formal/checks.cfg's `[csrs]` list.
  logic csr_readonly_write, instr_illegal;
  assign csr_readonly_write = instr_csr_access && csr_write_op && csr_addr[11:10] == 2'b11;
  assign instr_illegal = !instr_valid || csr_readonly_write;

  // Cause codes (ADR-0005). Instruction-address-misaligned (0) is unreachable
  // -- C makes 2-byte targets legal (ADR-0002) -- and is deliberately absent.
  localparam logic [31:0] CAUSE_ILLEGAL_INSTRUCTION = 32'd2;
  localparam logic [31:0] CAUSE_BREAKPOINT          = 32'd3;
  localparam logic [31:0] CAUSE_LOAD_MISALIGNED     = 32'd4;
  localparam logic [31:0] CAUSE_STORE_MISALIGNED    = 32'd6;
  localparam logic [31:0] CAUSE_ECALL_M             = 32'd11;

  logic trap_pending;
  assign trap_pending = instr_illegal || instr_ebreak || instr_ecall ||
                        load_misaligned || store_misaligned;

  // ADR-0030's priority order: illegal (2) -> breakpoint (3) -> environment
  // call (11) -> load misaligned (4) -> store misaligned (6). An instruction
  // that is not legal cannot meaningfully be said to have a misaligned
  // operand: the address computation is only defined for an instruction the
  // decoder recognises.
  //
  // Deliberately NOT `(* parallel_case *)`, and that is the point of ADR-0030.
  // The five cases cannot co-occur today -- an illegal instruction has every
  // execution flag suppressed and so has no memory operand; `ecall` and
  // `ebreak` differ in funct12 and are SYSTEM rather than LOAD/STORE; a load
  // and a store are different opcodes -- which makes this priority encoder
  // vacuous. Claiming one-hot to synthesis would turn that disjointness from
  // something the `ifdef FORMAL` block below ASSERTS into something the RTL
  // silently assumes, so a later cause that broke it would be a lie to
  // synthesis rather than a failed proof.
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

  // mepc is the address of the FAULTING instruction, not of the next one --
  // that is what lets a handler fix up and resume, and it is asserted directly
  // in test/asm/trap.S rather than inferred from the fact that the test
  // resumed.
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

  // ADR-0004 stall-only hazard scoreboard. Stall only when THIS instruction
  // actually consumes rs1/rs2 as a register operand — not, say, a shift's
  // shamt or a U-type/J-type immediate's overlapping bit position — and that
  // register has a live (non-x0) producer still in flight at decoder_out (the
  // module's own `out`, i.e. the instruction decode issued last cycle),
  // executor_out, or (ADR-0015) a load still in the accessor's one-cycle
  // memory turnaround. Write-through in the regfile covers the writeback
  // stage itself; the accessor check exists only because a load specifically
  // needs one more cycle there than every other instruction (see
  // accessor_stall/rtl/accessor.v) — not a general widening to a third stage.
  // `is_csr_imm` is excluded because instr[19:15] is a 5-bit zimm for those
  // three encodings, not a register index -- interlocking on it would stall
  // on a register the instruction never reads (ADR-0005).
  logic uses_rs1, uses_rs2;
  assign uses_rs1 = !(instr_lui || instr_jal || instr_auipc || is_csr_imm);
  assign uses_rs2 = (instr_math && !instr_math_immediate) || instr_sb || instr_sh || instr_sw ||
    instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu;

  // Is this operand's register the destination of a live (not-yet-retired)
  // producer at any of the three checked points? Spelled out twice rather than
  // shared through a `function automatic live_producer(r)` called from the two
  // continuous assigns below, and THAT IS A CORRECTNESS REQUIREMENT, not a
  // style preference:
  //
  //   iverilog builds a continuous assign's sensitivity list from the function
  //   CALL's arguments. A function whose body additionally reads module-level
  //   signals -- `out`, `executor_out`, `accessor_pending_*` -- therefore never
  //   re-evaluates when any of those change, only when `r` does. That is
  //   UNDER-sensitivity, which is the one direction CLAUDE.md's documented
  //   `sorry:` exception says is a real bug rather than harmless noise, and it
  //   is silent: iverilog issues no diagnostic for it at all.
  //
  // Measured, not theorised. With the function form, the iverilog leg froze at
  // the FIRST RAW hazard of every program -- `hazard_rs1` latched high and
  // never fell -- so `make waves`' own baked-in program executed two
  // instructions and then span, and no `.S` program could reach `tohost`. The
  // cxxrtl and formal legs were unaffected (yosys evaluates the function
  // correctly), which is exactly why it went unnoticed: the leg CLAUDE.md's
  // verification table calls the microscope was reporting nothing, and nothing
  // it reported was wrong.
  logic live_rs1, live_rs2;
  assign live_rs1 = (out.valid && out.rd == rs1) ||
    (executor_out.valid && executor_out.rd == rs1) ||
    (accessor_pending_valid && accessor_pending_rd == rs1);
  assign live_rs2 = (out.valid && out.rd == rs2) ||
    (executor_out.valid && executor_out.rd == rs2) ||
    (accessor_pending_valid && accessor_pending_rd == rs2);

  // CLAUDE.md invariant 5 / ADR-0005: a CSR instruction is held in decode
  // until execute, access and writeback have drained. ADR-0026: this is a
  // fourth stall *reason* on an existing mechanism, not a fourth mechanism.
  // It is bubble-shaped -- the CSR instruction has NOT issued, and the
  // executor reads `in` every cycle, so holding decoder_out would have it
  // reprocess whatever is sitting there -- which is exactly the scoreboard's
  // shape, so it folds into `hazard` rather than growing the stall protocol.
  //
  // All four in-flight slots are needed. `out`/`executor_out` are the two the
  // scoreboard already watches; `accessor_pending_valid` is ADR-0015's
  // load-response turnaround; `accessor_out_valid` is the one that is easy to
  // miss, because a store in the accessor shows up in none of the other
  // three (ADR-0026).
  //
  // What this buys is `minstret` exactness, NOT hazard safety (ADR-0027
  // amends ADR-0005 on exactly this point): the CSR read result reaches rd
  // through the ordinary `is_add` pass-through, where the scoreboard above
  // already covers RAW. Delete this stall and no hazard appears -- what
  // appears is a `csrr minstret` inflated by whatever is in flight behind it.
  //
  // `mret` joins the CSR instructions on this stall for the reason CLAUDE.md
  // invariant 5 names them together: it reads mepc and writes mstatus in
  // decode, the same one-cycle-wide architectural update a CSR instruction
  // makes, and holding it until the pipe drains keeps that update from being
  // interleaved with instructions that issued before it.
  logic pipe_drained, csr_serialize;
  assign pipe_drained = !out.valid && !executor_out.valid && !accessor_out_valid &&
    !accessor_pending_valid;
  assign csr_serialize = (instr_csr_access || instr_mret) && !pipe_drained;

  logic hazard_rs1, hazard_rs2, hazard, stall;
  assign hazard_rs1 = uses_rs1 && rs1 != 0 && live_rs1;
  assign hazard_rs2 = uses_rs2 && rs2 != 0 && live_rs2;
  assign hazard = hazard_rs1 || hazard_rs2 || csr_serialize;

 `ifdef RISCV_FORMAL
  // ADR-0006: rs1_valid/rs2_valid decide whether rvfi_rs{1,2}_addr
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
  // disagree with. Caught empirically: the `make test` run failed
  // error 132 ("mismatch with shadow rs2") on a correct core. `uses_rs2` is
  // exactly "does this instruction have a real rs2 operand", so it doesn't
  // have this hole.
  //
  // `is_csr_imm` is excluded for the same reason `uses_rs1` excludes it: the
  // three immediate CSR encodings have a zimm where rs1 would be, so
  // reporting instr[19:15] as an rs1 address (with that register's current
  // contents as rdata) would hand the monitor's shadow-register check a
  // register read this instruction never performed -- the rs1 twin of the
  // ADDI rs2 hole described above. riscv-formal's own rvfi_csrw_check reads
  // the zimm straight out of rvfi_insn for these, so it needs nothing here.
  logic rvfi_rs1_valid, rvfi_rs2_valid;
  assign rvfi_rs1_valid = !instr_lui && !instr_jal && !instr_auipc && !is_csr_imm;
  assign rvfi_rs2_valid = uses_rs2;
 `endif
  // ADR-0042: THE OPERAND-FETCH CYCLE. rtl/regfile.v's read is registered, so
  // the operands for the address pair presented in cycle N are on
  // reg_rs1/reg_rs2 in cycle N+1. Decode needs them in the cycle it issues --
  // it computes branch targets, jalr targets and load/store addresses from them
  // (and, per CLAUDE.md invariant 2, commits misalignment traps from them) --
  // so an instruction spends one cycle presenting its address pair and issues
  // on the next.
  //
  // The predicate is the direct statement of the rule: the registered read is
  // valid for exactly the address pair that was presented last cycle, and only
  // the ports this instruction actually reads have to be valid. It is NOT a
  // cycle counter, and both refinements are paid for in measurement -- ADR-0042
  // records these, over the 52-program suite:
  //
  //   every instruction waits one cycle    (not built)
  //   address pair unchanged               +27.8% cycles
  //   ...and only the ports it reads       +18.0% cycles   <- this
  //
  // `uses_rs1`/`uses_rs2` are the scoreboard's own predicates, reused verbatim,
  // so this reads the same shape as `hazard_rs1` a few lines up -- on purpose.
  // lui, jal, auipc and the zimm CSR forms have no register operand, and every
  // one of them OVERRIDES out.rs1/out.rs2 in the publish block below rather
  // than passing reg_rs1/reg_rs2 through, so skipping their fetch cycle cannot
  // leak a stale operand. Narrowing `uses_rs1` to exclude a memory operation
  // would break this exactly the way the publish block already warns it breaks
  // misalignment detection: the two now share a reason.
  //
  // `rvfi_rs1_valid` is byte-identical to `uses_rs1` and `rvfi_rs2_valid` IS
  // `uses_rs2`, so RVFI reports zero for exactly the operands whose fetch is
  // skipped. That is also what keeps this clear of ADR-0020: no `ifdef
  // RISCV_FORMAL` value reaches this predicate.
  //
  // THERE IS NO FLUSH HERE AND NONE IS NEEDED (invariant 1). The stalled
  // instruction is not speculative and is never killed: the publish block below
  // simply holds `pc`, so rtl/fetcher.v re-presents the same instruction next
  // cycle and the bubble carries no work. That is the whole reason this lands
  // as a stall rather than as a fetch-ahead pipeline register.
  //
  // `read_taken` covers reset, where `prev_rs1`/`prev_rs2` hold no read at all.
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

  // ADR-0009: a single global stall, combining the local RAW hazard and the
  // operand-fetch cycle with whatever's busy downstream (the divider, or the
  // accessor's one-cycle load turnaround). Any reason freezes the PC; see below
  // for why the two downstream reasons freeze decoder_out differently.
  assign stall = hazard || operand_stall || divider_stall || accessor_stall;

  // The one cycle an instruction actually issues: the publish block's `else`
  // arm below runs on exactly these edges, so everything that commits
  // architectural state OUTSIDE the pipeline registers -- the CSR write and
  // the minstret increment -- is gated on it. Getting this wrong is silent:
  // a CSR write that fired on a stalled cycle would apply once per stall
  // cycle instead of once per instruction.
  // Deliberately the publish block's own guard restated, term for term, and
  // NOT `... && in.valid`: the publish arm does not test `in.valid` either,
  // because rtl/fetcher.v drives it to exactly `!reset` (CLAUDE.md invariant
  // 1 -- fetch is combinational and never presents a wrong-path
  // instruction), so there is no third case. Adding the term would make this
  // stricter than the arm it is supposed to shadow, and the two disagreeing
  // is precisely the bug this signal exists to avoid.
  logic issuing;
  assign issuing = !reset && !stall;

  // The one cycle an instruction actually takes architectural effect: it
  // issued AND it did not trap. Everything that commits state outside the
  // pipeline registers hangs off this.
  logic committing;
  assign committing = issuing && !trap_pending;
  assign csr_ren = committing && csr_read_op;
  assign csr_wen = committing && csr_write_op;
  // ADR-0027: increment at issue, for NON-TRAPPING issues only. A trapping
  // instruction issues, and retires in RVFI with `rvfi_trap = 1`, but it did
  // not retire architecturally, so it must not be counted. `!trap_pending`
  // implies `instr_valid` (an unrecognised instruction is now cause 2), so
  // this is strictly the rule ADR-0027 states, not an approximation of it.
  assign instret = committing;

  // ADR-0028: the trap-entry pulse. rtl/csrs.v commits mepc/mcause/mstatus off
  // it and rtl/littlecpu.v exports it as the top-level `trap` port. High for
  // exactly one cycle per trap because `issuing` is, and no trap can be
  // committed twice: the publish block's `else` arm below runs on exactly the
  // same condition, so the redirect and the CSR update are the same edge.
  assign trap_entry = issuing && trap_pending;
  // `mret` is not a trap, so it commits on the ordinary non-trapping path.
  assign mret_entry = committing && instr_mret;

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
    end else if (hazard || operand_stall) begin
      // ADR-0009: upstream of the stalling stage freezes — the PC (and so
      // the fetch window) holds, so the stalled instruction re-presents next
      // cycle. ADR-0042's operand-fetch cycle shares this arm exactly: holding
      // the PC is what keeps rs1/rs2 pointed at the same pair for the second
      // cycle, so the bubble and the read are the same act.
      // Bubbles rather than holds: unlike divider_stall/accessor_stall
      // above, nothing stops the executor from reading `in` every cycle here,
      // so holding decoder_out unchanged would have it reprocess the same
      // instruction repeatedly instead of retrying the *hazarded* one.
      pc <= pc;
      out <= '0;
    end else begin
      // THIS ARM IS THE ONE CYCLE AN INSTRUCTION ISSUES, and every trap is
      // committed in it, at the bottom -- deliberately, because that single
      // placement gives four things for free that would otherwise each need
      // their own guard, and each of which a later edit can break silently:
      //
      //   * no trap on a stalled cycle -- this arm does not run then;
      //   * no trap from a bubble -- a bubble is the arm above, not this one;
      //   * no double commit -- an instruction reaches this arm exactly once;
      //   * `reg_rs1` is hazard-clear, so the misalignment test below is
      //     computed from the ARCHITECTURAL value of rs1 rather than a stale
      //     one. That holds because `uses_rs1` is true for every load and
      //     every store (it excludes only lui/jal/auipc and the zimm CSR
      //     forms), so the ADR-0004 scoreboard has already stalled this
      //     instruction on any live producer of rs1 before it can get here.
      //     Narrow `uses_rs1` to exclude a memory op and misalignment
      //     detection silently starts reading the wrong register.
      //
      // branches handled below
      pc <= fetcher_pc + pc_inc;
      out.valid <= 1'b1;
      out.mem_addr <= mem_addr_calc;
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
      // `instr` is already zero-extended for a compressed instruction (the
      // mask above), so this is RVFI's required zero-extended 16-bit report
      // for free -- no separate ternary needed.
      out.rvfi.insn <= instr;
      out.rvfi.pc_rdata <= fetcher_pc;
      // ADR-0028. Decode is the only stage that knows this, because decode is
      // the only stage that can fault (CLAUDE.md invariant 2).
      out.rvfi.trap <= trap_pending;
      out.rvfi.rs1_addr <= rvfi_rs1_valid ? rs1 : 5'b0;
      out.rvfi.rs2_addr <= rvfi_rs2_valid ? rs2 : 5'b0;
      out.rvfi.rs1_rdata <= rvfi_rs1_valid ? reg_rs1 : 32'b0;
      out.rvfi.rs2_rdata <= rvfi_rs2_valid ? reg_rs2 : 32'b0;
      // rtl/csrs.v builds these combinationally off the access being
      // presented this cycle (its `ren`/`wen` are this module's, gated on
      // `issuing`), so on this edge they describe exactly the instruction
      // being published -- and a non-CSR instruction gets all-zero masks for
      // free, because `csr_ren`/`csr_wen` are low for it.
      out.rvfi.csr_mcycle   <= csr_rvfi_mcycle;
      out.rvfi.csr_minstret <= csr_rvfi_minstret;
      out.rvfi.csr_mscratch <= csr_rvfi_mscratch;
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

        instr_csr_access: begin
          // ADR-0005: the CSR read result rides the pass-through the decoder
          // already uses for lui/jal/auipc -- the value goes out as rs1 with
          // rs2 zeroed and the executor adds them. The *write* has already
          // been committed this same edge, by rtl/csrs.v off `csr_wen`; the
          // whole access is one cycle wide.
          //
          // `rd` is the ordinary rd field, so CSRRW's read-suppression when
          // rd == x0 needs nothing extra: rd == 0 already gates
          // rtl/writeback.v's `wen`.
          out.rs1 <= csr_rdata;
          out.rs2 <= 32'b0;
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

      // ---- mret: a branch, taken exactly like every other branch here -----
      // The mstatus half (MIE <- MPIE, MPIE <- 1) is committed by rtl/csrs.v
      // off `mret_entry` on this same edge. Nothing else about `mret` reaches
      // the pipeline: rd is x0 in its encoding, so `out.rd` is already 0 and
      // no execution flag is set for it.
      if (instr_mret) begin
        pc <= mepc;
       `ifdef RISCV_FORMAL
        out.rvfi.pc_wdata <= mepc;
       `endif
      end

      // ---- trap entry (ADR-0028 / ADR-0030) -------------------------------
      // A TRAP IS A BRANCH. `pc <= mtvec` here is the same override the jump
      // and branch arms above use, on the same edge, through the same
      // register -- which is why no flush exists and none is needed
      // (CLAUDE.md invariant 1): the trapping instruction is the newest one in
      // the machine, and everything behind it is already correct.
      //
      // Last in the block on purpose. Non-blocking assignments take the last
      // write, so this beats the `pc` and `out.*` values the arms above set,
      // and the suppression below beats every flag they set -- no arm has to
      // know about traps and no ordering rule has to be remembered.
      //
      // The instruction still RETIRES (`out.valid` is 1, set at the top of
      // this arm): ADR-0028's convention is that a trapping instruction
      // retires having architecturally done nothing except redirect. Clearing
      // every execution flag is what makes `rvfi_mem_rmask`/`wmask` zero --
      // rtl/accessor.v builds them from the real bus, and with no flag set it
      // issues no request at all, so a trapping store never reaches memory.
      // That is the property `dmemcheck` catches (its environment shadow comes
      // from the real `mem_wstrb`/`mem_wdata` while `rvfi_dmem_check`'s comes
      // from `rvfi_mem_*`, so a suppressed-but-executed store desynchronises
      // them), and `rvfi_insn_check` explicitly does not.
      //
      // Forcing `out.rd` to 0 does the same job for the register file: it
      // gates rtl/writeback.v's `wen` and makes `rvfi_rd_addr`/`rd_wdata` zero.
      if (trap_pending) begin
        pc <= mtvec;
       `ifdef RISCV_FORMAL
        out.rvfi.pc_wdata <= mtvec;
       `endif
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
  //
  // `trap_pending` and `instr_mret` join the jumps and branches here because
  // they are jumps: both override `pc` in the publish block above, so a cycle
  // whose predecessor took one did not advance sequentially. That is the same
  // exclusion the six branches get, for the same reason, and forgetting it is
  // how "a trap is a branch" stops being true in the proof while staying true
  // in the RTL.
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

  // ADR-0009: a stalled cycle never advances pc.
  always_ff @(posedge clk) if (clocked && prev_stall && !prev_reset) assert(pc == past_pc);

  // ADR-0004: valid == 0 implies out.rd == 0 (a bubble is fully
  // zeroed, never a partial one that could sneak a spurious rd through).
  // Gated on `clocked`: before the first clock edge applies `reset`, `out`
  // is a free, uninitialized register as far as the solver is concerned, so
  // the property only holds once the design has actually been reset.
  always_comb if (clocked && !out.valid) assert(out.rd == 0);

  // ADR-0004: rs1 == 0 / rs2 == 0 never cause a stall — x0 has no
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
    // The three Zicsr forms join the list rather than being exempted from
    // it: `instr_valid` now admits a CSR access (when csr_implemented, a
    // free input in this standalone task), so leaving them out would make
    // the assertion below fail on every legal `csrr`. Listed separately, not
    // folded into one term, so a decode change that made two of them true at
    // once would still be caught -- they are distinguished only by funct3.
    instr_ebreak, instr_csrrw, instr_csrrs, instr_csrrc,
    // The four M3 additions. `mret` and `wfi` are SYSTEM/funct3==0 forms told
    // apart from ecall/ebreak (and from each other) by funct12 alone; `fence`
    // and `fence.i` are MISC-MEM and differ only in funct3. All four are now
    // in `instr_valid`, so omitting them here would fail this assertion on
    // every legal one of them -- which is exactly what makes the omission
    // catchable rather than silent.
    instr_mret, instr_wfi, instr_fence, instr_fencei});

  // we should only get one type of instruction
  always_comb if (instr_valid) assert(one_of);

  // ---- ADR-0030: the trap causes, and why the priority encoder is vacuous --
  //
  // The disjointness argument written out as an assertion, which is what keeps
  // the ADR honest as the core grows: an illegal instruction has no memory
  // operand (`instr_valid` is what admits a load or a store in the first
  // place, and `csr_readonly_write` is a SYSTEM encoding), `ecall`/`ebreak`
  // differ in funct12 and are neither LOAD nor STORE, and a load and a store
  // are different opcodes in both the 32-bit and the compressed forms. Add a
  // sixth cause that overlaps an existing one and this fails rather than the
  // behaviour quietly becoming whatever the encoder happened to produce.
  always_comb assert($onehot0({instr_illegal, instr_ebreak, instr_ecall,
                               load_misaligned, store_misaligned}));

  // ...and the committed cause is the one ADR-0030's order names. Written per
  // cause rather than as a restatement of the case statement, so a reordering
  // of the arms is caught rather than mirrored.
  always_comb if (instr_illegal)    assert(trap_cause == CAUSE_ILLEGAL_INSTRUCTION);
  always_comb if (instr_ebreak)     assert(trap_cause == CAUSE_BREAKPOINT);
  always_comb if (instr_ecall)      assert(trap_cause == CAUSE_ECALL_M);
  always_comb if (load_misaligned)  assert(trap_cause == CAUSE_LOAD_MISALIGNED);
  always_comb if (store_misaligned) assert(trap_cause == CAUSE_STORE_MISALIGNED);
  always_comb if (!trap_pending)    assert(trap_cause == 32'b0);

  // ---- ADR-0028: what a trapping retire is allowed to have done ------------
  // `mtvec`/`mepc` are free inputs in this standalone task (rtl/csrs.v is not
  // instantiated), so these are registered copies rather than reads of the
  // live input -- the same technique the pc-increment history above uses, and
  // for the same reason.
  logic        prev_trap_entry, prev_mret_entry;
  logic [31:0] prev_mtvec, prev_mepc;
  always_ff @(posedge clk) begin
    prev_trap_entry <= trap_entry;
    prev_mret_entry <= mret_entry;
    prev_mtvec      <= mtvec;
    prev_mepc       <= mepc;
  end

  // The redirect actually happened, and it went to mtvec -- which is the one
  // thing ADR-0028 records that NO check on the riscv-formal ladder can see:
  // `rvfi_insn_check` drops every value assertion under `spec_trap`, and
  // pc_fwd/pc_bwd are satisfied by any target so long as it is honestly
  // reported. This is the check that says it must be `mtvec`.
  always_comb if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_comb if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);

  // A trapping instruction writes no register and issues no memory access. The
  // executor and accessor read these flags off `out`, so clearing them here is
  // the whole mechanism -- there is no downstream kill signal and CLAUDE.md
  // invariant 1 forbids adding one.
  always_comb if (clocked && !prev_reset && prev_trap_entry) begin
    assert(out.rd == 5'b0);
    assert(!out.is_lb && !out.is_lbu && !out.is_lh && !out.is_lhu && !out.is_lw);
    assert(!out.is_sb && !out.is_sh && !out.is_sw);
  end

  // ADR-0027: a trapping issue does not increment minstret, and does not
  // commit a CSR write either. Both fall out of `committing`, asserted here so
  // that a future edit that re-gated one of them on `instr_valid` (the
  // pre-M3 spelling, which is weaker) fails.
  always_comb if (trap_pending) assert(!instret && !csr_wen && !csr_ren);
  // A trap and an mret are different instructions; nothing may commit both.
  always_comb assert(!(trap_entry && mret_entry));
 `endif
endmodule
