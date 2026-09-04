`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module decoder #(
  // The data bus's memory map: where some memory answers a plain load or store.
  // Handed down from the memories' own parameters so the map is stated once.
  parameter integer      LS_TEXT_WORDS = 2048,
  parameter logic [31:0] LS_RAM_BASE   = 32'h0001_0000,
  parameter integer      LS_RAM_WORDS  = 16384,
  parameter logic [31:0] LS_TIMER_BASE = 32'h0002_0000,
  parameter logic [31:0] LS_UART_BASE  = 32'h0002_0020,
  parameter logic [31:0] LS_FLASH_BASE = 32'h0002_0028
) (
  input  logic clk,
  input  logic reset,
  input  fetcher_output in,
  input  logic [31:0] reg_rs1,
  input  logic [31:0] reg_rs2,
  input  executor_output executor_out,
  input  logic divider_stall,
  // The instruction memory gave its port to a load or store, so `in.instr` is a
  // data word this cycle.
  input  logic fetch_stall,
  // The platform has not granted this core the shared data bus this cycle. A
  // single-initiator platform ties it low.
  input  logic bus_wait,
  // This cycle would publish a memory transaction if the bus were granted.
  // `bus_wait` is deliberately not a term: the platform ANDs this against its
  // grant, and a term here would close the loop through the arbiter.
  output logic bus_request,
  // The instruction memory has nothing at `pc`; it arrives with the word it is
  // about, so the fault is committed here with every other cause.
  input  logic imem_fault,
  // An atomic's effective address, for the platform to decode.
  output logic [31:0] atomic_addr,
  // The platform's answer about `atomic_addr`; low means the atomic faults.
  input  logic atomic_supported,
  // Serialization needs it: a store writes no register, so the scoreboard
  // cannot see one still in the accessor.
  input  logic       accessor_out_valid,
  output logic [31:0] pc,
  // Published a cycle early, so a synchronous memory latches it on the edge
  // `pc` moves.
  output logic [31:0] next_pc,
  // The pair this instruction names, read by instrumentation and by
  // formal/pcloop.sv; the register file is asked for `read_rs1`/`read_rs2`.
  output logic [4:0] rs1,
  output logic [4:0] rs2,
  // The pair presented to the register file: on an issuing cycle, a guess at
  // the next instruction's pair.
  output logic [4:0] read_rs1,
  output logic [4:0] read_rs2,
  // rtl/csrs.v answers in the same cycle and commits on the edge the
  // instruction issues.
  output logic [11:0] csr_addr,
  output logic        csr_ren,
  output logic        csr_wen,
  output logic [31:0] csr_wdata,
  input  logic [31:0] csr_rdata,
  input  logic        csr_implemented,
  output logic        instret,
  output logic        trap_entry,
  output logic [31:0] trap_cause,
  output logic [31:0] trap_epc,
  output logic [31:0] trap_tval,
  output logic        mret_entry,
  input  logic [31:0] mtvec,
  input  logic [31:0] mepc,
  // Already ANDed with `mie` and `mstatus.MIE` in rtl/csrs.v, and registered.
  input  logic        interrupt_pending,
 `ifdef RISCV_FORMAL
  input  rvfi_csr64   csr_rvfi_mcycle,
  input  rvfi_csr64   csr_rvfi_minstret,
  input  rvfi_csr32   csr_rvfi_mscratch,
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  input  rvfi_csr32   csr_rvfi_mcause,
  `endif
  // A plain load or store committing, for rtl/littlecpu.v's locality counters.
  output logic        probe_ls_issuing,
 `endif
  output decoder_output out
);
  // For a 16-bit instruction the upper half of the window is the next
  // instruction in memory, so it is zeroed rather than read by mistake.
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

  // Declared here because iverilog requires a declaration before the first use.
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
      // Not redundant with the default: an atomic's address is rs1 alone, and
      // its funct5, aq, rl and rs2 sit where the I-immediate is read.
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

  // instr[26:25] are `aq` and `rl`, read by nothing: one hart, in order, with
  // one outstanding access makes memory order program order.
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
  // lr.w's rs2 field is an encoding constant of zero, not a register number.
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

  // The immediate forms stay named apart: their rs1 field is a constant, not a
  // register to wait on.
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

  // Zicsr's suppression rules; skipping the write is what makes `csrr` legal
  // on a read-only CSR. Skipping the read when rd is x0 already happens
  // because rd == 0 gates rtl/writeback.v's `wen`; `csr_read_op` exists only
  // so RVFI reports the right read mask.
  logic csr_src_zero, csr_write_op, csr_read_op;
  assign csr_src_zero = rs1_field == 5'b0;
  assign csr_write_op = instr_csr_access && !((instr_csrrs || instr_csrrc) && csr_src_zero);
  assign csr_read_op  = instr_csr_access && !(instr_csrrw && rd_field == 5'b0);
  assign csr_wdata = instr_csrrw ? csr_arg :
                     instr_csrrs ? (csr_rdata | csr_arg) :
                                   (csr_rdata & ~csr_arg);

  // Read off the raw fields, not the muxed `rs1`/`rd`, which would put the
  // compressed register-select decode in the trap arm of `next_pc`. A SYSTEM
  // opcode is uncompressed, so the two agree.
  logic instr_error, instr_mret, instr_wfi, instr_cebreak;
  assign instr_error = opcode == 5'b11100 && uncompressed && funct3 == 0 &&
    rs1_field == 5'b0 && rd_field == 5'b0;
  assign instr_ecall = instr_error && instr[31:20] == 12'h0;
  // `instr_cjalr` and `instr_cadd` each exclude C.EBREAK by a different field,
  // so it needs its own line.
  assign instr_cebreak = quadrant == 2'b10 && cfunct4 == 4'b1001 &&
    instr[11:7] == 5'b0 && instr[6:2] == 5'b0;
  assign instr_ebreak = (instr_error && instr[31:20] == 12'h1) || instr_cebreak;
  assign instr_mret = instr_error && instr[31:20] == 12'h302;
  // `wfi` is a NOP; the spec lets an implementation resume for any reason.
  assign instr_wfi = instr_error && instr[31:20] == 12'h105;

  // Both are NOPs here, except that `fence.i` serializes (below).
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
    instr_fencei || instr_atomic || (instr_csr_access && csr_implemented);

  logic [31:0] mem_addr_calc;
  assign mem_addr_calc = $signed(immediate) + $signed(reg_rs1);

  // Deliberately not `mem_addr_calc`, which equals it for every atomic: the
  // point is the register output with no adder in front of it.
  assign atomic_addr = reg_rs1;

  logic instr_ls_load, instr_ls_store;
  assign instr_ls_load  = instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw;
  assign instr_ls_store = instr_sb || instr_sh || instr_sw;

  // Does some memory answer a load or store at `immediate + reg_rs1`? It drives
  // only the flip-flop below, so reading the whole sum keeps the carry chain
  // out of the fetch loop. The timer's window is the eight words the map
  // reserves, not the four one hart decodes.
  localparam logic [31:0] LS_TEXT_BYTES = LS_TEXT_WORDS * 4;
  localparam logic [31:0] LS_RAM_BYTES  = LS_RAM_WORDS * 4;
  logic ls_supported;
  assign ls_supported =
    ((mem_addr_calc & ~(LS_TEXT_BYTES - 32'd1)) == 32'd0) ||
    (((mem_addr_calc ^ LS_RAM_BASE) & ~(LS_RAM_BYTES - 32'd1)) == 32'd0) ||
    (mem_addr_calc[31:5] == LS_TIMER_BASE[31:5]) ||
    (mem_addr_calc[31:3] == LS_UART_BASE[31:3]) ||
    (mem_addr_calc[31:3] == LS_FLASH_BASE[31:3]);

  // Whether that answer can depend on the immediate at all, asked of `reg_rs1`
  // alone: a 12-bit offset reaches 2 KB, so a base block with a whole block of
  // the same window on each side is answered whatever the immediate is. Low
  // means "wait for the flip-flop", never "fault".
  localparam int LS_BLOCK_BITS = 11;
  localparam int LS_BLOCK_NUM  = 32 - LS_BLOCK_BITS;
  localparam logic [LS_BLOCK_NUM-1:0] LS_TEXT_BLOCK = '0;
  localparam logic [LS_BLOCK_NUM-1:0] LS_TEXT_BMASK =
    LS_BLOCK_NUM'((LS_TEXT_BYTES - 32'd1) >> LS_BLOCK_BITS);
  localparam logic [LS_BLOCK_NUM-1:0] LS_RAM_BLOCK =
    LS_BLOCK_NUM'(LS_RAM_BASE >> LS_BLOCK_BITS);
  localparam logic [LS_BLOCK_NUM-1:0] LS_RAM_BMASK =
    LS_BLOCK_NUM'((LS_RAM_BYTES - 32'd1) >> LS_BLOCK_BITS);

  logic [LS_BLOCK_NUM-1:0] ls_block;
  assign ls_block = reg_rs1[31:LS_BLOCK_BITS];

  // In the window, and neither its first block nor its last.
  logic ls_text_deep, ls_ram_deep, ls_settled;
  assign ls_text_deep = ((ls_block ^ LS_TEXT_BLOCK) & ~LS_TEXT_BMASK) == '0 &&
                        (ls_block & LS_TEXT_BMASK) != '0 &&
                        (ls_block & LS_TEXT_BMASK) != LS_TEXT_BMASK;
  assign ls_ram_deep  = ((ls_block ^ LS_RAM_BLOCK) & ~LS_RAM_BMASK) == '0 &&
                        (ls_block & LS_RAM_BMASK) != '0 &&
                        (ls_block & LS_RAM_BMASK) != LS_RAM_BMASK;
  assign ls_settled = ls_text_deep || ls_ram_deep;

  // Added here rather than read off `mem_addr_calc`: a misalignment trap
  // reaches `next_pc`, and this waits on two bits, not a 32-bit carry chain.
  logic [1:0] mem_addr_low;
  assign mem_addr_low = immediate[1:0] + reg_rs1[1:0];

  // A load or store whose base register sits near a window's edge bubbles one
  // cycle; the answer about its effective address is registered on that cycle
  // and read on the next. `ls_capture` requires every other stall reason low,
  // so nothing in flight can write rs1 between the capture and the issue.
  logic ls_access, ls_capture, ls_answer, ls_answer_valid, region_stall, ls_fault;
  logic stall_other, stall_own;
  assign ls_access = instr_ls_load || instr_ls_store;
  assign region_stall = ls_access && !ls_settled && !ls_answer_valid;
  assign ls_capture = region_stall && !stall_own;

  // Held until the access issues, not for one cycle: under a bus wait a
  // one-cycle answer expires, `region_stall` returns and drops `bus_request`,
  // and the two livelock -- invisible to any single-hart run. Held on exactly
  // the two reasons that can newly assert over a captured access; holding on
  // `stall` would carry the answer across an operand-fetch cycle into the next
  // access.
  always_ff @(posedge clk) begin
    if (reset) begin
      ls_answer       <= 1'b0;
      ls_answer_valid <= 1'b0;
    end else if (ls_capture) begin
      ls_answer       <= ls_supported;
      ls_answer_valid <= 1'b1;
    end else begin
      ls_answer_valid <= ls_answer_valid && ls_access &&
                         (bus_wait || fetch_stall);
    end
  end

  // An unaligned atomic faults rather than being split; `lr.w` reports as a
  // load and the ten that write as stores.
  logic instr_atomic_write, word_misaligned;
  assign instr_atomic_write = instr_amo || instr_sc;
  assign word_misaligned = mem_addr_low != 2'b00;

  logic load_misaligned, store_misaligned;
  assign load_misaligned  = (instr_lw && word_misaligned) ||
                            ((instr_lh || instr_lhu) && mem_addr_low[0] != 1'b0) ||
                            (instr_lr && word_misaligned);
  assign store_misaligned = (instr_sw && word_misaligned) ||
                            (instr_sh && mem_addr_low[0] != 1'b0) ||
                            (instr_atomic_write && word_misaligned);

  // `!word_misaligned` keeps the four data causes disjoint: an atomic both
  // unaligned and out of region reports the misalignment, as the reference
  // model does. One term reaches `next_pc`; which cause it is feeds only
  // `trap_cause`, off the fetch loop.
  logic atomic_fault;
  assign atomic_fault = instr_atomic && !atomic_supported && !word_misaligned;
  assign ls_fault = ls_access && ls_answer_valid && !ls_answer &&
                    !load_misaligned && !store_misaligned;

  logic load_access_fault, store_access_fault;
  assign load_access_fault  = (atomic_fault && instr_lr) || (ls_fault && instr_ls_load);
  assign store_access_fault = (atomic_fault && instr_atomic_write) ||
                              (ls_fault && instr_ls_store);

  logic csr_readonly_write, instr_illegal;
  assign csr_readonly_write = instr_csr_access && csr_write_op && csr_addr[11:10] == 2'b11;
  assign instr_illegal = !instr_valid || csr_readonly_write;

  // Instruction-address-misaligned (0) is absent because C makes 2-byte targets
  // legal, so it is unreachable.
  localparam logic [31:0] CAUSE_INSTRUCTION_FAULT   = 32'd1;
  localparam logic [31:0] CAUSE_ILLEGAL_INSTRUCTION = 32'd2;
  localparam logic [31:0] CAUSE_BREAKPOINT          = 32'd3;
  localparam logic [31:0] CAUSE_LOAD_MISALIGNED     = 32'd4;
  localparam logic [31:0] CAUSE_LOAD_ACCESS_FAULT   = 32'd5;
  localparam logic [31:0] CAUSE_STORE_MISALIGNED    = 32'd6;
  localparam logic [31:0] CAUSE_STORE_ACCESS_FAULT  = 32'd7;
  localparam logic [31:0] CAUSE_ECALL_M             = 32'd11;
  localparam logic [31:0] CAUSE_MACHINE_TIMER       = 32'h8000_0007;

 `ifdef RISCV_FORMAL
  // The strobe the refused store would have driven: the RVFI fault check wants
  // the write mask exact, where a superset of the read mask is legal.
  logic [3:0] ls_fault_wstrb;
  always_comb begin
    if (instr_sb)      ls_fault_wstrb = 4'b0001 << mem_addr_calc[1:0];
    else if (instr_sh) ls_fault_wstrb = 4'b0011 << mem_addr_calc[1:0];
    else               ls_fault_wstrb = 4'b1111;
  end
 `endif

  // One name for the trap chain and the mtval mux, so they cannot disagree.
  logic data_fault;
  assign data_fault = load_misaligned || store_misaligned || atomic_fault || ls_fault;

  logic trap_pending;
  assign trap_pending = imem_fault || instr_illegal || instr_ebreak || instr_ecall ||
                        data_fault;

  logic trap_taken;
  assign trap_taken = trap_pending || interrupt_pending;

  // No `(* parallel_case *)`: the top two arms deliberately overlap the eight
  // below, which the `ifdef FORMAL` block proves disjoint. An interrupt wins
  // because the displaced instruction does not execute; a fetch that read
  // nothing wins because its word is not an instruction -- out of range the
  // memory answers zero, which would report illegal.
  always_comb begin
    case (1'b1)
      interrupt_pending: trap_cause = CAUSE_MACHINE_TIMER;
      imem_fault:       trap_cause = CAUSE_INSTRUCTION_FAULT;
      instr_illegal:    trap_cause = CAUSE_ILLEGAL_INSTRUCTION;
      instr_ebreak:     trap_cause = CAUSE_BREAKPOINT;
      instr_ecall:      trap_cause = CAUSE_ECALL_M;
      load_misaligned:  trap_cause = CAUSE_LOAD_MISALIGNED;
      store_misaligned: trap_cause = CAUSE_STORE_MISALIGNED;
      load_access_fault:  trap_cause = CAUSE_LOAD_ACCESS_FAULT;
      store_access_fault: trap_cause = CAUSE_STORE_ACCESS_FAULT;
      default:          trap_cause = 32'b0;
    endcase
  end

  // For an interrupt this is the instruction that did not issue, so `mret`
  // re-executes it.
  assign trap_epc = fetcher_pc;

  // All four data causes share one arm: `mem_addr_calc` equals `atomic_addr`
  // for every atomic, which the `ifdef FORMAL` block asserts.
  always_comb begin
    case (1'b1)
      interrupt_pending: trap_tval = 32'b0;
      imem_fault:        trap_tval = fetcher_pc;
      instr_illegal:     trap_tval = instr;
      data_fault:        trap_tval = mem_addr_calc;
      default:           trap_tval = 32'b0;
    endcase
  end

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

  // One mapping instantiated twice, so the guess and the pair it is checked
  // against cannot disagree. Both take the raw word: rtl/regsel.v masks a
  // compressed one itself.
  logic [4:0] next_rs1, next_rs2;
  regsel current_regs (.word(in.instr), .rs1(rs1), .rs2(rs2));
  regsel next_regs (.word(in.next_instr), .rs1(next_rs1), .rs2(next_rs2));

  logic instr_math, instr_shift;
  assign instr_math = instr_add || instr_sub || instr_sll || instr_slt || instr_sltu || instr_xor || instr_srl ||
    instr_sra || instr_or || instr_and || instr_mul || instr_mulh || instr_mulhu || instr_mulhsu || instr_div ||
    instr_divu || instr_rem || instr_remu;
  assign instr_shift = instr_slli || instr_srli || instr_srai;

  // Whether the instruction reads the register its field names -- not a shift
  // amount, not a Zicsr constant.
  logic uses_rs1, uses_rs2;
  assign uses_rs1 = !(instr_lui || instr_jal || instr_auipc || is_csr_imm);
  assign uses_rs2 = (instr_math && !instr_math_immediate) || instr_sb || instr_sh || instr_sw ||
    instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu ||
    instr_amo || instr_sc;

  // Forwarding may only reach `out.rs1`/`out.rs2`, so an eligible encoding must
  // have no other decode-side reader of the same register: the branch
  // comparator, the jalr target, an effective address, an atomic's own address
  // and a register-form CSR's operand all read `reg_rs1`/`reg_rs2` directly.
  // Widening either set past that issues an instruction against a stale
  // register.
  logic rs1_fwd_eligible, rs2_fwd_eligible;
  assign rs1_fwd_eligible = instr_math;
  assign rs2_fwd_eligible = (instr_math && !instr_math_immediate) ||
    instr_sb || instr_sh || instr_sw || instr_amo || instr_sc;

  logic out_match_rs1, out_match_rs2, ex_match_rs1, ex_match_rs2;
  assign out_match_rs1 = out.valid && out.rd == rs1;
  assign out_match_rs2 = out.valid && out.rd == rs2;
  assign ex_match_rs1 = executor_out.valid && executor_out.rd == rs1;
  assign ex_match_rs2 = executor_out.valid && executor_out.rd == rs2;

  // `out`'s instruction has not reached the executor, so a match there is the
  // more recent write and has no result yet. `uses_rs1`/`uses_rs2` are not
  // retested because both eligible sets already imply them.
  logic ex_fwd_rs1, ex_fwd_rs2;
  assign ex_fwd_rs1 = rs1_fwd_eligible && rs1 != 0 && ex_match_rs1 &&
    executor_out.rd_ready && !out_match_rs1;
  assign ex_fwd_rs2 = rs2_fwd_eligible && rs2 != 0 && ex_match_rs2 &&
    executor_out.rd_ready && !out_match_rs2;

  // Continuous assigns rather than a struct field read inside an always_* block,
  // which iverilog cannot build a precise sensitivity list for and warns about.
  logic [31:0] rs1_forwarded, rs2_forwarded;
  assign rs1_forwarded = ex_fwd_rs1 ? executor_out.rd_data : reg_rs1;
  assign rs2_forwarded = ex_fwd_rs2 ? executor_out.rd_data : reg_rs2;

  logic [31:0] math_arg;
  always_comb
    if (instr_math_immediate) math_arg = instr_shift ? {27'b0, rs2} : immediate;
    else math_arg = rs2_forwarded;

  logic [31:0] pc_inc;
  assign pc_inc = uncompressed ? 4 : 2;

  // Do not fold these into a function: iverilog builds a continuous assign's
  // sensitivity list from the call's arguments, so a body that read `out` or
  // `executor_out` would stop re-evaluating when they changed. It gives no
  // warning, and yosys gets the function right, so every other check stays
  // green.
  logic live_rs1, live_rs2;
  assign live_rs1 = out_match_rs1 || ex_match_rs1;
  assign live_rs2 = out_match_rs2 || ex_match_rs2;

  // Two reasons share one wait, and narrowing it to suit one breaks the other:
  // a CSR access or `mret` must not interleave with older instructions still
  // in flight, and `fence.i` waits because text is writable and the fetch
  // address goes out a cycle early, so an older store's write edge must pass.
  logic pipe_drained, serialize;
  assign pipe_drained = !out.valid && !executor_out.valid && !accessor_out_valid;
  assign serialize = (instr_csr_access || instr_mret || instr_fencei) && !pipe_drained;

  logic hazard_rs1, hazard_rs2, hazard, stall;
  assign hazard_rs1 = uses_rs1 && rs1 != 0 && live_rs1 && !ex_fwd_rs1;
  assign hazard_rs2 = uses_rs2 && rs2 != 0 && live_rs2 && !ex_fwd_rs2;
  assign hazard = hazard_rs1 || hazard_rs2 || serialize;

 `ifdef RISCV_FORMAL
  // The monitor checks any register reported here against its own record of
  // writes, so report only what the instruction really reads.
  logic rvfi_rs1_valid, rvfi_rs2_valid;
  assign rvfi_rs1_valid = !instr_lui && !instr_jal && !instr_auipc && !is_csr_imm;
  assign rvfi_rs2_valid = uses_rs2;
 `endif
  // The register file answers a cycle late, so this asks whether what was
  // presented last cycle is what this instruction reads, for the ports it
  // reads. It does not care where that request came from, which is what lets
  // `read_rs1` below be a guess: wrong, and the instruction asks again.
  logic [4:0] prev_rs1, prev_rs2;
  logic       read_taken, operand_stall;
  always_ff @(posedge clk) begin
    if (reset) begin
      prev_rs1   <= 5'd0;
      prev_rs2   <= 5'd0;
      read_taken <= 1'b0;
    end else begin
      prev_rs1   <= read_rs1;
      prev_rs2   <= read_rs2;
      read_taken <= 1'b1;
    end
  end
  assign operand_stall = !read_taken || (uses_rs1 && prev_rs1 != rs1) ||
                                        (uses_rs2 && prev_rs2 != rs2);

  // An AMO's read-modify-write goes back out from rtl/accessor.v on the cycle
  // after the executor takes it, so that cycle is spent here to keep everything
  // else off the bus. `!divider_stall` because a held `out` has not been taken
  // yet, so the write cycle is not next.
  logic atomic_stall;
  assign atomic_stall = out.valid && out.is_amo && !divider_stall;

  // `stall_own` guards the region capture and leaves the bus out on purpose:
  // the region wait suppresses `bus_request`, so a capture that waited for the
  // grant would wait forever.
  assign stall_own = hazard || operand_stall || divider_stall || fetch_stall ||
                     atomic_stall;
  assign stall_other = stall_own || bus_wait;
  assign stall = stall_other || region_stall;

  // `stall` with `bus_wait` left out, over the encodings that reach the data
  // bus. Over-asking is deliberate -- a store-conditional with no reservation
  // makes no transaction -- because under-asking would put two initiators on
  // the bus at once.
  assign bus_request = !reset && !trap_taken && !region_stall && !stall_own &&
    (instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw ||
     instr_sb || instr_sh || instr_sw || instr_atomic);

  // On an issuing cycle, the next instruction's pair; on a stalled one, its
  // own, since the same instruction comes back and guessing again would
  // alternate forever; on a stolen fetch window, neither, since that word is
  // data. The register file's write-through bypass selects on a registered
  // copy of this and is right only because `operand_stall` lets nothing issue
  // until the held pair is the issuing instruction's.
  assign read_rs1 = fetch_stall ? prev_rs1 : stall ? rs1 : next_rs1;
  assign read_rs2 = fetch_stall ? prev_rs2 : stall ? rs2 : next_rs2;

  // All six branch tests from one subtraction: unsigned less-than is the
  // borrow, signed differs only when the sign bits disagree, equal is a zero
  // difference. Deliberately no signed expression: a signed comparison written
  // as an arm of a conditional takes its signedness from the other arms, and
  // has silently turned unsigned here twice.
  logic [32:0] cmp_sub;
  logic        cmp_eq, cmp_ltu, cmp_lt;
  assign cmp_sub = {1'b0, reg_rs1} - {1'b0, reg_rs2};
  assign cmp_eq  = ~|cmp_sub[31:0];
  assign cmp_ltu = cmp_sub[32];
  assign cmp_lt  = (reg_rs1[31] ^ reg_rs2[31]) ? reg_rs1[31] : cmp_sub[32];

  logic branch_taken;
  always_comb begin
    (* parallel_case *)
    case (1'b1)
      instr_beq:  branch_taken =  cmp_eq;
      instr_bne:  branch_taken = !cmp_eq;
      instr_blt:  branch_taken =  cmp_lt;
      instr_bge:  branch_taken = !cmp_lt;
      instr_bltu: branch_taken =  cmp_ltu;
      instr_bgeu: branch_taken = !cmp_ltu;
      default:    branch_taken = 1'b0;
    endcase
  end

  // No `(* parallel_case *)`: `stall` and `trap_taken` can both be true, and
  // `stall` winning is what makes an interrupt wait for the pipeline.
  always_comb begin
    case (1'b1)
      reset:                     next_pc = 32'b0;
      stall:                     next_pc = pc;
      trap_taken:                next_pc = mtvec;
      instr_mret:                next_pc = mepc;
      instr_jalr:                next_pc = ($signed(immediate) + $signed(reg_rs1)) & 32'hfffffffe;
      instr_jal || branch_taken: next_pc = fetcher_pc + immediate;
      default:                   next_pc = fetcher_pc + pc_inc;
    endcase
  end

  // Term for term the guard on the publish block's last two arms. Do not add
  // `&& in.valid`: those arms do not test it, and if the two disagree a CSR
  // write fires once per stalled cycle with nothing to say so.
  logic issuing;
  assign issuing = !reset && !stall;

  logic committing;
  assign committing = issuing && !trap_taken;
  assign csr_ren = committing && csr_read_op;
  assign csr_wen = committing && csr_write_op;
  assign instret = committing;

  assign trap_entry = issuing && trap_taken;
  assign mret_entry = committing && instr_mret;

 `ifdef RISCV_FORMAL
  // RVFI's `intr`: only an interrupt breaks pc continuity, since an exception
  // reports its own redirect in `pc_wdata`.
  logic intr_report;
  always_ff @(posedge clk) begin
    if (reset) intr_report <= 1'b0;
    else if (issuing) intr_report <= interrupt_pending;
  end

  assign probe_ls_issuing = committing &&
    (instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw ||
     instr_sb || instr_sh || instr_sw);
 `endif

  always_ff @(posedge clk) pc <= next_pc;

  always_ff @(posedge clk) begin
    if (reset) begin
      out <= '0;
    end else if (divider_stall) begin
      // The executor has not taken this instruction yet, so it is held; above
      // the bubble arm so a coinciding `fetch_stall` cannot zero it.
      out <= out;
    end else if (hazard || operand_stall || fetch_stall || atomic_stall || bus_wait ||
                 region_stall || interrupt_pending) begin
      // Bubbles: the executor reads `in` every cycle, so a held `out` would
      // execute twice.
      out <= '0;
    end else begin
      out.valid <= 1'b1;
      out.mem_addr <= mem_addr_calc;
      out.rs1 <= rs1_forwarded;
      out.rs2 <= instr_math ? math_arg : rs2_forwarded;
      out.rd <= rd;
     `ifdef RISCV_FORMAL
      // `next_pc`, not `pc`: this arm runs only when `stall` is low, so it is
      // the real target.
      out.rvfi.pc_wdata <= next_pc;
      out.rvfi.insn <= instr;
      out.rvfi.pc_rdata <= fetcher_pc;
      out.rvfi.trap <= trap_pending;
      out.rvfi.intr <= intr_report;
      out.rvfi.mem_fault <= imem_fault || load_access_fault || store_access_fault;
      // The access the instruction would have made: an AMO both reads and
      // writes, so its read mask is set under the store cause too.
      out.rvfi.mem_fault_rmask <= {4{load_access_fault || (store_access_fault && instr_amo)}};
      out.rvfi.mem_fault_wmask <= store_access_fault ? ls_fault_wstrb : 4'b0;
      out.rvfi.mem_fault_addr <= {mem_addr_calc[31:2], 2'b00};
      out.rvfi.rs1_addr <= rvfi_rs1_valid ? rs1 : 5'b0;
      out.rvfi.rs2_addr <= rvfi_rs2_valid ? rs2 : 5'b0;
      // The forwarded value, not the register file's: the monitor checks
      // `rd_wdata` against exactly these two fields, so reporting `reg_rs1`/
      // `reg_rs2` makes every forwarded retire self-contradictory.
      out.rvfi.rs1_rdata <= rvfi_rs1_valid ? rs1_forwarded : 32'b0;
      out.rvfi.rs2_rdata <= rvfi_rs2_valid ? rs2_forwarded : 32'b0;
      out.rvfi.csr_mcycle   <= csr_rvfi_mcycle;
      out.rvfi.csr_minstret <= csr_rvfi_minstret;
      out.rvfi.csr_mscratch <= csr_rvfi_mscratch;
     `ifdef RISCV_FORMAL_CSR_MCAUSE
      out.rvfi.csr_mcause   <= csr_rvfi_mcause;
     `endif
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
      out.is_lb <= instr_lb;
      out.is_lbu <= instr_lbu;
      out.is_lhu <= instr_lhu;
      out.is_lh <= instr_lh;
      out.is_lw <= instr_lw;
      out.is_sb <= instr_sb;
      out.is_sh <= instr_sh;
      out.is_sw <= instr_sw;
      out.is_amo <= instr_amo;
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
      out.is_valid_instr <= instr_valid;
      (* parallel_case *)
      case(1'b1)
        default: ;
        instr_auipc: begin
          out.rd <= rd;
          out.rs1 <= fetcher_pc;
          out.rs2 <= immediate;
          out.is_add <= 1;
        end

        instr_csr_access: begin
          out.rs1 <= csr_rdata;
          out.rs2 <= 32'b0;
          out.is_add <= 1;
        end

        instr_lui: begin
          out.rs1 <= immediate;
          out.rs2 <= 32'b0;
          out.is_add <= 1;
        end

        instr_jal || instr_jalr: begin
          out.rs1 <= fetcher_pc;
          out.rs2 <= pc_inc;
          out.rd <= rd;
          out.is_add <= 1;
        end

        instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu: begin
          out.rs1 <= 0;
          out.rs2 <= 0;
          out.rd <= 0;
        end
      endcase

      // Last so it wins over every arm above; `out.valid` stays high so the
      // trapping instruction still retires, with no flag and no rd.
      if (trap_pending) begin
        out.is_add <= 0; out.is_sub <= 0; out.is_xor <= 0; out.is_or <= 0; out.is_and <= 0;
        out.is_mul <= 0; out.is_mulh <= 0; out.is_mulhu <= 0; out.is_mulhsu <= 0;
        out.is_div <= 0; out.is_divu <= 0; out.is_rem <= 0; out.is_remu <= 0;
        out.is_sll <= 0; out.is_slt <= 0; out.is_sltu <= 0; out.is_srl <= 0; out.is_sra <= 0;
        out.is_lb <= 0; out.is_lbu <= 0; out.is_lhu <= 0; out.is_lh <= 0; out.is_lw <= 0;
        out.is_sb <= 0; out.is_sh <= 0; out.is_sw <= 0;
        out.is_amo <= 0;
        out.is_amoswap <= 0; out.is_amoadd <= 0; out.is_amoxor <= 0; out.is_amoand <= 0;
        out.is_amoor <= 0; out.is_amomin <= 0; out.is_amomax <= 0; out.is_amominu <= 0;
        out.is_amomaxu <= 0; out.is_lr <= 0; out.is_sc <= 0;
        out.rd <= 0;
      end
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // Reset is high before the first edge; every harness does it, nothing proves
  // it.
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // Reset never returns; the pc-increment assertions below would otherwise
  // have to allow for it.
  always_comb if (clocked) assume(!reset);

  // The fetcher hands back the pc it was given; formal/pcloop.sv asserts this
  // over the real fetcher.
  always_comb assume(in.pc == pc);

  // A held instruction is the same instruction: a stalled cycle holds the pc,
  // so the memory re-presents the same words, and issues nothing, so no write
  // to rs1 starts. This module can see neither, so it is assumed here and
  // dropped with `-noassume` where the composed proof can check it.
  fetcher_output prev_in;
  logic [31:0] prev_reg_rs1;
  logic        prev_issued;
  always_ff @(posedge clk) begin
    prev_in      <= in;
    prev_reg_rs1 <= reg_rs1;
    prev_issued  <= !stall || reset;
  end
  always_comb if (clocked && !prev_issued) begin
    assume(in == prev_in);
    assume(reg_rs1 == prev_reg_rs1);
  end

  // Own copies of last cycle's values rather than `$past()`, which through a
  // free input would let the solver fill in a value from before reset.
  // `trap_taken` and `instr_mret` are in `branch_jump` because both redirect.
  logic branch_jump;
  always_ff @(posedge clk) if (reset) branch_jump <= 1'b0;
    else branch_jump <= instr_jal || instr_jalr || instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu || trap_taken || instr_mret;
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

  // Arm order in the publish block, both ways round; nothing else enforces it
  // and both ways of getting it wrong are silent.
  decoder_output past_out;
  logic prev_hold_and_steal, prev_steal_only, prev_atomic_stall;
  logic prev_hold_and_wait, prev_wait_only;
  logic prev_hold_and_region, prev_region_only;
  always_ff @(posedge clk) begin
    past_out             <= out;
    prev_hold_and_steal  <= fetch_stall && divider_stall;
    prev_steal_only      <= fetch_stall && !divider_stall;
    prev_atomic_stall    <= atomic_stall;
    prev_hold_and_wait   <= bus_wait && divider_stall;
    prev_wait_only       <= bus_wait && !divider_stall;
    prev_hold_and_region <= region_stall && divider_stall;
    prev_region_only     <= region_stall && !divider_stall;
  end
  always_comb if (clocked && !prev_reset && prev_hold_and_steal) assert(out == past_out);
  always_comb if (clocked && !prev_reset && prev_steal_only)     assert(out == '0);
  always_comb if (clocked && !prev_reset && prev_hold_and_wait) assert(out == past_out);
  always_comb if (clocked && !prev_reset && prev_wait_only)     assert(out == '0);
  always_comb if (clocked && !prev_reset && prev_atomic_stall) assert(out == '0);
  always_comb if (clocked && !prev_reset && prev_hold_and_region) assert(out == past_out);
  always_comb if (clocked && !prev_reset && prev_region_only)     assert(out == '0);

  // Fails the moment anything else writes `pc`.
  logic [31:0] past_next_pc;
  always_ff @(posedge clk) past_next_pc <= next_pc;
  always_comb if (clocked) assert(pc == past_next_pc);

  always_comb if (clocked && !out.valid) assert(out.rd == 0);

  // `atomic_stall` and rtl/accessor.v both take `is_amo` on trust.
  always_comb if (clocked)
    assert(out.is_amo == (out.is_amoswap || out.is_amoadd || out.is_amoxor ||
      out.is_amoand || out.is_amoor || out.is_amomin || out.is_amomax ||
      out.is_amominu || out.is_amomaxu));

  // The one stall reason that reads a register value asserts only alongside
  // `ls_access`, and `ls_access` is exactly the eight base load/store
  // encodings; the Zkt isolation argument stands on both.
  always_comb if (clocked) assert(!region_stall || ls_access);
  always_comb if (clocked)
    assert(ls_access == (instr_lb || instr_lbu || instr_lh || instr_lhu ||
      instr_lw || instr_sb || instr_sh || instr_sw));

  always_comb if (rs1 == 0) assert(!hazard_rs1);
  always_comb if (rs2 == 0) assert(!hazard_rs2);

  logic one_of;
  assign one_of = $onehot({instr_auipc, instr_jal, instr_jalr, instr_beq, instr_bne, instr_blt,
    instr_bltu, instr_bge, instr_bgeu, instr_add, instr_sub, instr_xor, instr_or, instr_and,
    instr_mul, instr_mulh, instr_mulhu, instr_mulhsu, instr_div, instr_divu, instr_rem,
    instr_remu, instr_sll, instr_slt, instr_sltu, instr_srl, instr_sra, instr_lui, instr_lb,
    instr_lbu, instr_lh, instr_lhu, instr_lw, instr_sb, instr_sh, instr_sw, instr_ecall,
    // Every term of `instr_valid`, with forms that differ only in funct3 or
    // funct12 kept on separate terms.
    instr_ebreak, instr_csrrw, instr_csrrs, instr_csrrc,
    instr_mret, instr_wfi, instr_fence, instr_fencei,
    instr_amoswap, instr_amoadd, instr_amoxor, instr_amoand, instr_amoor, instr_amomin,
    instr_amomax, instr_amominu, instr_amomaxu, instr_lr, instr_sc});

  always_comb if (instr_valid) assert(one_of);

  // One list per `(* parallel_case *)` above, transcribed rather than shared,
  // so an arm added there and not here still trips.
  // immediate
  always_comb assert($onehot0({instr_load_op || instr_jalr_op, instr_store_op,
    instr_lui_op || instr_auipc, instr_jal_op, instr_branch_op, instr_math_immediate_op,
    instr_amo_op, instr_clwsp, instr_cswsp, instr_csw, instr_clw, instr_cj || instr_cjal,
    instr_cbeqz || instr_cbnez, instr_cli, instr_clui, instr_caddi, instr_caddi16sp,
    instr_caddi4spn, instr_candi}));
  // rd
  always_comb assert($onehot0({
    instr_beq || instr_bne || instr_blt || instr_bge || instr_bltu || instr_bgeu ||
      instr_sb || instr_sh || instr_sw || instr_cj || instr_cjr,
    instr_cjal || instr_cjalr,
    instr_clw || instr_caddi4spn,
    instr_csrai || instr_csrli || instr_candi || instr_cand ||
      instr_cor || instr_cxor || instr_csub}));
  // the operand overrides in the publish block
  always_comb assert($onehot0({instr_auipc, instr_csr_access, instr_lui,
    instr_jal || instr_jalr,
    instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu}));

  // What lets the trap-cause case go unmarked. Under `!imem_fault` because that
  // arm deliberately overlaps all of them: a word the memory never supplied
  // decodes as zero and can name any register.
  always_comb if (!imem_fault)
    assert($onehot0({instr_illegal, instr_ebreak, instr_ecall,
                     load_misaligned, store_misaligned,
                     load_access_fault, store_access_fault}));

  // `atomic_addr` skips the adder on purpose, so nothing else says the fault
  // and the transaction name one address.
  always_comb if (instr_atomic) assert(mem_addr_calc == atomic_addr);

  // The fast path never skips a fault: `ls_settled` is answered from `reg_rs1`
  // alone, so it has to imply the answer about the effective address. The
  // first line is what that stands on: a sign-extended immediate cannot leave
  // the block `reg_rs1` names or the two beside it.
  always_comb if (ls_access) begin
    assert(immediate[31:12] == {20{immediate[31]}});
    if (ls_settled) assert(ls_supported);
  end

  // The answer never outlives the access it was taken for; stated on
  // `prev_issued` rather than on the two hold reasons, so it is a claim about
  // the design and not the assignment read back.
  logic prev_answer_valid;
  always_ff @(posedge clk) prev_answer_valid <= ls_answer_valid;
  always_comb if (clocked && prev_answer_valid && prev_issued)
    assert(!ls_answer_valid);

  // `ls_fault` reads the flip-flop with no term of its own about the address,
  // so this is the whole of what keeps the region causes about the access that
  // raised them.
  always_comb if (clocked && ls_answer_valid) assert(ls_answer == ls_supported);

  // One line per cause rather than a copy of the case, so reordering its arms
  // trips these. `word_decides` is the two top arms' priority, named once.
  logic word_decides;
  assign word_decides = !interrupt_pending && !imem_fault;
  always_comb if (interrupt_pending) assert(trap_cause == CAUSE_MACHINE_TIMER);
  always_comb if (!interrupt_pending && imem_fault) assert(trap_cause == CAUSE_INSTRUCTION_FAULT);
  always_comb if (word_decides && instr_illegal)    assert(trap_cause == CAUSE_ILLEGAL_INSTRUCTION);
  always_comb if (word_decides && instr_ebreak)     assert(trap_cause == CAUSE_BREAKPOINT);
  always_comb if (word_decides && instr_ecall)      assert(trap_cause == CAUSE_ECALL_M);
  always_comb if (word_decides && load_misaligned)  assert(trap_cause == CAUSE_LOAD_MISALIGNED);
  always_comb if (word_decides && store_misaligned) assert(trap_cause == CAUSE_STORE_MISALIGNED);
  always_comb if (word_decides && load_access_fault)
    assert(trap_cause == CAUSE_LOAD_ACCESS_FAULT);
  always_comb if (word_decides && store_access_fault)
    assert(trap_cause == CAUSE_STORE_ACCESS_FAULT);
  always_comb if (!trap_taken)       assert(trap_cause == 32'b0);

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

  // No riscv-formal check sees that a trap lands on `mtvec`; its pc checks
  // accept any target the core reports.
  always_comb if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_comb if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);

  always_comb if (clocked && !prev_reset && prev_trap_entry) begin
    assert(out.rd == 5'b0);
    assert(!out.is_lb && !out.is_lbu && !out.is_lh && !out.is_lhu && !out.is_lw);
    assert(!out.is_sb && !out.is_sh && !out.is_sw);
    assert(!out.is_amo && !out.is_lr && !out.is_sc);
  end

  // On `trap_taken`, not `instr_valid`: an instruction can decode fine and
  // still trap.
  always_comb if (trap_taken) assert(!instret && !csr_wen && !csr_ren);
  always_comb assert(!(trap_entry && mret_entry));

  always_comb if (interrupt_pending && !stall && !reset) assert(trap_entry);
  always_comb if (stall || reset) assert(!trap_entry);

  // An interrupt publishes nothing: the instruction it displaced has not
  // issued.
  logic prev_interrupt_entry;
  always_ff @(posedge clk) prev_interrupt_entry <= !reset && !stall && interrupt_pending;
  always_comb if (clocked && !prev_reset && prev_interrupt_entry) assert(!out.valid);

  // The references for the branch compare and the two-bit misalignment add,
  // against the operators they replaced. Each is a self-determined signed
  // statement, never an arm of a conditional, which is where a signed
  // comparison loses its signedness.
  logic signed [31:0] cmp_ref_x, cmp_ref_y;
  assign cmp_ref_x = reg_rs1;
  assign cmp_ref_y = reg_rs2;
  always_comb assert(cmp_eq == (reg_rs1 == reg_rs2));
  always_comb assert(cmp_ltu == (reg_rs1 < reg_rs2));
  always_comb assert(cmp_lt == (cmp_ref_x < cmp_ref_y));
  always_comb assert(mem_addr_low == mem_addr_calc[1:0]);
 `endif
endmodule
