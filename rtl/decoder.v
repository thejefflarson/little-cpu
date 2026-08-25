`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module decoder #(
  // The data bus's memory map: where some memory answers a plain load or store.
  // The same four numbers rtl/littlecpu.v takes, in the units rtl/memory.v and
  // rtl/timer.v state their own windows in, handed down rather than restated --
  // test/memmap_test.sh compares the copies, and a second spelling of the map
  // here would be a copy that file does not know about.
  //
  // It arrives at elaboration rather than over a port, unlike the answer about
  // an atomic's address. The fault commits in the cycle the pc is chosen, so
  // constants that route nowhere are what make the region tests below
  // equalities on register-output bits; a question sent out to the memories and
  // back measured 3 ns more than keeping the whole cone inside decode.
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
  // This holds `out` instead of zeroing it: the executor has not taken this
  // cycle's `out` yet, so zeroing it would lose that instruction. It is the
  // only reason that holds.
  input  logic divider_stall,
  // The instruction memory gave its read port to a load or store, so the window
  // arriving this cycle holds a data word, not the instruction at `pc`. If a
  // divide lands on the same cycle, that one wins.
  input  logic fetch_stall,
  // The platform has not given this core the shared data bus this cycle. It
  // BUBBLES: nothing issues, the pc holds, and the same instruction decodes and
  // asks again next cycle, so nothing is lost because nothing was published.
  // Holding `out` here instead would hand the executor an instruction it has
  // already consumed -- the wait leaves the executor idle, unlike a divide.
  // A platform with one bus initiator ties it low and the core never waits.
  input  logic bus_wait,
  // This cycle would publish a memory transaction, if the bus were this core's
  // to publish on. It is the arbiter's request line and the one signal the
  // shared-bus surface was left owing: an arbiter that registers its grant has
  // to be told a cycle before the transaction, and the only stage that knows a
  // cycle early is this one. `bus_wait` is deliberately NOT a term of it --
  // the platform ANDs this against its own grant to make that wait, so a term
  // here would close the loop through the arbiter.
  output logic bus_request,
  // The instruction memory had nothing at `pc` -- the address is outside the
  // text window. It arrives with the word it is about, in the cycle decode is
  // looking at that word, so the fault is committed with everything else here
  // and nothing downstream has to be told.
  input  logic imem_fault,
  // The address an atomic would use, published for the platform to decode. It is
  // `reg_rs1` verbatim: the A encodings put funct5, aq, rl and rs2 exactly where
  // the I-immediate is read from, so the immediate mux forces zero and there is
  // no adder in front of this. That is the whole reason the region question is
  // affordable for the eleven and was not for a load or a store, whose answer
  // waits on the top of a 32-bit sum.
  output logic [31:0] atomic_addr,
  // ...and the platform's answer: this address is memory that answers an atomic.
  // Low means an lr.w there faults with cause 5 and an AMO or sc.w with cause 7,
  // committed in decode with every other cause. A platform whose memory answers
  // atomics everywhere ties it high and the core never takes either.
  input  logic atomic_supported,
  // A store writes no register, so the hazard check ignores it. Serialization
  // still has to wait for one. That is what this is for -- without it a CSR
  // access can issue while a store is still in the accessor.
  input  logic       accessor_out_valid,
  output logic [31:0] pc,
  // The value `pc` takes next cycle, ready during this one. A memory that needs
  // a cycle to answer latches this on the same edge `pc` moves, so its data is
  // there when decode wants it. Without it decode would always be one
  // instruction behind the address it is deciding from.
  output logic [31:0] next_pc,
  // The pair this instruction's encoding names. Nothing in the datapath reads
  // it: the register file is asked for the pair below instead. formal/pcloop.sv
  // needs both to say which cycles decode may hold the pc on, and a second copy
  // of the register-number decode over there is what that file exists to avoid.
  output logic [4:0] rs1,
  output logic [4:0] rs2,
  // The pair actually presented to the register file. On a cycle that issues it
  // is a guess at the NEXT instruction's pair, so the operands arrive with that
  // instruction rather than a cycle behind it. See `read_rs1` below.
  output logic [4:0] read_rs1,
  output logic [4:0] read_rs2,
  // rtl/csrs.v is a sibling module, not a stage. It answers in the same cycle,
  // and commits on the edge the instruction issues. No CSR value is ever held
  // downstream, so there is nothing to forward and nothing to replay.
  output logic [11:0] csr_addr,
  output logic        csr_ren,
  output logic        csr_wen,
  output logic [31:0] csr_wdata,
  input  logic [31:0] csr_rdata,
  input  logic        csr_implemented,
  output logic        instret,
  // A trap takes the same `next_pc` chain a branch does. Nothing downstream has
  // to be told about it, so there is no kill signal anywhere.
  output logic        trap_entry,
  output logic [31:0] trap_cause,
  output logic [31:0] trap_epc,
  // What the trap happened to, for mtval. Latched by rtl/csrs.v on the same
  // edge as the cause and the pc; nothing in the `next_pc` chain reads it, so
  // it is off the fetch loop however wide the mux below gets.
  output logic [31:0] trap_tval,
  output logic        mret_entry,
  input  logic [31:0] mtvec,
  input  logic [31:0] mepc,
  // An enabled interrupt source is asserting. rtl/csrs.v has already ANDed in
  // mie and mstatus.MIE, and every term of it is a flip-flop, so this arrives
  // as a registered level. It joins the trap arm of the `next_pc` chain below,
  // which is BELOW the stall arm -- so an interrupt waits out a divide and a
  // serialization the same way everything else does, and is taken only on a
  // cycle that would otherwise have issued an instruction. The
  // instruction it displaces has not issued, so there is nothing to un-commit.
  input  logic        interrupt_pending,
 `ifdef RISCV_FORMAL
  input  rvfi_csr64   csr_rvfi_mcycle,
  input  rvfi_csr64   csr_rvfi_minstret,
  input  rvfi_csr32   csr_rvfi_mscratch,
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  input  rvfi_csr32   csr_rvfi_mcause,
  `endif
  // A plain load or store is issuing this cycle and is not trapping. The
  // load/store locality counters in rtl/littlecpu.v are summed over exactly
  // these cycles; everything else they need -- the base register's value, the
  // register number and the write port -- is already up there.
  output logic        probe_ls_issuing,
 `endif
  output decoder_output out
);
  // The fetch window is always 32 bits. For a 16-bit instruction the upper half
  // is the next instruction in memory. Zeroing it means a decode line that reads
  // instr[31:16] without checking the width first sees zero, not its neighbour.
  logic [31:0] instr;
  assign instr = (in.instr[1:0] == 2'b11) ? in.instr : {16'b0, in.instr[15:0]};

  // Base encodings (RISC-V unprivileged spec, Ch. 2.2). Pulled out here rather
  // than read inside the always_comb blocks below: iverilog cannot work out a
  // precise sensitivity list for a bit-select taken inside one, and warns. Use a
  // continuous assign for any field read added later.
  logic [4:0] rd_field, rs1_field;
  assign rd_field  = instr[11:7];
  assign rs1_field = instr[19:15];

  // Compressed encodings (Ch. 16). instr[11:7] doubles as rd/rs1 in CI/CR
  // formats, so `rd_field` is reused there rather than aliased.
  logic [2:0] c_rd_rs1_prime, c_rs2_prime;
  assign c_rd_rs1_prime = instr[9:7];
  assign c_rs2_prime    = instr[4:2];
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

  // The A extension, single hart. `.aq` and `.rl` are decoded and ignored:
  // one hart, in order, one outstanding access and every access completing
  // before the next issues, so memory order is program order. A second agent on
  // memory, a store buffer, a cache or a posted MMIO write each end that.
  logic instr_amoswap, instr_amoadd, instr_amoxor, instr_amoand, instr_amoor, instr_amomin,
        instr_amomax, instr_amominu, instr_amomaxu, instr_lr, instr_sc;
  logic instr_amo_op, instr_amo, instr_atomic;

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
      // An A encoding's effective address is rs1 exactly. Its funct5, `aq`,
      // `rl` and rs2 fields sit where the I-immediate is read, so without this
      // arm the default below is the only thing keeping them out of the address
      // -- and any later arm that widened to this opcode would put them in it.
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

  // AMO (Ch. 14). funct3 picks the width and only the word form exists on RV32;
  // instr[26] and instr[25] are `aq` and `rl`, read by nothing, so every
  // ordering suffix of one instruction decodes identically.
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
  // lr.w's rs2 field is an encoding constant, not a register number, so a
  // non-zero one is a different encoding and not this instruction. sc.w reads
  // rs2 for real.
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

  // Keep the immediate forms named apart from the register forms. Their rs1
  // field holds a 5-bit constant, not a register number, so `uses_rs1` must not
  // wait on it and `csr_arg` must read the encoding instead of the register.
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

  // Skipping the write when the source is zero is what makes `csrr` legal on a
  // read-only CSR instead of an illegal instruction. Skipping the read when rd
  // is x0 already happens, because rd == 0 gates rtl/writeback.v's `wen`; it is
  // named here only so RVFI reports the right read mask.
  logic csr_src_zero, csr_write_op, csr_read_op;
  assign csr_src_zero = rs1_field == 5'b0;
  assign csr_write_op = instr_csr_access && !((instr_csrrs || instr_csrrc) && csr_src_zero);
  assign csr_read_op  = instr_csr_access && !(instr_csrrw && rd_field == 5'b0);
  assign csr_wdata = instr_csrrw ? csr_arg :
                     instr_csrrs ? (csr_rdata | csr_arg) :
                                   (csr_rdata & ~csr_arg);

  // SYSTEM with funct3 == 0. Leave one of these out and it decodes to nothing,
  // which traps as an illegal instruction rather than being ignored.
  // The raw encoding fields, not the muxed `rs1`/`rd`: reading those puts the
  // whole compressed register-select decode in front of `trap_pending`, and so
  // in front of the next pc. They hold the same value here for a different
  // reason on each side. `uncompressed` settles rs1, because every arm of the
  // mux that picks it is a compressed encoding; the rd mux needs one step more,
  // since its first arm also covers uncompressed branches and stores -- and a
  // SYSTEM opcode is neither, so it falls to the default arm, `rd_field`.
  logic instr_error, instr_mret, instr_wfi, instr_cebreak;
  assign instr_error = opcode == 5'b11100 && uncompressed && funct3 == 0 &&
    rs1_field == 5'b0 && rd_field == 5'b0;
  assign instr_ecall = instr_error && instr[31:20] == 12'h0;
  // C.EBREAK needs its own line. It sits in the same row as `instr_cjalr` and
  // `instr_cadd`, and each of those excludes it by a different field. Without
  // this it decodes to nothing and traps as illegal instead of as a breakpoint.
  assign instr_cebreak = quadrant == 2'b10 && cfunct4 == 4'b1001 &&
    instr[11:7] == 5'b0 && instr[6:2] == 5'b0;
  assign instr_ebreak = (instr_error && instr[31:20] == 12'h1) || instr_cebreak;
  assign instr_mret = instr_error && instr[31:20] == 12'h302;
  // `wfi` is a NOP, which the spec permits outright: an implementation may
  // resume for any reason, even with no enabled interrupt pending. The
  // permission does not depend on which interrupts this core can post.
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
    instr_fencei || instr_atomic || (instr_csr_access && csr_implemented);

  // The effective address, used here to decide misalignment and again in the
  // publish block as `out.mem_addr`. Having it this early is what lets a
  // misaligned access be turned into a trap before the instruction issues,
  // rather than by the memory stage refusing it after the fact.
  logic [31:0] mem_addr_calc;
  assign mem_addr_calc = $signed(immediate) + $signed(reg_rs1);

  // Deliberately not `mem_addr_calc`, whose value is identical for the eleven
  // encodings that read this: the whole worth of the term is that it is the
  // register output rather than the adder's result. The `ifdef FORMAL` block
  // below is what keeps those two facts from drifting apart.
  assign atomic_addr = reg_rs1;

  // The twelve plain load and store encodings, split the way the two region
  // causes are: a store-conditional that succeeds writes, so the split an atomic
  // makes is the same one.
  logic instr_ls_load, instr_ls_store;
  assign instr_ls_load  = instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw;
  assign instr_ls_store = instr_sb || instr_sh || instr_sw;

  // Does some memory answer a plain load or store at `immediate + reg_rs1`?
  // Five windows, each a power of two on a multiple of its own size, so each is
  // one equality on the address bits above the window. It reads the whole sum,
  // top bit included, which is affordable HERE and nowhere else: the only thing
  // it drives is the flip-flop below, so the effective-address carry chain ends
  // at a register instead of at the pc. Seven spellings that answered in the
  // issue cycle were built and priced, the cheapest exact one at +9.4% of the
  // median period and under the board clock at nine placements of sixteen.
  //
  // The timer's window is the eight words the map reserves for one `mtimecmp`
  // per hart, not the four a one-hart build decodes: rounding out reads zero
  // where rounding in would fault an address the two-hart machine answers.
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
  // by itself so it costs no adder. A 12-bit offset reaches 2 KB either way, so
  // an effective address lies in the 2 KB block `reg_rs1` names or in one either
  // side. When all three of those blocks are inside one window the access is
  // answered whatever the immediate is, and the cycle issues with no region term
  // in it.
  //
  // ONE-SIDED ON PURPOSE: low means "wait for the flip-flop", never "fault". So
  // a window narrower than three blocks -- the timer's, the UART's, the SPI
  // master's -- never reaches the fast path, and `mcause` stays a function of
  // the access rather than of the base register. That is the whole difference between this
  // and the neighbourhood test that meets the clock by calling rs1's own answer
  // final.
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

  // In the window, and neither its first block nor its last. A window of one or
  // two blocks has no such block and these fold to constant zero, which is the
  // safe direction.
  logic ls_text_deep, ls_ram_deep, ls_settled;
  assign ls_text_deep = ((ls_block ^ LS_TEXT_BLOCK) & ~LS_TEXT_BMASK) == '0 &&
                        (ls_block & LS_TEXT_BMASK) != '0 &&
                        (ls_block & LS_TEXT_BMASK) != LS_TEXT_BMASK;
  assign ls_ram_deep  = ((ls_block ^ LS_RAM_BLOCK) & ~LS_RAM_BMASK) == '0 &&
                        (ls_block & LS_RAM_BMASK) != '0 &&
                        (ls_block & LS_RAM_BMASK) != LS_RAM_BMASK;
  assign ls_settled = ls_text_deep || ls_ram_deep;

  // The wait that pays for all of the above. A load or store whose base register
  // sits near a window's edge bubbles one cycle, and the answer about its own
  // effective address is registered on that cycle and read on the next.
  //
  // `ls_answer_valid` is one cycle wide by construction -- it is set only by a
  // capture cycle and cleared by every other cycle -- so an answer that is read
  // is always exactly one cycle old. What makes it an answer about the same
  // ACCESS is `ls_capture` requiring every other stall reason to be low: the
  // decode scoreboard is clear, so no instruction is in flight that could write
  // rs1, and nothing issued on that cycle to start one. THAT IS THE ONE THING
  // THIS DESIGN NEEDS AND CANNOT SEE. rtl/regfile.v and the scoreboard provide
  // it; formal/traps.sv, which has no register file, assumes it and says so.
  // Misalignment needs two bits of the sum, and the low two bits of a sum depend
  // only on the low two bits of the operands -- so they are added here rather
  // than read off `mem_addr_calc`. Read from there, this test waits on a 32-bit
  // carry chain it never uses, and that chain lands in the fetch loop because a
  // misaligned access traps and a trap chooses the next pc.
  logic [1:0] mem_addr_low;
  assign mem_addr_low = immediate[1:0] + reg_rs1[1:0];

  logic ls_access, ls_capture, ls_answer, ls_answer_valid, region_stall, ls_fault;
  logic stall_other, stall_own;
  assign ls_access = instr_ls_load || instr_ls_store;
  assign region_stall = ls_access && !ls_settled && !ls_answer_valid;
  assign ls_capture = region_stall && !stall_own;

  // THE ANSWER IS HELD UNTIL THE INSTRUCTION IT IS ABOUT ISSUES, not for one
  // cycle. A one-cycle answer livelocks the moment anything else holds the same
  // instruction for longer: the answer expires, `region_stall` comes back, that
  // drops `bus_request`, the grant never arrives, and the pair oscillates
  // forever. `make dual-smoke` reproduced exactly that at cycle 17 -- a
  // single-hart build ties `bus_wait` low, so nothing there can hold a load
  // past its own capture and this is invisible.
  //
  // HELD FOR TWO REASONS AND NO OTHERS, and the list is short because the
  // capture already required every reason of the instruction's own to be low.
  // Nothing has issued since, so the scoreboard cannot have filled, the operand
  // pair cannot have changed and the divider cannot have started -- the only
  // reasons that can newly assert over a captured access are the two that come
  // from outside it: another hart holding the bus, and a text write stealing
  // the fetch window. Neither changes the instruction, so the answer is still
  // about the access it was taken for.
  //
  // Holding on `stall` at large is WRONG and the suite says so: an
  // operand-fetch cycle would carry the answer across into the NEXT access,
  // which then issues on a stale bit without waiting. `ls_access` is in the
  // term for the same reason -- parking on a nop must retire the answer.
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

  // An atomic is word-wide and never split, so an unaligned one faults rather
  // than being emulated. `lr.w` reports as a load and the other ten as stores,
  // which is what the privileged spec asks: an AMO writes, and so does a
  // store-conditional that succeeds. That split is the same one the region fault
  // below makes, so the ten that write are named once here.
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

  // An atomic the platform's memory does not answer. The address is `reg_rs1`
  // with nothing in front of it, so this equality does not wait on a carry
  // chain -- which is what a load's or a store's would, and is why those two
  // causes are still absent.
  //
  // `!word_misaligned` keeps the four data causes disjoint and states an order
  // at the same time: an atomic that is both unaligned and out of region reports
  // the misalignment, which is what the reference model does. Take it out and
  // two arms of the cause chain below match at once.
  //
  // ONE term reaches the pc, not two. Which of the two causes it is depends on
  // which way the instruction goes, and that question is answered off the fetch
  // loop entirely -- `trap_cause` and the RVFI report are its only readers. The
  // other spelling, two terms each carrying their own encoding test, measured
  // 11.98 MHz at a placement this one clears.
  logic atomic_fault;
  assign atomic_fault = instr_atomic && !atomic_supported && !word_misaligned;
  // The same refusal for a plain load or store, read a cycle late off the
  // flip-flop the wait above filled, so the trap chain and the pc never see the
  // effective-address adder. Misalignment outranks the region here too, so the
  // four data causes stay disjoint whichever access raised them.
  assign ls_fault = ls_access && ls_answer_valid && !ls_answer &&
                    !load_misaligned && !store_misaligned;

  logic load_access_fault, store_access_fault;
  assign load_access_fault  = (atomic_fault && instr_lr) || (ls_fault && instr_ls_load);
  assign store_access_fault = (atomic_fault && instr_atomic_write) ||
                              (ls_fault && instr_ls_store);

  // `csr_write_op` already has Zicsr's suppression rules applied, so `csrr misa`
  // stays legal while `csrw misa` does not.
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
  // Bit 31 says interrupt; 7 is the machine timer.
  localparam logic [31:0] CAUSE_MACHINE_TIMER       = 32'h8000_0007;

 `ifdef RISCV_FORMAL
  // The strobe the refused store would have driven, which is what
  // checks/rvfi_insn_check.sv compares the fault report against. The whole read
  // mask is legal for a load -- that check takes a superset there -- but the
  // write mask has to be the exact one, so the width and the two low address
  // bits are read here. An atomic and a word store are both word accesses and
  // report all four; nothing else reaches the `else`, because the arm below
  // publishes this only for a store the map refused.
  logic [3:0] ls_fault_wstrb;
  always_comb begin
    if (instr_sb)      ls_fault_wstrb = 4'b0001 << mem_addr_calc[1:0];
    else if (instr_sh) ls_fault_wstrb = 4'b0011 << mem_addr_calc[1:0];
    else               ls_fault_wstrb = 4'b1111;
  end
 `endif

  // The four causes that happened to a data address rather than to the
  // instruction. Named once and read twice -- here and by the mtval mux below
  // -- because those two must not be able to disagree: a fifth data cause added
  // to the trap chain alone would fault correctly and report nothing about
  // where.
  logic data_fault;
  assign data_fault = load_misaligned || store_misaligned || atomic_fault || ls_fault;

  logic trap_pending;
  assign trap_pending = imem_fault || instr_illegal || instr_ebreak || instr_ecall ||
                        data_fault;

  // The instruction's own fault and an interrupt really can be true together,
  // and the interrupt wins: the instruction does not execute, so its fault does
  // not happen. It faults instead when it re-executes after the handler's
  // `mret`, which is why nothing is lost by taking the interrupt first.
  logic trap_taken;
  assign trap_taken = trap_pending || interrupt_pending;

  // No `(* parallel_case *)` here. No two of the eight synchronous causes can be
  // true at once today, so their order never matters -- but marking the case
  // one-hot tells synthesis it may assume that, and the two arms on top
  // genuinely overlap them. Add a cause that overlaps an existing one and you
  // get whatever the optimiser chose, instead of a failed assertion. The
  // `ifdef FORMAL` block below checks the eight can't overlap each other.
  //
  // The two arms on top are the deliberate overlaps. An interrupt wins because
  // the instruction does not execute, so its own fault does not happen. A fetch
  // that read nothing wins over everything the word could have decoded to,
  // because the word is not an instruction: out of range the memory answers
  // zero, which would otherwise trap as an illegal instruction and report the
  // wrong cause.
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

  // The faulting instruction's address, not the next one's -- that is what lets
  // a handler fix up and resume. For an interrupt it is the address of the
  // instruction that did not issue, so `mret` re-executes it.
  assign trap_epc = fetcher_pc;

  // ...and what it happened to, for mtval. Which value each cause carries is a
  // platform statement firmware cannot derive, so the table is written out in
  // rtl/csrs.v's header, where a reader asking what a CSR holds will look; it
  // is stated once because two copies of it can disagree.
  //
  // Three things about the mux and not about the table. The order is the cause
  // chain's above, for the same reasons -- an interrupt and a fetch that read
  // nothing outrank the word. The arms that report zero are left to the
  // default. And all four data causes share one arm: an atomic's effective
  // address is rs1 verbatim and `mem_addr_calc` is provably equal to it for
  // those encodings, so the sum covers the region pair without their needing
  // `atomic_addr`.
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

  // The pair this instruction names, and the pair the word after it names. Two
  // instances of one mapping: the guess below is checked against the decoded
  // pair, so two copies that disagreed would cost a cycle for a reason no
  // oracle here reports. Both take a raw fetched word -- rtl/regsel.v masks a
  // compressed one itself, so `instr` above is not the input here.
  logic [4:0] next_rs1, next_rs2;
  regsel current_regs (.word(in.instr), .rs1(rs1), .rs2(rs2));
  regsel next_regs (.word(in.next_instr), .rs1(next_rs1), .rs2(next_rs2));

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

  // Does this instruction really read rsN? Not a shift amount, not immediate
  // bits that happen to sit where a register number would, not a Zicsr
  // constant. Both `hazard_rs1`/`hazard_rs2` and `operand_stall` use these.
  // Widen one and the core waits on registers nothing reads. Narrow one and an
  // instruction issues before its operand has arrived.
  logic uses_rs1, uses_rs2;
  assign uses_rs1 = !(instr_lui || instr_jal || instr_auipc || is_csr_imm);
  assign uses_rs2 = (instr_math && !instr_math_immediate) || instr_sb || instr_sh || instr_sw ||
    instr_beq || instr_bne || instr_blt || instr_bltu || instr_bge || instr_bgeu ||
    instr_amo || instr_sc;

  // Do not fold these two into a shared function. iverilog builds a continuous
  // assign's sensitivity list from the arguments at the call site. A function
  // body that also read `out` or `executor_out` would stop re-evaluating when
  // either of those changed. `hazard_rs1` then sticks high at the
  // first conflict and the core stops. iverilog gives no warning for this, and
  // yosys gets the same function right, so every other check stays green.
  logic live_rs1, live_rs2;
  assign live_rs1 = (out.valid && out.rd == rs1) ||
    (executor_out.valid && executor_out.rd == rs1);
  assign live_rs2 = (out.valid && out.rd == rs2) ||
    (executor_out.valid && executor_out.rd == rs2);

  // These three wait until the pipeline is empty, for two different reasons.
  // Narrowing the test to suit one of them breaks the other.
  //
  // A CSR access and `mret` are not waiting for an operand. `minstret` counts up
  // when an instruction issues, not when it finishes, so it already includes
  // instructions still moving down the pipeline. Waiting until they are done
  // makes the count match what has actually finished.
  //
  // `fence.i` waits because text is writable and the fetch address goes out a
  // cycle early. A store needs two edges to reach the memory array. By then the
  // instruction after the `fence.i` has already been fetched, and it read the
  // old text. An empty pipeline means the store has landed.
  logic pipe_drained, serialize;
  assign pipe_drained = !out.valid && !executor_out.valid && !accessor_out_valid;
  assign serialize = (instr_csr_access || instr_mret || instr_fencei) && !pipe_drained;

  logic hazard_rs1, hazard_rs2, hazard, stall;
  assign hazard_rs1 = uses_rs1 && rs1 != 0 && live_rs1;
  assign hazard_rs2 = uses_rs2 && rs2 != 0 && live_rs2;
  assign hazard = hazard_rs1 || hazard_rs2 || serialize;

 `ifdef RISCV_FORMAL
  // Report a register only if the instruction really reads one. The monitor
  // keeps its own record of every register's last write and checks any address
  // reported here against it. Name a register the instruction never read and the
  // two disagree, on a core that is working correctly.
  logic rvfi_rs1_valid, rvfi_rs2_valid;
  assign rvfi_rs1_valid = !instr_lui && !instr_jal && !instr_auipc && !is_csr_imm;
  assign rvfi_rs2_valid = uses_rs2;
 `endif
  // Register reads take one cycle, so reg_rs1/reg_rs2 hold the answer to
  // whatever addresses went out last cycle, not this one. The test below is
  // exactly that question -- was the register file asked for what this
  // instruction reads? -- and only for the ports it reads. lui, jal and auipc
  // read neither, so they never wait. Each writes its own operands into `out`
  // further down instead of passing reg_rs1/reg_rs2 through, so skipping the
  // read cannot leak a stale value.
  //
  // The comparison does not care where the request came from, which is what
  // lets `read_rs1` below be a guess: right and the instruction issues with no
  // bubble at all, wrong and it stalls one cycle and asks again, which is the
  // cycle every instruction used to pay.
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

  // An AMO owns the data bus for two cycles: the read goes out while the
  // executor takes it, and the read-modify-write goes back out from
  // rtl/accessor.v on the cycle after. Nothing else may present a transaction
  // on that cycle, so the cycle is spent here. It is read off `out` rather than
  // routed back from the accessor because `out` is what the accessor is looking
  // at, one register earlier and one module nearer the pc.
  //
  // `!divider_stall` because a held `out` has not been taken yet: no read has
  // gone out, so the write cycle is not next. The divider arm of the publish
  // block below wins for the same reason and this term keeps the two agreeing.
  logic atomic_stall;
  assign atomic_stall = out.valid && out.is_amo && !divider_stall;

  // Every reason to stall, in one signal. They all hold the PC; the block that
  // writes `out` below is where they differ.
  //
  // Split in two because the region wait is the one reason that is about an
  // answer rather than about a resource: `ls_capture` above registers the
  // region answer on the cycle nothing else is holding the instruction, which is
  // the cycle after which rs1 cannot move again. Declared beside the region
  // block; assigned here, where the other seven are born.
  // Everything that holds this instruction for a reason of its OWN, which is
  // every reason but the bus. Named once and read three times: `stall_other`
  // adds the bus to it, `bus_request` asks off it, and the region capture is
  // guarded by it rather than by `stall_other`.
  //
  // THE CAPTURE MUST NOT WAIT ON THE BUS, and the deadlock is immediate if it
  // does: the region wait suppresses `bus_request`, an unasked-for grant never
  // arrives, `bus_wait` stays high, and a capture guarded on it never fires --
  // so the wait never ends. A single-hart build ties `bus_wait` low and cannot
  // see any of that; `make dual-smoke` is what does, and it did.
  //
  // Leaving the bus out costs the guard nothing. What the capture registers is
  // a function of `immediate` and `reg_rs1`: the immediate is decoded from a
  // word that is being held, and rs1 cannot move while `hazard` and
  // `operand_stall` are low, since the scoreboard is clear and nothing issued
  // to start a write. The bus changes neither -- it says only that some other
  // hart is publishing this cycle.
  assign stall_own = hazard || operand_stall || divider_stall || fetch_stall ||
                     atomic_stall;
  assign stall_other = stall_own || bus_wait;
  assign stall = stall_other || region_stall;

  // The same conjunction as `stall` with `bus_wait` left out, and the nine
  // encodings that reach the data bus. It is an OVER-approximation on purpose:
  // a store-conditional that finds no reservation puts nothing on the bus, and
  // asking for a cycle this core then does not use costs an arbitration slot
  // where under-asking would put two initiators on the bus at once. Trapping
  // accesses are out, because decode clears their `is_l*`/`is_s*` flags and the
  // accessor never sees a transaction to make. So is the region wait's own
  // cycle: the transaction it precedes is two cycles away, not one, and asking
  // early there is an over-approximation with nothing to buy.
  assign bus_request = !reset && !trap_taken && !region_stall && !stall_own &&
    (instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw ||
     instr_sb || instr_sh || instr_sw || instr_atomic);

  // On a cycle that issues, ask for the NEXT instruction's pair. The guess runs
  // the fetch window's successor word through the same register-number mapping
  // this instruction went through, so a compressed successor is guessed as
  // accurately as an uncompressed one; what still misses is everything after a
  // redirect. `operand_stall` above is the check.
  //
  // A stalled cycle asks for the current instruction's own pair instead. The pc
  // holds while stalled, so the same instruction comes back; guess there too and
  // the two take turns forever. `stall` rather than `operand_stall` alone,
  // because during a hazard stall the guess is one instruction ahead and would
  // cost an extra cycle after every hazard cleared.
  //
  // A stolen fetch window asks for neither: the word arriving on that cycle is
  // a data word, so `rs1` is a pair decoded from something that is not an
  // instruction. Presenting it throws away whatever was presented the cycle
  // before -- usually the right guess -- and the instruction behind the steal
  // then pays an operand-fetch cycle it had already earned. Holding the pair
  // costs at worst what presenting garbage costs, because `operand_stall` is the
  // same compare either way.
  //
  // This is also what keeps the register file's write-through bypass honest.
  // That mux selects on its own registered copy of whatever was presented, and
  // is right because nothing issues until the held pair is the pair the issuing
  // instruction reads -- not because the held pair is the presented pair, which
  // it deliberately is not on an issuing cycle.
  assign read_rs1 = fetch_stall ? prev_rs1 : stall ? rs1 : next_rs1;
  assign read_rs2 = fetch_stall ? prev_rs2 : stall ? rs2 : next_rs2;

  // All six branch tests come from one subtraction. Unsigned less-than is the
  // borrow out; signed less-than is the same fact except when the operands'
  // sign bits disagree, and then the negative one is smaller; equal is the
  // difference being zero. Written as six comparisons this is six carry chains
  // answering one question.
  //
  // Nothing here is a signed expression, which is the point: a signed comparison
  // written as one arm of a conditional takes its signedness from the other arms
  // and has quietly turned unsigned in this repo twice. Bit tests cannot.
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

  // Every way the PC can change, in one priority chain. No `(* parallel_case *)`
  // here: `stall` and `trap_taken` really can both be true, so the order
  // matters and synthesis must not assume otherwise. `stall` above the trap arm
  // is also what makes an interrupt wait for the pipeline instead of cutting
  // into it.
  always_comb begin
    case (1'b1)
      reset:                     next_pc = 32'b0;
      stall:                     next_pc = pc;
      trap_taken:                next_pc = mtvec;
      instr_mret:                next_pc = mepc;
      instr_jalr:                next_pc = ($signed(immediate) + $signed(reg_rs1)) & 32'hfffffffe;
      // One arm: at 32 bits the signed and unsigned sums have identical bits,
      // so these were the same adder written twice.
      instr_jal || branch_taken: next_pc = fetcher_pc + immediate;
      // Sequential: +4 for a 32-bit instruction, +2 for a compressed one.
      default:                   next_pc = fetcher_pc + pc_inc;
    endcase
  end

  // `issuing` is a cycle the fetch window is consumed: no reset, no stall. It
  // is the guard on the last two arms of the publish block below taken
  // together, term for term -- an interrupt consumes the cycle and reaches the
  // bubble arm, everything else reaches the final one. In particular do not add
  // `&& in.valid`: those arms do not test it either. If they ever disagree a
  // CSR write fires once per stalled cycle instead of once per instruction, and
  // nothing says so.
  logic issuing;
  assign issuing = !reset && !stall;

  logic committing;
  assign committing = issuing && !trap_taken;
  assign csr_ren = committing && csr_read_op;
  assign csr_wen = committing && csr_write_op;
  // A trapping instruction still issues, and RVFI still reports it, but it
  // changed nothing. It must not be counted. An interrupted one did not even
  // issue.
  assign instret = committing;

  assign trap_entry = issuing && trap_taken;
  assign mret_entry = committing && instr_mret;

 `ifdef RISCV_FORMAL
  // RVFI's flag for "this retire is the first instruction of a handler the
  // previous retire did not hand off to". An exception reports its own redirect
  // in `pc_wdata`, so only an interrupt breaks the chain and only an interrupt
  // sets this. Both sim legs' monitor stops checking pc continuity across a
  // retire that carries it; without it, every interrupt is a monitor error.
  logic intr_report;
  always_ff @(posedge clk) begin
    if (reset) intr_report <= 1'b0;
    else if (issuing) intr_report <= interrupt_pending;
  end

  // The eight plain load and store encodings, atomics excluded: an atomic's
  // address is rs1 verbatim, so the questions counted up in rtl/littlecpu.v are
  // not asked about one. `committing` rather than `issuing`, so a trapping
  // access -- which makes no transaction -- is not counted as one.
  assign probe_ls_issuing = committing &&
    (instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw ||
     instr_sb || instr_sh || instr_sw);
 `endif

  always_ff @(posedge clk) pc <= next_pc;

  always_ff @(posedge clk) begin
    if (reset) begin
      out <= '0;
    end else if (divider_stall) begin
      // This fires a cycle after the divide that caused it, so `out` holds an
      // instruction the executor has not taken yet. Zeroing it would lose that
      // instruction. This arm has to come before the next one: if a `fetch_stall`
      // lands on the same cycle, holding still wins. Swapping the
      // two arms changes that and nothing would say so, which is why the
      // `ifdef FORMAL` block below checks both cases.
      out <= out;
    end else if (hazard || operand_stall || fetch_stall || atomic_stall || bus_wait ||
                 region_stall || interrupt_pending) begin
      // These zero `out` instead of holding it. The executor reads `in` every
      // cycle, so a held `out` would be executed again and again while the
      // stalled instruction waits. The interrupt is here rather than in the arm
      // below because the instruction it displaces did not issue: it publishes
      // nothing, retires nothing and re-executes after `mret`.
      //
      // The atomic wait BUBBLES, and that is the opposite ruling from the
      // divider's. Its cycle is raised after the executor has already taken the
      // instruction, so holding would present the same AMO a second time and
      // put a second read on the bus beside its own write. Holding here is a
      // duplicated transaction and a duplicated retire, not a lost instruction.
      //
      // The bus wait bubbles for the same reason arrived at from the other end:
      // the executor is idle through it, so it takes a held `out` and executes
      // it again. Nothing was published while waiting, so there is nothing to
      // lose by publishing a bubble.
      out <= '0;
    end else begin
      // This arm runs on exactly the cycles an instruction issues. Committing
      // traps at the bottom of it is what makes the rest free: no trap on a
      // stalled cycle, none from an empty cycle, and never twice for the same
      // instruction. `hazard_rs1` has also already cleared by now, so the
      // misalignment test reads the real value of rs1.
      out.valid <= 1'b1;
      out.mem_addr <= mem_addr_calc;
      out.rs1 <= reg_rs1;
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
      out.rvfi.intr <= intr_report;
      out.rvfi.mem_fault <= imem_fault || load_access_fault || store_access_fault;
      // An lr.w reads, an sc.w writes, and an AMO does both -- so the masks are
      // the access the instruction would have made, not the one it made. The
      // read half names the AMOs rather than excusing `sc.w`, because a plain
      // store raises this cause now and reads nothing either.
      out.rvfi.mem_fault_rmask <= {4{load_access_fault || (store_access_fault && instr_amo)}};
      out.rvfi.mem_fault_wmask <= store_access_fault ? ls_fault_wstrb : 4'b0;
      out.rvfi.mem_fault_addr <= {mem_addr_calc[31:2], 2'b00};
      out.rvfi.rs1_addr <= rvfi_rs1_valid ? rs1 : 5'b0;
      out.rvfi.rs2_addr <= rvfi_rs2_valid ? rs2 : 5'b0;
      out.rvfi.rs1_rdata <= rvfi_rs1_valid ? reg_rs1 : 32'b0;
      out.rvfi.rs2_rdata <= rvfi_rs2_valid ? reg_rs2 : 32'b0;
      // A non-CSR instruction gets all-zero masks for free, because
      // `csr_ren`/`csr_wen` are low for it.
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
          // AUIPC has no rs1 or rs2 field. The bits the default selection reads
          // are part of its immediate, so both operands are written here
          // directly and added as if it were an add.
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

        instr_lui: begin
          // The immediate IS the result, so it arrives as an add of zero rather
          // than as a flag of its own with an executor arm behind it. LUI reads
          // no register: the bits the default selection reads are its immediate.
          out.rs1 <= immediate;
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

      // Last in the block, so these writes win over every flag set above and no
      // arm has to know traps exist. `out.valid` stays high: the instruction
      // still reaches the end of the pipeline, having done nothing but change
      // the PC. Clearing the flags is the whole of that. rtl/accessor.v starts
      // no memory access with none of them set, and rd of 0 stops
      // rtl/writeback.v writing a register.
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
        out.rd <= 0; // prevent writeback to arbitrary register
      end
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // Assume reset is high before the first edge. Nothing proves this; every
  // harness in the tree happens to do it. Before that edge `out` and the history
  // registers below hold no defined value, so nothing here means anything
  // without it.
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // Assume reset never comes back. Nothing proves this either. The pc-increment
  // assertions below would otherwise have to allow for reset forcing pc to 0 at
  // any point. It is unguarded, so it also covers the trap assertions further
  // down; those already test `!prev_reset`, so it costs nothing there.
  always_comb if (clocked) assume(!reset);

  // Assume the fetcher hands back the pc this module gave it. rtl/fetcher.v does
  // that with a wire. formal/pcloop.sv builds the real fetcher and asserts it
  // instead of assuming it, so this is checked somewhere; that task runs on CI.
  // This assume covers the whole file, not just the assertions it was written
  // for, and nothing else below reads `in.pc`.
  always_comb assume(in.pc == pc);

  // Assume a held instruction is still the same instruction. The machine does
  // it: a stalled cycle holds the pc and `imem_addr` is the pc, so the memory
  // re-presents the same words; and a stalled cycle issues nothing, so no write
  // to rs1 can be started on it, while the decode scoreboard holds any
  // instruction whose rs1 is already in flight until that write has gone
  // through. This module can see none of that, which is why it is the same
  // shape of standalone environment model as the fetcher assume above --
  // formal/traps.sv states it over `fetcher_out` and formal/components.sby
  // drops both copies there with `-noassume`.
  //
  // The region answer below is the first state in this core computed from a
  // decode input on one cycle and read on the next, so it is the first thing
  // here that needs the fact written down rather than relied on.
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

  // Each of these keeps its own copy of last cycle's value instead of using
  // $past(). `in.pc` and `instr` are free inputs here, and $past() through
  // another register would let the solver fill in a value from before reset that
  // this proof cannot rule out. `trap_taken` and `instr_mret` belong in
  // `branch_jump` because both change the pc, and `trap_taken` rather than
  // `trap_pending` so an interrupt counts too. Leave any of them out and the
  // proof stops believing a redirect is a branch while the RTL still treats it
  // as one.
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

  // Both halves of the arm order above. Nothing else enforces it and both ways
  // of getting it wrong are silent: zeroing `out` when a stall and a freeze
  // coincide loses an instruction, and holding it on a plain stall hands the
  // executor the same instruction twice.
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

  // The bus wait's half of the same ruling, and it lands in the same two arms
  // for its own reasons: an ungranted cycle publishes nothing, and a wait
  // arriving during a divide must not throw away the instruction the executor
  // has not taken yet. Nothing else in the tree says which arm it belongs in.
  always_comb if (clocked && !prev_reset && prev_hold_and_wait) assert(out == past_out);
  always_comb if (clocked && !prev_reset && prev_wait_only)     assert(out == '0);

  // The atomic wait's half of that ruling, and it goes the other way. The
  // executor has already taken the AMO on the cycle this is raised, so holding
  // would re-present it: a second bus transaction beside the write it is
  // waiting for, and a second retire of one instruction. Nothing else in the
  // tree says which arm it belongs in.
  always_comb if (clocked && !prev_reset && prev_atomic_stall) assert(out == '0);

  // The region wait's half, and it bubbles: nothing has issued, so there is
  // nothing to re-present and holding would hand the executor an instruction
  // twice. It lands in the divider's arm for the same reason the stolen window
  // and the bus wait do -- a held `out` is an instruction the executor has not
  // taken. This is only arm order in the publish block above, so nothing else
  // in the tree says which arm it belongs in.
  always_comb if (clocked && !prev_reset && prev_hold_and_region) assert(out == past_out);
  always_comb if (clocked && !prev_reset && prev_region_only)     assert(out == '0);

  // Obviously true one flip-flop apart, and that is the point. It fails the
  // moment something else also writes `pc`. A second writer puts the
  // instruction memory a cycle out of step with decode, and the only symptom is
  // that every instruction executed is the wrong one.
  logic [31:0] past_next_pc;
  always_ff @(posedge clk) past_next_pc <= next_pc;
  always_comb if (clocked) assert(pc == past_next_pc);

  // An empty `out` is zeroed all the way through, so it can never carry an rd
  // that writes a register.
  always_comb if (clocked && !out.valid) assert(out.rd == 0);

  // The published AMO bit is exactly the nine functions beside it. Both readers
  // take it on trust now -- `atomic_stall` above and rtl/accessor.v, which
  // assumes this -- so nothing else would notice the two disagreeing, and a
  // disagreement is either a bus cycle nobody spent or an AMO with no read.
  always_comb if (clocked)
    assert(out.is_amo == (out.is_amoswap || out.is_amoadd || out.is_amoxor ||
      out.is_amoand || out.is_amoor || out.is_amomin || out.is_amomax ||
      out.is_amominu || out.is_amomaxu));

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
    // Everything `instr_valid` accepts has to appear here too. Keep the forms
    // that differ only in funct3 or funct12 on separate terms, so a decode
    // change that makes two of them true at once still trips this.
    instr_ebreak, instr_csrrw, instr_csrrs, instr_csrrc,
    instr_mret, instr_wfi, instr_fence, instr_fencei,
    instr_amoswap, instr_amoadd, instr_amoxor, instr_amoand, instr_amoor, instr_amomin,
    instr_amomax, instr_amominu, instr_amomaxu, instr_lr, instr_sc});

  always_comb if (instr_valid) assert(one_of);

  // Three case statements above are marked one-hot for synthesis, and the check
  // just above does not reach them -- their arms are opcode groups and
  // compressed flags, which it does not name. Each assertion below is one of
  // those arm lists. Without it an encoding change that made two arms match at
  // once would leave synthesis free to pick either, and nothing would say so.
  // Each list is transcribed, not shared with the statement it describes, so
  // adding an arm to one means adding it here too. The two marked statements
  // that pick register numbers moved to rtl/regsel.v and carry their own.
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

  // This is what makes the trap-cause case above safe without a one-hot marking.
  // Add a ninth cause that overlaps an existing one and this fails here. It is
  // stated under `!imem_fault` because that arm deliberately overlaps every one
  // of them: a word the memory never supplied decodes to zero, which is illegal,
  // and reads register fields that could name any address at all.
  always_comb if (!imem_fault)
    assert($onehot0({instr_illegal, instr_ebreak, instr_ecall,
                     load_misaligned, store_misaligned,
                     load_access_fault, store_access_fault}));

  // The address the platform decodes really is the address the instruction uses,
  // for every encoding that can raise the two region causes. Nothing else says
  // so: `atomic_addr` skips the adder on purpose, so a change to the immediate
  // mux would leave the fault talking about one address and the transaction
  // about another, with the fault's own tests still green.
  always_comb if (instr_atomic) assert(mem_addr_calc == atomic_addr);

  // THE FAST PATH NEVER SKIPS A FAULT. `ls_settled` is answered from `reg_rs1`
  // alone and lets a load or store issue with no region term in it, so it has to
  // imply the answer about the effective address; nothing else in the tree says
  // so, and getting it wrong reads memory that is not there instead of trapping.
  // The first line is the fact it stands on -- every encoding that can raise
  // these two causes sign-extends its immediate above bit 11, so the address
  // cannot leave the block `reg_rs1` names or the two beside it.
  always_comb if (ls_access) begin
    assert(immediate[31:12] == {20{immediate[31]}});
    if (ls_settled) assert(ls_supported);
  end

  // THE ANSWER NEVER OUTLIVES THE ACCESS IT WAS TAKEN FOR. That is what the
  // register file's guarantee has to cover -- rs1 cannot move under a held
  // instruction -- and it is asserted rather than argued, in two statements
  // neither of which is the RTL's own clause restated.
  //
  // Issuing retires it: if the instruction went out last cycle then decode has
  // moved on, and any answer surviving into this one is about an access that is
  // gone. Stated in terms of `stall` rather than of the two hold reasons, so it
  // is a claim about the design and not the assignment read back.
  //
  // The companion claim -- a valid answer implies a load or store is still in
  // decode -- is `test/decoder_tb.v`'s and deliberately NOT asserted here. This
  // harness drives `instr` as a free input, so it can put a different
  // instruction under a captured answer on the very next cycle, which the fetch
  // path cannot: the pc holds through a stall and re-presents the same word.
  // Asserting it here would fail on a trace the machine cannot reach. An
  // earlier spelling of the hold really did carry an answer across an
  // operand-fetch cycle into the next access, and the bench is what caught it.
  // `prev_issued` is the one the hold assume above keeps, `|| reset` included:
  // an answer cannot survive a reset cycle either, since reset clears it.
  logic prev_answer_valid;
  always_ff @(posedge clk) prev_answer_valid <= ls_answer_valid;
  always_comb if (clocked && prev_answer_valid && prev_issued)
    assert(!ls_answer_valid);

  // A VALID ANSWER IS AN ANSWER ABOUT THE ACCESS THAT IS IN DECODE NOW. The
  // region flip-flop is the only state here computed from a decode input on one
  // cycle and read on the next, and `ls_fault` reads it with no term of its own
  // about the effective address -- so this is the whole of what keeps the two
  // region causes about the access that raised them.
  //
  // It stands on the hold assume above and on nothing else, which is why it is
  // stated where the two flip-flops are rather than argued in a comment: it is
  // what formal/traps.sv's induction needs and cannot reach. That harness sees
  // no decoder internal, and both ways of reaching for one are silent --
  // `decoder.ls_answer_valid` parses as an implicitly declared undriven wire
  // and `bind` is ignored -- so an invariant written there is a claim about a
  // free wire, passing induction and failing the base case.
  always_comb if (clocked && ls_answer_valid) assert(ls_answer == ls_supported);

  // One line per cause, not a copy of the case statement, so reordering its arms
  // trips these instead of changing them to match. The eight synchronous causes
  // are each stated under the two arms above them, which is the priority those
  // arms buy: the interrupted instruction did not execute, so its own fault is
  // not what happened, and a word that was never fetched decoded to nothing.
  //
  // `word_decides` is the two arms' priority, named once rather than repeated on
  // every line below: neither an interrupt nor a fetch that read nothing has
  // anything to do with the word, so the word only settles the cause when both
  // are clear.
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

  // No riscv-formal check can see that the trap went to mtvec. Its instruction
  // check drops every value comparison once an instruction traps, and its pc
  // checks accept any target so long as the core reports the one it took.
  always_comb if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_comb if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);

  always_comb if (clocked && !prev_reset && prev_trap_entry) begin
    assert(out.rd == 5'b0);
    assert(!out.is_lb && !out.is_lbu && !out.is_lh && !out.is_lhu && !out.is_lw);
    assert(!out.is_sb && !out.is_sh && !out.is_sw);
    assert(!out.is_amo && !out.is_lr && !out.is_sc);
  end

  // Gating these on `instr_valid` instead would look right and be weaker: an
  // instruction can decode fine and still trap. This catches that.
  always_comb if (trap_taken) assert(!instret && !csr_wen && !csr_ren);
  always_comb assert(!(trap_entry && mret_entry));

  // An interrupt is only ever taken on a cycle that would otherwise have issued
  // an instruction. Below the stall arm, so a divide, a hazard, a serialization
  // drain and the operand-fetch cycle each hold it off.
  always_comb if (interrupt_pending && !stall && !reset) assert(trap_entry);
  always_comb if (stall || reset) assert(!trap_entry);

  // ...and it publishes nothing. The instruction it displaced has not issued,
  // so nothing downstream ever hears about it and there is nothing to take
  // back. `out` is registered, hence the one-cycle delay.
  logic prev_interrupt_entry;
  always_ff @(posedge clk) prev_interrupt_entry <= !reset && !stall && interrupt_pending;
  always_comb if (clocked && !prev_reset && prev_interrupt_entry) assert(!out.valid);

  // The six branch tests come from one subtraction, and the misalignment test
  // adds two bits rather than reading them off the effective address. Both are
  // arithmetic identities the code no longer states, so they are stated here
  // against the operators they replaced. Each reference is its own
  // self-determined statement over signed nets, never an arm of a conditional:
  // that is what a signed comparison loses its signedness to.
  logic signed [31:0] cmp_ref_x, cmp_ref_y;
  assign cmp_ref_x = reg_rs1;
  assign cmp_ref_y = reg_rs2;
  always_comb assert(cmp_eq == (reg_rs1 == reg_rs2));
  always_comb assert(cmp_ltu == (reg_rs1 < reg_rs2));
  always_comb assert(cmp_lt == (cmp_ref_x < cmp_ref_y));
  always_comb assert(mem_addr_low == mem_addr_calc[1:0]);
 `endif
endmodule
