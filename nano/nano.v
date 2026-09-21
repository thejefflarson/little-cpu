module riscv #(
  // The RAM window a load/store must land in or fault (cause 5/7); matches nano.lds.
  parameter logic [31:0] RAM_BASE  = 32'h0001_0000,
  parameter int          RAM_WORDS = 4096
) (
  input  logic        clk,
  input  logic        reset,
  output logic        mem_valid,
  output logic        mem_instr,
  input  logic        mem_ready,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
  // Machine external interrupt, level-triggered, synchronized here.
  input  logic        irq_meip,
  output logic        trap
 `ifdef RISCV_FORMAL
   , `RVFI_OUTPUTS
   // Not part of RVFI: a proof over trap entry needs these named, self-reported.
   , output logic [31:0] rvfi_dbg_mtvec
   , output logic [31:0] rvfi_dbg_mepc
   , output logic [31:0] rvfi_dbg_mcause
   , output logic [31:0] rvfi_dbg_mtval
   , output logic [31:0] rvfi_dbg_mstatus
 `endif
  );

  // Declared here, ahead of every use, since iverilog needs an identifier in scope first.
  logic [31:0] instr;
  logic [4:0] opcode;
  logic [1:0] quadrant, cfunct2, cmath_funct2;
  logic uncompressed;
  logic [2:0] funct3, cfunct3;
  logic [3:0] cfunct4;
  logic [5:0] cfunct6;
  logic [6:0] funct7;
  logic [31:0] i_immediate, s_immediate, b_immediate, u_immediate, j_immediate;
  logic [31:0] cl_immediate, clwsp_immediate, cli_immediate, css_immediate, cj_immediate,
    cb_immediate, clui_immediate, caddi_immediate, caddi16sp_immediate, caddi4spn_immediate;
  logic [31:0] immediate;
  logic is_lui, is_lui_op, is_auipc, is_jal, is_jal_op, is_jalr, is_jalr_op, is_cj, is_cjal, is_cjr,
    is_cjalr, is_clui;
  logic [31:0] jump_address;
  logic is_branch_op, is_branch, is_beq, is_bne, is_blt, is_bltu, is_bge, is_bgeu, is_cbeqz,
    is_cbnez;
  logic is_load_op, is_load, is_lb, is_lh, is_lw, is_lbu, is_lhu, is_clwsp, is_clw;
  logic is_store, is_store_op, is_sb, is_sh, is_sw, is_cswsp, is_csw;
  logic math_low;
  logic math_high;
  logic is_math_immediate_op, is_math_immediate, is_addi, is_slti, is_sltiu, is_xori, is_ori,
    is_andi, is_slli, is_srli, is_srai, is_cli, is_caddi, is_caddi16sp, is_caddi4spn, is_cslli,
    is_csrli, is_csrai, is_candi;
  logic is_math_op, is_math, is_add, is_sub, is_sll, is_slt, is_sltu, is_xor, is_srl, is_sra, is_or,
    is_and, is_cmv, is_cadd, is_cand, is_cor, is_cxor, is_csub;
  logic is_opm_encoding;
  logic [31:0] math_arg;
  logic [4:0] shamt;
  logic is_system_op, is_csr, is_csrrw, is_csrrs, is_csrrc, is_csrrwi, is_csrrsi, is_csrrci;
  logic is_error, is_ecall, is_ebreak, is_mret;
  logic rs1_valid, rs2_valid;
  logic is_e_illegal;
  logic is_valid;
  logic [31:0] regs[0:15];

  localparam logic [31:0] MISA_VALUE = 32'h4000_0014; // RV32, E, C
  localparam logic [11:0] CSR_MSTATUS    = 12'h300;
  localparam logic [11:0] CSR_MISA       = 12'h301;
  localparam logic [11:0] CSR_MSTATUSH   = 12'h310;
  localparam logic [11:0] CSR_MIE        = 12'h304;
  localparam logic [11:0] CSR_MTVEC      = 12'h305;
  localparam logic [11:0] CSR_MSCRATCH   = 12'h340;
  localparam logic [11:0] CSR_MEPC       = 12'h341;
  localparam logic [11:0] CSR_MCAUSE     = 12'h342;
  localparam logic [11:0] CSR_MTVAL      = 12'h343;
  localparam logic [11:0] CSR_MIP        = 12'h344;
  localparam logic [11:0] CSR_MCYCLE     = 12'hB00;
  localparam logic [11:0] CSR_MINSTRET   = 12'hB02;
  localparam logic [11:0] CSR_MCYCLEH    = 12'hB80;
  localparam logic [11:0] CSR_MINSTRETH  = 12'hB82;
  localparam logic [11:0] CSR_MVENDORID  = 12'hF11;
  localparam logic [11:0] CSR_MARCHID    = 12'hF12;
  localparam logic [11:0] CSR_MIMPID     = 12'hF13;
  localparam logic [11:0] CSR_MHARTID    = 12'hF14;
  localparam logic [11:0] CSR_MCONFIGPTR = 12'hF15;

  localparam logic [31:0] CAUSE_ILLEGAL_INSTRUCTION = 32'd2;
  localparam logic [31:0] CAUSE_BREAKPOINT          = 32'd3;
  localparam logic [31:0] CAUSE_LOAD_MISALIGNED     = 32'd4;
  localparam logic [31:0] CAUSE_LOAD_ACCESS_FAULT   = 32'd5;
  localparam logic [31:0] CAUSE_STORE_MISALIGNED    = 32'd6;
  localparam logic [31:0] CAUSE_STORE_ACCESS_FAULT  = 32'd7;
  localparam logic [31:0] CAUSE_ECALL_M             = 32'd11;
  localparam logic [31:0] CAUSE_MACHINE_EXTERNAL    = 32'h8000_000B;

  logic [63:0] mcycle, minstret;
  logic [31:0] mscratch, mtvec, mepc, mcause, mtval;
  logic        mstatus_mie, mstatus_mpie, mie_meie;
  logic        irq_meip_sync1, irq_meip_sync2;

  logic [11:0] csr_addr;
  logic [31:0] csr_rdata, csr_arg, csr_new_value;
  logic        csr_implemented, csr_readonly_write, csr_src_zero, csr_write_op, csr_wen;
  logic        wr_mcycle, wr_mcycleh, wr_minstret, wr_minstreth;
  logic        instret;
  logic        hpm_number;
  logic        hpm_counter_window, hpm_event_window, hpm_zero;

  logic        interrupt_pending, take_interrupt;
  logic        load_misaligned, store_misaligned, ls_in_range, load_region_fault,
               store_region_fault;
  logic        take_trap;
  logic [31:0] trap_cause_value, trap_tval_value;
  logic [31:0] pc;
  logic [4:0] rd, rs1, rs2;
  logic [31:0] load_store_address;
  logic [1:0] addr24;
  logic addr16;
  logic addr8;
  logic [3:0] store_wstrb;
  logic [31:0] next_pc;
  logic [31:0] pc_inc;
  logic [31:0] reg_wdata;
  logic [31:0] pc_wdata;
  logic [3:0] cpu_state;
  logic skip_reg_write;

`ifdef NANO_ONE_PORT_RF
  // One held register per operand; `rf_raddr` is the one address that reads `regs[]`.
  logic [31:0] op_rs1, op_rs2;
  logic [3:0] rf_raddr;
`define RF_RS1 op_rs1
`define RF_RS2 op_rs2
`else
`define RF_RS1 regs[rs1[3:0]]
`define RF_RS2 regs[rs2[3:0]]
`endif

  // instruction decoder (figure 2.3)
  assign opcode = instr[6:2];
  assign quadrant = instr[1:0];
  assign uncompressed = quadrant == 2'b11;
  assign funct3 = instr[14:12];
  assign cfunct3 = instr[15:13];
  assign cfunct2 = instr[11:10];
  assign cmath_funct2 = instr[6:5];
  assign cfunct4 = instr[15:12];
  assign cfunct6 = instr[15:10];
  assign funct7 = instr[31:25];

  // immediate decoder (figure 2.4 & table 16.1)
  assign i_immediate = {{20{instr[31]}}, instr[31:20]};
  assign s_immediate = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  assign b_immediate = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
  assign u_immediate = {instr[31], instr[30:20], instr[19:12], 12'b0};
  assign j_immediate = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

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
    (* parallel_case, full_case *)
    case (1'b1)
      is_load_op || is_jalr: immediate = i_immediate;
      is_store_op: immediate = s_immediate;
      is_lui_op || is_auipc: immediate = u_immediate;
      is_jal_op: immediate = j_immediate;
      is_branch_op: immediate = b_immediate;
      is_math_immediate_op: immediate = i_immediate;
      is_clwsp: immediate = clwsp_immediate;
      is_cswsp: immediate = css_immediate;
      is_csw: immediate = cl_immediate;
      is_clw: immediate = cl_immediate;
      is_cj || is_cjal: immediate = cj_immediate;
      is_cbeqz || is_cbnez: immediate = cb_immediate;
      is_cli: immediate = cli_immediate;
      is_clui: immediate = clui_immediate;
      is_caddi: immediate = caddi_immediate;
      is_caddi16sp: immediate = caddi16sp_immediate;
      is_caddi4spn: immediate = caddi4spn_immediate;
      is_candi: immediate = caddi_immediate;
      default: immediate = 32'b0;
    endcase
  end

  // Table 24.2 RV32I and Table 16.5-7
  assign is_lui_op = opcode == 5'b01101 && uncompressed;
  assign is_lui = is_lui_op || is_clui;
  assign is_clui = quadrant == 2'b01 && cfunct3 == 3'b011 && clui_immediate != 0 &&
    instr[11:7] != 2;
  assign is_auipc = opcode == 5'b00101 && uncompressed;
  assign is_jal_op = opcode == 5'b11011 && uncompressed;
  assign is_jal = is_jal_op || is_cj || is_cjal;
  assign is_jalr_op = opcode == 5'b11001 && uncompressed && funct3 == 3'b000;
  assign is_jalr = is_jalr_op || is_cjr || is_cjalr;
  assign is_cj = quadrant == 2'b01 && cfunct3 == 3'b101;
  assign is_cjal = quadrant == 2'b01 && cfunct3 == 3'b001;
  assign is_cjr = quadrant == 2'b10 && cfunct3 == 3'b100 && instr[12] == 0 && instr[6:2] == 0 &&
    instr[11:7] != 0;
  assign is_cjalr = quadrant == 2'b10 && cfunct3 == 3'b100 && instr[12] == 1 && instr[6:2] == 0 &&
    instr[11:7] != 0;
  assign jump_address = is_jalr || is_cjr || is_cjalr ?
    ($signed(immediate) + $signed(`RF_RS1)) & 32'hfffffffe :
    $signed(pc) + $signed(immediate);

  assign is_branch_op = opcode == 5'b11000 && uncompressed;
  assign is_beq = (is_branch_op && funct3 == 3'b000) || is_cbeqz;
  assign is_bne = (is_branch_op && funct3 == 3'b001) || is_cbnez;
  assign is_blt = is_branch_op && funct3 == 3'b100;
  assign is_bge = is_branch_op && funct3 == 3'b101;
  assign is_bltu = is_branch_op && funct3 == 3'b110;
  assign is_bgeu = is_branch_op && funct3 == 3'b111;
  assign is_cbeqz = quadrant == 2'b01 && cfunct3 == 3'b110;
  assign is_cbnez = quadrant == 2'b01 && cfunct3 == 3'b111;
  assign is_branch = is_beq || is_bne || is_blt || is_bge || is_bltu || is_bgeu;

  assign is_load_op = opcode == 5'b00000 && uncompressed;
  assign is_lb = is_load_op && funct3 == 3'b000;
  assign is_lh = is_load_op && funct3 == 3'b001;
  assign is_lw = (is_load_op && funct3 == 3'b010) || is_clwsp || is_clw;
  assign is_lbu = is_load_op && funct3 == 3'b100;
  assign is_lhu = is_load_op && funct3 == 3'b101;
  assign is_clwsp = quadrant == 2'b10 && cfunct3 == 3'b010 && instr[11:7] != 5'b0;
  assign is_clw = quadrant == 2'b00 && cfunct3 == 3'b010;
  assign is_load = is_lb || is_lh || is_lw || is_lbu || is_lhu;

  assign is_store_op = opcode == 5'b01000 && uncompressed;
  assign is_sb = is_store_op && funct3 == 3'b000;
  assign is_sh = is_store_op && funct3 == 3'b001;
  assign is_sw = (is_store_op && funct3 == 3'b010) || is_cswsp || is_csw;
  assign is_cswsp = quadrant == 2'b10 && cfunct3 == 3'b110;
  assign is_csw = quadrant == 2'b00 && cfunct3 == 3'b110;
  assign is_store = is_sb || is_sh || is_sw;

  assign math_low = funct7 == 7'b0000000;
  assign math_high = funct7 == 7'b0100000;
  assign is_math_immediate_op = opcode == 5'b00100 && uncompressed;
  assign is_addi = (is_math_immediate_op && funct3 == 3'b000) || is_cli || is_caddi ||
    is_caddi16sp || is_caddi4spn;
  assign is_caddi = quadrant == 2'b01 && cfunct3 == 3'b000;
  assign is_caddi16sp = quadrant == 2'b01 && cfunct3 == 3'b011 && instr[11:7] == 2 &&
    caddi16sp_immediate != 0;
  assign is_caddi4spn = quadrant == 2'b00 && cfunct3 == 3'b000 && caddi4spn_immediate != 0;
  // c.li is addi in disguise
  assign is_cli = quadrant == 2'b01 && cfunct3 == 3'b010;
  assign is_slti = is_math_immediate_op && funct3 == 3'b010;
  assign is_sltiu = is_math_immediate_op && funct3 == 3'b011;
  assign is_xori = is_math_immediate_op && funct3 == 3'b100;
  assign is_ori = is_math_immediate_op && funct3 == 3'b110;
  assign is_andi = (is_math_immediate_op && funct3 == 3'b111) || is_candi;
  assign is_candi = quadrant == 2'b01 && cfunct3 == 3'b100 && cfunct2 == 2'b10;
  assign is_slli = (is_math_immediate_op && math_low && funct3 == 3'b001) || is_cslli;
  assign is_srli = (is_math_immediate_op && math_low && funct3 == 3'b101) || is_csrli;
  assign is_srai = (is_math_immediate_op && math_high && funct3 == 3'b101) || is_csrai;
  assign is_cslli = quadrant == 2'b10 && cfunct4 == 4'b0000;
  assign is_csrli = quadrant == 2'b01 && cfunct4 == 4'b1000 && cfunct2 == 2'b00;
  assign is_csrai = quadrant == 2'b01 && cfunct4 == 4'b1000 && cfunct2 == 2'b01;
  assign is_math_immediate = is_addi || is_slti || is_sltiu || is_xori || is_ori || is_andi ||
    is_slli || is_srli || is_srai;

  assign is_math_op = opcode == 5'b01100 && uncompressed;
  // The eight cut M encodings: legal but unimplemented, traps like anything else this
  // core does not decode. Excluded from RVFI's retirement stream (not just reported
  // as trapping) since the shared sim monitor's spec model still claims M exists.
  assign is_opm_encoding = is_math_op && funct7 == 7'b0000001;
  assign is_add = (is_math_op && math_low && funct3 == 3'b000) || is_cmv || is_cadd;
  assign is_cmv = quadrant == 2'b10 && cfunct4 == 4'b1000 && instr[6:2] != 0;
  assign is_cadd = quadrant == 2'b10 && cfunct4 == 4'b1001 && instr[6:2] != 0;
  assign is_sub = (is_math_op && math_high && funct3 == 3'b000) || is_csub;
  assign is_csub = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b00;
  assign is_sll = is_math_op && math_low && funct3 == 3'b001;
  assign is_slt = is_math_op && math_low && funct3 == 3'b010;
  assign is_sltu = is_math_op && math_low && funct3 == 3'b011;
  assign is_xor = (is_math_op && math_low && funct3 == 3'b100) || is_cxor;
  assign is_cxor = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b01;
  assign is_srl = is_math_op && math_low && funct3 == 3'b101;
  assign is_sra = is_math_op && math_high && funct3 == 3'b101;
  assign is_or = (is_math_op && math_low && funct3 == 3'b110) || is_cor;
  assign is_cor = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b10;
  assign is_and = (is_math_op && math_low && funct3 == 3'b111) || is_cand;
  assign is_cand = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b11;
  assign is_math = is_add || is_sub || is_sll || is_slt || is_sltu || is_xor || is_srl || is_sra ||
    is_or || is_and;

  assign math_arg = is_math_immediate ? immediate : `RF_RS2;
  assign shamt = is_math_immediate ? rs2 : `RF_RS2[4:0];

  // is_system_op alone is not "is a CSR instruction": funct3 000 is ecall/ebreak/mret.
  assign is_system_op = opcode == 5'b11100 && uncompressed;
  assign is_csrrw = is_system_op && funct3 == 3'b001;
  assign is_csrrs = is_system_op && funct3 == 3'b010;
  assign is_csrrc = is_system_op && funct3 == 3'b011;
  assign is_csrrwi = is_system_op && funct3 == 3'b101;
  assign is_csrrsi = is_system_op && funct3 == 3'b110;
  assign is_csrrci = is_system_op && funct3 == 3'b111;
  assign is_csr = is_csrrw || is_csrrs || is_csrrc || is_csrrwi || is_csrrsi || is_csrrci;

  // rs1 holds instr[19:15] for every CSR form, register or immediate.
  assign csr_addr = instr[31:20];
  assign csr_src_zero = rs1 == 5'b0;
  assign csr_write_op = is_csr &&
    !((is_csrrs || is_csrrc || is_csrrsi || is_csrrci) && csr_src_zero);
  assign csr_readonly_write = is_csr && csr_write_op && csr_addr[11:10] == 2'b11;
  assign csr_arg = (is_csrrwi || is_csrrsi || is_csrrci) ? {27'b0, rs1} : `RF_RS1;
  assign csr_new_value = (is_csrrw || is_csrrwi) ? csr_arg :
                         (is_csrrs || is_csrrsi) ? (csr_rdata | csr_arg) :
                                                    (csr_rdata & ~csr_arg);

  assign is_error = is_system_op && funct3 == 0 && rs1 == 0 && rd == 0;
  assign is_ecall = is_error && instr[31:20] == 12'h000;
  assign is_ebreak = is_error && instr[31:20] == 12'h001;
  assign is_mret = is_error && instr[31:20] == 12'h302;

  // RV32E: x0-x15 only, bit 4 of a decoded field says whether it named x16-x31.
  assign rs1_valid = !is_lui && !is_jal && !is_auipc &&
    !(is_csrrwi || is_csrrsi || is_csrrci) && !is_error;
  assign rs2_valid = !is_lui && !is_jal && !is_auipc && !is_jalr && !is_load &&
    !is_math_immediate && !is_csr && !is_error;
  assign is_e_illegal = rd[4] || (rs1_valid && rs1[4]) || (rs2_valid && rs2[4]);

  assign is_valid = (is_lui ||
    is_auipc ||
    is_jal ||
    is_jalr ||
    is_branch ||
    is_load ||
    is_store ||
    is_math ||
    is_math_immediate ||
    is_ecall ||
    is_ebreak ||
    is_mret ||
    (is_csr && csr_implemented && !csr_readonly_write)) && !is_e_illegal;

  assign load_store_address = $signed(immediate) + $signed(`RF_RS1);
  assign addr24 = load_store_address[1:0];
  assign addr16 = load_store_address[1];
  assign addr8 = load_store_address[0];
  // A faulting store never runs its own case arm, so RVFI's fault-channel write mask
  // needs the intended strobe computed independently, here.
  assign store_wstrb = is_sw ? 4'b1111 :
                        is_sh ? (addr16 ? 4'b1100 : 4'b0011) :
                        is_sb ? (4'b0001 << addr24) : 4'b0000;

  assign pc_inc = uncompressed ? 4 : 2;

  // A plain load or store outside the RAM window faults with the address, decided
  // the same cycle decode would otherwise issue the transaction, since the bus has
  // no fault line. Alignment outranks the region, matching littlecpu.
  assign load_misaligned = (is_load_op || is_clwsp || is_clw) &&
    ((is_lw && |addr24) || ((is_lh || is_lhu) && addr8));
  assign store_misaligned = (is_store_op || is_cswsp || is_csw) &&
    ((is_sw && |addr24) || (is_sh && addr8));
  assign ls_in_range = load_store_address >= RAM_BASE &&
    load_store_address < RAM_BASE + RAM_WORDS * 4;
  assign load_region_fault = (is_load_op || is_clwsp || is_clw) &&
    !load_misaligned && !ls_in_range;
  assign store_region_fault = (is_store_op || is_cswsp || is_csw) &&
    !store_misaligned && !ls_in_range;

  assign take_trap = !is_valid || is_ecall || is_ebreak ||
    load_misaligned || store_misaligned || load_region_fault || store_region_fault;

  always_comb begin
    if (!is_valid) begin
      trap_cause_value = CAUSE_ILLEGAL_INSTRUCTION;
      trap_tval_value  = instr;
    end else if (is_ebreak) begin
      trap_cause_value = CAUSE_BREAKPOINT;
      trap_tval_value  = 32'b0;
    end else if (is_ecall) begin
      trap_cause_value = CAUSE_ECALL_M;
      trap_tval_value  = 32'b0;
    end else if (load_misaligned) begin
      trap_cause_value = CAUSE_LOAD_MISALIGNED;
      trap_tval_value  = load_store_address;
    end else if (store_misaligned) begin
      trap_cause_value = CAUSE_STORE_MISALIGNED;
      trap_tval_value  = load_store_address;
    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
      trap_tval_value  = load_store_address;
    end else if (store_region_fault) begin
      trap_cause_value = CAUSE_STORE_ACCESS_FAULT;
      trap_tval_value  = load_store_address;
    end else begin
      trap_cause_value = 32'b0;
      trap_tval_value  = 32'b0;
    end
  end

  // The 87 performance-monitor addresses, every one of them read-only zero.
  localparam logic [6:0] MHPMCOUNTER_WINDOW  = 7'h58; // 0xB00-0xB1F
  localparam logic [6:0] MHPMCOUNTERH_WINDOW = 7'h5C; // 0xB80-0xB9F
  localparam logic [6:0] MHPMEVENT_WINDOW    = 7'h19; // 0x320-0x33F
  assign hpm_number = csr_addr[4:0] > 5'd2;
  assign hpm_counter_window = csr_addr[11:5] == MHPMCOUNTER_WINDOW ||
                              csr_addr[11:5] == MHPMCOUNTERH_WINDOW;
  assign hpm_event_window = csr_addr[11:5] == MHPMEVENT_WINDOW;
  assign hpm_zero = hpm_number && (hpm_counter_window || hpm_event_window);

  logic [31:0] mstatus_value, mie_value, mip_value;
  // MPP is hardwired 2'b11: this core has no mode below machine.
  assign mstatus_value = {19'b0, 2'b11, 3'b0, mstatus_mpie, 3'b0, mstatus_mie, 3'b0};
  assign mie_value = {20'b0, mie_meie, 11'b0};
  assign mip_value = {20'b0, irq_meip_sync2, 11'b0};
  assign interrupt_pending = irq_meip_sync2 && mie_meie && mstatus_mie;

  // Sliced here, not inside the case below: a constant part-select of a wider signal
  // inside an always_comb/always_ff is an iverilog "sorry" (over-sensitive, not an
  // error), avoided by slicing in a continuous assign instead.
  logic [31:0] mcycle_lo, mcycle_hi, minstret_lo, minstret_hi;
  assign mcycle_lo   = mcycle[31:0];
  assign mcycle_hi   = mcycle[63:32];
  assign minstret_lo = minstret[31:0];
  assign minstret_hi = minstret[63:32];

  always_comb begin
    csr_implemented = 1'b1;
    (* parallel_case *)
    case (csr_addr)
      CSR_MSTATUS:   csr_rdata = mstatus_value;
      CSR_MSTATUSH:  csr_rdata = 32'b0;
      CSR_MISA:      csr_rdata = MISA_VALUE;
      CSR_MIE:       csr_rdata = mie_value;
      CSR_MTVEC:     csr_rdata = mtvec;
      CSR_MSCRATCH:  csr_rdata = mscratch;
      CSR_MEPC:      csr_rdata = mepc;
      CSR_MCAUSE:    csr_rdata = mcause;
      CSR_MTVAL:     csr_rdata = mtval;
      CSR_MIP:       csr_rdata = mip_value;
      CSR_MCYCLE:    csr_rdata = mcycle_lo;
      CSR_MCYCLEH:   csr_rdata = mcycle_hi;
      CSR_MINSTRET:  csr_rdata = minstret_lo;
      CSR_MINSTRETH: csr_rdata = minstret_hi;
      CSR_MHARTID:   csr_rdata = 32'b0;
      CSR_MVENDORID, CSR_MARCHID, CSR_MIMPID, CSR_MCONFIGPTR: csr_rdata = 32'b0;
      default: begin
        csr_rdata = 32'b0;
        csr_implemented = hpm_zero;
      end
    endcase
  end

  localparam cpu_trap = 4'b0000;
  localparam fetch_instr = 4'b0001;
  localparam ready_instr = 4'b0010;
  localparam decode_instr = 4'b0011;
  localparam execute_instr = 4'b0100;
  localparam finish_load = 4'b0101;
  localparam finish_store = 4'b0110;
  localparam check_pc = 4'b0111;
  localparam reg_write = 4'b1000;
`ifdef NANO_ONE_PORT_RF
  localparam fetch_rs1 = 4'b1100;
  localparam fetch_rs2 = 4'b1101;

  assign rf_raddr = cpu_state == fetch_rs2 ? rs2[3:0] : rs1[3:0];
`endif

  // Nano completes one instruction fully before returning here to redirect.
  assign take_interrupt = interrupt_pending && cpu_state == fetch_instr;

  always_ff @(posedge clk) begin
    if (reset) begin
      pc <= 0;
      instr <= 0;
      next_pc <= 0;
      mem_addr <= 0;
      mem_wdata <= 0;
      mem_wstrb <= 0;
      trap <= 0;
      cpu_state <= fetch_instr;
      mem_valid <= 0;
    end else begin
      (* parallel_case, full_case *)
      case (cpu_state)
        fetch_instr: begin
          skip_reg_write <= 0;
`ifndef NANO_LATCH_RF
          regs[0] <= 0;
`endif
          if (take_interrupt) begin
            // Nothing issues this cycle: next_pc becomes mtvec, and mstatus_mie
            // reads already cleared next cycle, so the fetch below runs then.
            next_pc <= mtvec;
          end else begin
            mem_wstrb <= 4'b0000;
            mem_instr <= 1;
            mem_valid <= 1;
            cpu_state <= ready_instr;
            mem_addr <= next_pc;
          end
        end

        ready_instr: begin
          if (mem_ready) begin
            mem_valid <= 0;
            pc <= mem_addr;
            instr <= mem_rdata[1:0] == 2'b11 ? mem_rdata : {16'b0, mem_rdata[15:0]};
            cpu_state <= decode_instr;
          end
        end

        decode_instr: begin
          (* parallel_case, full_case *)
          case (1'b1)
            is_branch || is_store || is_cj || is_cjr: rd <= 0;
            is_cjal || is_cjalr: rd <= 1;
            is_clw || is_caddi4spn: rd <= {2'b01, instr[4:2]};
            is_csrai || is_csrli || is_candi || is_cand ||
              is_cor || is_cxor || is_csub: rd <= {2'b01, instr[9:7]};
            default: rd <= instr[11:7];
          endcase

          (* parallel_case, full_case *)
          case (1'b1)
            is_clwsp || is_cswsp || is_caddi4spn: rs1 <= 2;
            is_clw || is_csw || is_cbeqz || is_cbnez ||
              is_csrai || is_csrli || is_candi || is_cand ||
              is_cor || is_cxor || is_csub: rs1 <= {2'b01, instr[9:7]};
            is_cjr || is_cjalr || is_cslli: rs1 <= instr[11:7];
            is_cli || is_cmv: rs1 <= 0;
            is_caddi || is_caddi16sp || is_cadd: rs1 <= instr[11:7];
            default: rs1 <= instr[19:15];
          endcase

          (* parallel_case, full_case *)
          case(1'b1)
            is_cswsp || is_cslli || is_csrai || is_csrli || is_cmv || is_cadd: rs2 <= instr[6:2];
            is_csw || is_cand || is_cor || is_cxor || is_csub: rs2 <= {2'b01, instr[4:2]};
            is_cbeqz || is_cbnez: rs2 <= 0;
            default: rs2 <= instr[24:20];
          endcase
`ifdef NANO_ONE_PORT_RF
          cpu_state <= fetch_rs1;
`else
          cpu_state <= execute_instr;
`endif
        end

`ifdef NANO_ONE_PORT_RF
        fetch_rs1: begin
          op_rs1 <= regs[rf_raddr];
          cpu_state <= rs2_valid ? fetch_rs2 : execute_instr;
        end

        fetch_rs2: begin
          op_rs2 <= regs[rf_raddr];
          cpu_state <= execute_instr;
        end
`endif

        execute_instr: begin
          if (take_trap) begin
            skip_reg_write <= 1;
            next_pc <= mtvec;
            cpu_state <= fetch_instr;
          end else begin
            (* parallel_case, full_case *)
            case (1'b1)
              is_lui: begin
                reg_wdata <= immediate;
                cpu_state <= reg_write;
                next_pc <= pc + pc_inc;
              end

              is_auipc: begin
                reg_wdata <= immediate + pc;
                cpu_state <= reg_write;
                next_pc <= pc + 4;
              end

              is_jal || is_jalr: begin
                pc_wdata <= jump_address;
                reg_wdata <= pc + pc_inc;
                skip_reg_write <= 0;
                cpu_state <= check_pc;
              end

              is_branch: begin
                (* parallel_case, full_case *)
                case(1'b1)
                  is_beq: pc_wdata <= `RF_RS1 == `RF_RS2 ? pc + immediate : pc + pc_inc;
                  is_bne: pc_wdata <= `RF_RS1 != `RF_RS2 ? pc + immediate : pc + pc_inc;
                  is_blt: pc_wdata <= $signed(`RF_RS1) < $signed(`RF_RS2) ? pc + immediate : pc + 4;
                  is_bltu: pc_wdata <= `RF_RS1 < `RF_RS2 ? pc + immediate : pc + 4;
                  is_bge: pc_wdata <= $signed(`RF_RS1) >= $signed(`RF_RS2) ? pc + immediate : pc + 4;
                  is_bgeu: pc_wdata <= `RF_RS1 >= `RF_RS2 ? pc + immediate : pc + 4;
                endcase
                skip_reg_write <= 1;
                cpu_state <= check_pc;
              end

              is_math || is_math_immediate: begin
                cpu_state <= reg_write;
                next_pc <= pc + pc_inc;
                (* parallel_case, full_case *)
                case(1'b1)
                  is_add || is_addi: begin
                    reg_wdata <= `RF_RS1 + math_arg;
                  end

                  is_sub: begin
                    reg_wdata <= `RF_RS1 - math_arg;
                  end

                  is_sll || is_slli: begin
                    reg_wdata <= `RF_RS1 << shamt;
                  end

                  is_slt || is_slti: begin
                    reg_wdata <= {31'b0, $signed(`RF_RS1) < $signed(math_arg)};
                  end

                  is_sltu || is_sltiu: begin
                    reg_wdata <= {31'b0, `RF_RS1 < math_arg};
                  end

                  is_xor || is_xori: begin
                    reg_wdata <= `RF_RS1 ^ math_arg;
                  end

                  is_srl || is_srli: begin
                    reg_wdata <= `RF_RS1 >> shamt;
                  end

                  is_sra || is_srai: begin
                    reg_wdata <= $signed(`RF_RS1) >>> shamt;
                  end

                  is_or || is_ori: begin
                    reg_wdata <= `RF_RS1 | math_arg;
                  end

                  is_and || is_andi: begin
                    reg_wdata <= `RF_RS1 & math_arg;
                  end
                endcase
              end

              is_load_op || is_clwsp || is_clw: begin
                // Misaligned and out-of-window accesses are excluded by take_trap above.
                mem_wstrb <= 4'b0000;
                mem_addr <= {load_store_address[31:2], 2'b00};
                mem_instr <= 0; // can we have data
                mem_valid <= 1; // kick off a memory request
                cpu_state <= finish_load;
              end

              is_store_op || is_cswsp || is_csw: begin
                (* parallel_case, full_case *)
                case (1'b1)
                  is_sw: begin
                    mem_addr <= load_store_address;
                    mem_wstrb <= 4'b1111;
                    mem_wdata <= `RF_RS2;
                  end

                  is_sh: begin
                    mem_wstrb <= addr16 ? 4'b1100 : 4'b0011;
                    mem_wdata <= {2{`RF_RS2[15:0]}};
                  end

                  is_sb: begin
                    mem_wstrb <= 4'b0001 << addr24;
                    mem_wdata <= {4{`RF_RS2[7:0]}};
                  end
                endcase
                mem_addr <= {load_store_address[31:2], 2'b00};
                mem_instr <= 0;
                mem_valid <= 1; // kick off a memory request
                cpu_state <= finish_store;
              end

              is_csr: begin
                reg_wdata <= csr_rdata;
                cpu_state <= reg_write;
                next_pc   <= pc + pc_inc;
              end

              is_mret: begin
                skip_reg_write <= 1;
                next_pc <= mepc;
                cpu_state <= fetch_instr;
              end

              default: begin
                cpu_state <= cpu_trap;
              end
            endcase
          end
        end

        check_pc: begin
          if (pc_wdata[0]) begin
            cpu_state <= cpu_trap;
          end else begin
            next_pc <= pc_wdata;
            cpu_state <= skip_reg_write ? fetch_instr : reg_write;
          end
        end

        reg_write: begin
`ifndef NANO_LATCH_RF
          regs[rd[3:0]] <= reg_wdata;
`endif
          cpu_state <= fetch_instr;
        end

        finish_load: begin
          if (mem_ready) begin
            (* parallel_case, full_case *)
            case (1'b1)
              is_lb: begin
                case (addr24)
                  2'b00: reg_wdata <= {{24{mem_rdata[7]}}, mem_rdata[7:0]};
                  2'b01: reg_wdata <= {{24{mem_rdata[15]}}, mem_rdata[15:8]};
                  2'b10: reg_wdata <= {{24{mem_rdata[23]}}, mem_rdata[23:16]};
                  2'b11: reg_wdata <= {{24{mem_rdata[31]}}, mem_rdata[31:24]};
                endcase
              end

              is_lbu: begin
                case (addr24)
                  2'b00: reg_wdata <= {24'b0, mem_rdata[7:0]};
                  2'b01: reg_wdata <= {24'b0, mem_rdata[15:8]};
                  2'b10: reg_wdata <= {24'b0, mem_rdata[23:16]};
                  2'b11: reg_wdata <= {24'b0, mem_rdata[31:24]};
                endcase
              end

              is_lh: begin
                case (addr16)
                  1'b0: reg_wdata <= {{16{mem_rdata[15]}}, mem_rdata[15:0]};
                  1'b1: reg_wdata <= {{16{mem_rdata[31]}}, mem_rdata[31:16]};
                endcase
              end

              is_lhu: begin
                case (addr16)
                  1'b0: reg_wdata <= {16'b0, mem_rdata[15:0]};
                  1'b1: reg_wdata <= {16'b0, mem_rdata[31:16]};
                endcase
              end

              is_lw: reg_wdata <= mem_rdata;
            endcase
            cpu_state <= reg_write;
            mem_valid <= 0;
            next_pc <= pc + pc_inc;
          end
        end

        finish_store: begin
          if (mem_ready) begin
            cpu_state <= fetch_instr;
            mem_valid <= 0;
            next_pc <= pc + pc_inc;
          end
        end

        cpu_trap: begin
          trap <= 1;
        end
      endcase
    end
  end

  // !take_trap excludes an E-illegal CSR instruction, which is_valid alone does not.
  assign csr_wen = is_csr && csr_write_op && cpu_state == execute_instr && !take_trap;
  assign wr_mcycle    = csr_wen && csr_addr == CSR_MCYCLE;
  assign wr_mcycleh   = csr_wen && csr_addr == CSR_MCYCLEH;
  assign wr_minstret  = csr_wen && csr_addr == CSR_MINSTRET;
  assign wr_minstreth = csr_wen && csr_addr == CSR_MINSTRETH;
  assign instret = cpu_state == execute_instr && !take_trap;

  always_ff @(posedge clk) begin
    if (reset) begin
      mcycle         <= 64'b0;
      minstret       <= 64'b0;
      mscratch       <= 32'b0;
      mtvec          <= 32'b0;
      mepc           <= 32'b0;
      mcause         <= 32'b0;
      mtval          <= 32'b0;
      mstatus_mie    <= 1'b0;
      mstatus_mpie   <= 1'b0;
      mie_meie       <= 1'b0;
      irq_meip_sync1 <= 1'b0;
      irq_meip_sync2 <= 1'b0;
    end else begin
      irq_meip_sync1 <= irq_meip;
      irq_meip_sync2 <= irq_meip_sync1;

      mcycle   <= wr_mcycle   ? {mcycle_hi, csr_new_value} :
                 wr_mcycleh   ? {csr_new_value, mcycle_lo}  :
                                mcycle + 64'd1;
      minstret <= wr_minstret  ? {minstret_hi, csr_new_value} :
                 wr_minstreth  ? {csr_new_value, minstret_lo}  :
                 instret       ? minstret + 64'd1 : minstret;

      if (csr_wen) begin
        (* parallel_case *)
        case (csr_addr)
          CSR_MSTATUS: begin
            mstatus_mie  <= csr_new_value[3];
            mstatus_mpie <= csr_new_value[7];
          end
          CSR_MIE:      mie_meie <= csr_new_value[11];
          CSR_MTVEC:    mtvec    <= {csr_new_value[31:2], 2'b00};
          CSR_MSCRATCH: mscratch <= csr_new_value;
          CSR_MEPC:     mepc     <= {csr_new_value[31:1], 1'b0};
          CSR_MCAUSE:   mcause   <= csr_new_value;
          CSR_MTVAL:    mtval    <= csr_new_value;
          default: ;
        endcase
      end else if (take_interrupt) begin
        mepc         <= next_pc;
        mcause       <= CAUSE_MACHINE_EXTERNAL;
        mtval        <= 32'b0;
        mstatus_mpie <= mstatus_mie;
        mstatus_mie  <= 1'b0;
      end else if (cpu_state == execute_instr && take_trap) begin
        mepc         <= pc;
        mcause       <= trap_cause_value;
        mtval        <= trap_tval_value;
        mstatus_mpie <= mstatus_mie;
        mstatus_mie  <= 1'b0;
      end else if (cpu_state == execute_instr && is_mret) begin
        mstatus_mie  <= mstatus_mpie;
        mstatus_mpie <= 1'b1;
      end
    end
  end

`ifdef NANO_LATCH_RF
  // Latches open only while clk is LOW, on a select/data pair captured a period earlier.
  logic [15:0] we, we_q;
  logic [31:0] wdata_q;

  assign we[0] = cpu_state == fetch_instr;
  genvar gw;
  generate
    for (gw = 1; gw < 16; gw = gw + 1) begin : g_we
      assign we[gw] = cpu_state == reg_write && rd[3:0] == gw[3:0];
    end
  endgenerate

  always_ff @(posedge clk) begin
    we_q <= we;
    wdata_q <= reg_wdata;
  end

  genvar gi;
  generate
    for (gi = 0; gi < 16; gi = gi + 1) begin : g_regs_latch
      logic sel_q;
      assign sel_q = we_q[gi];
      if (gi == 0) begin : g_zero
        always_latch if (!clk && sel_q) regs[0] = 32'b0;
      end else begin : g_write
        always_latch if (!clk && sel_q) regs[gi] = wdata_q;
      end
    end
  endgenerate
`endif

 `ifdef RISCV_FORMAL
  assign rvfi_dbg_mtvec   = mtvec;
  assign rvfi_dbg_mepc    = mepc;
  assign rvfi_dbg_mcause  = mcause;
  assign rvfi_dbg_mtval   = mtval;
  assign rvfi_dbg_mstatus = mstatus_value;

  // prev_cpu_state tells a genuine arrival in fetch_instr apart from a cycle merely
  // spent dwelling here doing interrupt-entry bookkeeping.
  logic [3:0] prev_cpu_state;
  always_ff @(posedge clk)
    prev_cpu_state <= reset ? fetch_instr : cpu_state;
  logic is_fetch, is_fetch_entry;
  assign is_fetch = cpu_state == fetch_instr;
  assign is_fetch_entry = is_fetch && prev_cpu_state != fetch_instr;

  // Held from execute_instr, not re-read live: a load/store whose rd aliases its rs1
  // moves regs[rs1] (and so load_store_address/take_trap) before its own retirement.
  logic captured_is_valid, captured_take_trap, captured_is_opm, captured_load_fault,
        captured_store_fault;
  logic [3:0]  captured_store_wstrb;
  logic [31:0] captured_ls_addr;
  always_ff @(posedge clk) begin
    if (cpu_state == execute_instr) begin
      captured_is_valid    <= is_valid;
      captured_take_trap   <= take_trap;
      captured_is_opm      <= is_opm_encoding;
      captured_load_fault  <= load_region_fault;
      captured_store_fault <= store_region_fault;
      captured_store_wstrb <= store_wstrb;
      // Word-aligned, matching both the non-faulting mem_addr and RISCV_FORMAL_ALIGNED_MEM's spec model.
      captured_ls_addr     <= {load_store_address[31:2], 2'b00};
    end
  end

  // Set the cycle an interrupt redirects next_pc, cleared at the handler's first
  // retirement -- the two can be cycles apart if a load/store was in flight.
  logic pending_rvfi_intr;
  always_ff @(posedge clk) begin
    if (reset) pending_rvfi_intr <= 1'b0;
    else if (take_interrupt) pending_rvfi_intr <= 1'b1;
    else if (is_fetch_entry) pending_rvfi_intr <= 1'b0;
  end

  // `RVFI_OUTPUTS types these `wire` under iverilog; each gets a same-width shadow.
  `define RVFI_SHADOW(name) \
    logic [$bits(name)-1:0] name``_q; \
    assign name = name``_q;
  `RVFI_SHADOW(rvfi_valid)
  `RVFI_SHADOW(rvfi_order)
  `RVFI_SHADOW(rvfi_insn)
  `RVFI_SHADOW(rvfi_trap)
  `RVFI_SHADOW(rvfi_halt)
  `RVFI_SHADOW(rvfi_intr)
  `RVFI_SHADOW(rvfi_mode)
  `RVFI_SHADOW(rvfi_ixl)
  `RVFI_SHADOW(rvfi_rs1_addr)
  `RVFI_SHADOW(rvfi_rs2_addr)
  `RVFI_SHADOW(rvfi_rs1_rdata)
  `RVFI_SHADOW(rvfi_rs2_rdata)
  `RVFI_SHADOW(rvfi_rd_addr)
  `RVFI_SHADOW(rvfi_rd_wdata)
  `RVFI_SHADOW(rvfi_pc_rdata)
  `RVFI_SHADOW(rvfi_pc_wdata)
  `RVFI_SHADOW(rvfi_mem_addr)
  `RVFI_SHADOW(rvfi_mem_rmask)
  `RVFI_SHADOW(rvfi_mem_wmask)
  `RVFI_SHADOW(rvfi_mem_rdata)
  `RVFI_SHADOW(rvfi_mem_wdata)
`ifdef RISCV_FORMAL_MEM_FAULT
  `RVFI_SHADOW(rvfi_mem_fault)
  `RVFI_SHADOW(rvfi_mem_fault_rmask)
  `RVFI_SHADOW(rvfi_mem_fault_wmask)
`endif
  `undef RVFI_SHADOW

`ifdef RISCV_FORMAL_CSR_MCYCLE
  logic [63:0] rvfi_csr_mcycle_rmask_q, rvfi_csr_mcycle_wmask_q, rvfi_csr_mcycle_rdata_q,
               rvfi_csr_mcycle_wdata_q;
  assign rvfi_csr_mcycle_rmask = rvfi_csr_mcycle_rmask_q;
  assign rvfi_csr_mcycle_wmask = rvfi_csr_mcycle_wmask_q;
  assign rvfi_csr_mcycle_rdata = rvfi_csr_mcycle_rdata_q;
  assign rvfi_csr_mcycle_wdata = rvfi_csr_mcycle_wdata_q;
`endif
`ifdef RISCV_FORMAL_CSR_MINSTRET
  logic [63:0] rvfi_csr_minstret_rmask_q, rvfi_csr_minstret_wmask_q, rvfi_csr_minstret_rdata_q,
               rvfi_csr_minstret_wdata_q;
  assign rvfi_csr_minstret_rmask = rvfi_csr_minstret_rmask_q;
  assign rvfi_csr_minstret_wmask = rvfi_csr_minstret_wmask_q;
  assign rvfi_csr_minstret_rdata = rvfi_csr_minstret_rdata_q;
  assign rvfi_csr_minstret_wdata = rvfi_csr_minstret_wdata_q;
`endif

  always_ff @(posedge clk) begin
    // is_fetch_entry, not is_fetch: this must fire once per retirement, not once
    // per cycle dwelled in fetch_instr doing interrupt-entry bookkeeping.
    rvfi_valid_q <= !reset &&
      ((is_fetch_entry && (captured_is_valid || captured_take_trap)) || trap);

    if (cpu_state == execute_instr) begin
      rvfi_rs1_rdata_q <= rs1_valid ? `RF_RS1 : 0;
      rvfi_rs2_rdata_q <= rs2_valid ? `RF_RS2 : 0;
    end

    rvfi_rs1_addr_q <= rs1_valid ? rs1 : 0;
    rvfi_rs2_addr_q <= rs2_valid ? rs2 : 0;
    rvfi_insn_q <= instr;

    // RVFI requires a trapping retirement to report no destination register.
    rvfi_rd_addr_q <= (is_fetch_entry && captured_take_trap) ? 5'b0 : rd;
`ifdef NANO_LATCH_RF
    // A retiring write's latch has not opened yet; reg_wdata already holds the value.
    rvfi_rd_wdata_q <=
      (is_fetch_entry && captured_take_trap) ? 32'b0 : (|rd ? reg_wdata : 0);
`else
    rvfi_rd_wdata_q <=
      (is_fetch_entry && captured_take_trap) ? 32'b0 : (|rd ? regs[rd[3:0]] : 0);
`endif
    rvfi_trap_q <= (is_fetch_entry && captured_take_trap) || trap;
    rvfi_halt_q <= trap;
`ifdef RISCV_FORMAL_MEM_FAULT
    rvfi_mem_fault_q       <= is_fetch_entry &&
      (captured_is_opm || captured_load_fault || captured_store_fault);
    rvfi_mem_fault_rmask_q <= (is_fetch_entry && captured_load_fault) ? 4'b1111 : 4'b0;
    rvfi_mem_fault_wmask_q <=
      (is_fetch_entry && captured_store_fault) ? captured_store_wstrb : 4'b0;
`endif
    rvfi_pc_rdata_q <= pc;
    rvfi_pc_wdata_q <= next_pc;
    rvfi_mode_q <= 3;
    rvfi_ixl_q <= 1;
    rvfi_intr_q <= is_fetch_entry && pending_rvfi_intr;
    rvfi_order_q <= !reset ? rvfi_order_q + rvfi_valid_q : 0;

`ifdef RISCV_FORMAL_CSR_MCYCLE
    // Held, not sampled every cycle: mcycle keeps ticking between this instruction's
    // execute_instr and the fetch_instr that reports it.
    if (cpu_state == execute_instr) begin
      rvfi_csr_mcycle_rmask_q <= {64{is_csr && (csr_addr == CSR_MCYCLE || csr_addr == CSR_MCYCLEH)}};
      rvfi_csr_mcycle_wmask_q <= {{32{wr_mcycleh}}, {32{wr_mcycle}}};
      rvfi_csr_mcycle_rdata_q <= mcycle;
      rvfi_csr_mcycle_wdata_q <= {wr_mcycleh ? csr_new_value : mcycle_hi,
                                   wr_mcycle  ? csr_new_value : mcycle_lo};
    end
`endif
`ifdef RISCV_FORMAL_CSR_MINSTRET
    if (cpu_state == execute_instr) begin
      rvfi_csr_minstret_rmask_q <= {64{is_csr &&
        (csr_addr == CSR_MINSTRET || csr_addr == CSR_MINSTRETH)}};
      rvfi_csr_minstret_wmask_q <= {{32{wr_minstreth}}, {32{wr_minstret}}};
      rvfi_csr_minstret_rdata_q <= minstret;
      rvfi_csr_minstret_wdata_q <= {wr_minstreth ? csr_new_value : minstret_hi,
                                     wr_minstret  ? csr_new_value : minstret_lo};
    end
`endif

    if (mem_instr) begin
      rvfi_mem_addr_q <= 0;
      rvfi_mem_wmask_q <= 0;
      rvfi_mem_rmask_q <= 0;
      rvfi_mem_rdata_q <= 0;
      rvfi_mem_wdata_q <= 0;
    end else if (mem_valid && mem_ready) begin
      rvfi_mem_addr_q <= mem_addr;
      rvfi_mem_wmask_q <= mem_wstrb;
      rvfi_mem_rmask_q <= |mem_wstrb ? 0 : ~0;
      rvfi_mem_rdata_q <= mem_rdata;
      rvfi_mem_wdata_q <= mem_wdata;
    end
    // A faulting load/store never reaches the case arm that sets mem_addr, so its
    // address is reported here instead (the generic spec model has no separate
    // fault-address field, so this overrides the ordinary rvfi_mem_addr too).
`ifdef RISCV_FORMAL_MEM_FAULT
    if (is_fetch_entry && (captured_load_fault || captured_store_fault))
      rvfi_mem_addr_q <= captured_ls_addr;
`endif
  end
 `endif
endmodule
