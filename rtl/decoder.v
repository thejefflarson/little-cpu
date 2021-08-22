`default_nettype none
module decoder (
  input  var logic        clk,
  input  var logic        reset,
  // handshake
  input  var logic        fetcher_valid,
  output var logic        decoder_ready,
  output var logic        decoder_valid,
  input  var logic        executor_ready,
  // inputs
  input  fetcher_output   in,
  input  var logic [31:0] reg_rs1,
  input  var logic [31:0] reg_rs2,
  // forwards
  output var logic [4:0]  decoder_rd,
  output var logic [31:0] decoder_reg_rs1,
  output var logic [31:0] decoder_reg_rs2,
  // outputs
  output var logic [31:0] pc,
  // rs1 and rs2 are synchronous outputs
  output var logic [4:0]  rs1,
  output var logic [4:0]  rs2,
  // asynchrous
  output var logic [31:0] decoder_mem_addr,
  output var logic        is_valid_instr,
  output var logic        is_add,
  output var logic        is_sub,
  output var logic        is_xor,
  output var logic        is_or,
  output var logic        is_and,
  output var logic        is_mul,
  output var logic        is_mulh,
  output var logic        is_mulhu,
  output var logic        is_mulhsu,
  output var logic        is_div,
  output var logic        is_divu,
  output var logic        is_rem,
  output var logic        is_remu,
  output var logic        is_sll,
  output var logic        is_slt,
  output var logic        is_sltu,
  output var logic        is_srl,
  output var logic        is_sra,
  output var logic        is_lui,
  output var logic        is_lb,
  output var logic        is_lbu,
  output var logic        is_lhu,
  output var logic        is_lh,
  output var logic        is_lw,
  output var logic        is_sb,
  output var logic        is_sh,
  output var logic        is_sw,
  output var logic        is_ecall,
  output var logic        is_ebreak,
  output var logic        is_csrrw,
  output var logic        is_csrrs,
  output var logic        is_csrrc
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
    (* parallel_case, full_case *)
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
  logic instr_lui_op, instr_jal_op, instr_jalr_op, instr_cj, instr_cjal, instr_cjr, instr_cjalr,
    instr_clui;
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

  logic instr_branch_op, instr_cbeqz, instr_cbnez;
  assign instr_branch_op = opcode == 5'b11000 && uncompressed;
  assign instr_beq = (instr_branch_op && funct3 == 3'b000) || instr_cbeqz;
  assign instr_bne = (instr_branch_op && funct3 == 3'b001) || instr_cbnez;
  assign instr_blt = instr_branch_op && funct3 == 3'b100;
  assign instr_bge = instr_branch_op && funct3 == 3'b101;
  assign instr_bltu = instr_branch_op && funct3 == 3'b110;
  assign instr_bgeu = instr_branch_op && funct3 == 3'b111;
  assign instr_cbeqz = quadrant == 2'b01 && cfunct3 == 3'b110;
  assign instr_cbnez = quadrant == 2'b01 && cfunct3 == 3'b111;

  logic instr_load_op, instr_clwsp, instr_clw;
  assign instr_load_op = opcode == 5'b00000 && uncompressed;
  assign instr_lb = instr_load_op && funct3 == 3'b000;
  assign instr_lh = instr_load_op && funct3 == 3'b001;
  assign instr_lw = (instr_load_op && funct3 == 3'b010) || instr_clwsp || instr_clw;
  assign instr_lbu = instr_load_op && funct3 == 3'b100;
  assign instr_lhu = instr_load_op && funct3 == 3'b101;
  assign instr_clwsp = quadrant == 2'b10 && cfunct3 == 3'b010 && instr[11:7] != 5'b0;
  assign instr_clw = quadrant == 2'b00 && cfunct3 == 3'b010;

  logic instr_store_op, instr_cswsp, instr_csw;
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
  logic instr_math_immediate, instr_math_immediate_op, instr_cli, instr_caddi, instr_caddi16sp,
    instr_caddi4spn, instr_cslli, instr_csrli, instr_csrai, instr_candi, instr_addi, instr_slti,
    instr_sltiu, instr_xori, instr_ori, instr_andi, instr_slli, instr_srli, instr_srai;
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
  assign instr_ecall = instr_error && !{|instr[31:20]};
  assign instr_ebreak = instr_error && |instr[31:20];
  logic instr_valid;

  assign instr_valid = instr_auipc || instr_jal || instr_jalr || instr_beq || instr_bne || instr_blt
    || instr_bltu || instr_bge || instr_bgeu || instr_add || instr_sub || instr_xor || instr_or ||
    instr_and || instr_mul || instr_mulh || instr_mulhu || instr_mulhsu || instr_div || instr_divu
    || instr_rem || instr_remu || instr_sll || instr_slt || instr_sltu || instr_srl || instr_sra ||
    instr_lui || instr_lb || instr_lbu || instr_lh || instr_lhu || instr_lw || instr_sb || instr_sh
    || instr_sw || instr_ecall || instr_ebreak;

  logic [4:0] rd;
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
    endcase

    (* parallel_case, full_case *)
    case(1'b1)
      instr_cswsp || instr_cslli || instr_csrai || instr_csrli || instr_cmv || instr_cadd: rs2 = instr[6:2];
      instr_csw || instr_cand || instr_cor || instr_cxor || instr_csub: rs2 = {2'b01, instr[4:2]};
      instr_cbeqz || instr_cbnez: rs2 = 0;
      default: rs2 = instr[24:20];
    endcase
  end // always_comb

  // ALU handling
  logic instr_math, instr_shift;
  assign instr_math = instr_add || instr_sub || instr_sll || instr_slt || instr_sltu || instr_xor || instr_srl ||
    instr_sra || instr_or || instr_and || instr_mul || instr_mulh || instr_mulhu || instr_mulhsu || instr_div ||
    instr_divu || instr_rem || instr_remu;
  assign instr_shift = instr_sll || instr_slt || instr_sltu || instr_xor || instr_srl || instr_sra;

  logic [31:0] math_arg;
  always_comb begin
    if (instr_math_immediate) begin
      math_arg = instr_shift ? {27'b0, rs2} : immediate;
    end else begin
      math_arg = reg_rs2;
    end
  end

  // handshake
  always_ff @(posedge clk) begin
    if (reset) begin
      decoder_ready <= 1;
      decoder_valid <= 0;
    end else begin
      decoder_ready <= 1;
      decoder_valid <= 0;
      if (!executor_ready || !fetcher_valid) begin
        decoder_ready <= 0;
      end else if(fetcher_valid && executor_ready) begin
        decoder_valid <= 1;
      end
    end
  end

  logic [31:0] pc_inc;
  assign pc_inc = uncompressed ? 4 : 2;
  // publish the decoded results
  always_ff @(posedge clk) begin
    if (reset) begin
      // zero out the pc
      pc <= 0;
    end else if (fetcher_valid && executor_ready) begin
      // update pc
      pc <= fetcher_pc + pc_inc;
      decoder_mem_addr <= $signed(immediate) + $signed(reg_rs1);
      // forwards
      decoder_reg_rs1 <= reg_rs1;
      decoder_reg_rs2 <= instr_math ? math_arg : reg_rs2;
      decoder_rd <= rd;
      // outputs
      is_add <= instr_add;
      is_sub <= instr_sub;
      is_xor <= instr_xor;
      is_or <= instr_or;
      is_and <= instr_and;
      is_mul <= instr_mul;
      is_mulh <= instr_mulh;
      is_mulhu <= instr_mulhu;
      is_mulhsu <= instr_mulhsu;
      is_div <= instr_div;
      is_divu <= instr_divu;
      is_rem <= instr_rem;
      is_remu <= instr_remu;
      is_sll <= instr_sll;
      is_slt <= instr_slt;
      is_sltu <= instr_sltu;
      is_srl <= instr_srl;
      is_sra <= instr_sra;
      is_lui <= instr_lui;
      is_lb <= instr_lb;
      is_lbu <= instr_lbu;
      is_lhu <= instr_lhu;
      is_lh <= instr_lh;
      is_lw <= instr_lw;
      is_sb <= instr_sb;
      is_sh <= instr_sh;
      is_sw <= instr_sw;
      is_ecall <= instr_ecall;
      is_ebreak <= instr_ebreak;
      is_csrrw <= instr_csrrw;
      is_csrrs <= instr_csrrs;
      is_csrrc <= instr_csrrc;
      is_valid_instr <= instr_valid;
      // calculate branch
      (* parallel_case *)
      case(1'b1)
        instr_auipc: begin
          decoder_rd <= rd;
          decoder_reg_rs1 <= reg_rs1;
          decoder_reg_rs2 <= reg_rs2;
          is_add <= 1;
        end

        instr_jal || instr_jalr: begin
          pc <= instr_jalr ?
            ($signed(immediate) + $signed(reg_rs1)) & 32'hfffffffe :
            $signed(pc) + $signed(immediate);
          decoder_reg_rs1 <= pc;
          decoder_reg_rs2 <= pc_inc;
          decoder_rd <= rd;
          is_add <= 1;
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
          decoder_reg_rs1 <= 0;
          decoder_reg_rs2 <= 0;
          decoder_rd <= 0;
        end
      endcase
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // assume we've reset at clk 0
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  // if we've been valid but stalled, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(decoder_valid) && $past(decoder_valid && !executor_ready)) assert(!decoder_valid);

  // if we've been valid but the next stage is busy, we're not valid anymore
  always_ff @(posedge clk) if(clocked && $past(decoder_valid) && $past(!executor_ready)) assert(!decoder_valid);

  logic one_of;
  assign one_of = instr_auipc ^ instr_jal ^ instr_jalr ^ instr_beq ^ instr_bne ^ instr_blt ^
    instr_bltu ^ instr_bge ^ instr_bgeu ^ instr_add ^ instr_sub ^ instr_xor ^ instr_or ^ instr_and ^
    instr_mul ^ instr_mulh ^ instr_mulhu ^ instr_mulhsu ^ instr_div ^ instr_divu ^ instr_rem ^
    instr_remu ^ instr_sll ^ instr_slt ^ instr_sltu ^ instr_srl ^ instr_sra ^ instr_lui ^ instr_lb ^
    instr_lbu ^ instr_lh ^ instr_lhu ^ instr_lw ^ instr_sb ^ instr_sh ^ instr_sw ^ instr_ecall ^
    instr_ebreak;

  // we should only get one type of instruction
  always_comb if (instr_valid) assert(one_of);
 `endif
endmodule
