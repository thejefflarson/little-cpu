module decoder (
  input logic         clk,
  input logic [31:0]  instr,
  input logic [31:0]  pc,
  input logic         decode,
  output logic        decoded,
  output logic [4:0]  rd,
  output logic [4:0]  rs1,
  output logic [4:0]  rs2,
  output logic [31:0] immediate,
  output logic        is_valid,
  output logic        uncompressed,
  output logic        is_auipc,
  output logic        is_jal,
  output logic        is_jalr,
  output logic        is_beq,
  output logic        is_bne,
  output logic        is_blt,
  output logic        is_bltu,
  output logic        is_bge,
  output logic        is_bgeu,
  output logic        is_add,
  output logic        is_addi,
  output logic        is_sub,
  output logic        is_xor,
  output logic        is_xori,
  output logic        is_or,
  output logic        is_ori,
  output logic        is_and,
  output logic        is_andi,
  output logic        is_mul,
  output logic        is_div,
  output logic        is_divu,
  output logic        is_sll,
  output logic        is_slli,
  output logic        is_slt,
  output logic        is_slti,
  output logic        is_sltu,
  output logic        is_sltiu,
  output logic        is_srl,
  output logic        is_srli,
  output logic        is_sra,
  output logic        is_srai,
  output logic        is_lui,
  output logic        is_lb,
  output logic        is_lh,
  output logic        is_lw,
  output logic        is_sb,
  output logic        is_sh,
  output logic        is_sw
  );
  // instruction decoder (figure 2.3)
  logic [4:0] opcode;
  assign opcode = instr[6:2];
  logic [1:0] quadrant, cfunct2, cmath_funct2;
  assign quadrant = instr[1:0];
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

  // immediate decoder (figure 2.4 & table 16.1)
  logic [31:0] i_immediate, s_immediate, b_immediate, u_immediate, j_immediate;
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
  logic is_lui_op, is_jal_op, is_jalr_op, is_cj, is_cjal, is_cjr,
    is_cjalr, is_clui;
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

  logic is_branch_op;
  assign is_branch_op = opcode == 5'b11000 && uncompressed;
  assign is_beq = (is_branch_op && funct3 == 3'b000) || is_cbeqz;
  assign is_bne = (is_branch_op && funct3 == 3'b001) || is_cbnez;
  assign is_blt = is_branch_op && funct3 == 3'b100;
  assign is_bge = is_branch_op && funct3 == 3'b101;
  assign is_bltu = is_branch_op && funct3 == 3'b110;
  assign is_bgeu = is_branch_op && funct3 == 3'b111;
  assign is_cbeqz = quadrant == 2'b01 && cfunct3 == 3'b110;
  assign is_cbnez = quadrant == 2'b01 && cfunct3 == 3'b111;

  logic is_load_op;
  assign is_load_op = opcode == 5'b00000 && uncompressed;
  assign is_lb = is_load_op && funct3 == 3'b000;
  assign is_lh = is_load_op && funct3 == 3'b001;
  assign is_lw = (is_load_op && funct3 == 3'b010) || is_clwsp || is_clw;
  assign is_lbu = is_load_op && funct3 == 3'b100;
  assign is_lhu = is_load_op && funct3 == 3'b101;
  assign is_clwsp = quadrant == 2'b10 && cfunct3 == 3'b010 && instr[11:7] != 5'b0;
  assign is_clw = quadrant == 2'b00 && cfunct3 == 3'b010;

  logic is_store_op;
  assign is_store_op = opcode == 5'b01000 && uncompressed;
  assign is_sb = is_store_op && funct3 == 3'b000;
  assign is_sh = is_store_op && funct3 == 3'b001;
  assign is_sw = (is_store_op && funct3 == 3'b010) || is_cswsp || is_csw;
  assign is_cswsp = quadrant == 2'b10 && cfunct3 == 3'b110;
  assign is_csw = quadrant == 2'b00 && cfunct3 == 3'b110;

  logic math_low;
  assign math_low = funct7 == 7'b0000000;
  logic math_high;
  assign math_high = funct7 == 7'b0100000;
  logic is_math_immediate_op, is_cli, is_caddi, is_caddi16sp, is_caddi4spn, is_cslli,
    is_csrli, is_csrai, is_candi;
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

  logic is_math_op, is_cmv, is_cadd, is_cand, is_cor, is_cxor, is_csub;
  assign is_math_op = opcode == 5'b01100 && uncompressed;
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

  logic is_m;
  assign is_m = is_math_op && funct7 == 7'b0000001;
  assign is_mul = is_m && funct3 == 3'b000;
  assign is_mulh = is_m && funct3 == 3'b001;
  assign is_mulhu = is_m && funct3 == 3'b011;
  assign is_mulhsu = is_m && funct3 == 3'b010;
  assign is_div = is_m && funct3 == 3'b100;
  assign is_divu = is_m && funct3 == 3'b101;
  assign is_rem = is_m && funct3 == 3'b110;
  assign is_remu = is_m && funct3 == 3'b111;

  logic is_csr;
  assign is_csr = opcode == 5'b11100 && uncompressed;
  assign is_csrrw = is_csr && funct3 == 3'b001;
  assign is_csrrs = is_csr && funct3 == 3'b010;
  assign is_csrrc = is_csr && funct3 == 3'b011;
  assign is_csrrwi = is_csr && funct3 == 3'b101;
  assign is_csrrsi = is_csr && funct3 == 3'b110;
  assign is_csrrci = is_csr && funct3 == 3'b111;

  logic is_error, is_ecall, is_ebreak;
  assign is_error = opcode == 5'b11100 && uncompressed && funct3 == 0 && rs1 == 0 && rd == 0;
  assign is_ecall = is_error && !{|instr[31:20]};
  assign is_ebreak = is_error && |instr[31:20];
  assign is_valid = is_auipc ||
    is_jal ||
    is_jalr ||
    is_beq ||
    is_bne ||
    is_blt ||
    is_bltu ||
    is_bge ||
    is_bgeu ||
    is_add ||
    is_sub ||
    is_mul ||
    is_div ||
    is_divu ||
    is_xor ||
    is_or ||
    is_and ||
    is_sll ||
    is_slt ||
    is_sltu ||
    is_srl ||
    is_sra ||
    is_lui ||
    is_lb ||
    is_lh ||
    is_sb
    ;

  always_ff @(posedge clk) begin
    if (!decode) begin
      decoded <= 0;
    end else begin
       (* parallel_case, full_case *)
       case (1'b1)
         is_beq || is_bne || is_blt || is_bge || is_bltu || is_bgeu ||
           is_sb || is_sh || is_sw || is_cj || is_cjr: rd <= 0;
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
       decoded <= 1;
    end
  end
endmodule
