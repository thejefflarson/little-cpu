module riscv (
  input  logic        clk,
  input  logic        reset,
  // picorv32 memory interface, cuz it is nice
  output logic        mem_valid,
  output logic        mem_instr,
  input  logic        mem_ready,
  output logic [31:0] mem_addr,
  output logic [31:0] mem_wdata,
  output logic [3:0]  mem_wstrb,
  input  logic [31:0] mem_rdata,
  output logic        trap
 `ifdef RISCV_FORMAL
   , `RVFI_OUTPUTS
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
  logic is_m, is_multiply, is_mul, is_mulh, is_mulhu, is_mulhsu, is_divide, is_div, is_divu, is_rem,
    is_remu;
  logic [31:0] math_arg;
  logic [4:0] shamt;
  logic is_csr, is_csrrw, is_csrrs, is_csrrc, is_csrrwi, is_csrrsi, is_csrrci;
  logic is_error, is_ecall, is_ebreak;
  logic rs1_valid, rs2_valid;
  logic is_e_illegal;
  logic is_valid;
  logic [31:0] regs[0:15];
  logic [31:0] pc;
  logic [4:0] rd, rs1, rs2;
  logic [31:0] load_store_address;
  logic [1:0] addr24;
  logic addr16;
  logic addr8;
  logic [31:0] next_pc;
  logic [31:0] pc_inc;
  logic [31:0] reg_wdata;
  logic [31:0] pc_wdata;
  logic [63:0] mul_div_store;
  logic [5:0] mul_div_counter;
  logic [31:0] mul_div_operand;
  logic want_abs;
  logic [31:0] div_abs_rs1, div_abs_rs2;
  logic want_neg_mul;
  logic [31:0] mul_mag_rs1, mul_mag_rs2;
  logic [31:0] mul_div_a;
  logic mul_div_op_sub;
  logic [32:0] mul_div_sum;
  logic div_qbit;
  logic [3:0] cpu_state;
  logic skip_reg_write;

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

  // compressed instructions
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
    ($signed(immediate) + $signed(regs[rs1[3:0]])) & 32'hfffffffe :
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

  assign is_m = is_math_op && funct7 == 7'b0000001;
  assign is_mul = is_m && funct3 == 3'b000;
  assign is_mulh = is_m && funct3 == 3'b001;
  assign is_mulhu = is_m && funct3 == 3'b011;
  assign is_mulhsu = is_m && funct3 == 3'b010;
  assign is_multiply = is_mul || is_mulh || is_mulhu || is_mulhsu;
  assign is_div = is_m && funct3 == 3'b100;
  assign is_divu = is_m && funct3 == 3'b101;
  assign is_rem = is_m && funct3 == 3'b110;
  assign is_remu = is_m && funct3 == 3'b111;
  assign is_divide = is_div || is_divu || is_rem || is_remu;
  assign math_arg = is_math_immediate ? immediate : regs[rs2[3:0]];
  assign shamt = is_math_immediate ? rs2 : regs[rs2[3:0]][4:0];

  assign is_csr = opcode == 5'b11100 && uncompressed;
  assign is_csrrw = is_csr && funct3 == 3'b001;
  assign is_csrrs = is_csr && funct3 == 3'b010;
  assign is_csrrc = is_csr && funct3 == 3'b011;
  assign is_csrrwi = is_csr && funct3 == 3'b101;
  assign is_csrrsi = is_csr && funct3 == 3'b110;
  assign is_csrrci = is_csr && funct3 == 3'b111;

  assign is_error = opcode == 5'b11100 && uncompressed && funct3 == 0 && rs1 == 0 && rd == 0;
  assign is_ecall = is_error && !{|instr[31:20]};
  assign is_ebreak = is_error && |instr[31:20];

  // RV32E: x0-x15 only. rd's decode case always fills a real register field from
  // instr[11:7] except where it hard-codes a value under 16, so bit 4 alone says whether
  // the raw field named x16-x31; rs1/rs2 are the same but must first be gated to the
  // encodings that actually read a register there, since lui/auipc/jal have no rs1, and
  // jalr/load/math_immediate's would-be rs2 field is immediate or shamt bits instead
  // (math_arg and shamt both read `immediate`/`rs2` directly rather than regs[rs2] there).
  assign rs1_valid = !is_lui && !is_jal && !is_auipc;
  assign rs2_valid = !is_lui && !is_jal && !is_auipc && !is_jalr && !is_load &&
    !is_math_immediate;
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
    is_multiply ||
    is_divide ||
    is_ecall ||
    is_ebreak) && !is_e_illegal;

  // registers
  assign load_store_address = $signed(immediate) + $signed(regs[rs1[3:0]]);
  assign addr24 = load_store_address[1:0];
  assign addr16 = load_store_address[1];
  assign addr8 = load_store_address[0];

  // storage for the next program counter
  assign pc_inc = uncompressed ? 4 : 2;

  // register write addr
  // pc write
  // multiply and divide state

  // The shift-subtract loop below compares magnitudes, so a signed dividend or divisor
  // is negated before it starts and the sign is restored on the way out; DIVU/REMU read
  // their operand as its own magnitude already.
  assign want_abs = is_div || is_rem;
  assign div_abs_rs1 = want_abs && regs[rs1[3:0]][31] ? -regs[rs1[3:0]] : regs[rs1[3:0]];
  assign div_abs_rs2 = want_abs && regs[rs2[3:0]][31] ? -regs[rs2[3:0]] : regs[rs2[3:0]];

  // MUL's low 32 bits are sign-agnostic, so only MULH/MULHSU take a magnitude.
  assign mul_mag_rs1 = (is_mulh || is_mulhsu) && regs[rs1[3:0]][31] ? -regs[rs1[3:0]] : regs[rs1[3:0]];
  assign mul_mag_rs2 = is_mulh && regs[rs2[3:0]][31] ? -regs[rs2[3:0]] : regs[rs2[3:0]];
  assign want_neg_mul = (is_mulh && (regs[rs1[3:0]][31] ^ regs[rs2[3:0]][31])) ||
    (is_mulhsu && regs[rs1[3:0]][31]);

  // state machine
  localparam cpu_trap = 4'b0000;
  localparam fetch_instr = 4'b0001;
  localparam ready_instr = 4'b0010;
  localparam decode_instr = 4'b0011;
  localparam execute_instr = 4'b0100;
  localparam finish_load = 4'b0101;
  localparam finish_store = 4'b0110;
  localparam check_pc = 4'b0111;
  localparam reg_write = 4'b1000;
  localparam multiply = 4'b1001;
  localparam divide = 4'b1011;

  // One 32-bit adder/subtractor serves both loops; divide's carry-out doubles as "no
  // borrow", ORed with the shifted-out top bit of a remainder too large for any divisor.
  assign mul_div_a = cpu_state == divide ?
    {mul_div_store[62:32], mul_div_store[31]} : mul_div_store[63:32];
  assign mul_div_op_sub = cpu_state == divide;
  assign mul_div_sum = {1'b0, mul_div_a} +
    (mul_div_op_sub ? ({1'b0, ~mul_div_operand} + 33'b1) : {1'b0, mul_div_operand});
  assign div_qbit = mul_div_store[63] | mul_div_sum[32];

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
          mem_wstrb <= 4'b0000;
          mem_instr <= 1;
          mem_valid <= 1;
          cpu_state <= ready_instr;
          mem_addr <= next_pc;
          skip_reg_write <= 0;
`ifndef NANO_LATCH_RF
          regs[0] <= 0;
`endif
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
          cpu_state <= execute_instr;
        end

        execute_instr: begin
          if (!is_valid) begin
            cpu_state <= cpu_trap;
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
                  is_beq: pc_wdata <= regs[rs1[3:0]] == regs[rs2[3:0]] ? pc + immediate : pc + pc_inc;
                  is_bne: pc_wdata <= regs[rs1[3:0]] != regs[rs2[3:0]] ? pc + immediate : pc + pc_inc;
                  is_blt: pc_wdata <= $signed(regs[rs1[3:0]]) < $signed(regs[rs2[3:0]]) ? pc + immediate : pc + 4;
                  is_bltu: pc_wdata <= regs[rs1[3:0]] < regs[rs2[3:0]] ? pc + immediate : pc + 4;
                  is_bge: pc_wdata <= $signed(regs[rs1[3:0]]) >= $signed(regs[rs2[3:0]]) ? pc + immediate : pc + 4;
                  is_bgeu: pc_wdata <= regs[rs1[3:0]] >= regs[rs2[3:0]] ? pc + immediate : pc + 4;
                endcase
                skip_reg_write <= 1;
                cpu_state <= check_pc;
              end

              is_math || is_math_immediate || is_m: begin
                cpu_state <= reg_write;
                next_pc <= pc + pc_inc;
                (* parallel_case, full_case *)
                case(1'b1)
                  is_add || is_addi: begin
                    reg_wdata <= regs[rs1[3:0]] + math_arg;
                  end

                  is_sub: begin
                    reg_wdata <= regs[rs1[3:0]] - math_arg;
                  end

                  is_sll || is_slli: begin
                    reg_wdata <= regs[rs1[3:0]] << shamt;
                  end

                  is_slt || is_slti: begin
                    reg_wdata <= {31'b0, $signed(regs[rs1[3:0]]) < $signed(math_arg)};
                  end

                  is_sltu || is_sltiu: begin
                    reg_wdata <= {31'b0, regs[rs1[3:0]] < math_arg};
                  end

                  is_xor || is_xori: begin
                    reg_wdata <= regs[rs1[3:0]] ^ math_arg;
                  end

                  is_srl || is_srli: begin
                    reg_wdata <= regs[rs1[3:0]] >> shamt;
                  end

                  is_sra || is_srai: begin
                    reg_wdata <= $signed(regs[rs1[3:0]]) >>> shamt;
                  end

                  is_or || is_ori: begin
                    reg_wdata <= regs[rs1[3:0]] | math_arg;
                  end

                  is_and || is_andi: begin
                    reg_wdata <= regs[rs1[3:0]] & math_arg;
                  end

                  is_multiply: begin
                    mul_div_counter <= 32;
                    cpu_state <= multiply;
                    mul_div_operand <= mul_mag_rs1;
                    mul_div_store <= {32'b0, mul_mag_rs2};
                  end

                  is_divide: begin
                    mul_div_counter <= 32;
                    cpu_state <= divide;
                    mul_div_operand <= div_abs_rs2;
                    mul_div_store <= {32'b0, div_abs_rs1};
                  end
                endcase
              end

              is_load_op || is_clwsp || is_clw: begin
                if ((is_lw && |addr24) ||
                    ((is_lh || is_lhu) && addr8)) begin
                  cpu_state <= cpu_trap;
                end else begin
                  mem_wstrb <= 4'b0000;
                  mem_addr <= {load_store_address[31:2], 2'b00};
                  mem_instr <= 0; // can we have data
                  mem_valid <= 1; // kick off a memory request
                  cpu_state <= finish_load;
                end
              end

              is_store_op || is_cswsp || is_csw: begin
                if ((is_sw && |addr24) ||
                    (is_sh && addr8)) begin
                  cpu_state <= cpu_trap;
                end else begin
                  (* parallel_case, full_case *)
                  case (1'b1)
                    is_sw: begin
                      mem_addr <= load_store_address;
                      mem_wstrb <= 4'b1111;
                      mem_wdata <= regs[rs2[3:0]];
                    end

                    is_sh: begin
                      // Offset to the right position
                      mem_wstrb <= addr16 ? 4'b1100 : 4'b0011;
                      mem_wdata <= {2{regs[rs2[3:0]][15:0]}};
                    end

                    is_sb: begin
                      mem_wstrb <= 4'b0001 << addr24;
                      mem_wdata <= {4{regs[rs2[3:0]][7:0]}};
                    end
                  endcase
                  mem_addr <= {load_store_address[31:2], 2'b00};
                  mem_instr <= 0;
                  mem_valid <= 1; // kick off a memory request
                  cpu_state <= finish_store;
                end
              end

              is_error: begin
                cpu_state <= cpu_trap;
              end

              default: begin
                cpu_state <= cpu_trap;
              end
            endcase
          end
        end

        multiply: begin
         `ifndef RISCV_FORMAL_ALTOPS
          if (mul_div_counter > 0) begin
            mul_div_store <= mul_div_store[0]
              ? {mul_div_sum, mul_div_store[31:1]}
              : {1'b0, mul_div_store[63:32], mul_div_store[31:1]};
            mul_div_counter <= mul_div_counter - 1;
          end else begin
            (* parallel_case, full_case *)
            case (1'b1)
              is_mul: reg_wdata <= mul_div_store[31:0];
              is_mulhu: reg_wdata <= mul_div_store[63:32];
              default: reg_wdata <= want_neg_mul
                ? ~mul_div_store[63:32] + {31'b0, mul_div_store[31:0] == 32'b0}
                : mul_div_store[63:32];
            endcase
            cpu_state <= reg_write;
          end
         `else
          cpu_state <= reg_write;
          (* parallel_case, full_case *)
          case (1'b1)
            is_mul: reg_wdata <= (regs[rs1[3:0]] + regs[rs2[3:0]]) ^ 32'h5876063e;
            is_mulh: reg_wdata <= (regs[rs1[3:0]] + regs[rs2[3:0]]) ^ 32'hf6583fb7;
            is_mulhu: reg_wdata <= (regs[rs1[3:0]] + regs[rs2[3:0]]) ^ 32'h949ce5e8;
            is_mulhsu: reg_wdata <= (regs[rs1[3:0]] - regs[rs2[3:0]]) ^ 32'hecfbe137;
          endcase
         `endif
        end

        divide: begin
         `ifndef RISCV_FORMAL_ALTOPS
          if (mul_div_counter > 0) begin
            // Restoring division: keep the subtraction only where it did not borrow.
            mul_div_store <= {div_qbit ? mul_div_sum[31:0] : mul_div_a,
                               mul_div_store[30:0], div_qbit};
            mul_div_counter <= mul_div_counter - 1;
          end else begin
            // A zero divisor is answered from the operands: unlike a remainder register
            // the loop leaves untouched, this shifting one never settles on all-ones.
            (* parallel_case, full_case *)
            case (1'b1)
              is_div: reg_wdata <= (regs[rs2[3:0]] == 32'b0) ? 32'hffffffff :
                ((regs[rs1[3:0]][31] ^ regs[rs2[3:0]][31]) ?
                  -mul_div_store[31:0] : mul_div_store[31:0]);
              is_divu: reg_wdata <= (regs[rs2[3:0]] == 32'b0) ? 32'hffffffff : mul_div_store[31:0];
              is_rem: reg_wdata <= (regs[rs2[3:0]] == 32'b0) ? regs[rs1[3:0]] :
                (regs[rs1[3:0]][31] ? -mul_div_store[63:32] : mul_div_store[63:32]);
              is_remu: reg_wdata <= (regs[rs2[3:0]] == 32'b0) ? regs[rs1[3:0]] : mul_div_store[63:32];
            endcase
            cpu_state <= reg_write;
          end
         `else
          cpu_state <= reg_write;
          (* parallel_case, full_case *)
          case (1'b1)
            is_div: reg_wdata <= (regs[rs1[3:0]] - regs[rs2[3:0]]) ^ 32'h7f8529ec;
            is_divu: reg_wdata <= (regs[rs1[3:0]] - regs[rs2[3:0]]) ^ 32'h10e8fd70;
            is_rem: reg_wdata <= (regs[rs1[3:0]] - regs[rs2[3:0]]) ^ 32'h8da68fa5;
            is_remu: reg_wdata <= (regs[rs1[3:0]] - regs[rs2[3:0]]) ^ 32'h3138d0e1;
          endcase
         `endif
        end

        // for branches and jumps: if the next program counter is misaligned we need to trap
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
              // unpack the alignment from above
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
  logic is_fetch;
  assign is_fetch = cpu_state == fetch_instr;

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
  `undef RVFI_SHADOW

  always_ff @(posedge clk) begin
    rvfi_valid_q <= !reset && ((is_fetch && is_valid) || trap);

    // what were our read registers while this instruction was executing?
    if (cpu_state == execute_instr) begin
      rvfi_rs1_rdata_q <= rs1_valid ? regs[rs1[3:0]] : 0;
      rvfi_rs2_rdata_q <= rs2_valid ? regs[rs2[3:0]] : 0;
    end

    rvfi_rs1_addr_q <= rs1_valid ? rs1 : 0;
    rvfi_rs2_addr_q <= rs2_valid ? rs2 : 0;
    rvfi_insn_q <= instr;

    rvfi_rd_addr_q <= rd;
`ifdef NANO_LATCH_RF
    // A retiring write's latch has not opened yet; reg_wdata already holds the value.
    rvfi_rd_wdata_q <= |rd ? reg_wdata : 0;
`else
    rvfi_rd_wdata_q <= |rd ? regs[rd[3:0]] : 0;
`endif
    rvfi_trap_q <= trap;
    rvfi_halt_q <= trap;
    rvfi_pc_rdata_q <= pc;
    rvfi_pc_wdata_q <= next_pc;
    rvfi_mode_q <= 3;
    rvfi_ixl_q <= 1;
    rvfi_intr_q <= 0;
    rvfi_order_q <= !reset ? rvfi_order_q + rvfi_valid_q : 0;

    if (mem_instr) begin
      rvfi_mem_addr_q <= 0;
      rvfi_mem_wmask_q <= 0;
      rvfi_mem_rmask_q <= 0;
      rvfi_mem_rdata_q <= 0;
      rvfi_mem_wdata_q <= 0;
    // what exactly came back from memory?
    end else if (mem_valid && mem_ready) begin
      rvfi_mem_addr_q <= mem_addr;
      rvfi_mem_wmask_q <= mem_wstrb;
      rvfi_mem_rmask_q <= |mem_wstrb ? 0 : ~0;
      rvfi_mem_rdata_q <= mem_rdata;
      rvfi_mem_wdata_q <= mem_wdata;
    end
  end
 `endif
endmodule
