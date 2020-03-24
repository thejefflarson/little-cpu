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
 `ifdef RISCV_FORMAL
  output logic        rvfi_valid,
  output logic [63:0] rvfi_order,
  output logic [31:0] rvfi_insn,
  output logic        rvfi_trap,
  output logic        rvfi_halt,
  output logic        rvfi_intr,
  output logic [ 1:0] rvfi_mode,
  output logic [ 1:0] rvfi_ixl,
  output logic [ 4:0] rvfi_rs1_addr,
  output logic [ 4:0] rvfi_rs2_addr,
  output logic [31:0] rvfi_rs1_rdata,
  output logic [31:0] rvfi_rs2_rdata,
  output logic [ 4:0] rvfi_rd_addr,
  output logic [31:0] rvfi_rd_wdata,
  output logic [31:0] rvfi_pc_rdata,
  output logic [31:0] rvfi_pc_wdata,
  output logic [31:0] rvfi_mem_addr,
  output logic [ 3:0] rvfi_mem_rmask,
  output logic [ 3:0] rvfi_mem_wmask,
  output logic [31:0] rvfi_mem_rdata,
  output logic [31:0] rvfi_mem_wdata,
 `endif
  output logic        trap
  );

  // instruction decoder (figure 2.3)
  logic [31:0] instr;
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

  logic [31:0] immediate;
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
  logic is_lui, is_lui_op, is_auipc, is_jal, is_jal_op, is_jalr, is_jalr_op, is_cj, is_cjal, is_cjr,
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
  logic [31:0] jump_address;
  assign jump_address = is_jalr || is_cjr || is_cjalr ?
    ($signed(immediate) + $signed(regs[rs1])) & 32'hfffffffe :
    $signed(pc) + $signed(immediate);

  logic is_branch_op, is_branch, is_beq, is_bne, is_blt, is_bltu, is_bge, is_bgeu, is_cbeqz,
    is_cbnez;
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

  logic is_load_op, is_load, is_lb, is_lh, is_lw, is_lbu, is_lhu, is_clwsp, is_clw;
  assign is_load_op = opcode == 5'b00000 && uncompressed;
  assign is_lb = is_load_op && funct3 == 3'b000;
  assign is_lh = is_load_op && funct3 == 3'b001;
  assign is_lw = (is_load_op && funct3 == 3'b010) || is_clwsp || is_clw;
  assign is_lbu = is_load_op && funct3 == 3'b100;
  assign is_lhu = is_load_op && funct3 == 3'b101;
  assign is_clwsp = quadrant == 2'b10 && cfunct3 == 3'b010 && instr[11:7] != 5'b0;
  assign is_clw = quadrant == 2'b00 && cfunct3 == 3'b010;
  assign is_load = is_lb || is_lh || is_lw || is_lbu || is_lhu;

  logic is_store, is_store_op, is_sb, is_sh, is_sw, is_cswsp;
  assign is_store_op = opcode == 5'b01000 && uncompressed;
  assign is_sb = is_store_op && funct3 == 3'b000;
  assign is_sh = is_store_op && funct3 == 3'b001;
  assign is_sw = (is_store_op && funct3 == 3'b010) || is_cswsp;
  assign is_cswsp = quadrant == 2'b10 && cfunct3 == 3'b110;
  assign is_store = is_sb || is_sh || is_sw;

  logic math_low;
  assign math_low = funct7 == 7'b0000000;
  logic math_high;
  assign math_high = funct7 == 7'b0100000;
  logic is_math_immediate_op, is_math_immediate, is_addi, is_slti, is_sltiu, is_xori, is_ori,
    is_andi, is_slli, is_srli, is_srai, is_cli, is_caddi, is_caddi16sp, is_caddi4spn, is_cslli,
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
  assign is_math_immediate = is_addi || is_slti || is_sltiu || is_xori || is_ori || is_andi ||
    is_slli || is_srli || is_srai;

  logic is_math_op, is_math, is_add, is_sub, is_sll, is_slt, is_sltu, is_xor, is_srl, is_sra, is_or,
    is_and, is_cmv, is_cadd, is_cand, is_cor, is_cxor, is_csub;
  assign is_math_op = opcode == 5'b01100 && uncompressed;
  assign is_add = (is_math_op && math_low && funct3 == 3'b000) || is_cmv || is_cadd;
  assign is_cmv = quadrant == 2'b10 && cfunct4 == 4'b1000 && instr[6:2] != 0;
  assign is_cadd = quadrant == 2'b10 && cfunct4 == 4'b1001 && instr[6:2] != 0;
  assign is_sub = is_math_op && math_high && funct3 == 3'b000;
  assign is_sll = is_math_op && math_low && funct3 == 3'b001;
  assign is_slt = is_math_op && math_low && funct3 == 3'b010;
  assign is_sltu = is_math_op && math_low && funct3 == 3'b011;
  assign is_xor = is_math_op && math_low && funct3 == 3'b100;
  assign is_srl = is_math_op && math_low && funct3 == 3'b101;
  assign is_sra = is_math_op && math_high && funct3 == 3'b101;
  assign is_or = (is_math_op && math_low && funct3 == 3'b110) || is_cor;
  assign is_cor = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b10;
  assign is_and = (is_math_op && math_low && funct3 == 3'b111) || is_cand;
  assign is_cand = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b11;
  assign is_math = is_add || is_sub || is_sll || is_slt || is_sltu || is_xor || is_srl || is_sra ||
    is_or || is_and;

  logic is_m, is_multiply, is_mul, is_mulh, is_mulhu, is_mulhsu, is_divide, is_div, is_divu, is_rem,
    is_remu;
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

  logic [31:0] math_arg;
  assign math_arg = is_math_immediate ? immediate : regs[rs2];
  logic [4:0] shamt;
  assign shamt = is_math_immediate ? rs2 : regs[rs2][4:0];
  logic is_error, is_ecall, is_ebreak;
  assign is_error = opcode == 5'b11100 && uncompressed && funct3 == 0 && rs1 == 0 && rd == 0;
  assign is_ecall = is_error && !{|instr[31:20]};
  assign is_ebreak = is_error && |instr[31:20];
  logic is_valid;
  assign is_valid = is_lui ||
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
    is_ebreak;

  // registers
  logic [31:0] regs[0:31];
  logic [31:0] pc;
  logic [4:0] rd, rs1, rs2;
  logic [31:0] load_store_address;
  assign load_store_address = $signed(immediate) + $signed(regs[rs1]);
  logic [1:0] addr24;
  assign addr24 = load_store_address[1:0];
  logic addr16;
  assign addr16 = load_store_address[1];
  logic addr8;
  assign addr8 = load_store_address[0];

  // storage for the next program counter
  logic [31:0] next_pc;
  logic [31:0] pc_inc;
  assign pc_inc = uncompressed ? 4 : 2;

  // register write addr
  logic [31:0] reg_wdata;
  // pc write
  logic [31:0] pc_wdata;
  // multiply and divide state
  logic [63:0] mul_div_store;
  logic [6:0] mul_div_counter;
  logic [63:0] mul_div_x;
  logic [63:0] mul_div_y;

  // state machine
  logic [3:0] cpu_state;
  logic skip_reg_write;
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
          regs[0] <= 0;
        end

        ready_instr: begin
          if (mem_ready) begin
            mem_valid <= 0;
            pc <= mem_addr;
            instr <= mem_rdata;
            cpu_state <= decode_instr;
          end
        end

        decode_instr: begin
          (* parallel_case, full_case *)
          case (1'b1)
            is_branch || is_store || is_cj || is_cjr: rd <= 0;
            is_cjal || is_cjalr: rd <= 1;
            is_clw || is_caddi4spn: rd <= {2'b01, instr[4:2]};
            is_csrai || is_csrli || is_candi || is_cand || is_cor: rd <= {2'b01, instr[9:7]};
            default: rd <= instr[11:7];
          endcase

          (* parallel_case, full_case *)
          case (1'b1)
            is_clwsp || is_cswsp || is_caddi4spn: rs1 <= 2;
            is_clw || is_cbeqz || is_cbnez || is_csrai ||
              is_csrli || is_candi || is_cand || is_cor: rs1 <= {2'b01, instr[9:7]};
            is_cjr || is_cjalr || is_cslli: rs1 <= instr[11:7];
            is_cli || is_cmv: rs1 <= 0;
            is_caddi || is_caddi16sp || is_cadd: rs1 <= instr[11:7];
            default: rs1 <= instr[19:15];
          endcase

          (* parallel_case, full_case *)
          case(1'b1)
            is_cswsp || is_cslli || is_csrai || is_csrli || is_cmv || is_cadd: rs2 <= instr[6:2];
            is_cand || is_cor: rs2 <= {2'b01, instr[4:2]};
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
                  is_beq: pc_wdata <= regs[rs1] == regs[rs2] ? pc + immediate : pc + pc_inc;
                  is_bne: pc_wdata <= regs[rs1] != regs[rs2] ? pc + immediate : pc + pc_inc;
                  is_blt: pc_wdata <= $signed(regs[rs1]) < $signed(regs[rs2]) ? pc + immediate : pc + 4;
                  is_bltu: pc_wdata <= regs[rs1] < regs[rs2] ? pc + immediate : pc + 4;
                  is_bge: pc_wdata <= $signed(regs[rs1]) >= $signed(regs[rs2]) ? pc + immediate : pc + 4;
                  is_bgeu: pc_wdata <= regs[rs1] >= regs[rs2] ? pc + immediate : pc + 4;
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
                    reg_wdata <= regs[rs1] + math_arg;
                  end

                  is_sub: begin
                    reg_wdata <= regs[rs1] - math_arg;
                  end

                  is_sll || is_slli: begin
                    reg_wdata <= regs[rs1] << shamt;
                  end

                  is_slt || is_slti: begin
                    reg_wdata <= {31'b0, $signed(regs[rs1]) < $signed(math_arg)};
                  end

                  is_sltu || is_sltiu: begin
                    reg_wdata <= {31'b0, regs[rs1] < math_arg};
                  end

                  is_xor || is_xori: begin
                    reg_wdata <= regs[rs1] ^ math_arg;
                  end

                  is_srl || is_srli: begin
                    reg_wdata <= regs[rs1] >> shamt;
                  end

                  is_sra || is_srai: begin
                    reg_wdata <= $signed(regs[rs1]) >>> shamt;
                  end

                  is_or || is_ori: begin
                    reg_wdata <= regs[rs1] | math_arg;
                  end

                  is_and || is_andi: begin
                    reg_wdata <= regs[rs1] & math_arg;
                  end

                  is_multiply: begin
                    mul_div_counter <= is_mul ? 32 : 64;
                    cpu_state <= multiply;
                    mul_div_store <= 0;
                    (* parallel_case, full_case *)
                    case(1'b1)
                      is_mul || is_mulhu: begin
                        mul_div_x <= {32'b0,regs[rs1]};
                        mul_div_y <= {32'b0,regs[rs2]};
                      end

                      is_mulh: begin
                        mul_div_x <= {{32{regs[rs1][31]}},regs[rs1]};
                        mul_div_y <= {{32{regs[rs2][31]}},regs[rs2]};
                      end

                      is_mulhsu: begin
                        mul_div_x <= {{32{regs[rs1][31]}},regs[rs1]};
                        mul_div_y <= {{32'b0},regs[rs2]};
                      end
                    endcase
                  end

                  is_divide: begin
                    mul_div_counter <= 65;
                    cpu_state <= divide;
                    mul_div_store <= 0;
                    mul_div_x <= {32'b0,regs[rs1]};
                    mul_div_y <= {1'b0,regs[rs2],31'b0};
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

              is_store_op || is_cswsp: begin
                if ((is_sw && |addr24) ||
                    (is_sh && addr8)) begin
                  cpu_state <= cpu_trap;
                end else begin
                  (* parallel_case, full_case *)
                  case (1'b1)
                    is_sw: begin
                      mem_addr <= load_store_address;
                      mem_wstrb <= 4'b1111;
                      mem_wdata <= regs[rs2];
                    end

                    is_sh: begin
                      // Offset to the right position
                      mem_wstrb <= addr16 ? 4'b1100 : 4'b0011;
                      mem_wdata <= {2{regs[rs2][15:0]}};
                    end

                    is_sb: begin
                      mem_wstrb <= 4'b0001 << addr24;
                      mem_wdata <= {4{regs[rs2][7:0]}};
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
            mul_div_store <= mul_div_y[0] ? mul_div_store + mul_div_x : mul_div_store;
            mul_div_x <= mul_div_x << 1;
            mul_div_y <= mul_div_y >> 1;
            mul_div_counter <= mul_div_counter - 1;
          end else begin
            if (is_mul) begin
              reg_wdata <= mul_div_store[31:0];
            end else begin
              reg_wdata <= mul_div_store[63:32];
            end
            cpu_state <= reg_write;
          end
         `else
          cpu_state <= reg_write;
          (* parallel_case, full_case *)
          case (1'b1)
            is_mul: reg_wdata <= (regs[rs1] + regs[rs2]) ^ 32'h5876063e;
            is_mulh: reg_wdata <= (regs[rs1] + regs[rs2]) ^ 32'hf6583fb7;
            is_mulhu: reg_wdata <= (regs[rs1] + regs[rs2]) ^ 32'h949ce5e8;
            is_mulhsu: reg_wdata <= (regs[rs1] - regs[rs2]) ^ 32'hecfbe137;
          endcase
         `endif
        end

        divide: begin
         `ifndef RISCV_FORMAL_ALTOPS
          if (mul_div_counter > 0) begin
            if (mul_div_x <= mul_div_y) begin
              mul_div_store <= (mul_div_store << 1) | 1;
              mul_div_x <= mul_div_x - mul_div_y;
            end else begin
              mul_div_store <= mul_div_store << 1;
            end
            mul_div_y <= mul_div_y >> 1;
            mul_div_counter <= mul_div_counter - 1;
          end else begin
            (* parallel_case, full_case *)
            case(1'b1)
              is_div: reg_wdata <= regs[rs1][31] != regs[rs2][31] ? -mul_div_store[31:0] : mul_div_store[31:0];
              is_divu: reg_wdata <= mul_div_store[31:0];
              is_rem: reg_wdata <= regs[rs1][31] ? -mul_div_x[31:0] : mul_div_x[31:0];
              is_remu: reg_wdata <= mul_div_x[31:0];
            endcase
            cpu_state <= reg_write;
          end
         `else
          cpu_state <= reg_write;
          (* parallel_case, full_case *)
          case (1'b1)
            is_div: reg_wdata <= (regs[rs1] - regs[rs2]) ^ 32'h7f8529ec;
            is_divu: reg_wdata <= (regs[rs1] - regs[rs2]) ^ 32'h10e8fd70;
            is_rem: reg_wdata <= (regs[rs1] - regs[rs2]) ^ 32'h8da68fa5;
            is_remu: reg_wdata <= (regs[rs1] - regs[rs2]) ^ 32'h3138d0e1;
          endcase
         `endif
        end

        reg_write: begin
          regs[rd] <= reg_wdata;
          cpu_state <= fetch_instr;
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

 `ifdef RISCV_FORMAL
  logic is_fetch;
  assign is_fetch = cpu_state == fetch_instr;
  logic rs1_valid, rs2_valid;
  assign rs1_valid = !is_lui && !is_jal && !is_auipc;
  assign rs2_valid = !is_lui && !is_jal && !is_auipc && !is_jalr && !is_load;
  always_ff @(posedge clk) begin
    rvfi_valid <= !reset && ((is_fetch && is_valid) || trap);

    // what were our read registers while this instruction was executing?
    if (cpu_state == execute_instr) begin
      rvfi_rs1_rdata <= rs1_valid ? regs[rs1] : 0;
      rvfi_rs2_rdata <= rs2_valid ? regs[rs2] : 0;
    end

    rvfi_rs1_addr <= rs1_valid ? rs1 : 0;
    rvfi_rs2_addr <= rs2_valid ? rs2 : 0;
    rvfi_insn <= instr;

    rvfi_rd_addr <= rd;
    rvfi_rd_wdata <= |rd ? regs[rd] : 0;
    rvfi_trap <= trap;
    rvfi_halt <= trap;
    rvfi_pc_rdata <= pc;
    rvfi_pc_wdata <= next_pc;
    rvfi_mode <= 3;
    rvfi_ixl <= 1;
    rvfi_intr <= 0;
    rvfi_order <= !reset ? rvfi_order + rvfi_valid : 0;

    if (mem_instr) begin
      rvfi_mem_addr <= 0;
      rvfi_mem_wmask <= 0;
      rvfi_mem_rmask <= 0;
      rvfi_mem_rdata <= 0;
      rvfi_mem_wdata <= 0;
    // what exactly came back from memory?
    end else if (mem_valid && mem_ready) begin
      rvfi_mem_addr <= mem_addr;
      rvfi_mem_wmask <= mem_wstrb;
      rvfi_mem_rmask <= |mem_wstrb ? 0 : ~0;
      rvfi_mem_rdata <= mem_rdata;
      rvfi_mem_wdata <= mem_wdata;
    end
  end
 `endif
endmodule
