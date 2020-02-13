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
  logic [6:0] opcode;
  logic [4:0] rd, rs1, rs2;
  logic [2:0] funct3;
  logic [6:0] funct7;
  assign opcode = instr[6:0];
  assign rd = is_branch || is_store ? 0 : instr[11:7];
  assign rs1 = instr[19:15];
  assign rs2 = instr[24:20];
  assign funct3 = instr[14:12];
  assign funct7 = instr[31:25];
  logic [31:0] load_store_address;
  assign load_store_address = $signed(immediate) + $signed(regs[rs1]);

  // immediate decoder (figure 2.4)
  logic [31:0] i_immediate, s_immediate, b_immediate, u_immediate, j_immediate;
  assign i_immediate = {{20{instr[31]}}, instr[31:20]};
  assign s_immediate = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  assign b_immediate = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
  assign u_immediate = {instr[31], instr[30:20], instr[19:12], 12'b0};
  assign j_immediate = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

  // Table 24.2 RV32I
  logic is_lui, is_auipc, is_jal, is_jalr;
  assign is_lui = opcode == 7'b0110111;
  assign is_auipc = opcode == 7'b0010111;
  assign is_jal = opcode == 7'b1101111;
  assign is_jalr = opcode == 7'b1100111 && funct3 == 3'b000;
  logic [31:0] jump_address;
  assign jump_address = is_jalr ?
    ($signed(immediate) + $signed(regs[rs1])) & 32'hfffffffe  :
    $signed(pc) + $signed(immediate);

  logic is_branch, is_beq, is_bne, is_blt, is_bltu, is_bge, is_bgeu;
  assign is_branch = opcode == 7'b1100011;
  assign is_beq = is_branch && funct3 == 3'b000;
  assign is_bne = is_branch && funct3 == 3'b001;
  assign is_blt = is_branch && funct3 == 3'b100;
  assign is_bge = is_branch && funct3 == 3'b101;
  assign is_bltu = is_branch && funct3 == 3'b110;
  assign is_bgeu = is_branch && funct3 == 3'b111;

  logic is_load, is_lb, is_lh, is_lw, is_lbu, is_lhu;
  assign is_load = opcode == 7'b0000011;
  assign is_lb = is_load && funct3 == 3'b000;
  assign is_lh = is_load && funct3 == 3'b001;
  assign is_lw = is_load && funct3 == 3'b010;
  assign is_lbu = is_load && funct3 == 3'b100;
  assign is_lhu = is_load && funct3 == 3'b101;

  logic is_store, is_sb, is_sh, is_sw;
  assign is_store = opcode == 7'b0100011;
  assign is_sb = is_store && funct3 == 3'b000;
  assign is_sh = is_store && funct3 == 3'b001;
  assign is_sw = is_store && funct3 == 3'b010;

  logic is_math_immediate, is_addi, is_slti, is_sltiu, is_xori, is_ori, is_andi, is_slli, is_srli, is_srai;
  assign is_math_immediate = opcode == 7'b0010011;
  assign is_addi = is_math_immediate && funct3 == 3'b000;
  assign is_slti = is_math_immediate && funct3 == 3'b010;
  assign is_sltiu = is_math_immediate && funct3 == 3'b011;
  assign is_xori = is_math_immediate && funct3 == 3'b100;
  assign is_ori = is_math_immediate && funct3 == 3'b110;
  assign is_andi = is_math_immediate && funct3 == 3'b111;
  assign is_slli = is_math_immediate && funct7 == 7'b0000000 && funct3 == 3'b001;
  assign is_srli = is_math_immediate && funct7 == 7'b0000000 && funct3 == 3'b101;
  assign is_srai = is_math_immediate && funct7 == 7'b0100000 && funct3 == 3'b101;

  logic is_math, is_add, is_sub, is_sll, is_slt, is_sltu, is_xor, is_srl, is_sra, is_or, is_and;
  assign is_math = opcode == 7'b0110011;
  assign is_add = is_math && funct7 == 7'b0000000 && funct3 == 3'b000;
  assign is_sub = is_math && funct7 == 7'b0100000 && funct3 == 3'b000;
  assign is_sll = is_math && funct3 == 3'b001;
  assign is_slt = is_math && funct3 == 3'b010;
  assign is_sltu = is_math && funct3 == 3'b011;
  assign is_xor = is_math && funct7 == 7'b0000000 && funct3 == 3'b100;
  assign is_srl = is_math && funct7 == 7'b0000000 && funct3 == 3'b101;
  assign is_sra = is_math && funct7 == 7'b0100000 && funct3 == 3'b101;
  assign is_or = is_math && funct7 == 7'b0000000 && funct3 == 3'b110;
  assign is_and = is_math && funct7 == 7'b0000000 && funct3 == 3'b111;
  logic [31:0] math_arg;
  assign math_arg = is_math_immediate ? immediate : regs[rs2];
  logic [4:0] shamt;
  assign shamt = is_math_immediate ? rs2 : regs[rs2][4:0];

  logic is_error, is_ecall, is_ebreak;
  assign is_error = opcode == 7'b1110011;
  assign is_ecall = is_error && !instr[20];
  assign is_ebreak = is_error && instr[20];
  logic is_valid;
  assign is_valid = is_lui ||
    is_auipc ||
    is_jal ||
    is_jalr ||
    is_beq ||
    is_bne ||
    is_blt ||
    is_bge ||
    is_bltu ||
    is_bgeu ||
    is_lb ||
    is_lh ||
    is_lbu ||
    is_lhu ||
    is_lw ||
    is_sb ||
    is_sh ||
    is_sw ||
    is_addi ||
    is_slti ||
    is_sltiu ||
    is_xori ||
    is_ori ||
    is_andi ||
    is_slli ||
    is_srai ||
    is_add ||
    is_sub ||
    is_sll ||
    is_slt ||
    is_sltu ||
    is_xor ||
    is_srl ||
    is_or ||
    is_and ||
    is_error ||
    is_ecall ||
    is_ebreak;

  logic [31:0]immediate;
  always_comb begin
    (* parallel_case, full_case *)
    case (1'b1)
      is_load || is_jalr: immediate = i_immediate;
      is_store: immediate = s_immediate;
      is_lui || is_auipc: immediate = u_immediate;
      is_jal: immediate = j_immediate;
      is_branch: immediate = b_immediate;
      is_math_immediate: immediate = i_immediate;
      default: immediate = 32'b0;
    endcase
  end

  // registers
  logic [31:0] regs[0:31];
  logic [31:0] pc;
  logic [31:0] instr;
  // storage for the next program counter
  logic [31:0] next_pc;
  // register write addr
  logic [31:0] reg_wdata;

  // state_machine
  logic [2:0] cpu_state;
  logic skip_pc_check;
  localparam fetch_instr = 3'b001;
  localparam ready_instr = 3'b010;
  localparam execute_instr = 3'b011;
  localparam finish_load = 3'b100;
  localparam finish_store = 3'b101;
  localparam check_pc = 3'b111;
  localparam reg_write = 3'b110;
  localparam cpu_trap = 3'b000;

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
          skip_pc_check <= 0;
          regs[0] <= 0;
        end

        ready_instr: begin
          if (mem_ready) begin
            mem_valid <= 0;
            pc <= mem_addr;
            instr <= mem_rdata;
            cpu_state <= execute_instr;
          end
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
                skip_pc_check <= 1;
                next_pc <= pc + 4;
              end

              is_auipc: begin
                reg_wdata <= immediate + pc;
                cpu_state <= reg_write;
                skip_pc_check <= 1;
                next_pc <= pc + 4;
              end

              is_jal || is_jalr: begin
                reg_wdata <= pc + 4;
                next_pc <= jump_address;
                cpu_state <= reg_write;
              end

              is_branch: begin
                (* parallel_case, full_case *)
                case(1'b1)
                  is_beq: next_pc <= regs[rs1] == regs[rs2] ? pc + immediate : pc + 4;
                  is_bne: next_pc <= regs[rs1] != regs[rs2] ? pc + immediate : pc + 4;
                  is_blt: next_pc <= $signed(regs[rs1]) < $signed(regs[rs2]) ? pc + immediate : pc + 4;
                  is_bltu: next_pc <= regs[rs1] < regs[rs2] ? pc + immediate : pc + 4;
                  is_bge: next_pc <= $signed(regs[rs1]) >= $signed(regs[rs2]) ? pc + immediate : pc + 4;
                  is_bgeu: next_pc <= regs[rs1] >= regs[rs2] ? pc + immediate : pc + 4;
                endcase
                cpu_state <= check_pc;
              end

              is_math || is_math_immediate: begin
                cpu_state <= reg_write;
                next_pc <= pc + 4;
                skip_pc_check <= 1;
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
                endcase
              end

              is_load: begin
                mem_wstrb <= 4'b0000;
                mem_addr <= load_store_address;
                mem_instr <= 0; // can we have data
                mem_valid <= 1; // kick off a memory request
                cpu_state <= finish_load;
              end

              is_store: begin
                mem_addr <= load_store_address;
                mem_wdata <= regs[rs2];
                (* parallel_case, full_case *)
                case (1'b1)
                  is_sw: mem_wstrb <= 4'b1111;
                  is_sh: mem_wstrb <= 4'b0011;
                  is_sb: mem_wstrb <= 4'b0001;
                endcase
                mem_instr <= 0;
                mem_valid <= 1; // kick off a memory request
                cpu_state <= finish_store;
              end

              default: begin
                cpu_state <= cpu_trap;
              end
            endcase
          end
        end

        reg_write: begin
          regs[rd] <= reg_wdata;
          if (|skip_pc_check) begin
            cpu_state <= fetch_instr;
          end else begin
            cpu_state <= check_pc;
          end
        end

        // for branches and jumps: if the next program counter is misaligned we need to trap
        check_pc: begin
          cpu_state <= |next_pc[1:0] ? cpu_trap : fetch_instr;
        end

        finish_load: begin
          if (mem_ready) begin
            (* parallel_case, full_case *)
            case (1'b1)
              is_lb: regs[rd] <= {{24{mem_rdata[7]}}, mem_rdata[7:0]};
              is_lbu: regs[rd] <= {24'b0, mem_rdata[7:0]};
              is_lh: regs[rd] <= {{16{mem_rdata[15]}}, mem_rdata[15:0]};
              is_lhu: regs[rd] <= {16'b0, mem_rdata[15:0]};
              is_lw: regs[rd] <= mem_rdata;
            endcase
            cpu_state <= fetch_instr;
            mem_valid <= 0;
            next_pc <= pc + 4;
          end
        end

        finish_store: begin
          if (mem_ready) begin
            cpu_state <= fetch_instr;
            mem_valid <= 0;
            next_pc <= pc + 4;
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
  logic skip;

  always_ff @(posedge clk) begin
    if (reset) begin
      skip <= 1;
    end else if (cpu_state == fetch_instr) begin // not our first rodeo
      skip <= 0;
    end
    rvfi_valid <= !reset && (trap || is_fetch) && is_valid && !skip;

    // what were our read registers while this instruction was executing?
    if (cpu_state == execute_instr) begin
      rvfi_rs1_rdata <= regs[rs1];
      rvfi_rs2_rdata <= regs[rs2];
    end

    rvfi_rs1_addr <= rs1;
    rvfi_rs2_addr <= rs2;
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
