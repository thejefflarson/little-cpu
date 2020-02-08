module riscv (
  input  logic        clk, reset,
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
  logic [4:0] rd, rs1, rs2, shamt;
  logic [2:0] funct3;
  logic [6:0] funct7;
  logic math_flag;
  assign opcode = instr[6:0];
  assign rd = instr[11:7];
  assign rs1 = instr[19:15];
  assign rs2 = instr[24:20];
  // For shift immediates
  assign funct3 = instr[14:12];
  assign funct7 = instr[31:25];
  assign math_flag = funct7 == 7'b0100000;
  // for load and store
  logic [31:0] load_store_address;
  assign load_store_address = immediate + regs[rs1];

  // for jump and link
  logic [31:0] jump_address;
  assign jump_address = is_jalr ? ($signed(immediate) + $signed(regs[rs2] & 32'hfffffe)) : $signed(pc) + $signed(immediate);

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
  assign is_jalr = opcode == 7'b1100111;

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
  assign is_slli = is_math_immediate && funct3 == 3'b001;
  assign is_srli = is_math_immediate && !math_flag && funct3 == 3'b101;
  assign is_srai = is_math_immediate && math_flag && funct3 == 3'b101;

  logic is_math, is_add, is_sub, is_sll, is_slt, is_sltu, is_xor, is_srl, is_sra, is_or, is_and;
  assign is_math = opcode == 7'b0110011;
  assign is_add = is_math && !math_flag && funct3 == 3'b000;
  assign is_sub = is_math && math_flag && funct3 == 3'b000;
  assign is_sll = is_math && funct3 == 3'b001;
  assign is_slt = is_math && funct3 == 3'b010;
  assign is_sltu = is_math && funct3 == 3'b011;
  assign is_xor = is_math && funct3 == 3'b100;
  assign is_srl = is_math && !math_flag && funct3 == 3'b101;
  assign is_sra = is_math && math_flag && funct3 == 3'b101;
  assign is_or = is_math && funct3 == 3'b110;
  assign is_and = is_math && funct3 == 3'b111;
  assign shamt = is_math_immediate ? rs2 : regs[rs2][4:0];

  logic is_error, is_ecall, is_ebreak;
  assign is_error = opcode == 7'b1110011;
  assign is_ecall = is_error && !instr[20];
  assign is_ebreak = is_error && instr[20];

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

  logic [31:0] math_arg;
  always_comb begin
    if (is_math_immediate) begin
      math_arg = immediate;
    end else begin
      math_arg = regs[rs2];
    end
  end

  // registers
  logic [31:0] regs[0:31];
  logic [31:0] pc;
  logic [31:0] instr;
  // storage for the next program counter
  logic [31:0] next_pc;

  // state_machine
  logic [4:0] cpu_state;
  localparam fetch_instr = 5'b00001;
  localparam ready_instr = 5'b00010;
  localparam execute_instr = 5'b00011;
  localparam finish_load = 5'b00100;
  localparam finish_store = 5'b00101;
  localparam cpu_trap = 5'b00000;

  task automatic do_next_instr;
    cpu_state <= fetch_instr;
    next_pc <= pc + 4;
  endtask

  integer i;
  always_ff @(posedge clk) begin
    if (!reset) begin
      for (i = 0; i < 32; i = i + 1) begin
        regs[i] <= 0;
      end
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
          (* parallel_case, full_case *)
          case (1'b1)
            is_lui: begin
              regs[rd] <= immediate;
              do_next_instr();
            end

            is_auipc: begin
              regs[rd] <= immediate + pc;
              do_next_instr();
            end

            is_jal || is_jalr: begin
              if (is_jalr) begin
                regs[rd] <= pc + 4;
              end
              if (|jump_address[1:0]) begin
                cpu_state <= cpu_trap;
              end else begin
                next_pc <= jump_address;
                cpu_state <= fetch_instr;
              end
            end

            is_branch: begin
              (* parallel_case, full_case *)
              case(1'b1)
                is_beq: next_pc <= regs[rs1] == regs[rs2] ? immediate : pc + 4;
                is_bne: next_pc <= regs[rs1] != regs[rs2] ? immediate : pc + 4;
                is_blt: next_pc <= $signed(regs[rs1]) < $signed(regs[rs2]) ? immediate : pc + 4;
                is_bltu: next_pc <= regs[rs1] < regs[rs2] ? immediate : pc + 4;
                is_bge: next_pc <= $signed(regs[rs1]) >= $signed(regs[rs2]) ? immediate : pc + 4;
                is_bgeu: next_pc <= regs[rs1] >= regs[rs2] ? immediate : pc + 4;
              endcase
              cpu_state <= fetch_instr;
            end

            is_math || is_math_immediate: begin
              (* parallel_case, full_case *)
              case(1'b1)
                is_add || is_addi: begin
                  regs[rd] <= regs[rs1] + math_arg;
                end

                is_sub: begin
                  regs[rd] <= regs[rs1] - math_arg;
                end

                is_sll || is_slli: begin
                  regs[rd] <= regs[rs1] << shamt;
                end

                is_slt || is_slti: begin
                  regs[rd] <= {31'b0, $signed(regs[rs1]) < $signed(math_arg)};
                end

                is_sltu || is_sltiu: begin
                  regs[rd] <= {31'b0, regs[rs1] < math_arg};
                end

                is_xor || is_xori: begin
                  regs[rd] <= regs[rs1] ^ math_arg;
                end

                is_srl || is_srli: begin
                  regs[rd] <= regs[rs1] >> shamt;
                end

                is_sra || is_srai: begin
                  regs[rd] <= $signed(regs[rs1]) >>> shamt;
                end

                is_or || is_ori: begin
                  regs[rd] <= regs[rs1] | math_arg;
                end

                is_and || is_andi: begin
                  regs[rd] <= regs[rs1] & math_arg;
                end
              endcase
              do_next_instr();
            end

            is_load: begin
              // can't load into x0 and can't load misaligned addresses
              if (rd == 0 || |load_store_address[1:0]) begin
                cpu_state <= cpu_trap;
              end else begin
                mem_wstrb <= 4'b0000;
                mem_addr <= load_store_address;
                mem_instr <= 0; // can we have data
                mem_valid <= 1; // kick off a memory request
                cpu_state <= finish_load;
              end
            end

            is_store: begin
              // Check for misalignment
              if ((is_sw && |load_store_address[1:0]) ||
                  (is_sh && load_store_address[0])) begin
                cpu_state <= cpu_trap;
              end else begin
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
            end

            default: begin
              cpu_state <= cpu_trap;
           end
         endcase
        end

        finish_load: begin
          if (mem_ready) begin
            case (1'b1)
              is_lb: regs[rd] <= {24'b0, mem_rdata[7:0]};
              is_lbu: regs[rd] <= {{24{mem_rdata[7]}}, mem_rdata[7:0]};
              is_lh: regs[rd] <= {16'b0, mem_rdata[15:0]};
              is_lhu: regs[rd] <= {{16{mem_rdata[7]}}, mem_rdata[15:0]};
              is_lw: regs[rd] <= mem_rdata;
            endcase
            cpu_state <= fetch_instr;
            mem_valid <= 0;
            do_next_instr();
          end
        end

        finish_store: begin
          if (mem_ready) begin
            cpu_state <= fetch_instr;
            mem_valid <= 0;
            do_next_instr();
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

  always_ff @(posedge clk) begin
    rvfi_valid <= reset && (trap || is_fetch); // && is_valid;

    // what were our read registers while this instruction was executing?
    if (cpu_state == execute_instr) begin
      rvfi_rs1_rdata <= regs[rs1];
      rvfi_rs2_rdata <= regs[rs2];
    end

    rvfi_rs1_addr <= rs1;
    rvfi_rs2_addr <= rs2;
    rvfi_insn <= instr;
    rvfi_rd_addr <= rd;
    rvfi_rd_wdata <= regs[rd];
    rvfi_trap <= trap;
    rvfi_halt <= trap;
    rvfi_pc_rdata <= pc;
    rvfi_mem_rdata <= mem_rdata;
    rvfi_pc_wdata <= next_pc;
    rvfi_mode <= 3;
    rvfi_ixl <= 1;
    rvfi_intr <= 0;
    rvfi_order <= reset ? rvfi_order + rvfi_valid : 0;

    if (mem_instr) begin
      rvfi_mem_addr <= 0;
      rvfi_mem_wmask <= 0;
      rvfi_mem_rmask <= 0;
      rvfi_mem_wmask <= 0;
      rvfi_mem_wdata <= 0;
    // what exactly came back from memory?
    end else if (mem_valid && mem_ready) begin
      rvfi_mem_addr <= mem_addr;
      rvfi_mem_wmask <= mem_wstrb;
      rvfi_mem_wdata <= mem_wdata;
      rvfi_mem_rmask <= (mem_wstrb == 4'b0000) && is_load ? ~0 : 0;
      rvfi_mem_addr <= mem_addr;
    end
  end
`endif
endmodule
