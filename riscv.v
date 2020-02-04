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
  output logic        trap,
  output logic [1:0]  trap_code
  // Formal
  `ifdef RISCV_FORMAL
     ,
     `RVFI_OUTPUTS
  `endif
  );
  // memory error code
  localparam trap_mem = 2'b00;
  // privileged, insecure and instruction protection flag
  localparam inst_prot = 3'b101;
  // privileged, insecure and data protection flag
  localparam data_prot = 3'b000;
  localparam rresp_ok = 2'b00;
  localparam rresp_xok = 2'b01;
  localparam bresp_ok = 2'b01;
  localparam bresp_xok = 2'b00;

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
  assign is_sh = is_store && funct3 == 3'b010;
  assign is_sw = is_store && funct3 == 3'b100;

  logic [31:0] math_arg;
  logic [31:0] math_arg_signed;
  assign math_arg_signed = {{27{rs2[4]}}, rs2};
  logic [31:0] math_arg_unsigned;
  assign math_arg_unsigned = {27'b0, rs2};

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

  always_comb begin
    if (is_math_immediate) begin
      // signed operations
      if (is_srai || is_slti) begin
        math_arg = math_arg_signed;
      end else begin
        math_arg = math_arg_unsigned;
      end
    end else begin
      math_arg = regs[rs2];
    end
  end

  logic is_error, is_ecall, is_ebreak, instr_valid;
  assign is_error = opcode == 7'b1110011;
  assign is_ecall = is_error && !instr[20];
  assign is_ebreak = is_error && instr[20];
  assign instr_valid = is_add ||
    is_load ||
    is_math_immediate ||
    is_math ||
    is_error ||
    is_ecall ||
    is_ebreak;

  logic [31:0]immediate;
  always_comb begin
    case (1'b1)
      is_load || is_jalr: immediate = i_immediate;
      is_store: immediate = s_immediate;
      is_lui || is_auipc: immediate = u_immediate;
      is_jal: immediate = j_immediate;
      is_branch: immediate = b_immediate;
      default: immediate = 32'b0;
    endcase
  end

  // registers
  logic [31:0] regs[0:31];
  `define zero regs[0];
  `define ra regs[1];
  `define sp regs[2];
  `define gp regs[3];
  `define fp regs[8];
  logic [31:0] pc;
  logic [31:0] instr;
  // storage for the next program counter and instruction
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
      next_pc <= 0;
      cpu_state <= fetch_instr;
      mem_valid <= 0;
    end else begin
      case (cpu_state)
        fetch_instr: begin
          mem_wstrb <= 4'b0;
          mem_instr <= 1;
          mem_valid <= 1;
          cpu_state <= ready_instr;
          mem_addr <= next_pc;
        end

        ready_instr: begin
          if (mem_ready) begin
            $display(mem_rdata);

            mem_valid <= 0;
            pc <= next_pc;
            instr <= mem_rdata;
            cpu_state <= execute_instr;
          end
        end

        execute_instr: begin
          $display("next");

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
              regs[rd] <= pc + 4;
              if (|jump_address[1:0]) begin
                cpu_state <= cpu_trap;
              end else begin
                next_pc <= jump_address;
                cpu_state <= fetch_instr;
              end
            end

            is_branch: begin
              case(1'b1)
                is_beq: next_pc <= regs[rs1] == regs[rs2] ? immediate : pc + 4;
                is_bne: next_pc <= regs[rs1] != regs[rs2] ? immediate : pc + 4;
                is_blt: next_pc <= $signed(regs[rs1]) < $signed(regs[rs2]) ? immediate : pc + 4;
                is_bltu: next_pc <= regs[rs1] < regs[rs2] ? immediate : pc + 4;
                is_bge: next_pc <= $signed(regs[rs1]) >= $signed(regs[rs2]) ? immediate : pc + 4;
                is_bgeu: next_pc <= regs[rs1] >= regs[rs2] ? immediate : pc + 4;
                default: cpu_state <= cpu_trap;
              endcase
              cpu_state <= fetch_instr;
            end

            is_math || is_math_immediate: begin
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
                case (1'b1)
                  is_sw: mem_wstrb <= 4'b1111;
                  is_sh: mem_wstrb <= 4'b0011;
                  is_sb: mem_wstrb <= 4'b0001;
                endcase
                mem_valid <= 1; // kick off a memory request
                cpu_state <= finish_store;
              end
            end

            default: begin
              $display("oppee");

              cpu_state <= fetch_instr;
            end
          endcase
        end

        finish_load: begin
          if (mem_ready) begin
            case (1'b1)
              is_lb: regs[rs2] <= {24'b0, mem_rdata[7:0]};
              is_lbu: regs[rs2] <= {{24{mem_rdata[7]}}, mem_rdata[7:0]};
              is_lh: regs[rs2] <= {16'b0, mem_rdata[15:0]};
              is_lhu: regs[rs2] <= {{16{mem_rdata[7]}}, mem_rdata[15:0]};
              is_lw: regs[rs2] <= mem_rdata;
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

        default: begin
          cpu_state <= fetch_instr;
        end
      endcase
    end
  end

`ifdef RISCV_FORMAL
  assign rvfi_valid = !reset && instr_valid;
  assign rvfi_rs2_addr = rs2;
  assign rvfi_rs1_addr = rs1;
  assign rvfi_insn = opcode;
  assign rvfi_rd_addr = rd;
  assign rvfi_trap = trap;
  assign rvfi_halt = trap;
  assign rvfi_pc_rdata = pc;
  assign rvfi_mem_rdata = rdata;
  assign rvfi_rs2_rdata = regs[rs2];
  assign rvfi_rs1_rdata = regs[rs1];
  assign rvfi_rd_wdata = regs[rd];
  assign rvfi_pc_wdata = next_pc;
  assign rvfi_mode = 3;
  assign rvfi_ixl = 1;
  assign rvfi_mem_wmask = wstrb;
  assign rvfi_mem_wdata = regs[rs2];
  assign rvfi_mem_rmask = 4'b1111;
  assign rvfi_mem_addr = load_store_address;
  assign rvfi_intr = 0;
  reg [63:0] order;
  assign rvfi_order = order;

  always_ff @(posedge clk) begin
    if (!reset)
      order <= 0;
    order <= order + rvfi_valid;
  end
`endif
endmodule
