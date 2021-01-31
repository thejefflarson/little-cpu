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
  output logic [63:0] rvfi_csr_mcycle_rmask,
  output logic [63:0] rvfi_csr_mcycle_wmask,
  output logic [63:0] rvfi_csr_mcycle_rdata,
  output logic [63:0] rvfi_csr_mcycle_wdata,
  output logic [63:0] rvfi_csr_minstret_rmask,
  output logic [63:0] rvfi_csr_minstret_wmask,
  output logic [63:0] rvfi_csr_minstret_rdata,
  output logic [63:0] rvfi_csr_minstret_wdata,
 `endif //  `ifdef RISCV_FORMAL
  output logic        trap
  );

  logic [31:0] instr;
  logic [31:0] immediate;
  logic [31:0] pc;
  logic is_valid;
  logic decode, decoded;
  logic uncompressed;
  // all instructions
  logic is_auipc, is_jal, is_jalr, is_beq, is_bne, is_blt, is_bltu, is_bge, is_bgeu, is_add,
        is_sub, is_mul, is_mulh, is_mulhu, is_mulhsu, is_div, is_divu, is_rem, is_remu,
        is_xor, is_or, is_and, is_sll, is_slt, is_sltu, is_srl, is_sra, is_lui, is_lb,
        is_lbu, is_lhu, is_lh, is_lw, is_sb, is_sh, is_sw, is_ecall, is_ebreak, is_csrrw,
        is_csrrs, is_csrrc;

  logic [4:0] rd, rs1, rs2;

  // ALU helpers
  logic is_math, is_math_immediate;
  assign is_math = is_add || is_sub || is_sll || is_slt || is_sltu || is_xor || is_srl ||
    is_sra || is_or || is_and || is_mul || is_mulh || is_mulhu || is_mulhsu || is_div ||
    is_divu || is_rem || is_remu;
  logic [31:0] math_arg;
  assign math_arg = is_math_immediate ? immediate : regs[rs2];
  logic [4:0] shamt;
  assign shamt = is_math_immediate ? rs2 : regs[rs2][4:0];

  decoder decoder (
    .clk(clk),
    .instr(instr),
    .pc(pc),
    .immediate(immediate),
    .is_math_immediate(is_math_immediate),
    .is_valid(is_valid),
    .decode(decode),
    .decoded(decoded),
    .uncompressed(uncompressed),
    .rd(rd),
    .rs1(rs1),
    .rs2(rs2),
    .is_auipc(is_auipc),
    .is_jal(is_jal),
    .is_jalr(is_jalr),
    .is_beq(is_beq),
    .is_bne(is_bne),
    .is_blt(is_blt),
    .is_bltu(is_bltu),
    .is_bge(is_bge),
    .is_bgeu(is_bgeu),
    .is_add(is_add),
    .is_sub(is_sub),
    .is_xor(is_xor),
    .is_or(is_or),
    .is_and(is_and),
    .is_mul(is_mul),
    .is_mulh(is_mulh),
    .is_mulhu(is_mulhu),
    .is_mulhsu(is_mulhsu),
    .is_div(is_div),
    .is_divu(is_divu),
    .is_rem(is_rem),
    .is_remu(is_remu),
    .is_sll(is_sll),
    .is_slt(is_slt),
    .is_sltu(is_sltu),
    .is_srl(is_srl),
    .is_sra(is_sra),
    .is_lui(is_lui),
    .is_lb(is_lb),
    .is_lbu(is_lbu),
    .is_lh(is_lh),
    .is_lhu(is_lhu),
    .is_lw(is_lw),
    .is_sb(is_sb),
    .is_sh(is_sh),
    .is_sw(is_sw),
    .is_ecall(is_ecall),
    .is_ebreak(is_ebreak),
    .is_csrrw(is_csrrw),
    .is_csrrs(is_csrrs),
    .is_csrrc(is_csrrc)
  );

  logic [31:0] alu_rs1;
  logic [31:0] alu_rs2;
  logic [4:0]  alu_shamt;
  logic [31:0] alu_out;
  logic       alu_ready;
  logic       alu_valid;
  alu alu (
    .clk(clk),
    .rs1(alu_rs1),
    .rs2(alu_rs2),
    .shamt(alu_shamt),
    .out(alu_out),
    .is_add(is_add),
    .is_sub(is_sub),
    .is_xor(is_xor),
    .is_or(is_or),
    .is_and(is_and),
    .is_sll(is_sll),
    .is_slt(is_slt),
    .is_sltu(is_sltu),
    .is_srl(is_srl),
    .is_sra(is_sra),
    .is_mul(is_mul),
    .is_mulh(is_mulh),
    .is_mulhu(is_mulhu),
    .is_mulhsu(is_mulhsu),
    .is_div(is_div),
    .is_divu(is_divu),
    .is_rem(is_rem),
    .is_remu(is_remu),
    .ready(alu_ready),
    .valid(alu_valid)
  );


  // registers
  logic [31:0] regs[0:31];
  logic [31:0] regs_rs1, regs_rs2;
  logic [4:0] rd_addr;

  // memory
  logic [31:0] load_store_address;
  assign load_store_address = $signed(immediate) + $signed(regs[rs1]);
  logic [1:0] addr24;
  logic addr16, addr8;

  // storage for the next program counter
  logic [31:0] next_pc;
  logic [31:0] pc_inc;
  assign pc_inc = uncompressed ? 4 : 2;

  // register write addr
  logic [31:0] reg_wdata;
  // pc write
  logic [31:0] pc_wdata;

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
  localparam alu_wait = 4'b1010;
  localparam multiply = 4'b1001;
  localparam divide = 4'b1011;

  always_ff @(posedge clk) begin
    if (reset) begin
      pc <= 0;
      instr <= 0;
      next_pc <= 0;
      decode <= 0;
      alu_valid <= 0;
      mem_addr <= 0;
      mem_wdata <= 0;
      mem_wstrb <= 0;
      rd_addr <= 0;
      regs_rs1 <= 0;
      regs_rs2 <= 0;
      trap <= 0;
      cpu_state <= fetch_instr;
      mem_valid <= 0;
    end else begin
      (* parallel_case, full_case *)
      case (cpu_state)
        fetch_instr: begin
          mem_valid <= 1;
          mem_wstrb <= 4'b0000;
          mem_instr <= 1;
          cpu_state <= ready_instr;
          mem_addr <= next_pc;
          skip_reg_write <= 0;
          rd_addr <= 0;
          reg_wdata <= 0;
          regs[0] <= 0;
        end

        ready_instr: begin
          if (mem_ready) begin
            mem_valid <= 0;
            pc <= mem_addr;
            instr <= mem_rdata[1:0] == 2'b11 ? mem_rdata : {16'b0, mem_rdata[15:0]};
            cpu_state <= execute_instr;
            decode <= 1;
          end
        end

        execute_instr: begin
          if (decoded) begin
            decode <= 0;
            if (!is_valid) begin
              cpu_state <= cpu_trap;
            end else begin
              (* parallel_case, full_case *)
              case (1'b1)
                is_lui: begin
                  rd_addr <= rd;
                  reg_wdata <= immediate;
                  cpu_state <= reg_write;
                  next_pc <= pc + pc_inc;
                end

                is_auipc: begin
                  rd_addr <= rd;
                  reg_wdata <= immediate + pc;
                  cpu_state <= reg_write;
                  next_pc <= pc + 4;
                end

                is_jal || is_jalr: begin
                  pc_wdata <= is_jalr ?
                    ($signed(immediate) + $signed(regs[rs1])) & 32'hfffffffe :
                    $signed(pc) + $signed(immediate);
                  rd_addr <= rd;
                  reg_wdata <= pc + pc_inc;
                  skip_reg_write <= 0;
                  cpu_state <= check_pc;
                end

                is_beq || is_bne || is_blt || is_bltu || is_bge || is_bgeu: begin
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

                is_math: begin
                  rd_addr <= rd;
                  cpu_state <= alu_wait;
                  next_pc <= pc + pc_inc;
                  alu_rs1 <= regs[rs1];
                  alu_rs2 <= math_arg;
                  alu_shamt <= shamt;
                  alu_valid <= 1;
                end

                is_lw || is_lh || is_lhu || is_lb || is_lbu: begin
                  addr24 <= load_store_address[1:0];
                  addr16 <= load_store_address[1];
                  addr8 <= load_store_address[0];
                  if ((is_lw && |load_store_address[1:0]) ||
                      ((is_lh || is_lhu) && load_store_address[0])) begin
                    cpu_state <= cpu_trap;
                  end else begin
                    rd_addr <= rd;
                    mem_wstrb <= 4'b0000;
                    mem_addr <= {load_store_address[31:2], 2'b00};
                    mem_instr <= 0; // can we have data
                    mem_valid <= 1; // kick off a memory request
                    cpu_state <= finish_load;
                  end
                end

                is_sw || is_sh || is_sb: begin
                  if ((is_sw && |load_store_address[1:0]) ||
                      (is_sh && load_store_address[0])) begin
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
                        mem_wstrb <= load_store_address[1] ? 4'b1100 : 4'b0011;
                        mem_wdata <= {2{regs[rs2][15:0]}};
                      end

                      is_sb: begin
                        mem_wstrb <= 4'b0001 << load_store_address[1:0];
                        mem_wdata <= {4{regs[rs2][7:0]}};
                      end
                    endcase // case (1'b1)
                    mem_addr <= {load_store_address[31:2], 2'b00};
                    mem_instr <= 0;
                    mem_valid <= 1; // kick off a memory request
                    cpu_state <= finish_store;
                  end
                end

                is_ecall || is_ebreak: begin
                  cpu_state <= cpu_trap;
                end

                default: begin
                  cpu_state <= cpu_trap;
                end
              endcase
            end // else: !if(!is_valid)
          end
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

        alu_wait: begin
          if (alu_ready) begin
            alu_valid <= 0;
            cpu_state <= reg_write;
            reg_wdata <= alu_out;
          end
        end

        reg_write: begin
          regs[rd_addr] <= reg_wdata;
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

 `ifdef RISCV_FORMAL
  logic is_fetch;
  assign is_fetch = cpu_state == fetch_instr;
  logic rs1_valid, rs2_valid;
  assign rs1_valid = !is_lui && !is_jal && !is_auipc;
  assign rs2_valid = !is_lui && !is_jal && !is_auipc && !is_jalr && !is_lh && !is_lw && !is_lb && !is_lbu && !is_lhu;
  always_ff @(posedge clk) begin
    rvfi_valid <= !reset && ((is_fetch && is_valid) || trap);

    // what were our registers while this instruction was executing?
    if (cpu_state == execute_instr && decoded == 1) begin
      rvfi_rs1_rdata <= rs1_valid ? regs[rs1] : 0;
      rvfi_rs2_rdata <= rs2_valid ? regs[rs2] : 0;
    end

    rvfi_rd_addr <= rd_addr;
    rvfi_rd_wdata <= |rd_addr ? reg_wdata : 0;
    rvfi_rs1_addr <= rs1_valid ? rs1 : 0;
    rvfi_rs2_addr <= rs2_valid ? rs2 : 0;
    rvfi_insn <= instr;
    rvfi_trap <= trap;
    rvfi_halt <= trap;
    rvfi_pc_rdata <= pc;
    rvfi_pc_wdata <= next_pc;
    rvfi_mode <= 3;
    rvfi_ixl <= 1;
    rvfi_intr <= 0;
    rvfi_order <= !reset ? rvfi_order + rvfi_valid : 0;
    rvfi_csr_mcycle_rmask <= 64'b0;
    rvfi_csr_mcycle_wmask <= 64'b0;
    rvfi_csr_mcycle_rdata <= 64'b0;
    rvfi_csr_mcycle_wdata <= 64'b0;
    rvfi_csr_minstret_rmask <= 64'b0;
    rvfi_csr_minstret_wmask <= 64'b0;
    rvfi_csr_minstret_rdata <= 64'b0;
    rvfi_csr_minstret_wdata <= 64'b0;

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
