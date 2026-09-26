`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// X is where a register value first exists: the ALU, branch compare, address/region
// test, every trap but the timer interrupt, and CSR access land here.
module executor #(
  parameter integer      LS_TEXT_WORDS = 2048,
  parameter logic [31:0] LS_RAM_BASE   = 32'h0001_0000,
  parameter integer      LS_RAM_WORDS  = 16384,
  parameter logic [31:0] LS_TIMER_BASE = 32'h0002_0000,
  parameter logic [31:0] LS_UART_BASE  = 32'h0002_0020,
  parameter logic [31:0] LS_FLASH_BASE = 32'h0002_0028
) (
  input  logic clk,
  input  logic reset,

  input  dx_output in,
  input  logic [31:0] reg_rs1,
  input  logic [31:0] reg_rs2,
  output logic x_busy,

  output logic [31:0] atomic_addr,
  input  logic        atomic_supported,

  output logic [11:0] csr_addr,
  output logic        csr_ren,
  output logic        csr_wen,
  output logic [31:0] csr_wdata,
  input  logic [31:0] csr_rdata,
  input  logic        csr_implemented,
  output logic        instret,
  output logic        trap_entry,
  output logic [31:0] trap_cause,
  output logic [31:0] trap_epc,
  output logic [31:0] trap_tval,
  output logic        mret_entry,
  input  logic [31:0] mtvec,
  input  logic [31:0] mepc,

  output logic         redirect,  // resolved target != D's guess, a trap, or mret
  output logic [31:0]  redirect_target,

  output decoder_output launch,
  output executor_output out
 `ifdef RISCV_FORMAL
  ,
  input  rvfi_csr64   csr_rvfi_mcycle,
  input  rvfi_csr64   csr_rvfi_minstret,
  input  rvfi_csr32   csr_rvfi_mscratch
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  , input  rvfi_csr32   csr_rvfi_mcause
  `endif
 `endif
);
  // Named continuous assigns, not part-selects inside the always_* blocks below: iverilog
  // cannot build a precise sensitivity entry for those (ADR-0037's class of defect).
  logic        in_valid, in_is_interrupt, in_imem_fault;
  logic [31:0] in_pc, in_instr, in_immediate;
  logic [4:0]  in_rd, in_rs1, in_rs2;
  logic        in_is_add, in_is_sub, in_is_xor, in_is_or, in_is_and, in_is_mul, in_is_mulh,
    in_is_mulhu, in_is_mulhsu, in_is_div, in_is_divu, in_is_rem, in_is_remu, in_is_sll,
    in_is_slt, in_is_sltu, in_is_srl, in_is_sra, in_is_lb, in_is_lbu, in_is_lhu, in_is_lh,
    in_is_lw, in_is_sb, in_is_sh, in_is_sw, in_is_amoswap, in_is_amoadd, in_is_amoxor,
    in_is_amoand, in_is_amoor, in_is_amomin, in_is_amomax, in_is_amominu, in_is_amomaxu,
    in_is_lr, in_is_sc, in_is_auipc, in_is_lui, in_is_jal, in_is_jalr, in_is_beq, in_is_bne,
    in_is_blt, in_is_bltu, in_is_bge, in_is_bgeu, in_is_ecall, in_is_ebreak, in_is_mret,
    in_is_wfi, in_is_fence, in_is_fencei, in_is_csrrw, in_is_csrrs, in_is_csrrc, in_is_csr_imm,
    in_is_csr_access, in_is_math_imm, in_fwd_rs1, in_fwd_rs2;
  assign {in_valid, in_is_interrupt, in_imem_fault, in_pc, in_instr, in_immediate, in_rd,
    in_rs1, in_rs2, in_is_add, in_is_sub, in_is_xor, in_is_or, in_is_and, in_is_mul, in_is_mulh,
    in_is_mulhu, in_is_mulhsu, in_is_div, in_is_divu, in_is_rem, in_is_remu, in_is_sll,
    in_is_slt, in_is_sltu, in_is_srl, in_is_sra, in_is_lb, in_is_lbu, in_is_lhu, in_is_lh,
    in_is_lw, in_is_sb, in_is_sh, in_is_sw, in_is_amoswap, in_is_amoadd, in_is_amoxor,
    in_is_amoand, in_is_amoor, in_is_amomin, in_is_amomax, in_is_amominu, in_is_amomaxu,
    in_is_lr, in_is_sc, in_is_auipc, in_is_lui, in_is_jal, in_is_jalr, in_is_beq, in_is_bne,
    in_is_blt, in_is_bltu, in_is_bge, in_is_bgeu, in_is_ecall, in_is_ebreak, in_is_mret,
    in_is_wfi, in_is_fence, in_is_fencei, in_is_csrrw, in_is_csrrs, in_is_csrrc, in_is_csr_imm,
    in_is_csr_access, in_is_math_imm, in_fwd_rs1, in_fwd_rs2} = in;

  // D precomputed these selects from register NUMBERS alone -- the one case the
  // write-through bypass (commitment 4) reaches too late.
  logic [31:0] fwd_rs1_val, fwd_rs2_val;
  assign fwd_rs1_val = in_fwd_rs1 ? out.rd_data : reg_rs1;
  assign fwd_rs2_val = in_fwd_rs2 ? out.rd_data : reg_rs2;

  logic [4:0] rs1_field;
  assign rs1_field = in_instr[19:15];

  // Never forwarded: `csr_arg` is the one X operand D's own select excludes.
  logic [31:0] csr_arg;
  assign csr_arg = in_is_csr_imm ? {27'b0, rs1_field} : reg_rs1;

  // Zicsr's suppression rules make `csrr` legal on a read-only CSR; `csr_read_op` exists only so RVFI reports the right read mask.
  logic csr_src_zero, csr_write_op, csr_read_op;
  assign csr_src_zero = rs1_field == 5'b0;
  assign csr_write_op = in_is_csr_access && !((in_is_csrrs || in_is_csrrc) && csr_src_zero);
  assign csr_read_op  = in_is_csr_access && !(in_is_csrrw && in_rd == 5'b0);
  assign csr_addr = in_instr[31:20];
  assign csr_wdata = in_is_csrrw ? csr_arg :
                     in_is_csrrs ? (csr_rdata | csr_arg) :
                                   (csr_rdata & ~csr_arg);

  logic [31:0] mem_addr_calc;
  assign mem_addr_calc = $signed(in_immediate) + $signed(fwd_rs1_val);
 `ifdef RISCV_FORMAL
  logic [31:0] mem_fault_word_addr;
  assign mem_fault_word_addr = {mem_addr_calc[31:2], 2'b00};
 `endif
  assign atomic_addr = fwd_rs1_val;

  logic instr_ls_load, instr_ls_store, ls_access;
  assign instr_ls_load  = in_is_lb || in_is_lbu || in_is_lh || in_is_lhu || in_is_lw;
  assign instr_ls_store = in_is_sb || in_is_sh || in_is_sw;
  assign ls_access = instr_ls_load || instr_ls_store;

  localparam logic [31:0] LS_TEXT_BYTES = LS_TEXT_WORDS * 4;
  localparam logic [31:0] LS_RAM_BYTES  = LS_RAM_WORDS * 4;
  logic ls_supported;
  assign ls_supported =
    ((mem_addr_calc & ~(LS_TEXT_BYTES - 32'd1)) == 32'd0) ||
    (((mem_addr_calc ^ LS_RAM_BASE) & ~(LS_RAM_BYTES - 32'd1)) == 32'd0) ||
    (mem_addr_calc[31:5] == LS_TIMER_BASE[31:5]) ||
    (mem_addr_calc[31:3] == LS_UART_BASE[31:3]) ||
    (mem_addr_calc[31:3] == LS_FLASH_BASE[31:3]);

  logic [1:0] mem_addr_low;
  assign mem_addr_low = in_immediate[1:0] + fwd_rs1_val[1:0];

  logic ls_fault;

  // Read again by the RVFI fault mask below, ungated by `executing`, for a trapping amo.
  logic is_amo;
  assign is_amo = in_is_amoswap || in_is_amoadd || in_is_amoxor || in_is_amoand ||
    in_is_amoor || in_is_amomin || in_is_amomax || in_is_amominu || in_is_amomaxu;

  logic instr_atomic, instr_atomic_write, word_misaligned;
  assign instr_atomic = in_is_lr || in_is_sc || is_amo;
  assign instr_atomic_write = in_is_sc || is_amo;
  assign word_misaligned = mem_addr_low != 2'b00;

  logic load_misaligned, store_misaligned;
  assign load_misaligned  = (in_is_lw && word_misaligned) ||
                            ((in_is_lh || in_is_lhu) && mem_addr_low[0] != 1'b0) ||
                            (in_is_lr && word_misaligned);
  assign store_misaligned = (in_is_sw && word_misaligned) ||
                            (in_is_sh && mem_addr_low[0] != 1'b0) ||
                            (instr_atomic_write && word_misaligned);

  logic atomic_fault;
  assign atomic_fault = instr_atomic && !atomic_supported && !word_misaligned;
  assign ls_fault = ls_access && !ls_supported && !load_misaligned && !store_misaligned;

  logic load_access_fault, store_access_fault;
  assign load_access_fault  = (atomic_fault && in_is_lr) || (ls_fault && instr_ls_load);
  assign store_access_fault = (atomic_fault && instr_atomic_write) ||
                              (ls_fault && instr_ls_store);

  logic instr_valid, csr_readonly_write, instr_illegal;
  assign instr_valid = in_is_auipc || in_is_jal || in_is_jalr || in_is_beq || in_is_bne ||
    in_is_blt || in_is_bltu || in_is_bge || in_is_bgeu || in_is_add || in_is_sub || in_is_xor ||
    in_is_or || in_is_and || in_is_mul || in_is_mulh || in_is_mulhu || in_is_mulhsu ||
    in_is_div || in_is_divu || in_is_rem || in_is_remu || in_is_sll || in_is_slt || in_is_sltu ||
    in_is_srl || in_is_sra || in_is_lui || in_is_lb || in_is_lbu || in_is_lh || in_is_lhu ||
    in_is_lw || in_is_sb || in_is_sh || in_is_sw || in_is_ecall || in_is_ebreak || in_is_mret ||
    in_is_wfi || in_is_fence || in_is_fencei ||
    instr_atomic || (in_is_csr_access && csr_implemented);
  assign csr_readonly_write = in_is_csr_access && csr_write_op && csr_addr[11:10] == 2'b11;
  assign instr_illegal = in_valid && !in_is_interrupt && (!instr_valid || csr_readonly_write);

  localparam logic [31:0] CAUSE_INSTRUCTION_FAULT   = 32'd1;
  localparam logic [31:0] CAUSE_ILLEGAL_INSTRUCTION = 32'd2;
  localparam logic [31:0] CAUSE_BREAKPOINT          = 32'd3;
  localparam logic [31:0] CAUSE_LOAD_MISALIGNED     = 32'd4;
  localparam logic [31:0] CAUSE_LOAD_ACCESS_FAULT   = 32'd5;
  localparam logic [31:0] CAUSE_STORE_MISALIGNED    = 32'd6;
  localparam logic [31:0] CAUSE_STORE_ACCESS_FAULT  = 32'd7;
  localparam logic [31:0] CAUSE_ECALL_M             = 32'd11;
  localparam logic [31:0] CAUSE_MACHINE_TIMER       = 32'h8000_0007;

 `ifdef RISCV_FORMAL
  // A plain `assign`: a constant select in an always_* process is Icarus's own "sorry", allowlisted nowhere but rtl/writeback.v.
  logic [3:0] ls_fault_wstrb;
  assign ls_fault_wstrb = in_is_sb ? (4'b0001 << mem_addr_calc[1:0]) :
                           in_is_sh ? (4'b0011 << mem_addr_calc[1:0]) :
                           4'b1111;
 `endif

  logic data_fault, trap_pending, trap_taken;
  assign data_fault = load_misaligned || store_misaligned || atomic_fault || ls_fault;
  assign trap_pending = in_valid && !in_is_interrupt &&
    (in_imem_fault || instr_illegal || in_is_ebreak || in_is_ecall || data_fault);
  assign trap_taken = in_valid && (in_is_interrupt || trap_pending);

  always_comb begin
    case (1'b1)
      in_is_interrupt:    trap_cause = CAUSE_MACHINE_TIMER;
      in_imem_fault:      trap_cause = CAUSE_INSTRUCTION_FAULT;
      instr_illegal:      trap_cause = CAUSE_ILLEGAL_INSTRUCTION;
      in_is_ebreak:       trap_cause = CAUSE_BREAKPOINT;
      in_is_ecall:        trap_cause = CAUSE_ECALL_M;
      load_misaligned:    trap_cause = CAUSE_LOAD_MISALIGNED;
      store_misaligned:   trap_cause = CAUSE_STORE_MISALIGNED;
      load_access_fault:  trap_cause = CAUSE_LOAD_ACCESS_FAULT;
      store_access_fault: trap_cause = CAUSE_STORE_ACCESS_FAULT;
      default:            trap_cause = 32'b0;
    endcase
  end

  assign trap_epc = in_pc;

  always_comb begin
    case (1'b1)
      in_is_interrupt: trap_tval = 32'b0;
      in_imem_fault:   trap_tval = in_pc;
      instr_illegal:   trap_tval = in_instr;
      data_fault:      trap_tval = mem_addr_calc;
      default:         trap_tval = 32'b0;
    endcase
  end

  logic [32:0] cmp_sub;
  logic        cmp_eq, cmp_ltu, cmp_lt;
  assign cmp_sub = {1'b0, fwd_rs1_val} - {1'b0, fwd_rs2_val};
  assign cmp_eq  = ~|cmp_sub[31:0];
  assign cmp_ltu = cmp_sub[32];
  assign cmp_lt  = (fwd_rs1_val[31] ^ fwd_rs2_val[31]) ? fwd_rs1_val[31] : cmp_sub[32];

  logic branch_taken;
  always_comb begin
    (* parallel_case *)
    case (1'b1)
      in_is_beq:  branch_taken =  cmp_eq;
      in_is_bne:  branch_taken = !cmp_eq;
      in_is_blt:  branch_taken =  cmp_lt;
      in_is_bge:  branch_taken = !cmp_lt;
      in_is_bltu: branch_taken =  cmp_ltu;
      in_is_bgeu: branch_taken = !cmp_ltu;
      default:    branch_taken = 1'b0;
    endcase
  end

  logic [31:0] pc_inc, seq_pc, resolved_target;
  assign pc_inc = in_instr[1:0] == 2'b11 ? 4 : 2;
  assign seq_pc = in_pc + pc_inc;
  always_comb begin
    case (1'b1)
      trap_taken:                resolved_target = mtvec;
      in_is_mret:                resolved_target = mepc;
      in_is_jalr:                resolved_target = ($signed(in_immediate) + $signed(fwd_rs1_val)) &
                                                     32'hfffffffe;
      in_is_jal || branch_taken: resolved_target = in_pc + in_immediate;
      default:                   resolved_target = seq_pc;
    endcase
  end

  assign redirect = in_valid && !x_busy && (resolved_target != seq_pc ||
    trap_taken || in_is_mret);
  assign redirect_target = resolved_target;

  logic committing;
  assign committing = in_valid && !x_busy && !trap_taken;
  assign csr_ren = committing && in_is_csr_access && csr_read_op;
  assign csr_wen = committing && in_is_csr_access && csr_write_op;
  assign instret = committing;
  assign trap_entry = in_valid && !x_busy && trap_taken;
  assign mret_entry = committing && in_is_mret;

  logic [1:0]  state;
  localparam init = 2'b00;
  localparam divide = 2'b10;
  // x_busy is exactly divider_busy now that region_stall is gone, but restated rather
  // than aliased: a bare `x_busy = divider_busy` collapses to the same netlist bit,
  // which would make test/zkt_isolation_test.py's one-hop block land on x_busy itself.
  logic divider_busy;
  assign divider_busy = state != init;
  assign x_busy = state != init;

  logic [31:0] alu_rs1, alu_rs2;
  always_comb begin
    (* parallel_case *)
    case (1'b1)
      in_is_auipc: begin
        alu_rs1 = in_pc;
        alu_rs2 = in_immediate;
      end
      in_is_csr_access: begin
        alu_rs1 = csr_rdata;
        alu_rs2 = 32'b0;
      end
      in_is_lui: begin
        alu_rs1 = in_immediate;
        alu_rs2 = 32'b0;
      end
      in_is_jal || in_is_jalr: begin
        alu_rs1 = in_pc;
        alu_rs2 = pc_inc;
      end
      default: begin
        alu_rs1 = fwd_rs1_val;
        alu_rs2 = in_is_math_imm ? in_immediate : fwd_rs2_val;  // shift imm uses shift_amt below
      end
    endcase
  end

  // A trap still retires; `executing` gates that, and `launch.valid` must not.
  logic executing;
  assign executing = in_valid && !in_is_interrupt && !x_busy && !trap_taken;
  assign launch.valid = in_valid && !in_is_interrupt && !x_busy;
  assign launch.rd = executing ? in_rd : 5'b0;
  assign launch.rs1 = alu_rs1;
  assign launch.rs2 = fwd_rs2_val;
  assign launch.mem_addr = mem_addr_calc;
  assign launch.is_valid_instr = instr_valid;
  assign launch.is_add = executing && (in_is_add || in_is_auipc || in_is_lui || in_is_jal ||
    in_is_jalr || in_is_csr_access);
  assign launch.is_sub = executing && in_is_sub;
  assign launch.is_xor = executing && in_is_xor;
  assign launch.is_or = executing && in_is_or;
  assign launch.is_and = executing && in_is_and;
  assign launch.is_mul = executing && in_is_mul;
  assign launch.is_mulh = executing && in_is_mulh;
  assign launch.is_mulhu = executing && in_is_mulhu;
  assign launch.is_mulhsu = executing && in_is_mulhsu;
  assign launch.is_div = executing && in_is_div;
  assign launch.is_divu = executing && in_is_divu;
  assign launch.is_rem = executing && in_is_rem;
  assign launch.is_remu = executing && in_is_remu;
  assign launch.is_sll = executing && in_is_sll;
  assign launch.is_slt = executing && in_is_slt;
  assign launch.is_sltu = executing && in_is_sltu;
  assign launch.is_srl = executing && in_is_srl;
  assign launch.is_sra = executing && in_is_sra;
  assign launch.is_lb = executing && in_is_lb;
  assign launch.is_lbu = executing && in_is_lbu;
  assign launch.is_lhu = executing && in_is_lhu;
  assign launch.is_lh = executing && in_is_lh;
  assign launch.is_lw = executing && in_is_lw;
  assign launch.is_sb = executing && in_is_sb;
  assign launch.is_sh = executing && in_is_sh;
  assign launch.is_sw = executing && in_is_sw;
  // Not a read of `launch.is_amo`: that would read as feedback through the struct port.
  assign launch.is_amo = executing && is_amo;
  assign launch.is_amoswap = executing && in_is_amoswap;
  assign launch.is_amoadd = executing && in_is_amoadd;
  assign launch.is_amoxor = executing && in_is_amoxor;
  assign launch.is_amoand = executing && in_is_amoand;
  assign launch.is_amoor = executing && in_is_amoor;
  assign launch.is_amomin = executing && in_is_amomin;
  assign launch.is_amomax = executing && in_is_amomax;
  assign launch.is_amominu = executing && in_is_amominu;
  assign launch.is_amomaxu = executing && in_is_amomaxu;
  assign launch.is_lr = executing && in_is_lr;
  assign launch.is_sc = executing && in_is_sc;

 `ifdef RISCV_FORMAL
  logic rvfi_rs1_valid, rvfi_rs2_valid;
  assign rvfi_rs1_valid = !in_is_lui && !in_is_jal && !in_is_auipc && !in_is_csr_imm;
  logic uses_rs2_rvfi;
  assign uses_rs2_rvfi = ((in_is_add || in_is_sub || in_is_sll || in_is_slt || in_is_sltu ||
    in_is_xor || in_is_srl || in_is_sra || in_is_or || in_is_and || in_is_mul || in_is_mulh ||
    in_is_mulhu || in_is_mulhsu || in_is_div || in_is_divu || in_is_rem || in_is_remu) &&
    !in_is_math_imm) ||
    in_is_sb || in_is_sh || in_is_sw || in_is_beq || in_is_bne || in_is_blt || in_is_bltu ||
    in_is_bge || in_is_bgeu || is_amo || in_is_sc;
  assign rvfi_rs2_valid = uses_rs2_rvfi;

  // The interrupt bubble never retires, so rvfi_intr latches for the next real retire.
  logic pending_intr;
  always_ff @(posedge clk) begin
    if (reset) pending_intr <= 1'b0;
    else if (in_valid && in_is_interrupt) pending_intr <= 1'b1;
    else if (launch.valid) pending_intr <= 1'b0;
  end

  // Plain `assign`, matching every other `launch.*` field: Icarus conflicts a packed struct's procedural and continuous drivers even on disjoint fields.
  assign launch.rvfi.pc_wdata = resolved_target;
  assign launch.rvfi.insn = in_instr;
  assign launch.rvfi.pc_rdata = in_pc;
  assign launch.rvfi.trap = trap_pending;
  assign launch.rvfi.intr = pending_intr;
  assign launch.rvfi.mem_fault = in_imem_fault || load_access_fault || store_access_fault;
  assign launch.rvfi.mem_fault_rmask = {4{load_access_fault || (store_access_fault && is_amo)}};
  assign launch.rvfi.mem_fault_wmask = store_access_fault ? ls_fault_wstrb : 4'b0;
  assign launch.rvfi.mem_fault_addr = mem_fault_word_addr;
  assign launch.rvfi.rs1_addr = rvfi_rs1_valid ? in_rs1 : 5'b0;
  assign launch.rvfi.rs2_addr = rvfi_rs2_valid ? in_rs2 : 5'b0;
  // The forwarded value: the monitor checks rd_wdata against these two fields, so the
  // unforwarded operand would make every forwarded retire self-contradictory.
  assign launch.rvfi.rs1_rdata = rvfi_rs1_valid ? fwd_rs1_val : 32'b0;
  assign launch.rvfi.rs2_rdata = rvfi_rs2_valid ? fwd_rs2_val : 32'b0;
  assign launch.rvfi.csr_mcycle   = csr_rvfi_mcycle;
  assign launch.rvfi.csr_minstret = csr_rvfi_minstret;
  assign launch.rvfi.csr_mscratch = csr_rvfi_mscratch;
 `ifdef RISCV_FORMAL_CSR_MCAUSE
  assign launch.rvfi.csr_mcause   = csr_rvfi_mcause;
 `endif
 `endif

  logic in_has_result;
  assign in_has_result = launch.is_add || launch.is_sub || launch.is_xor || launch.is_or ||
    launch.is_and || launch.is_sll || launch.is_slt || launch.is_sltu || launch.is_srl ||
    launch.is_sra || launch.is_mul || launch.is_mulh || launch.is_mulhu || launch.is_mulhsu ||
    launch.is_div || launch.is_divu || launch.is_rem || launch.is_remu;

  logic [32:0] alu_sub;
  logic        alu_ltu, alu_lt;
  assign alu_sub = {1'b0, alu_rs1} - {1'b0, alu_rs2};
  assign alu_ltu = alu_sub[32];
  assign alu_lt  = (alu_rs1[31] ^ alu_rs2[31]) ? alu_rs1[31] : alu_sub[32];

  // `in_rs2` is an immediate shift's shamt field; a register shift's amount is alu_rs2's low bits.
  logic [4:0] shift_amt;
  assign shift_amt = in_is_math_imm ? in_rs2 : alu_rs2[4:0];

  logic [31:0] rs1_rev, shift_src, shift_res, shift_rev;
  logic        shift_fill;
  logic signed [32:0] shift_wide;
  for (genvar i = 0; i < 32; i++) begin : l_shift_rev
    assign rs1_rev[i]   = alu_rs1[31-i];
    assign shift_rev[i] = shift_res[31-i];
  end
  assign shift_src  = in_is_sll ? rs1_rev : alu_rs1;
  assign shift_fill = in_is_sra ? alu_rs1[31] : 1'b0;
  assign shift_wide = $signed({shift_fill, shift_src}) >>> shift_amt;
  assign shift_res  = shift_wide[31:0];

  // The divider is unsigned; signed div/rem hand it magnitudes and restore the sign on completion.
  logic [31:0] div_x, div_y;
  assign div_x = (in_is_div || in_is_rem) && fwd_rs1_val[31] ? ~(fwd_rs1_val - 32'd1) : fwd_rs1_val;
  assign div_y = (in_is_div || in_is_rem) && fwd_rs2_val[31] ? ~(fwd_rs2_val - 32'd1) : fwd_rs2_val;

  // A zero-top-half dividend skips the 16 iterations that would just shift zeros past it.
  logic div_skip;
  assign div_skip = div_x[31:16] == 16'b0;

  logic [6:0]  mul_div_counter;
  // div_quot holds the dividend; a quotient bit shifts in as each dividend bit leaves.
  logic [31:0] div_rem, div_quot, div_divisor_n;
  logic [31:0] div_divisor;
  assign div_divisor = ~div_divisor_n;

  logic [32:0] rem_shifted, rem_sub;
  assign rem_shifted = {div_rem, div_quot[31]};
  assign rem_sub     = rem_shifted + {1'b1, div_divisor_n} + 33'd1;

  logic [31:0] div_quot_next, div_rem_next;
  assign div_quot_next = {div_quot[30:0], ~rem_sub[32]};
  assign div_rem_next  = rem_sub[32] ? rem_shifted[31:0] : rem_sub[31:0];

  logic op_is_div, op_is_divu, op_is_rem, op_is_remu, op_sign_x, op_sign_y;

  logic [31:0] div_result_mag;
  logic        div_negate;
  assign div_result_mag = (op_is_div || op_is_divu) ? div_quot_next : div_rem_next;
  assign div_negate     = (op_is_div && (op_sign_x != op_sign_y)) || (op_is_rem && op_sign_x);

 `ifdef RISCV_FORMAL_ALTOPS
  logic [31:0] div_alt_rs1, div_alt_rs2;
  assign div_alt_rs1 = div_quot;
  assign div_alt_rs2 = div_divisor;
 `endif

  logic mul_sign_x, mul_sign_y;
  assign mul_sign_x = fwd_rs1_val[31] & (in_is_mulh | in_is_mulhsu);
  assign mul_sign_y = fwd_rs2_val[31] & in_is_mulh;

  // A negative operand contributes one subtraction at bit 32: two conditional subtracts.
  logic [63:0] mul_unsigned;
  logic [31:0] mul_lo, mul_hi;
  assign mul_unsigned = fwd_rs1_val * fwd_rs2_val;
  assign mul_lo = mul_unsigned[31:0];
  assign mul_hi = mul_unsigned[63:32] - (mul_sign_x ? fwd_rs2_val : 32'b0)
                                      - (mul_sign_y ? fwd_rs1_val : 32'b0);

  always_ff @(posedge clk) begin
    if (reset) begin
      state <= init;
      out <= 0;
      mul_div_counter <= 0;
      div_rem <= 0;
      div_quot <= 0;
      div_divisor_n <= 0;
      op_is_div <= 0;
      op_is_divu <= 0;
      op_is_rem <= 0;
      op_is_remu <= 0;
      op_sign_x <= 0;
      op_sign_y <= 0;
    end else begin
      // Assigned outside the case: a divide's completing cycle must publish its own answer.
      out.rd_ready <= in_has_result;
      (* parallel_case, full_case *)
      case (state)
        init: begin
          out.valid <= launch.valid;
         `ifdef RISCV_FORMAL
          out.rvfi <= launch.rvfi;
         `endif
          out.rd <= launch.rd;
          out.rd_data <= 0;
          (* parallel_case, full_case *)
          case (1'b1)
            launch.is_add: out.rd_data <= alu_rs1 + alu_rs2;
            launch.is_sub: out.rd_data <= alu_sub[31:0];
            launch.is_sll: out.rd_data <= shift_rev;
            launch.is_slt: out.rd_data <= {31'b0, alu_lt};
            launch.is_sltu: out.rd_data <= {31'b0, alu_ltu};
            launch.is_xor: out.rd_data <= alu_rs1 ^ alu_rs2;
            launch.is_srl || launch.is_sra: out.rd_data <= shift_res;
            launch.is_or: out.rd_data <= alu_rs1 | alu_rs2;
            launch.is_and: out.rd_data <= alu_rs1 & alu_rs2;
            launch.is_mul || launch.is_mulh || launch.is_mulhu || launch.is_mulhsu: begin
             `ifndef RISCV_FORMAL_ALTOPS
              if (launch.is_mul) begin
                out.rd_data <= mul_lo;
              end else begin
                out.rd_data <= mul_hi;
              end
             `else
              (* parallel_case, full_case *)
              case (1'b1)
                launch.is_mul: out.rd_data <= (fwd_rs1_val + fwd_rs2_val) ^ 32'h5876063e;
                launch.is_mulh: out.rd_data <= (fwd_rs1_val + fwd_rs2_val) ^ 32'hf6583fb7;
                launch.is_mulhu: out.rd_data <= (fwd_rs1_val + fwd_rs2_val) ^ 32'h949ce5e8;
                launch.is_mulhsu: out.rd_data <= (fwd_rs1_val - fwd_rs2_val) ^ 32'hecfbe137;
              endcase
             `endif
            end

            launch.is_div || launch.is_divu || launch.is_rem || launch.is_remu: begin
              op_is_div <= launch.is_div;
              op_is_divu <= launch.is_divu;
              op_is_rem <= launch.is_rem;
              op_is_remu <= launch.is_remu;
              op_sign_x <= fwd_rs1_val[31];
              op_sign_y <= fwd_rs2_val[31];
             `ifndef RISCV_FORMAL_ALTOPS
              if (fwd_rs2_val == 0) begin
                if (launch.is_rem || launch.is_remu) out.rd_data <= fwd_rs1_val;
                else out.rd_data <= 32'hffffffff;
              end else if ((launch.is_div || launch.is_rem) &&
                           fwd_rs1_val == 32'h80000000 && fwd_rs2_val == 32'hffffffff) begin
                if (launch.is_div) out.rd_data <= 32'h80000000;
                else out.rd_data <= 32'b0;
              end else begin
                mul_div_counter <= div_skip ? 7'd16 : 7'd32;
                state <= divide;
                div_rem <= 0;
                div_quot <= div_skip ? {div_x[15:0], 16'b0} : div_x;
                div_divisor_n <= ~div_y;
                out.valid <= 1'b0;
              end
             `else
              mul_div_counter <= 32;
              state <= divide;
              div_rem <= 0;
              div_quot <= fwd_rs1_val;
              div_divisor_n <= ~fwd_rs2_val;
              out.valid <= 1'b0;
             `endif
            end
            default: ;
          endcase // case (1'b1)
        end // case: init

        divide: begin
         `ifndef RISCV_FORMAL_ALTOPS
          div_quot <= div_quot_next;
          div_rem  <= div_rem_next;
          mul_div_counter <= mul_div_counter - 1;
          if (mul_div_counter == 7'd1) begin
            out.rd_data <= div_negate ? -div_result_mag : div_result_mag;
            out.valid <= 1'b1;
            state <= init;
          end
         `else
          (* parallel_case, full_case *)
          case (1'b1)
            op_is_div: out.rd_data <= (div_alt_rs1 - div_alt_rs2) ^ 32'h7f8529ec;
            op_is_divu: out.rd_data <= (div_alt_rs1 - div_alt_rs2) ^ 32'h10e8fd70;
            op_is_rem: out.rd_data <= (div_alt_rs1 - div_alt_rs2) ^ 32'h8da68fa5;
            op_is_remu: out.rd_data <= (div_alt_rs1 - div_alt_rs2) ^ 32'h3138d0e1;
          endcase
          out.valid <= 1'b1;
          state <= init;
         `endif
        end // case: divide
        default: ;
      endcase
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  initial assume(reset);
  always_comb if(!clocked) assume(reset);
  initial state = init;
  always_comb if (clocked) assume(!reset);

  // D writes the whole struct `'0` on every bubble path, never just `valid`.
  always_comb if (!in_valid) assume(in == '0);

  // decoder.v's own `one_of` set: a free `in` must not manufacture a spurious trap-cause conflict.
  always_comb assume($onehot0({in_is_auipc, in_is_jal, in_is_jalr,
    in_is_beq, in_is_bne, in_is_blt, in_is_bltu, in_is_bge, in_is_bgeu,
    in_is_add, in_is_sub, in_is_xor, in_is_or, in_is_and,
    in_is_sll, in_is_slt, in_is_sltu, in_is_srl, in_is_sra,
    in_is_mul, in_is_mulh, in_is_mulhu, in_is_mulhsu,
    in_is_div, in_is_divu, in_is_rem, in_is_remu,
    in_is_lui,
    in_is_lb, in_is_lbu, in_is_lh, in_is_lhu, in_is_lw, in_is_sb, in_is_sh, in_is_sw,
    in_is_ecall, in_is_ebreak,
    in_is_csrrw, in_is_csrrs, in_is_csrrc,
    in_is_mret, in_is_wfi, in_is_fence, in_is_fencei,
    in_is_amoswap, in_is_amoadd, in_is_amoxor, in_is_amoand, in_is_amoor,
    in_is_amomin, in_is_amomax, in_is_amominu, in_is_amomaxu,
    in_is_lr, in_is_sc}));

  always_comb assume(in_is_csr_access == (in_is_csrrw || in_is_csrrs || in_is_csrrc));

  // D sign-extends every I/S-type immediate and hands an atomic a zero one (rs1 alone).
  logic [19:0] assume_immediate_hi;
  logic        assume_immediate_lo_sign;
  assign assume_immediate_hi = in_immediate[31:12];
  assign assume_immediate_lo_sign = in_immediate[11];
  always_comb if (ls_access) assume(assume_immediate_hi == {20{assume_immediate_lo_sign}});
  always_comb if (instr_atomic) assume(in_immediate == 32'b0);

  // Held so the divide proof sees stable operands; composed proofs drop this via `-formal -noassume`.
  dx_output prev_in;
  logic [31:0] prev_reg_rs1, prev_reg_rs2, prev_fwd_rs1_val, prev_fwd_rs2_val;
  logic        prev_x_busy;
  always_ff @(posedge clk) begin
    prev_in          <= in;
    prev_reg_rs1     <= reg_rs1;
    prev_reg_rs2     <= reg_rs2;
    prev_fwd_rs1_val <= fwd_rs1_val;
    prev_fwd_rs2_val <= fwd_rs2_val;
    prev_x_busy      <= x_busy;
  end
  always_comb if (clocked && prev_x_busy) begin
    assume(in == prev_in);
    assume(reg_rs1 == prev_reg_rs1);
    assume(reg_rs2 == prev_reg_rs2);
    assume(fwd_rs1_val == prev_fwd_rs1_val);
    assume(fwd_rs2_val == prev_fwd_rs2_val);
  end

  // Named continuous assigns, not part-selects inside the always_* blocks below: iverilog
  // cannot build a precise sensitivity entry for those (ADR-0037's class of defect).
  logic [31:0] alu_sub_lo;
  assign alu_sub_lo = alu_sub[31:0];
  logic rem_sub_hi, rem_shifted_hi;
  assign rem_sub_hi = rem_sub[32];
  assign rem_shifted_hi = rem_shifted[32];
  logic [1:0] mem_addr_calc_lo;
  assign mem_addr_calc_lo = mem_addr_calc[1:0];
  logic [19:0] in_immediate_hi;
  logic        in_immediate_sign;
  assign in_immediate_hi = in_immediate[31:12];
  assign in_immediate_sign = in_immediate[31];
  logic launch_valid, launch_is_amo;
  logic [4:0] launch_rd;
  logic launch_is_amoswap, launch_is_amoadd, launch_is_amoxor, launch_is_amoand,
    launch_is_amoor, launch_is_amomin, launch_is_amomax, launch_is_amominu, launch_is_amomaxu,
    launch_is_lb, launch_is_lbu, launch_is_lh, launch_is_lhu, launch_is_lw,
    launch_is_sb, launch_is_sh, launch_is_sw, launch_is_lr, launch_is_sc,
    launch_is_mul, launch_is_mulh, launch_is_mulhu, launch_is_mulhsu;
  assign launch_valid = launch.valid;
  assign launch_is_amo = launch.is_amo;
  assign launch_rd = launch.rd;
  assign launch_is_amoswap = launch.is_amoswap;
  assign launch_is_amoadd = launch.is_amoadd;
  assign launch_is_amoxor = launch.is_amoxor;
  assign launch_is_amoand = launch.is_amoand;
  assign launch_is_amoor = launch.is_amoor;
  assign launch_is_amomin = launch.is_amomin;
  assign launch_is_amomax = launch.is_amomax;
  assign launch_is_amominu = launch.is_amominu;
  assign launch_is_amomaxu = launch.is_amomaxu;
  assign launch_is_lb = launch.is_lb;
  assign launch_is_lbu = launch.is_lbu;
  assign launch_is_lh = launch.is_lh;
  assign launch_is_lhu = launch.is_lhu;
  assign launch_is_lw = launch.is_lw;
  assign launch_is_sb = launch.is_sb;
  assign launch_is_sh = launch.is_sh;
  assign launch_is_sw = launch.is_sw;
  assign launch_is_lr = launch.is_lr;
  assign launch_is_sc = launch.is_sc;
  assign launch_is_mul = launch.is_mul;
  assign launch_is_mulh = launch.is_mulh;
  assign launch_is_mulhu = launch.is_mulhu;
  assign launch_is_mulhsu = launch.is_mulhsu;
  logic [31:0] out_rd_data;
  assign out_rd_data = out.rd_data;

  logic signed [31:0] alu_ref_x, alu_ref_y;
  assign alu_ref_x = alu_rs1;
  assign alu_ref_y = alu_rs2;
  always_comb if (clocked) assert(alu_sub_lo == alu_rs1 - alu_rs2);
  always_comb if (clocked) assert(alu_ltu == (alu_rs1 < alu_rs2));
  always_comb if (clocked) assert(alu_lt == (alu_ref_x < alu_ref_y));

  logic [31:0] shift_sll_ref, shift_srl_ref;
  logic signed [31:0] shift_sra_ref;
  assign shift_sll_ref = alu_rs1 << shift_amt;
  assign shift_srl_ref = alu_rs1 >> shift_amt;
  assign shift_sra_ref = alu_ref_x >>> shift_amt;
  always_comb if (clocked && in_is_sll) assert(shift_rev == shift_sll_ref);
  always_comb if (clocked && in_is_srl) assert(shift_res == shift_srl_ref);
  always_comb if (clocked && in_is_sra) assert(shift_res == shift_sra_ref);

  always_comb
    if (clocked && div_rem < div_divisor)
      assert(rem_sub_hi == (rem_shifted < {1'b0, div_divisor}));
  always_comb
    if (clocked && div_rem < div_divisor && rem_sub_hi) assert(rem_shifted_hi == 1'b0);

  logic [32:0] rs1_sext33, rs2_sext33, rs1_zext33, rs2_zext33;
  assign rs1_sext33 = $signed(fwd_rs1_val);
  assign rs2_sext33 = $signed(fwd_rs2_val);
  assign rs1_zext33 = {1'b0, fwd_rs1_val};
  assign rs2_zext33 = {1'b0, fwd_rs2_val};
  logic [32:0] mul_op_x_ref, mul_op_y_ref;
  assign mul_op_x_ref = (in_is_mulh || in_is_mulhsu) ? rs1_sext33 : rs1_zext33;
  assign mul_op_y_ref = in_is_mulh ? rs2_sext33 : rs2_zext33;
  always_comb if (clocked) assert({mul_sign_x, fwd_rs1_val} == mul_op_x_ref);
  always_comb if (clocked) assert({mul_sign_y, fwd_rs2_val} == mul_op_y_ref);

  // Proven by components_executor; excluded from traps.sv's own composition below.
 `ifndef TRAPS_SKIP_EXEC_ARITH
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(launch_is_mul))
      assert(out_rd_data == $past(mul_lo));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(launch_is_mulh))
      assert(out_rd_data == $past(mul_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(launch_is_mulhu))
      assert(out_rd_data == $past(mul_hi));
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(launch_is_mulhsu))
      assert(out_rd_data == $past(mul_hi));

  // The Zkt constant-latency claim for the four multiplies: no operand-dependent second
  // cycle. formal/executor-zkt-probe.py is this assertion's forced-red prerequisite.
  always_ff @(posedge clk)
    if (clocked && !reset && !$past(reset) && $past(state) == init &&
        $past(launch_is_mul || launch_is_mulh || launch_is_mulhu || launch_is_mulhsu))
      assert(state == init);
 `endif

  logic [63:0] mul_result;
  assign mul_result = {mul_hi, mul_lo};
  always_comb if (clocked && fwd_rs1_val == 32'b0) assert(mul_result == 64'b0);
  always_comb if (clocked && fwd_rs2_val == 32'b0) assert(mul_result == 64'b0);
  always_comb if (clocked && fwd_rs2_val == 32'h1 && !mul_sign_y)
    assert(mul_result == {{32{mul_sign_x}}, fwd_rs1_val});
  always_comb if (clocked && fwd_rs1_val == 32'h1 && !mul_sign_x)
    assert(mul_result == {{32{mul_sign_y}}, fwd_rs2_val});

  logic [31:0] div_ghost_rs1, div_ghost_rs2;
  logic div_ghost_rs1_sign, div_ghost_rs2_sign;
  assign div_ghost_rs1_sign = div_ghost_rs1[31];
  assign div_ghost_rs2_sign = div_ghost_rs2[31];
  always_ff @(posedge clk)
    if (!reset && state == init) begin
      div_ghost_rs1 <= fwd_rs1_val;
      div_ghost_rs2 <= fwd_rs2_val;
    end

  logic [31:0] div_mag_x, div_mag_y;
  assign div_mag_x = (op_is_div || op_is_rem) && div_ghost_rs1_sign ? -div_ghost_rs1 : div_ghost_rs1;
  assign div_mag_y = (op_is_div || op_is_rem) && div_ghost_rs2_sign ? -div_ghost_rs2 : div_ghost_rs2;

  always_comb
    if (state == divide) assert($onehot({op_is_div, op_is_divu, op_is_rem, op_is_remu}));
  always_comb if (state == divide) assert(op_sign_x == div_ghost_rs1_sign);
  always_comb if (state == divide) assert(op_sign_y == div_ghost_rs2_sign);
 `ifndef TRAPS_SKIP_EXEC_ARITH
  always_comb if (state == divide) assert(div_divisor == div_mag_y);
 `endif

  always_comb if (state == divide) assert(mul_div_counter <= 32);
  always_comb if (state == divide) assert(mul_div_counter != 0);

  localparam [31:0] div_proof_cap = 32'h000000ff;
  always_comb if (state == divide) assume(div_mag_x <= div_proof_cap);
  always_comb if (state == divide) assume(div_mag_y <= div_proof_cap);

 `ifndef TRAPS_SKIP_EXEC_ARITH
  logic [5:0]  div_done;
  logic [63:0] div_quot_done, div_quot_left, div_mag_x_done, div_mag_x_left;
  assign div_done       = 6'd32 - mul_div_counter[5:0];
  assign div_quot_done  = {32'b0, div_quot} & ((64'b1 << div_done) - 64'b1);
  assign div_quot_left  = {32'b0, div_quot} >> div_done;
  assign div_mag_x_done = {32'b0, div_mag_x} >> mul_div_counter;
  assign div_mag_x_left = {32'b0, div_mag_x} & ((64'b1 << mul_div_counter) - 64'b1);
  always_comb
    if (state == divide)
      assert(div_quot_done * {32'b0, div_divisor} + {32'b0, div_rem} == div_mag_x_done);
  always_comb if (state == divide) assert(div_rem < div_divisor);
  always_comb if (state == divide) assert(div_quot_left == div_mag_x_left);

  logic signed [31:0] div_srs1, div_srs2;
  assign div_srs1 = $signed(div_ghost_rs1);
  assign div_srs2 = $signed(div_ghost_rs2);
  logic signed [31:0] div_q, div_r;
  assign div_q = div_srs1 / div_srs2;
  assign div_r = div_srs1 % div_srs2;

  logic [31:0] divu_ref, remu_ref, div_ref, rem_ref;
  assign divu_ref = (div_ghost_rs2 == 0) ? 32'hffffffff : (div_ghost_rs1 / div_ghost_rs2);
  assign remu_ref = (div_ghost_rs2 == 0) ? div_ghost_rs1 : (div_ghost_rs1 % div_ghost_rs2);
  assign div_ref  = (div_ghost_rs2 == 0) ? 32'hffffffff : div_q;
  assign rem_ref  = (div_ghost_rs2 == 0) ? div_ghost_rs1 : div_r;

  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(op_is_divu))
      assert(out_rd_data == divu_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(op_is_remu))
      assert(out_rd_data == remu_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(op_is_div))
      assert(out_rd_data == div_ref);
  always_ff @(posedge clk)
    if (clocked && !reset && $past(state) == divide && state == init && $past(op_is_rem))
      assert(out_rd_data == rem_ref);
 `endif

  // The Zkt isolation claim's other half: `ls_access` is exactly the eight base
  // load/store encodings. formal/decoder-zkt-probe.py is this assertion's forced-red
  // prerequisite.
  always_comb if (clocked)
    assert(ls_access == (in_is_lb || in_is_lbu || in_is_lh || in_is_lhu ||
      in_is_lw || in_is_sb || in_is_sh || in_is_sw));

  always_comb if (clocked && !launch_valid) assert(launch_rd == 0);
  always_comb if (clocked)
    assert(launch_is_amo == (launch_is_amoswap || launch_is_amoadd || launch_is_amoxor ||
      launch_is_amoand || launch_is_amoor || launch_is_amomin || launch_is_amomax ||
      launch_is_amominu || launch_is_amomaxu));

  always_comb if (clocked && instr_atomic) assert(mem_addr_calc == atomic_addr);

  always_comb if (clocked && ls_access) assert(in_immediate_hi == {20{in_immediate_sign}});

  // The trap-cause priority chain: exactly one arm decides, in this order, whenever the
  // word alone (no interrupt, no fetch fault) is what is deciding.
  logic word_decides;
  assign word_decides = !in_is_interrupt && !in_imem_fault;
  always_comb if (clocked && in_is_interrupt) assert(trap_cause == CAUSE_MACHINE_TIMER);
  always_comb if (clocked && !in_is_interrupt && in_imem_fault) assert(trap_cause == CAUSE_INSTRUCTION_FAULT);
  always_comb if (clocked && word_decides && instr_illegal) assert(trap_cause == CAUSE_ILLEGAL_INSTRUCTION);
  always_comb if (clocked && word_decides && in_is_ebreak) assert(trap_cause == CAUSE_BREAKPOINT);
  always_comb if (clocked && word_decides && in_is_ecall) assert(trap_cause == CAUSE_ECALL_M);
  always_comb if (clocked && word_decides && load_misaligned) assert(trap_cause == CAUSE_LOAD_MISALIGNED);
  always_comb if (clocked && word_decides && store_misaligned) assert(trap_cause == CAUSE_STORE_MISALIGNED);
  always_comb if (clocked && word_decides && load_access_fault)
    assert(trap_cause == CAUSE_LOAD_ACCESS_FAULT);
  always_comb if (clocked && word_decides && store_access_fault)
    assert(trap_cause == CAUSE_STORE_ACCESS_FAULT);
  always_comb if (clocked && !trap_taken) assert(trap_cause == 32'b0);

  // X is the single commit point: a trap redirects to mtvec and an mret to mepc, both
  // same-cycle claims (X owns no registered pc of its own for a $past version to check).
  always_comb if (clocked && trap_entry) assert(redirect_target == mtvec);
  always_comb if (clocked && mret_entry) assert(redirect_target == mepc);
  always_comb if (clocked && trap_entry) begin
    assert(launch_rd == 5'b0);
    assert(!launch_is_lb && !launch_is_lbu && !launch_is_lh && !launch_is_lhu && !launch_is_lw);
    assert(!launch_is_sb && !launch_is_sh && !launch_is_sw);
    assert(!launch_is_amo && !launch_is_lr && !launch_is_sc);
  end

  always_comb if (clocked && trap_taken) assert(!instret && !csr_wen && !csr_ren);
  always_comb if (clocked) assert(!(trap_entry && mret_entry));
  // NOT asserted here: "in_is_interrupt implies trap_entry" depends on D never handing X
  // an interrupt while x_busy holds -- a claim about D's own behavior this module cannot
  // see standalone. formal/traps.sv checks it composed.

  logic signed [31:0] cmp_ref_x, cmp_ref_y;
  assign cmp_ref_x = fwd_rs1_val;
  assign cmp_ref_y = fwd_rs2_val;
  always_comb if (clocked) assert(cmp_eq == (fwd_rs1_val == fwd_rs2_val));
  always_comb if (clocked) assert(cmp_ltu == (fwd_rs1_val < fwd_rs2_val));
  always_comb if (clocked) assert(cmp_lt == (cmp_ref_x < cmp_ref_y));
  always_comb if (clocked) assert(mem_addr_low == mem_addr_calc_lo);
 `endif
endmodule
