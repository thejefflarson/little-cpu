`default_nettype none
`ifndef STRUCTS_V
`define STRUCTS_V
typedef struct packed {
  logic        valid;
  logic [31:0] pc;
  logic [31:0] instr;
  logic [31:0] next_instr;  // The raw 32 bits following `instr`, not necessarily an instruction.
} fetcher_output;

`ifdef RISCV_FORMAL
typedef struct packed {
  logic [63:0] rmask;
  logic [63:0] wmask;
  logic [63:0] rdata;
  logic [63:0] wdata;
} rvfi_csr64;

typedef struct packed {
  logic [31:0] rmask;
  logic [31:0] wmask;
  logic [31:0] rdata;
  logic [31:0] wdata;
} rvfi_csr32;

typedef struct packed {
  logic [31:0] insn;
  logic [31:0] pc_rdata;
  logic [31:0] pc_wdata;
  logic [4:0]  rs1_addr;
  logic [4:0]  rs2_addr;
  logic [31:0] rs1_rdata;
  logic [31:0] rs2_rdata;
  logic        trap;
  // Only an interrupt sets this. An exception reports mtvec in the faulting instruction's
  // own pc_wdata, so the pc chain stays unbroken there.
  logic        intr;
  // The platform had no memory at an address this instruction touched, so its address and
  // masks are reported from here rather than by the accessor it never reached.
  logic        mem_fault;
  logic [3:0]  mem_fault_rmask;
  logic [3:0]  mem_fault_wmask;
  logic [31:0] mem_fault_addr;
  rvfi_csr64   csr_mcycle;
  rvfi_csr64   csr_minstret;
  rvfi_csr32   csr_mscratch;
 `ifdef RISCV_FORMAL_CSR_MCAUSE
  rvfi_csr32   csr_mcause;
 `endif
} rvfi_shadow;
`endif

// D's register into X: `rs1`/`rs2` are register NUMBERS, and `instr` rides along so X
// can pull the CSR-immediate's uimm field and report RVFI's `insn`/`trap_tval`.
typedef struct packed {
  logic        valid;
  logic        is_interrupt;  // the one-cycle interrupt bubble D injects; else all zero
  logic        imem_fault;
  logic [31:0] pc;
  logic [31:0] instr;
  logic [31:0] immediate;
  logic [4:0]  rd;
  logic [4:0]  rs1;
  logic [4:0]  rs2;
  logic        is_add;
  logic        is_sub;
  logic        is_xor;
  logic        is_or;
  logic        is_and;
  logic        is_mul;
  logic        is_mulh;
  logic        is_mulhu;
  logic        is_mulhsu;
  logic        is_div;
  logic        is_divu;
  logic        is_rem;
  logic        is_remu;
  logic        is_sll;
  logic        is_slt;
  logic        is_sltu;
  logic        is_srl;
  logic        is_sra;
  logic        is_lb;
  logic        is_lbu;
  logic        is_lhu;
  logic        is_lh;
  logic        is_lw;
  logic        is_sb;
  logic        is_sh;
  logic        is_sw;
  logic        is_amoswap;
  logic        is_amoadd;
  logic        is_amoxor;
  logic        is_amoand;
  logic        is_amoor;
  logic        is_amomin;
  logic        is_amomax;
  logic        is_amominu;
  logic        is_amomaxu;
  logic        is_lr;
  logic        is_sc;
  logic        is_auipc;
  logic        is_lui;
  logic        is_jal;
  logic        is_jalr;
  logic        is_beq;
  logic        is_bne;
  logic        is_blt;
  logic        is_bltu;
  logic        is_bge;
  logic        is_bgeu;
  logic        is_ecall;
  logic        is_ebreak;
  logic        is_mret;
  logic        is_wfi;
  logic        is_fence;
  logic        is_fencei;
  logic        is_csrrw;
  logic        is_csrrs;
  logic        is_csrrc;
  logic        is_csr_imm;
  logic        is_csr_access;
  // True for addi..srai and their compressed forms: `rs2` is a shamt/immediate, not a register.
  logic        is_math_imm;
  // Set when this operand's producer is `out` and will publish a ready result in
  // executor_out one cycle later, which X then selects over the regfile's answer.
  logic        fwd_rs1;
  logic        fwd_rs2;
} dx_output;

typedef struct packed {
  logic        valid;
 `ifdef RISCV_FORMAL
  rvfi_shadow  rvfi;
 `endif
  logic [4:0]  rd;
  logic [31:0] rs1;
  logic [31:0] rs2;
  logic [31:0] mem_addr;
  logic        is_valid_instr;
  logic        is_add;
  logic        is_sub;
  logic        is_xor;
  logic        is_or;
  logic        is_and;
  logic        is_mul;
  logic        is_mulh;
  logic        is_mulhu;
  logic        is_mulhsu;
  logic        is_div;
  logic        is_divu;
  logic        is_rem;
  logic        is_remu;
  logic        is_sll;
  logic        is_slt;
  logic        is_sltu;
  logic        is_srl;
  logic        is_sra;
  logic        is_lb;
  logic        is_lbu;
  logic        is_lhu;
  logic        is_lh;
  logic        is_lw;
  logic        is_sb;
  logic        is_sh;
  logic        is_sw;
  logic        is_amo;
  logic        is_amoswap;
  logic        is_amoadd;
  logic        is_amoxor;
  logic        is_amoand;
  logic        is_amoor;
  logic        is_amomin;
  logic        is_amomax;
  logic        is_amominu;
  logic        is_amomaxu;
  logic        is_lr;
  logic        is_sc;
} decoder_output;

typedef struct packed {
  logic        valid;
 `ifdef RISCV_FORMAL
  rvfi_shadow  rvfi;
 `endif
  logic [4:0]  rd;
  logic [31:0] rd_data;
  logic        rd_ready;
} executor_output;

typedef struct packed {
  logic        valid;
 `ifdef RISCV_FORMAL
  rvfi_shadow  rvfi;
  logic [31:0] rvfi_mem_addr;
  logic [3:0]  rvfi_mem_rmask;
  logic [3:0]  rvfi_mem_wmask;
  logic [31:0] rvfi_mem_rdata;
  logic [31:0] rvfi_mem_wdata;
 `endif
  logic [4:0] rd;
  logic [31:0] rd_data;
} accessor_output;
`endif
