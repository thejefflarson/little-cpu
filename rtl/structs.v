`default_nettype none
`ifndef STRUCTS_V
`define STRUCTS_V
// Every inter-stage struct carries a `valid` bit: a bubble is `valid = 0` with
// the rest zeroed, and retire is `valid` reaching writeback.
typedef struct packed {
  logic        valid;
  logic [31:0] pc;
  logic [31:0] instr;
  // The raw 32 bits at pc + 2 or pc + 4, whichever follows `instr`. Decode
  // reads a register-number guess out of it a cycle early; it need not be an
  // instruction.
  logic [31:0] next_instr;
} fetcher_output;

// RVFI rides each stage struct as a shadow payload the core never reads, so a
// bubble zeroes it and a stall holds it with no plumbing of its own.
`ifdef RISCV_FORMAL
// The counters are 64-bit because mcycle/mcycleh are one RVFI CSR, not two.
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
  // A trapping instruction still retires; it has done nothing but redirect the
  // PC.
  logic        trap;
  // Set only on the first instruction of an interrupt handler. An exception
  // reports mtvec in the faulting instruction's own pc_wdata, so the pc chain
  // stays unbroken there.
  logic        intr;
  // The platform had no memory at an address this instruction touched. The
  // access never reached the accessor, so its address and masks are reported
  // from here: both masks clear is a refused fetch, a read mask alone a load
  // or lr.w, any write mask a store, sc.w or AMO.
  logic        mem_fault;
  logic [3:0]  mem_fault_rmask;
  logic [3:0]  mem_fault_wmask;
  logic [31:0] mem_fault_addr;
  // Captured in decode at issue, the stage that reads and commits every CSR
  // access.
  rvfi_csr64   csr_mcycle;
  rvfi_csr64   csr_minstret;
  rvfi_csr32   csr_mscratch;
 `ifdef RISCV_FORMAL_CSR_MCAUSE
  rvfi_csr32   csr_mcause;
 `endif
} rvfi_shadow;
`endif

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
  // The OR of the nine `is_amo*` flags below; lr.w and sc.w are not in it.
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
  // These flags must stay mutually exclusive: the executor selects over them
  // with `(* parallel_case *)`, and a CSR read and `lui` already ride `is_add`
  // with the value handed over as rs1. No flag names a trap either: the trap
  // arm clears `is_add` through `is_sc` on a trapping issue, so one like
  // `is_ecall` would read constant zero.
} decoder_output;

// The bus request goes out from `decoder_output` during the executor's cycle,
// so nothing memory-related needs to reach here.
typedef struct packed {
  logic        valid;
 `ifdef RISCV_FORMAL
  rvfi_shadow  rvfi;
 `endif
  logic [4:0]  rd;
  logic [31:0] rd_data;
  // `rd_data` is this instruction's real result rather than a placeholder a
  // later stage still has to fill in. Every arithmetic op sets it the same
  // cycle it issues; a load, a store, fence, wfi and every atomic do not --
  // rtl/accessor.v produces theirs, or there is none. rtl/decoder.v's
  // forwarding path is the one reader: it may only take `rd_data` from here
  // when this bit says the value behind it is the finished one.
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
