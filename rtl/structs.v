`default_nettype none
`ifndef STRUCTS_V
`define STRUCTS_V
// Every inter-stage struct carries a `valid` bit. A bubble is `valid = 0` with
// the rest of the struct zeroed (reset does this unconditionally; a stalled
// stage does it whenever it withholds a real instruction). Retire is `valid`
// reaching writeback.
typedef struct packed {
  logic        valid;
  logic [31:0] pc;
  logic [31:0] instr;
  // The 32 bits sitting at pc + 2 or pc + 4, whichever follows `instr`, raw:
  // rtl/regsel.v masks a compressed one's upper half itself. Decode reads
  // register numbers out of it a cycle early and checks what it read, so this is
  // never decoded and is not required to be an instruction at all.
  logic [31:0] next_instr;
} fetcher_output;

// RVFI rides each stage struct as a shadow payload, so a bubble zeroes it and
// a stall holds it with no plumbing of its own. It carries what decode alone
// knows -- the word, the pc fetched at and the pc handed off to, the rs1/rs2
// read side; the mem_* report is captured in rtl/accessor.v, where it is
// known. None of this exists unless RISCV_FORMAL is defined, and
// `make -C formal nonperturbation` is what says the core never reads it.
`ifdef RISCV_FORMAL
// One RVFI CSR report: what the retiring instruction read and wrote of one
// CSR. Two widths because riscv-formal's own port widths differ -- the
// counters are 64-bit (mcycle/mcycleh are one RVFI CSR, not two), everything
// else is XLEN. rtl/csrs.v builds these; the decoder latches them at issue.
// Field ORDER is not arbitrary: rtl/writeback.v names the fields, but a
// packed struct is still a bit vector, so keep rmask/wmask/rdata/wdata
// together and change them in one place.
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
  // `rvfi_trap` for this instruction. Decode is the one stage that knows it --
  // every trap is detected and committed there -- so it rides down with the
  // rest of the shadow rather than being recomputed at retire. A trapping
  // instruction still retires (`valid` reaching writeback); it just retires
  // having architecturally done nothing except redirect the PC.
  logic        trap;
  // `rvfi_intr`: this instruction is the first of a handler that no earlier
  // retire handed off to. Only an interrupt sets it -- an exception rides out
  // on the faulting instruction's own retire, which reports mtvec in pc_wdata
  // and leaves the chain unbroken. Both sim legs' monitor and riscv-formal's
  // two pc checks read it to stop expecting continuity across that gap.
  logic        intr;
  // `rvfi_mem_fault`: the platform had no memory at an address this instruction
  // touched. Decode is the one stage that knows it -- the fetch that failed is
  // the one it is looking at; an atomic whose address rtl/memory.v refuses
  // (its `atomic_supported` input) never reaches the bus; and neither does a
  // plain load or store, which decode tests against its own elaboration-time
  // copy of the map and never issues once that test fails -- so the flag and
  // its two masks ride down here rather than coming from rtl/accessor.v with
  // the rest of the mem_* report.
  //
  // The two masks are how checks/rvfi_fault_check.sv tells the three refusals
  // apart, and it reads them as the cause: both clear is a fetch and cause 1, a
  // read mask alone is cause 5, and any write mask is cause 7 whatever the read
  // mask says -- which is why an AMO sets both and gets the store cause.
  //
  // The address goes with them, because a refused access never reaches
  // rtl/accessor.v and so has nothing to report through `rvfi_mem_addr` with
  // the rest of the mem_* report. checks/rvfi_insn_check.sv compares it against
  // the spec model's own effective address whenever `mem_fault` is set, which is
  // what grades the region decode's arithmetic rather than only its verdict.
  logic        mem_fault;
  logic [3:0]  mem_fault_rmask;
  logic [3:0]  mem_fault_wmask;
  logic [31:0] mem_fault_addr;
  // Exactly the CSRs formal/checks.cfg's `[csrs]` list names, captured in
  // decode -- the one stage that knows them, since every CSR access is read
  // and committed there -- and forwarded on the same valid-bit protocol as
  // everything above.
  rvfi_csr64   csr_mcycle;
  rvfi_csr64   csr_minstret;
  rvfi_csr32   csr_mscratch;
 `ifdef RISCV_FORMAL_CSR_MCAUSE
  // Not one of those, and carried for one consumer: checks/rvfi_fault_check.sv
  // asserts that a refused access wrote the whole of mcause with the cause the
  // spec names. rtl/csrs.v exports it under the same macro.
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
  // Is this one of the nine AMO functions? Decoded once here rather than ORed
  // back together by each reader: rtl/decoder.v needs it to spend the atomic
  // write cycle and rtl/accessor.v to route the read-modify-write, and the two
  // spellings of one nine-term OR were a fetch-loop input that had already been
  // computed a stage earlier.
  logic        is_amo;
  // The nine AMO functions plus lr.w and sc.w. One flag each rather than the
  // encoding's funct5, because rtl/accessor.v's result mux is a
  // `(* parallel_case *)` over them and a marking is spent against a $onehot0
  // check of the exact arm list. rtl/executor.v passes all eleven through
  // untouched: an AMO's operands are the memory word and rs2, and the word does
  // not exist until the accessor has it.
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
  // No flag here may match an instruction `is_add` already matches: a CSR read
  // and `lui` both ride the add pass-through with the value handed over as rs1,
  // and rtl/executor.v's op select is a `(* parallel_case *)` over these flags,
  // so a second arm matching the same instruction is a lie to synthesis rather
  // than redundancy. Nothing here names a trap either: the trap arm clears
  // every execution flag on a trapping issue, so an `is_ecall` would be
  // constant zero.
} decoder_output;

// Nothing memory-related survives this far. The bus transaction goes out from
// `decoder_output` during the executor's own cycle, so the address, the store
// data and the width are all spent a stage earlier than this struct and only
// rtl/accessor.v's own registered copy of them outlives the request.
typedef struct packed {
  logic        valid;
 `ifdef RISCV_FORMAL
  rvfi_shadow  rvfi;
 `endif
  logic [4:0]  rd;
  logic [31:0] rd_data;
} executor_output;

typedef struct packed {
  logic        valid;
 `ifdef RISCV_FORMAL
  rvfi_shadow  rvfi;
  // Everything mem-related is captured here, not forwarded from
  // executor_output: this is the stage that actually issues the bus
  // request and (one cycle later, for loads) sees the response, so it is
  // the only stage that knows the real rmask/wmask/wdata/rdata.
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
