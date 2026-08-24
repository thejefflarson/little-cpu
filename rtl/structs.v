`default_nettype none
`ifndef STRUCTS_V
`define STRUCTS_V
// ADR-0004 / ADR-0009: every inter-stage struct carries a `valid` bit. A
// bubble is `valid = 0` with the rest of the struct zeroed (reset does this
// unconditionally; the stall-only interlock does it whenever a stage
// withholds a real instruction). Retire is `valid` reaching writeback.
typedef struct packed {
  logic        valid;
  logic [31:0] pc;
  logic [31:0] instr;
  // The 32 bits sitting at pc + 2 or pc + 4, whichever follows `instr`, masked
  // above the low half when they are a compressed instruction. Decode reads
  // register numbers out of it a cycle early and checks what it read, so this is
  // never decoded and is not required to be an instruction at all.
  logic [31:0] next_instr;
} fetcher_output;

// ADR-0006: RVFI via per-stage shadow payloads rather than a separate retire
// tracker. This carries the fields decode alone knows (the instruction word,
// the pc this instruction was fetched at and the pc it hands off to the next
// one, and the register-file read side of rs1/rs2) down through the pipeline
// unchanged, riding the same struct/valid-bit protocol as everything else --
// so a bubble zeroes it for free and a stall holds it for free. Everything
// mem-related is captured separately in rtl/accessor.v, where it's actually
// known (see accessor_output below). `ifdef`'d out of synthesis so it costs
// no LUTs (ADR-0006): none of this exists unless
// RISCV_FORMAL is defined.
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
  // ADR-0028: `rvfi_trap` for this instruction. Decode is the one stage that
  // knows it -- every trap is detected and committed there -- so it rides down
  // with the rest of the shadow rather than being recomputed at retire. A
  // trapping instruction still retires (`valid` reaching writeback); it just
  // retires having
  // architecturally done nothing except redirect the PC.
  logic        trap;
  // `rvfi_intr`: this instruction is the first of a handler that no earlier
  // retire handed off to. Only an interrupt sets it -- an exception rides out
  // on the faulting instruction's own retire, which reports mtvec in pc_wdata
  // and leaves the chain unbroken. Both sim legs' monitor and riscv-formal's
  // two pc checks read it to stop expecting continuity across that gap.
  logic        intr;
  // `rvfi_mem_fault`: the platform had no memory at this instruction's address.
  // Decode is the one stage that knows it -- the fetch that failed is the one it
  // is looking at, and an atomic the data memory refuses never reaches the bus
  // either -- so all three ride down here rather than coming from
  // rtl/accessor.v with the rest of the mem_* report.
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
  // decode -- the one stage that knows them, since ADR-0005 reads and
  // commits every CSR access there -- and forwarded on the same valid-bit
  // protocol as everything else above (ADR-0006: no new plumbing concept, a
  // bubble zeroes them for free and a stall holds them for free).
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
  // Zba's sh2add: rd = rs2 + (rs1 << 2). rtl/decoder.v hands rs1 through
  // unshifted, the same as every other arm here; rtl/executor.v is the one
  // reader that shifts it, ahead of the adder it already has.
  logic        is_sh2add;
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
  // What is deliberately NOT here, and must not come back:
  //
  //   `is_csrrw`/`is_csrrs`/`is_csrrc`, which this struct used to carry with
  //   rtl/executor.v an empty statement for them. They cannot coexist with
  //   ADR-0005's `is_add` pass-through: the executor's op select is one
  //   `(* parallel_case *) case (1'b1)`, and setting both `is_add` and a CSR
  //   flag would make two arms match at once -- a lie to synthesis, not
  //   merely redundant.
  //
  //   `csr_addr`/`is_csr_imm`, which ADR-0034 kept as scaffolding with an
  //   explicit sunset condition: strike them in the trap-entry change if
  //   neither has acquired a downstream consumer by then. Neither did --
  //   trap detection, CSR read/write and trap commit all happen in decode --
  //   so they are gone. Thirteen bits of struct that nothing reads is the
  //   incidental machinery this project's stated goal warns against.
  //
  //   `is_lui`, which took the same route the CSR read did: decode hands the
  //   executor the immediate as rs1 with rs2 zeroed, so an add produces it and
  //   a flag of its own would be a second arm matching the same instruction.
  //
  //   `is_ecall`/`is_ebreak`. Everything that sets them raises `trap_pending`,
  //   and rtl/decoder.v's trap arm clears every execution flag on a trapping
  //   issue -- so both were constant zero at this boundary, with an empty
  //   executor arm and a one-hot assumption as their only readers.
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
  // the only stage that knows the real rmask/wmask/wdata/rdata (ADR-0006).
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
