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
  // Exactly the CSRs formal/checks.cfg's `[csrs]` list names, captured in
  // decode -- the one stage that knows them, since ADR-0005 reads and
  // commits every CSR access there -- and forwarded on the same valid-bit
  // protocol as everything else above (ADR-0006: no new plumbing concept, a
  // bubble zeroes them for free and a stall holds them for free).
  rvfi_csr64   csr_mcycle;
  rvfi_csr64   csr_minstret;
  rvfi_csr32   csr_mscratch;
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
  logic        is_lui;
  logic        is_lb;
  logic        is_lbu;
  logic        is_lhu;
  logic        is_lh;
  logic        is_lw;
  logic        is_sb;
  logic        is_sh;
  logic        is_sw;
  logic        is_ecall;
  logic        is_ebreak;
  // ADR-0005's decode-side record of a Zicsr access: which CSR, and whether
  // the operand was a zimm rather than rs1. No stage after decode reads
  // either -- a CSR access is read, computed and committed entirely in
  // decode (rtl/csrs.v is the decoder's sibling, not a pipeline stage), and
  // the *result* reaches rd through `is_add` like lui/jal/auipc already do.
  // They are carried anyway, per ADR-0005, because they are the only place
  // downstream a reader (or a waveform) can tell a CSR access apart from the
  // add it is disguised as.
  //
  // What is deliberately NOT here: `is_csrrw`/`is_csrrs`/`is_csrrc`, which
  // this struct used to carry with rtl/executor.v:198 an empty statement for
  // them. They cannot coexist with the `is_add` pass-through: the executor's
  // op select is one `(* parallel_case *) case (1'b1)`, and setting both
  // `is_add` and a CSR flag would make two arms match at once -- a lie to
  // synthesis, not merely redundant.
  logic [11:0] csr_addr;
  logic        is_csr_imm;
} decoder_output;

typedef struct packed {
  logic        valid;
 `ifdef RISCV_FORMAL
  rvfi_shadow  rvfi;
 `endif
  logic [4:0]  rd;
  logic [31:0] rd_data;
  logic [31:0] mem_addr;
  logic [31:0] mem_data;
  logic        is_lb;
  logic        is_lbu;
  logic        is_lh;
  logic        is_lhu;
  logic        is_lw;
  logic        is_sb;
  logic        is_sh;
  logic        is_sw;
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

typedef struct packed {
  logic        valid;
  logic        wen;
  logic [31:0] waddr;
  logic [31:0] wdata;
} writeback_output;
`endif
