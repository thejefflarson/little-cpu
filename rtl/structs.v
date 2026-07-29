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
// no LUTs (the constraint note on JEF-628): none of this exists unless
// RISCV_FORMAL is defined.
`ifdef RISCV_FORMAL
typedef struct packed {
  logic [31:0] insn;
  logic [31:0] pc_rdata;
  logic [31:0] pc_wdata;
  logic [4:0]  rs1_addr;
  logic [4:0]  rs2_addr;
  logic [31:0] rs1_rdata;
  logic [31:0] rs2_rdata;
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
  logic        is_csrrw;
  logic        is_csrrs;
  logic        is_csrrc;
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
  // the only stage that knows the real rmask/wmask/wdata/rdata (JEF-628).
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
