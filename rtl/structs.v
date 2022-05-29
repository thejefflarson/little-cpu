`default_nettype none

typedef struct packed {
  logic [31:0] pc;
  logic [31:0] instr;
} fetcher_output;

typedef struct packed {
  logic [4:0]  rd;
  logic [4:0]  reg_rs1;
  logic [4:0]  reg_rs2;
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
  logic [4:0]  rd;
  logic [31:0] rd_data;
  logic [31:0] mem_addr;
  logic [31:0] mem_data;
  logic        is_lui;
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
  logic [4:0] rd;
  logic [31:0] rd_data;
} accessor_output;

typedef struct packed {
  logic        wen;
  logic [31:0] waddr;
  logic [31:0] wdata;
} writeback_output;
