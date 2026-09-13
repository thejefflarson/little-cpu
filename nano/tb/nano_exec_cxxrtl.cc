// A randomized differential oracle for nano's real multiply and divide, in the shape of
// test/exec_tb.v, but bypassing fetch/decode by poking `regs`/`instr`/`cpu_state`
// directly through cxxrtl's debug_items rather than a decoder_output struct: nano has no
// separate executor module, and no path here ever calls `$display`, which is also why
// the build passes -Wno-unused-parameter for the generated eval()'s unused `performer *`.
#include "nano_exec_rtl.cc"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

namespace {

// nano.v's cpu_state localparams, duplicated so this can drive the state machine directly.
constexpr uint32_t DECODE_INSTR = 0x3;
constexpr uint32_t FETCH_INSTR  = 0x1;

constexpr uint32_t OPCODE_OP = 0x33; // R-type, quadrant 11, opcode field 01100
constexpr uint32_t FUNCT7_M  = 0x01;

uint32_t encode_r(uint32_t funct7, uint32_t rs2, uint32_t rs1, uint32_t funct3, uint32_t rd) {
  return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | OPCODE_OP;
}

// C99 truncates toward zero, matching RISC-V DIV/REM, so these need none of exec_tb.v's
// care around Verilog's unsigned `?:` -- but ref_selftest() below still pins each one
// against a hand-computed literal, since a wrong reference model hides a defect too.
uint32_t ref_mul(uint32_t a, uint32_t b) {
  return static_cast<uint32_t>(static_cast<uint64_t>(a) * static_cast<uint64_t>(b));
}

uint32_t ref_mulh(uint32_t a, uint32_t b) {
  int64_t p = static_cast<int64_t>(static_cast<int32_t>(a)) *
              static_cast<int64_t>(static_cast<int32_t>(b));
  return static_cast<uint32_t>(static_cast<uint64_t>(p) >> 32);
}

uint32_t ref_mulhu(uint32_t a, uint32_t b) {
  uint64_t p = static_cast<uint64_t>(a) * static_cast<uint64_t>(b);
  return static_cast<uint32_t>(p >> 32);
}

uint32_t ref_mulhsu(uint32_t a, uint32_t b) {
  int64_t p = static_cast<int64_t>(static_cast<int32_t>(a)) * static_cast<int64_t>(b);
  return static_cast<uint32_t>(static_cast<uint64_t>(p) >> 32);
}

uint32_t ref_div(uint32_t a, uint32_t b) {
  if (b == 0u) return 0xffffffffu;
  if (a == 0x80000000u && b == 0xffffffffu) return 0x80000000u;
  return static_cast<uint32_t>(static_cast<int32_t>(a) / static_cast<int32_t>(b));
}

uint32_t ref_divu(uint32_t a, uint32_t b) {
  return (b == 0u) ? 0xffffffffu : (a / b);
}

uint32_t ref_rem(uint32_t a, uint32_t b) {
  if (b == 0u) return a;
  if (a == 0x80000000u && b == 0xffffffffu) return 0u;
  return static_cast<uint32_t>(static_cast<int32_t>(a) % static_cast<int32_t>(b));
}

uint32_t ref_remu(uint32_t a, uint32_t b) {
  return (b == 0u) ? a : (a % b);
}

int errors = 0;

void ref_selftest(const char *what, uint32_t got, uint32_t want) {
  if (got != want) {
    std::fprintf(stderr, "ORACLE BROKEN: %s got=%08x expected=%08x\n", what, got, want);
    errors++;
  }
}

struct Op {
  const char *name;
  uint32_t funct3;
  uint32_t (*ref)(uint32_t, uint32_t);
};

constexpr Op OPS[] = {
    {"mul", 0x0, ref_mul},     {"mulh", 0x1, ref_mulh},   {"mulhsu", 0x2, ref_mulhsu},
    {"mulhu", 0x3, ref_mulhu}, {"div", 0x4, ref_div},     {"divu", 0x5, ref_divu},
    {"rem", 0x6, ref_rem},     {"remu", 0x7, ref_remu},
};
constexpr int NUM_OPS = sizeof(OPS) / sizeof(OPS[0]);

struct Directed {
  int op;
  uint32_t a, b, expected;
  const char *why;
};

int op_index(const char *name) {
  for (int i = 0; i < NUM_OPS; i++)
    if (std::strcmp(OPS[i].name, name) == 0) return i;
  return -1;
}

} // namespace

int main() {
  ref_selftest("div(fffffff9,00000002)", ref_div(0xfffffff9u, 0x00000002u), 0xfffffffdu);
  ref_selftest("div(00000007,fffffffe)", ref_div(0x00000007u, 0xfffffffeu), 0xfffffffdu);
  ref_selftest("div(fffffff9,fffffffe)", ref_div(0xfffffff9u, 0xfffffffeu), 0x00000003u);
  ref_selftest("div(00000064,00000000)", ref_div(0x00000064u, 0x00000000u), 0xffffffffu);
  ref_selftest("div(80000000,ffffffff)", ref_div(0x80000000u, 0xffffffffu), 0x80000000u);
  ref_selftest("divu(fffffff9,00000002)", ref_divu(0xfffffff9u, 0x00000002u), 0x7ffffffcu);
  ref_selftest("divu(00000064,00000000)", ref_divu(0x00000064u, 0x00000000u), 0xffffffffu);
  ref_selftest("rem(fffffff9,00000002)", ref_rem(0xfffffff9u, 0x00000002u), 0xffffffffu);
  ref_selftest("rem(00000007,fffffffe)", ref_rem(0x00000007u, 0xfffffffeu), 0x00000001u);
  ref_selftest("rem(00000064,00000000)", ref_rem(0x00000064u, 0x00000000u), 0x00000064u);
  ref_selftest("rem(80000000,ffffffff)", ref_rem(0x80000000u, 0xffffffffu), 0x00000000u);
  ref_selftest("remu(fffffff9,00000002)", ref_remu(0xfffffff9u, 0x00000002u), 0x00000001u);
  ref_selftest("remu(00000064,00000000)", ref_remu(0x00000064u, 0x00000000u), 0x00000064u);
  ref_selftest("div(ffffffec,00000006)", ref_div(0xffffffecu, 0x00000006u), 0xfffffffdu);
  ref_selftest("rem(ffffffec,00000006)", ref_rem(0xffffffecu, 0x00000006u), 0xfffffffeu);
  ref_selftest("mulh(ffffffff,ffffffff)", ref_mulh(0xffffffffu, 0xffffffffu), 0x00000000u);
  ref_selftest("mulhsu(ffffffff,00000001)", ref_mulhsu(0xffffffffu, 0x00000001u), 0xffffffffu);
  ref_selftest("mulhu(ffffffff,ffffffff)", ref_mulhu(0xffffffffu, 0xffffffffu), 0xfffffffeu);
  if (errors != 0) {
    std::fprintf(stderr, "FAILED: the reference model is broken; no core result below "
                          "means anything\n");
    return 1;
  }

  const Directed directed[] = {
      {op_index("div"), 0x00000064u, 0x00000000u, 0xffffffffu, "div by zero"},
      {op_index("divu"), 0x00000064u, 0x00000000u, 0xffffffffu, "divu by zero"},
      {op_index("rem"), 0x00000064u, 0x00000000u, 0x00000064u, "rem by zero"},
      {op_index("remu"), 0x00000064u, 0x00000000u, 0x00000064u, "remu by zero"},
      {op_index("div"), 0xffffffecu, 0x00000000u, 0xffffffffu, "div by zero, negative dividend"},
      {op_index("rem"), 0xffffffecu, 0x00000000u, 0xffffffecu, "rem by zero, negative dividend"},
      {op_index("div"), 0x80000000u, 0xffffffffu, 0x80000000u, "div INT_MIN/-1"},
      {op_index("rem"), 0x80000000u, 0xffffffffu, 0x00000000u, "rem INT_MIN/-1"},
      {op_index("div"), 0xffffffecu, 0x00000006u, 0xfffffffdu, "div negative/positive"},
      {op_index("rem"), 0xffffffecu, 0x00000006u, 0xfffffffeu, "rem negative/positive"},
      {op_index("div"), 0x00000014u, 0xfffffffau, 0xfffffffdu, "div positive/negative"},
      {op_index("rem"), 0x00000014u, 0xfffffffau, 0x00000002u, "rem positive/negative"},
  };
  constexpr int DIRECTED_N = sizeof(directed) / sizeof(directed[0]);
  for (const auto &d : directed)
    if (d.op < 0) {
      std::fprintf(stderr, "BENCH BUG: directed vector names an unknown op\n");
      return 1;
    }

  cxxrtl_design::p_nano__exec__tb top;
  cxxrtl::debug_items items;
  top.debug_info(&items, nullptr, "");

  const cxxrtl::debug_item &instr = items.at("dut instr").at(0);
  const cxxrtl::debug_item &cpu_state = items.at("dut cpu_state").at(0);
  const cxxrtl::debug_item &trap = items.at("dut trap").at(0);
  const cxxrtl::debug_item &regs = items.at("dut regs").at(0);
  if (regs.type != cxxrtl::debug_item::MEMORY || regs.depth < 16) {
    std::fprintf(stderr, "BENCH BUG: 'dut regs' is not the 16-entry register file expected\n");
    return 1;
  }

  auto half_step = [&](bool clk) {
    top.p_clk.set<bool>(clk);
    top.step();
  };

  top.p_reset.set<bool>(true);
  half_step(false);
  half_step(true);
  top.p_reset.set<bool>(false);
  half_step(false); // run_op() pokes state on a low clock; leave it there before the first one

  auto run_op = [&](int op, uint32_t a, uint32_t b) -> uint32_t {
    regs.curr[1] = a;
    regs.curr[2] = b;
    instr.curr[0] = encode_r(FUNCT7_M, /*rs2=*/2, /*rs1=*/1, OPS[op].funct3, /*rd=*/4);
    cpu_state.curr[0] = DECODE_INSTR;

    for (int guard = 0; guard < 128; guard++) {
      half_step(true);
      half_step(false);
      if (trap.curr[0] != 0) {
        std::fprintf(stderr, "TRAP: %s(rs1=%08x rs2=%08x) trapped instead of retiring\n",
                     OPS[op].name, a, b);
        std::exit(1);
      }
      if (cpu_state.curr[0] == FETCH_INSTR) return regs.curr[4];
    }
    std::fprintf(stderr, "HANG: %s(rs1=%08x rs2=%08x) never reached fetch_instr\n",
                 OPS[op].name, a, b);
    std::exit(1);
  };

  auto check = [&](const char *op_name, uint32_t a, uint32_t b, uint32_t got,
                    uint32_t expected, const char *why = nullptr) {
    if (got != expected) {
      std::fprintf(stderr, "MISMATCH %s%s%s%s rs1=%08x rs2=%08x got=%08x expected=%08x\n",
                   op_name, why ? "(" : "", why ? why : "", why ? ")" : "", a, b, got, expected);
      errors++;
    }
  };

  for (const auto &d : directed)
    check(OPS[d.op].name, d.a, d.b, run_op(d.op, d.a, d.b), d.expected, d.why);

  std::mt19937 rng(0xd591a5edu);
  std::uniform_int_distribution<uint32_t> dist;
  constexpr int RANDOM_VECTORS = 2000;
  int vec_count[NUM_OPS] = {0};
  for (int i = 0; i < RANDOM_VECTORS; i++) {
    uint32_t a = dist(rng);
    uint32_t b = dist(rng);
    for (int op = 0; op < NUM_OPS; op++) {
      uint32_t got = run_op(op, a, b);
      check(OPS[op].name, a, b, got, OPS[op].ref(a, b));
      vec_count[op]++;
    }
  }
  for (const auto &d : directed) vec_count[d.op]++;

  if (errors != 0) {
    std::fprintf(stderr, "FAILED: %d mismatches\n", errors);
    return 1;
  }
  std::printf("PASSED: 0 mismatches over %d directed and %d randomized-per-op vectors\n",
              DIRECTED_N, RANDOM_VECTORS);
  for (int op = 0; op < NUM_OPS; op++)
    std::printf("        %-7s %d\n", OPS[op].name, vec_count[op]);
  return 0;
}
