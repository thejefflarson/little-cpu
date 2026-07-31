// The co-simulation runner: a second cxxrtl runner (alongside test/cxxrtl.cc)
// whose only job is to emit the core's REAL architectural register-file state
// as it changes, so test/cosim.py can diff it against the Sail RISC-V model.
//
// This is a spike, not a fourth verification leg. `make test` does not build
// or run it; `make cosim` does. See docs/adr/0031.
//
// WHY A SEPARATE BINARY, AND WHY IT READS `regs` RATHER THAN RVFI.
//
// The RVFI monitor (test/monitor.v) and the riscv-formal ladder both check
// what the core SAYS it did: rvfi_rd_wdata against a spec model evaluated on
// rvfi_rs1_rdata / rvfi_rs2_rdata / rvfi_insn. A core that mis-reports a
// value and then computes with that same mis-reported value tells an
// internally consistent story and passes both. This runner never reads a
// single rvfi_* signal. It samples the `uut regfile regs` array -- the actual
// 32x32 memory in rtl/regfile.v -- once per cycle and prints a line whenever
// it changes.
//
// Comparing STATE, not WRITE EVENTS, is deliberate. A write of a value a
// register already holds is architecturally invisible, and Sail traces it
// while a state snapshot does not; comparing the sequence of distinct
// register-file states makes both sides agree on what counts as an event
// without either having to special-case that.
//
// The PC is printed as diagnostic context only, and is deliberately NOT
// compared: `uut decoder pc` is the PC of the instruction currently in
// DECODE, several stages ahead of the write retiring this cycle, so it is a
// pipeline landmark rather than an architectural fact. test/cosim.py reports
// the Sail-side PC as the authoritative location of a divergence.
//
// Output format, one line per architectural change (stdout):
//     CS <change-index> <cycle> x<n>=<hex> [x<n>=<hex> ...] @pc=<hex>
// followed by exactly one terminator line:
//     CS END PASS <cycle> | CS END FAIL <testnum> <cycle> | CS END TIMEOUT
//
// The `CS ` prefix exists because test/testbench.v $displays a line or two
// per cycle (ifetch/read/write chatter) onto the same stdout, and cxxrtl
// implements $display natively. Prefixing is what lets test/cosim.py treat
// an unrecognised `CS ` line as a hard error instead of having to guess
// which unparseable lines were the bench talking.
//
// Exit codes match test/cxxrtl.cc where they overlap: 0 = ran to a tohost
// verdict (PASS or FAIL -- either is a successful TRACE, and cosim.py, not
// this runner, decides whether the trace matches), 2 = cycle-limit timeout,
// 3 = usage/setup error.
#include "rtl.cc"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

// Kept in sync with test/cxxrtl.cc, test/testbench.v and test/asm/sections.lds
// (ADR-0008).
constexpr uint32_t kRamBase = 0x00010000;

using HexImage = std::map<uint32_t, uint32_t>;

bool parse_verilog_hex(const std::string &path, HexImage &image) {
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "error: cannot open %s\n", path.c_str());
    return false;
  }
  uint32_t addr = 0;
  std::string line;
  while (std::getline(in, line)) {
    size_t start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
      continue;
    if (line[start] == '@') {
      addr = std::strtoul(line.c_str() + start + 1, nullptr, 16);
      continue;
    }
    std::istringstream words(line);
    std::string word;
    while (words >> word)
      image[addr++] = std::strtoul(word.c_str(), nullptr, 16);
  }
  return true;
}

bool load_image(cxxrtl::debug_items &items, const std::string &name,
                const HexImage &image, uint32_t base_words) {
  const cxxrtl::debug_item &item = items.at(name).at(0);
  if (item.type != cxxrtl::debug_item::MEMORY) {
    std::fprintf(stderr, "error: debug item %s is not a memory\n", name.c_str());
    return false;
  }
  uint32_t *data = item.curr;
  for (const auto &[addr, word] : image) {
    if (addr < base_words || addr - base_words >= item.depth) {
      std::fprintf(stderr,
                   "error: %s image address 0x%08x is outside the simulated "
                   "%s (%zu words at base 0x%08x)\n",
                   name.c_str(), addr * 4, name.c_str(), item.depth,
                   base_words * 4);
      return false;
    }
    data[addr - base_words] = word;
  }
  return true;
}

struct Args {
  std::string rom_path;
  std::string ram_path;
  long cycles = 0;
};

bool parse_args(int argc, char **argv, Args &args) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next = [&](const char *flag) -> const char * {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: %s requires an argument\n", flag);
        return nullptr;
      }
      return argv[++i];
    };
    if (arg == "--rom") {
      const char *v = next("--rom");
      if (!v) return false;
      args.rom_path = v;
    } else if (arg == "--ram") {
      const char *v = next("--ram");
      if (!v) return false;
      args.ram_path = v;
    } else if (arg == "--cycles") {
      const char *v = next("--cycles");
      if (!v) return false;
      args.cycles = std::strtol(v, nullptr, 10);
    } else {
      std::fprintf(stderr, "error: unrecognized argument '%s'\n", arg.c_str());
      return false;
    }
  }
  if (args.rom_path.empty() || args.ram_path.empty() || args.cycles <= 0) {
    std::fprintf(stderr, "usage: cosim --rom <hex> --ram <hex> --cycles N\n");
    return false;
  }
  return true;
}

} // namespace

int main(int argc, char **argv) {
  Args args;
  if (!parse_args(argc, argv, args))
    return 3;

  HexImage rom_image, ram_image;
  if (!parse_verilog_hex(args.rom_path, rom_image))
    return 3;
  if (!parse_verilog_hex(args.ram_path, ram_image))
    return 3;

  cxxrtl_design::p_testbench top;
  cxxrtl::debug_items all_debug_items;
  top.debug_info(&all_debug_items, nullptr, "");

  if (!load_image(all_debug_items, "rom", rom_image, 0))
    return 3;
  if (!load_image(all_debug_items, "memory", ram_image, kRamBase / 4))
    return 3;

  const cxxrtl::debug_item &memory_item = all_debug_items.at("memory").at(0);
  uint32_t *ram_data = memory_item.curr;

  // The real register file inside rtl/regfile.v. `hierarchy -top testbench`
  // flattens the instance path to a space-separated debug-item name, the same
  // convention test/cxxrtl.cc uses for "monitor errcode".
  const cxxrtl::debug_item *regs_item = nullptr;
  const cxxrtl::debug_item *pc_item = nullptr;
  try {
    regs_item = &all_debug_items.at("uut regfile regs").at(0);
    pc_item = &all_debug_items.at("uut decoder pc").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                 "error: 'uut regfile regs' / 'uut decoder pc' not found in "
                 "the simulated design -- did the RTL hierarchy change?\n");
    return 3;
  }
  if (regs_item->type != cxxrtl::debug_item::MEMORY || regs_item->depth != 32) {
    std::fprintf(stderr, "error: 'uut regfile regs' is not a 32-word memory\n");
    return 3;
  }
  const uint32_t *regs = regs_item->curr;

  uint32_t shadow[32];
  std::memset(shadow, 0, sizeof(shadow));

  top.p_reset.set(true);
  top.step();
  // The regfile has no reset, so its power-on contents are whatever cxxrtl
  // zero-initialised them to. Seed the shadow from the post-reset state so
  // the first printed change is a real one and not the initialisation.
  std::memcpy(shadow, regs, sizeof(shadow));

  long change_index = 0;
  int verdict = -1; // 0 = PASS, >0 = FAIL testnum
  long verdict_cycle = 0;

  for (long cycle = 0; cycle < args.cycles; ++cycle) {
    if (cycle == 0)
      top.p_reset.set(false);
    top.p_clk.set<bool>(false);
    top.step();
    top.p_clk.set<bool>(true);
    top.step();

    if (std::memcmp(shadow, regs, sizeof(shadow)) != 0) {
      std::printf("CS %ld %ld", change_index, cycle);
      for (int r = 0; r < 32; ++r) {
        if (shadow[r] != regs[r])
          std::printf(" x%d=%08x", r, regs[r]);
      }
      std::printf(" @pc=%08x\n", pc_item->curr[0]);
      std::memcpy(shadow, regs, sizeof(shadow));
      ++change_index;
    }

    uint32_t tohost = ram_data[0];
    if (tohost != 0) {
      verdict = (tohost == 1) ? 0 : static_cast<int>(tohost >> 1);
      verdict_cycle = cycle;
      break;
    }
  }

  if (verdict < 0) {
    std::printf("CS END TIMEOUT\n");
    return 2;
  }
  if (verdict == 0)
    std::printf("CS END PASS %ld\n", verdict_cycle);
  else
    std::printf("CS END FAIL %d %ld\n", verdict, verdict_cycle);
  return 0;
}
