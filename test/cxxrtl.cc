// The cxxrtl test runner: loads a `.text` image into test/testbench.v's two ROM banks
// (`imem rom_even`/`rom_odd`) and a `.data`/`.rodata`/`.bss` image into `dmem ram`, both
// via debug_items, runs the design for a bounded number of cycles, and watches `tohost`
// (test/asm/riscv_test.h) for the riscv-tests pass/fail encoding.
#include <cxxrtl/cxxrtl_vcd.h>
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
#include <utility>
#include <vector>

namespace {

// test/asm/sections.lds' `ram` region starts here; the cxxrtl runner subtracts it back
// out of the `--ram` image's word addresses so they land at the right index in
// test/testbench.v's `memory` array.
constexpr uint32_t kRamBase = 0x00010000;

// A parsed `objcopy -O verilog --verilog-data-width=4` image: word address (byte address
// / 4, per that format) -> 32-bit word.
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

// Pokes `image` into the MEMORY debug item named `name`, offset by `base_words`.
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
                    name.c_str(), addr * 4, name.c_str(), item.depth, base_words * 4);
      return false;
    }
    data[addr - base_words] = word;
  }
  return true;
}

// The instruction ROM is two INTERLEAVED BANKS: word W lives in `imem rom_even` at index
// W/2 when W is even, and in `imem rom_odd` at the same index when it is odd.
bool load_rom_banks(cxxrtl::debug_items &items, const HexImage &image) {
  static const char *kBankName[2] = {"imem rom_even", "imem rom_odd"};
  const cxxrtl::debug_item *bank[2];
  for (int b = 0; b < 2; ++b) {
    bank[b] = &items.at(kBankName[b]).at(0);
    if (bank[b]->type != cxxrtl::debug_item::MEMORY) {
      std::fprintf(stderr, "error: debug item %s is not a memory\n", kBankName[b]);
      return false;
    }
  }
  for (const auto &[addr, word] : image) {
    const int b = addr & 1;
    const uint32_t index = addr >> 1;
    if (index >= bank[b]->depth) {
      std::fprintf(stderr,
                   "error: rom image address 0x%08x is outside the simulated "
                   "ROM (%zu words per bank, 2 banks)\n",
                   addr * 4, bank[b]->depth);
      return false;
    }
    bank[b]->curr[index] = word;
  }
  return true;
}

// The decoder's stall reasons, each read as the named signal rtl/decoder.v drives rather
// than rebuilt here.
struct StallReason {
  const char *item;
  int bucket;
};

constexpr const char *kStallLabels[] = {"divider", "atomic",  "hazard",
                                        "serialize", "operand", "fetch", "bus",
                                        "region"};
constexpr int kStallBuckets = sizeof(kStallLabels) / sizeof(kStallLabels[0]);

constexpr StallReason kStallReasons[] = {
    {"uut decoder divider_stall", 0},
    {"uut decoder atomic_stall", 1},
    {"uut decoder hazard_rs1", 2},
    {"uut decoder hazard_rs2", 2},
    {"uut decoder serialize", 3},
    {"uut decoder operand_stall", 4},
    {"uut decoder fetch_stall", 5},
    // The shared bus given to another initiator.
    {"uut decoder bus_wait", 6},
    // The load/store region wait.
    {"uut decoder region_stall", 7},
};

struct Args {
  std::string rom_path;
  std::string ram_path;
  std::string vcd_path;
  long cycles = 0;
  bool stalls = false;
  bool console = false;
  uint32_t console_addr = 0;
};

// Walks `ram_data` from `addr` and writes what it finds to stdout, stopping at the first
// NUL or at the end of the simulated RAM. Bytes are taken out of the little-endian words
// the array holds, which is the same order the core's `sb` writes them in.
void print_console(const uint32_t *ram_data, size_t ram_words, uint32_t addr) {
  if (addr < kRamBase) {
    std::fprintf(stderr, "error: --console address 0x%08x is below RAM base 0x%08x\n",
                 addr, kRamBase);
    return;
  }
  for (uint32_t offset = addr - kRamBase; offset / 4 < ram_words; ++offset) {
    char byte = (char)((ram_data[offset / 4] >> (8 * (offset % 4))) & 0xff);
    if (byte == '\0')
      return;
    std::fputc(byte, stdout);
  }
}

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
    } else if (arg == "--vcd") {
      const char *v = next("--vcd");
      if (!v) return false;
      args.vcd_path = v;
    } else if (arg == "--stalls") {
      args.stalls = true;
    } else if (arg == "--console") {
      const char *v = next("--console");
      if (!v) return false;
      args.console = true;
      args.console_addr = (uint32_t)std::strtoul(v, nullptr, 0);
    } else {
      std::fprintf(stderr, "error: unrecognized argument '%s'\n", arg.c_str());
      return false;
    }
  }
  if (args.rom_path.empty() || args.ram_path.empty() || args.cycles <= 0) {
    std::fprintf(stderr,
                  "usage: sim --rom <hex> --ram <hex> --cycles N [--vcd out.vcd] "
                  "[--stalls]\n");
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

  if (!load_rom_banks(all_debug_items, rom_image))
    return 3;
  if (!load_image(all_debug_items, "dmem ram", ram_image, kRamBase / 4))
    return 3;

  const cxxrtl::debug_item &memory_item = all_debug_items.at("dmem ram").at(0);
  uint32_t *ram_data = memory_item.curr;
  const uint32_t tohost_index = 0;

  const cxxrtl::debug_item *monitor_errcode = nullptr;
  try {
    monitor_errcode = &all_debug_items.at("monitor errcode").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: RVFI monitor ('monitor errcode') not found in the "
                  "simulated design -- was test/rtl.cc built without "
                  "-D RISCV_FORMAL?\n");
    return 3;
  }

  const cxxrtl::debug_item *trap_to_zero = nullptr;
  try {
    trap_to_zero = &all_debug_items.at("trap_to_zero").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: the trap-to-zero check ('trap_to_zero') not "
                  "found in the simulated design -- did test/testbench.v lose "
                  "the (* keep *) on it?\n");
    return 3;
  }

  // The observation counters (test/testbench.v).
  const cxxrtl::debug_item *retires = nullptr;
  const cxxrtl::debug_item *spec_retires = nullptr;
  try {
    retires = &all_debug_items.at("rvfi_retires").at(0);
    spec_retires = &all_debug_items.at("rvfi_spec_retires").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: the monitor observation counters ('rvfi_retires', "
                  "'rvfi_spec_retires') were not found in the simulated design "
                  "-- did test/testbench.v lose the (* keep *) on them?\n");
    return 3;
  }

  std::vector<std::pair<const cxxrtl::debug_item *, int>> stall_probes;
  const cxxrtl::debug_item *stall_any = nullptr;
  const cxxrtl::debug_item *hazard_rs1_item = nullptr;
  const cxxrtl::debug_item *hazard_rs2_item = nullptr;
  const cxxrtl::debug_item *out_match_rs1 = nullptr;
  const cxxrtl::debug_item *out_match_rs2 = nullptr;
  const cxxrtl::debug_item *rs1_fwd_eligible = nullptr;
  const cxxrtl::debug_item *rs2_fwd_eligible = nullptr;
  const cxxrtl::debug_item *instr_csr_access = nullptr;
  // The load/store locality counters (rtl/littlecpu.v).
  const cxxrtl::debug_item *ls_issues = nullptr;
  const cxxrtl::debug_item *ls_edges = nullptr;
  const cxxrtl::debug_item *ls_bypasses = nullptr;
  if (args.stalls) {
    try {
      stall_any = &all_debug_items.at("uut decoder stall").at(0);
      for (const StallReason &reason : kStallReasons)
        stall_probes.emplace_back(&all_debug_items.at(reason.item).at(0),
                                  reason.bucket);
      hazard_rs1_item = &all_debug_items.at("uut decoder hazard_rs1").at(0);
      hazard_rs2_item = &all_debug_items.at("uut decoder hazard_rs2").at(0);
      out_match_rs1 = &all_debug_items.at("uut decoder out_match_rs1").at(0);
      out_match_rs2 = &all_debug_items.at("uut decoder out_match_rs2").at(0);
      rs1_fwd_eligible = &all_debug_items.at("uut decoder rs1_fwd_eligible").at(0);
      rs2_fwd_eligible = &all_debug_items.at("uut decoder rs2_fwd_eligible").at(0);
      instr_csr_access = &all_debug_items.at("uut decoder instr_csr_access").at(0);
    } catch (const std::out_of_range &) {
      std::fprintf(stderr,
                    "error: --stalls needs the decoder's stall signals as debug "
                    "items, and at least one of them is not in the simulated "
                    "design. They are plain named wires in rtl/decoder.v; a "
                    "rename there means renaming them in kStallReasons here.\n");
      return 3;
    }
    try {
      ls_issues = &all_debug_items.at("uut probe_ls_issues").at(0);
      ls_edges = &all_debug_items.at("uut probe_ls_edges").at(0);
      ls_bypasses = &all_debug_items.at("uut probe_ls_bypasses").at(0);
    } catch (const std::out_of_range &) {
      std::fprintf(stderr,
                    "error: --stalls needs the load/store locality counters as "
                    "debug items, and at least one of them is not in the "
                    "simulated design. They are the `probe_ls_*` registers in "
                    "rtl/littlecpu.v's RISCV_FORMAL block; printing zeros for a "
                    "counter that is not there would read as a workload with no "
                    "loads in it.\n");
      return 3;
    }
  }

  uint64_t counted_cycles = 0;
  uint64_t issue_cycles = 0;
  uint64_t unattributed_cycles = 0;
  uint64_t stall_cycles[kStallBuckets] = {};
  uint64_t hazard_a = 0, hazard_b = 0, hazard_c = 0, hazard_c_csr = 0;

  auto report_counts = [&]() {
    if (args.console)
      print_console(ram_data, memory_item.depth, args.console_addr);
    std::printf("RETIRES %u SPEC-CHECKED %u\n", retires->curr[0],
                 spec_retires->curr[0]);
    if (!args.stalls)
      return;
    std::printf("STALLS cycles=%llu issue=%llu",
                 (unsigned long long)counted_cycles,
                 (unsigned long long)issue_cycles);
    for (int b = 0; b < kStallBuckets; ++b)
      std::printf(" %s=%llu", kStallLabels[b],
                   (unsigned long long)stall_cycles[b]);
    std::printf(" hzA=%llu hzB=%llu hzC=%llu hzCcsr=%llu",
                 (unsigned long long)hazard_a, (unsigned long long)hazard_b,
                 (unsigned long long)hazard_c, (unsigned long long)hazard_c_csr);
    std::printf(" unattributed=%llu lsissue=%u lsedge=%u lsbypass=%u\n",
                 (unsigned long long)unattributed_cycles, ls_issues->curr[0],
                 ls_edges->curr[0], ls_bypasses->curr[0]);
  };

  auto finish = [&](int code) {
    report_counts();
    uint32_t r = retires->curr[0];
    uint32_t s = spec_retires->curr[0];
    if (r == 0 || s == 0) {
      std::fprintf(stderr,
                    "the RVFI monitor observed nothing this run: %u retires, %u "
                    "of them spec-checked. The per-retire oracle was blind, so "
                    "this run's verdict (exit %d) means nothing -- see the "
                    "counter block in test/testbench.v.\n",
                    r, s, code);
      return 6;
    }
    return code;
  };

  std::unique_ptr<cxxrtl::vcd_writer> vcd;
  std::ofstream vcd_out;
  if (!args.vcd_path.empty()) {
    vcd = std::make_unique<cxxrtl::vcd_writer>();
    vcd->timescale(1, "us");
    vcd->add_without_memories(all_debug_items);
    vcd_out.open(args.vcd_path);
  }

  auto sample = [&](int64_t time) {
    if (!vcd) return;
    vcd->sample(time);
    vcd_out << vcd->buffer;
    vcd->buffer.clear();
  };

  top.p_reset.set(true);
  top.step();

  for (long cycle = 0; cycle < args.cycles; ++cycle) {
    top.p_clk.set<bool>(false);
    top.step();
    if (args.stalls) {
      top.debug_eval();
      counted_cycles++;
      if ((stall_any->curr[0] & 1) == 0) {
        issue_cycles++;
      } else {
        bool charged = false;
        const cxxrtl::debug_item *charged_item = nullptr;
        for (const auto &[item, bucket] : stall_probes) {
          if ((item->curr[0] & 1) != 0) {
            stall_cycles[bucket]++;
            charged = true;
            charged_item = item;
            break;
          }
        }
        if (!charged)
          unattributed_cycles++;

        const cxxrtl::debug_item *out_match = nullptr;
        const cxxrtl::debug_item *eligible = nullptr;
        if (charged_item == hazard_rs1_item) {
          out_match = out_match_rs1;
          eligible = rs1_fwd_eligible;
        } else if (charged_item == hazard_rs2_item) {
          out_match = out_match_rs2;
          eligible = rs2_fwd_eligible;
        }
        if (out_match != nullptr) {
          if ((out_match->curr[0] & 1) != 0) {
            hazard_a++;
          } else if ((eligible->curr[0] & 1) != 0) {
            hazard_b++;
          } else {
            hazard_c++;
            if ((instr_csr_access->curr[0] & 1) != 0)
              hazard_c_csr++;
          }
        }
      }
    }
    sample(cycle * 2 + 0);
    top.p_clk.set<bool>(true);
    top.step();
    sample(cycle * 2 + 1);

    if (cycle == 0)
      top.p_reset.set(false);

    uint32_t errcode = monitor_errcode->curr[0] & 0xffff;
    if (errcode != 0) {
      std::fprintf(stderr, "RVFI monitor error %u at cycle %ld\n", errcode, cycle);
      report_counts();
      return 4;
    }

    if ((trap_to_zero->curr[0] & 1) != 0) {
      std::fprintf(stderr,
                    "trap taken with mtvec == 0 at cycle %ld -- the handler was "
                    "never installed and the program has restarted at _start\n",
                    cycle);
      report_counts();
      return 5;
    }

    uint32_t tohost = ram_data[tohost_index];
    if (tohost != 0) {
      if (tohost == 1) {
        std::printf("PASS\n");
        return finish(0);
      }
      uint32_t testnum = tohost >> 1;
      std::printf("FAIL %u\n", testnum);
      return finish(1);
    }
  }

  std::printf("TIMEOUT\n");
  return finish(2);
}
