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

// test/asm/sections.lds' `ram` region starts here; the runner subtracts it back out of
// the `--ram` image's word addresses so they land at the right index in `memory`.
constexpr uint32_t kRamBase = 0x00010000;

// A parsed `objcopy -O verilog --verilog-data-width=4` image: word address -> 32-bit word.
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

// The ROM is two INTERLEAVED BANKS: word W lives in `rom_even` at W/2 if even, else `rom_odd`.
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

// The stall reasons, each read as the named signal the RTL drives rather than rebuilt here.
struct StallReason {
  const char *item;
  int bucket;
};

// The D/X split deletes "operand" outright and moves the divider reason to
// rtl/executor.v's `divider_busy`, folded into `x_busy` for D. B3 deletes the region
// wait outright rather than folding it: `x_busy` is now exactly `divider_busy`.
constexpr const char *kStallLabels[] = {"divider", "atomic",  "hazard",
                                        "serialize", "fetch", "bus"};
constexpr int kStallBuckets = sizeof(kStallLabels) / sizeof(kStallLabels[0]);

constexpr StallReason kStallReasons[] = {
    {"uut executor divider_busy", 0},
    {"uut decoder atomic_stall", 1},
    {"uut decoder hazard_rs1", 2},
    {"uut decoder hazard_rs2", 2},
    {"uut decoder serialize", 3},
    {"uut decoder fetch_stall", 4},
    // The shared bus given to another initiator.
    {"uut decoder bus_wait", 5},
};

// Where a redirect's cycles go, and what a return-address guess would have saved. Counted
// from X's own signals, nothing in the RTL. A redirect's cost is the run of cycles X
// resolves nothing after it, up to the next resolving cycle.
struct RedirectAccount {
  uint64_t commits = 0, jalr = 0, returns = 0;
  uint64_t jalr_redirects = 0, other_redirects = 0;
  uint64_t jalr_window = 0, other_window = 0;
  // A one-entry return register, and an unbounded stack as the nesting-free upper bound.
  uint64_t hit1 = 0, hit_deep = 0, saved1 = 0, saved_deep = 0;
  bool window_open = false, window_jalr = false, window_hit1 = false, window_hit_deep = false;
  uint64_t window_len = 0;
  bool ras1_valid = false;
  uint32_t ras1 = 0;
  std::vector<uint32_t> deep;

  void close_window() {
    if (!window_open)
      return;
    (window_jalr ? jalr_window : other_window) += window_len;
    if (window_hit1)
      saved1 += window_len;
    if (window_hit_deep)
      saved_deep += window_len;
    window_open = false;
  }

  void cycle(bool resolving, bool commit, bool redirect, bool is_jal, bool is_jalr,
             uint32_t pc, uint32_t pc_inc, uint32_t rd, uint32_t rs1, uint32_t target) {
    if (window_open && !resolving)
      window_len++;
    if (!resolving)
      return;
    close_window();
    bool hit1_now = false, hit_deep_now = false;
    if (commit) {
      commits++;
      if (is_jalr) {
        jalr++;
        if (rd == 0 && (rs1 == 1 || rs1 == 5)) {
          returns++;
          hit1_now = ras1_valid && ras1 == target;
          hit_deep_now = !deep.empty() && deep.back() == target;
          hit1 += hit1_now;
          hit_deep += hit_deep_now;
          ras1_valid = false;
          if (!deep.empty())
            deep.pop_back();
        }
      }
      if ((is_jal || is_jalr) && (rd == 1 || rd == 5)) {
        ras1_valid = true;
        ras1 = pc + pc_inc;
        deep.push_back(pc + pc_inc);
        if (deep.size() > 64)
          deep.erase(deep.begin());
      }
    }
    if (redirect) {
      (is_jalr ? jalr_redirects : other_redirects)++;
      window_open = true;
      window_jalr = is_jalr;
      window_hit1 = hit1_now;
      window_hit_deep = hit_deep_now;
      window_len = 0;
    }
  }
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

// Walks `ram_data` from `addr` to stdout, stopping at the first NUL or RAM's end, reading
// bytes out of the little-endian words the array holds -- the order the core's `sb` writes.
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

  const cxxrtl::debug_item *retires = nullptr;  // observation counters, test/testbench.v
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
  const cxxrtl::debug_item *hazard_rs1_dx_item = nullptr;
  const cxxrtl::debug_item *hazard_rs1_ex_item = nullptr;
  const cxxrtl::debug_item *hazard_rs2_dx_item = nullptr;
  const cxxrtl::debug_item *hazard_rs2_ex_item = nullptr;
  // The load/store locality counters (rtl/littlecpu.v).
  const cxxrtl::debug_item *ls_issues = nullptr;
  const cxxrtl::debug_item *ls_edges = nullptr;
  const cxxrtl::debug_item *ls_bypasses = nullptr;
  // D's own branch/jal predictor counters (rtl/littlecpu.v).
  const cxxrtl::debug_item *guesses = nullptr;
  const cxxrtl::debug_item *guess_hits = nullptr;
  const cxxrtl::debug_item *guess_misses = nullptr;
  if (args.stalls) {
    try {
      stall_any = &all_debug_items.at("uut decoder stall").at(0);
      for (const StallReason &reason : kStallReasons)
        stall_probes.emplace_back(&all_debug_items.at(reason.item).at(0),
                                  reason.bucket);
      hazard_rs1_dx_item = &all_debug_items.at("uut decoder hazard_rs1_dx").at(0);
      hazard_rs1_ex_item = &all_debug_items.at("uut decoder hazard_rs1_ex").at(0);
      hazard_rs2_dx_item = &all_debug_items.at("uut decoder hazard_rs2_dx").at(0);
      hazard_rs2_ex_item = &all_debug_items.at("uut decoder hazard_rs2_ex").at(0);
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
    try {
      guesses = &all_debug_items.at("uut probe_guesses").at(0);
      guess_hits = &all_debug_items.at("uut probe_guess_hits").at(0);
      guess_misses = &all_debug_items.at("uut probe_guess_misses").at(0);
    } catch (const std::out_of_range &) {
      std::fprintf(stderr,
                    "error: --stalls needs the branch/jal predictor counters as "
                    "debug items, and at least one of them is not in the "
                    "simulated design. They are the `probe_guess_*` registers in "
                    "rtl/littlecpu.v's RISCV_FORMAL block.\n");
      return 3;
    }
  }

  const cxxrtl::debug_item *x_committing = nullptr, *x_in_valid = nullptr, *x_busy_item = nullptr,
                           *x_redirect_item = nullptr, *x_is_jalr = nullptr, *x_is_jal = nullptr,
                           *x_pc = nullptr, *x_pc_inc = nullptr, *x_rd = nullptr, *x_rs1 = nullptr,
                           *x_target = nullptr;
  if (args.stalls) {
    try {
      x_committing = &all_debug_items.at("uut executor committing").at(0);
      x_in_valid = &all_debug_items.at("uut executor in_valid").at(0);
      x_busy_item = &all_debug_items.at("uut executor x_busy").at(0);
      x_redirect_item = &all_debug_items.at("uut executor redirect").at(0);
      x_is_jalr = &all_debug_items.at("uut executor in_is_jalr").at(0);
      x_is_jal = &all_debug_items.at("uut executor in_is_jal").at(0);
      x_pc = &all_debug_items.at("uut executor in_pc").at(0);
      x_pc_inc = &all_debug_items.at("uut executor pc_inc").at(0);
      x_rd = &all_debug_items.at("uut executor in_rd").at(0);
      x_rs1 = &all_debug_items.at("uut executor in_rs1").at(0);
      x_target = &all_debug_items.at("uut executor resolved_target").at(0);
    } catch (const std::out_of_range &) {
      std::fprintf(stderr,
                    "error: --stalls needs rtl/executor.v's `committing`, `in_valid`, "
                    "`x_busy`, `redirect`, `in_is_jal`, `in_is_jalr`, `in_pc`, `pc_inc`, "
                    "`in_rd`, `in_rs1` and `resolved_target` as debug items, and at "
                    "least one is not in the simulated design.\n");
      return 3;
    }
  }
  RedirectAccount redirects;

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
    std::printf(" unattributed=%llu lsissue=%u lsedge=%u lsbypass=%u"
                 " guesses=%u guesshits=%u guessmisses=%u",
                 (unsigned long long)unattributed_cycles, ls_issues->curr[0],
                 ls_edges->curr[0], ls_bypasses->curr[0], guesses->curr[0],
                 guess_hits->curr[0], guess_misses->curr[0]);
    redirects.close_window();
    std::printf(" commits=%llu jalr=%llu jalrret=%llu jalrredir=%llu otherredir=%llu"
                 " jalrwin=%llu otherwin=%llu rashit1=%llu rassave1=%llu"
                 " rashitdeep=%llu rassavedeep=%llu\n",
                 (unsigned long long)redirects.commits, (unsigned long long)redirects.jalr,
                 (unsigned long long)redirects.returns,
                 (unsigned long long)redirects.jalr_redirects,
                 (unsigned long long)redirects.other_redirects,
                 (unsigned long long)redirects.jalr_window,
                 (unsigned long long)redirects.other_window,
                 (unsigned long long)redirects.hit1, (unsigned long long)redirects.saved1,
                 (unsigned long long)redirects.hit_deep,
                 (unsigned long long)redirects.saved_deep);
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
      redirects.cycle((x_in_valid->curr[0] & 1) && !(x_busy_item->curr[0] & 1),
                      x_committing->curr[0] & 1, x_redirect_item->curr[0] & 1,
                      x_is_jal->curr[0] & 1, x_is_jalr->curr[0] & 1, x_pc->curr[0],
                      x_pc_inc->curr[0], x_rd->curr[0], x_rs1->curr[0], x_target->curr[0]);
      if ((stall_any->curr[0] & 1) == 0) {
        issue_cycles++;
      } else {
        bool charged = false;
        const cxxrtl::debug_item *charged_item = nullptr;
        int charged_bucket = -1;
        for (const auto &[item, bucket] : stall_probes) {
          if ((item->curr[0] & 1) != 0) {
            stall_cycles[bucket]++;
            charged = true;
            charged_item = item;
            charged_bucket = bucket;
            break;
          }
        }
        if (!charged)
          unattributed_cycles++;
        (void)charged_item;

        // hzA: dx_match with no forward select yet. hzB: ex_match not yet unpacked. A
        // ready ex_match no longer stalls (the write-through bypass reaches it).
        if (charged_bucket == 2) {
          bool dx = (hazard_rs1_dx_item->curr[0] & 1) != 0 ||
                    (hazard_rs2_dx_item->curr[0] & 1) != 0;
          bool ex = (hazard_rs1_ex_item->curr[0] & 1) != 0 ||
                    (hazard_rs2_ex_item->curr[0] & 1) != 0;
          if (dx)
            hazard_a++;
          else if (ex)
            hazard_b++;
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
