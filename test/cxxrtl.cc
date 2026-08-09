// The cxxrtl test runner: loads a `.text` and a
// `.data`/`.rodata`/`.bss` image straight into test/testbench.v's `rom` and
// `memory` arrays via debug_items, runs the design for a bounded number of
// cycles, and watches `tohost` (test/asm/riscv_test.h) for the riscv-tests
// pass/fail encoding.
//
// Exit codes: 0 = pass, 1 = fail (test number printed), 2 = cycle-limit
// timeout, 3 = usage/setup error (bad args, missing file, image too big for
// the simulated memories), 4 = RVFI monitor error (a per-retire mismatch,
// distinct from 1 because tohost never got a say: the failure is
// in the pipeline's own bookkeeping, not the test program's assertions),
// 5 = trap taken with mtvec == 0 (the program has silently
// restarted at `_start`; distinct from 2 because the timeout that would
// otherwise result says nothing about the fault that caused it),
// 6 = the per-retire monitor observed nothing: zero retires, or zero retires
// whose values its spec model checked. See the counter block in
// test/testbench.v for why that is a failure and not a curiosity — a monitor
// that never fires leaves every program passing off `tohost` alone, which
// co-simulation measured to be blind to real architectural corruption.
//
// Every run that actually simulates prints one machine-readable line before
// it exits:
//
//     RETIRES <n> SPEC-CHECKED <m>
//
// test/run_tests.sh parses it into the pass/fail table and grades it against
// test/OBSERVED_FLOOR. Making observation a printed, graded quantity rather
// than something inferred from a green run is the whole point of the two
// counters; a number nobody reads is the defect this closes, not a fix for it.
//
// `--stalls` adds a second line, splitting the run's cycles between the
// decoder's stall reasons and the cycles that issue an instruction:
//
//     STALLS cycles=<n> issue=<n> hazard=<n> ... unattributed=<n>
//
// It is off by default because it costs a debug_eval() per cycle; test/
// stall_report.py turns those lines into a table. See the counter block below
// for what each name means and why the order it tries them in is the decoder's.
//
// `--console <addr>` copies the NUL-terminated string the program left at that
// RAM address to stdout when the run ends. This machine has no output device,
// so a program with something to say -- test/bench/dhry_port.c formats a whole
// benchmark report -- builds it in RAM and this reads it back out. Nothing is
// graded on it, and without the flag not a byte of RAM is read.
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

// test/asm/sections.lds' `ram` region starts here; the cxxrtl runner
// subtracts it back out of the `--ram` image's word addresses so they land
// at the right index in test/testbench.v's `memory` array. Keep in sync
// with sections.lds and test/testbench.v's RAM_BASE.
constexpr uint32_t kRamBase = 0x00010000;

// A parsed `objcopy -O verilog --verilog-data-width=4` image: word address
// (byte address / 4, per that format) -> 32-bit word.
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

// Pokes `image` into the MEMORY debug item named `name`, offset by
// `base_words`. Returns false (with a diagnostic on stderr) if any address
// falls outside the simulated array — that means the test image no longer
// fits test/testbench.v's memories, not that the test failed.
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

// The instruction ROM is two INTERLEAVED BANKS: word W lives in
// `imem rom_even` at index W/2 when W is even, and in `imem rom_odd` at the
// same index when it is odd. Banking is what removed the duplicated ROM the SoC
// needed for the dual-word fetch window, so the de-interleaving has to happen
// somewhere -- here, at load time, rather than in RTL that would cost hardware.
//
// The split is derived from the word address alone, so it cannot disagree with
// rtl/imemory.v's read side without test/imem_tb.v -- which checks that side
// against a flat reference at every alignment -- saying so.
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

// The decoder's stall reasons, each read as the named signal rtl/decoder.v
// drives rather than rebuilt here. Several of them are true on the same cycle
// often enough that the count needs an order, and the order below is the
// decoder's own: it holds `decoder_out` for the divider and the accessor before
// it bubbles for anything else, and `stall` ORs the remaining four left to
// right. Rebuilding the equation in C++ would let this drift from the RTL with
// nothing to say so, which is worse than not counting at all.
//
// `hazard_rs1` and `hazard_rs2` share a bucket: they are one reason, the decode
// scoreboard finding a producer still in flight, and they sit next to each
// other in the order so merging them cannot move a cycle past anything else.
struct StallReason {
  const char *item;
  int bucket;
};

constexpr const char *kStallLabels[] = {"divider", "accessor", "hazard",
                                        "serialize", "operand", "fetch"};
constexpr int kStallBuckets = sizeof(kStallLabels) / sizeof(kStallLabels[0]);

constexpr StallReason kStallReasons[] = {
    {"uut decoder divider_stall", 0},
    {"uut decoder accessor_stall", 1},
    {"uut decoder hazard_rs1", 2},
    {"uut decoder hazard_rs2", 2},
    {"uut decoder serialize", 3},
    {"uut decoder operand_stall", 4},
    {"uut decoder fetch_stall", 5},
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

// Walks `ram_data` from `addr` and writes what it finds to stdout, stopping at
// the first NUL or at the end of the simulated RAM. Bytes are taken out of the
// little-endian words the array holds, which is the same order the core's `sb`
// writes them in.
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

  // tohost is always the first word riscv_test.h places in the ram region
  // (RVTEST_DATA_BEGIN's `.tohost` input section is placed first in
  // sections.lds' `.data` output section), so it is always at kRamBase.
  const cxxrtl::debug_item &memory_item = all_debug_items.at("dmem ram").at(0);
  uint32_t *ram_data = memory_item.curr;
  const uint32_t tohost_index = 0;

  // test/testbench.v instantiates the RVFI monitor
  // (test/monitor.v, riding along under -D RISCV_FORMAL) alongside the DUT,
  // so this leg is self-checking per-retire, not merely end-state-checking
  // via tohost above. "monitor errcode" is the top-level error code the
  // generated monitor module registers for exactly the one cycle a
  // per-retire check fails (0 otherwise) -- its own $display diagnostics
  // (rvfi_* vs spec_* dump) already reach stdout via cxxrtl's native
  // $display support, so this only needs to turn a nonzero code into a
  // distinct, non-ignorable exit status.
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

  // `mtvec` resets to 0 and sections.lds links `.text` there, so a
  // trap taken before a handler is installed restarts the program at `_start`
  // — a livelock that reads as a timeout with no hint of the fault behind it.
  // test/testbench.v raises this the cycle after any trap that redirects to
  // address 0; see the comment there for why that needs no hierarchical
  // reference into the CSR file. Looked up the same way as the monitor's
  // errcode above, and missing is a setup error rather than something to
  // skip: a silently absent check is exactly what this exists to prevent.
  const cxxrtl::debug_item *trap_to_zero = nullptr;
  try {
    trap_to_zero = &all_debug_items.at("trap_to_zero").at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                  "error: ADR-0029's trap-to-zero check ('trap_to_zero') not "
                  "found in the simulated design -- did test/testbench.v lose "
                  "the (* keep *) on it?\n");
    return 3;
  }

  // The observation counters (test/testbench.v). `rvfi_retires` counts the
  // cycles the monitor examined; `rvfi_spec_retires` counts the subset whose
  // values it actually compared against its spec model. Looked up the same way
  // and for the same reason as the two items above: a silently absent counter
  // would report the exact false green this exists to prevent, so missing is a
  // setup error rather than something to skip.
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

  // `--stalls` only. Each of these is a signal rtl/decoder.v drives, looked up
  // the same way as the items above and a setup error when missing: a stall
  // reason that quietly stopped being counted would report the cycles it costs
  // as issue cycles, which is the one wrong answer this measurement can give.
  //
  // `uut decoder stall` is the decoder's own OR of the reasons below. Reading it
  // rather than OR-ing the seven probes is what makes `unattributed` mean
  // something: it counts the cycles the decoder called a stall and none of the
  // named reasons explains, i.e. a stall reason nobody has written down.
  std::vector<std::pair<const cxxrtl::debug_item *, int>> stall_probes;
  const cxxrtl::debug_item *stall_any = nullptr;
  if (args.stalls) {
    try {
      stall_any = &all_debug_items.at("uut decoder stall").at(0);
      for (const StallReason &reason : kStallReasons)
        stall_probes.emplace_back(&all_debug_items.at(reason.item).at(0),
                                  reason.bucket);
    } catch (const std::out_of_range &) {
      std::fprintf(stderr,
                    "error: --stalls needs the decoder's stall signals as debug "
                    "items, and at least one of them is not in the simulated "
                    "design. They are plain named wires in rtl/decoder.v; a "
                    "rename there means renaming them in kStallReasons here.\n");
      return 3;
    }
  }

  uint64_t counted_cycles = 0;
  uint64_t issue_cycles = 0;
  uint64_t unattributed_cycles = 0;
  uint64_t stall_cycles[kStallBuckets] = {};

  // Printed on every path that reaches the simulation loop. Setup failures
  // above return before this point on purpose: nothing ran, so there is no
  // observation to report, and a "RETIRES 0" line for a run that never started
  // would be a claim rather than a measurement.
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
    std::printf(" unattributed=%llu\n", (unsigned long long)unattributed_cycles);
  };

  // Silence outranks the run's own verdict. A run whose oracle never fired has
  // no verdict worth reporting: `tohost` saying PASS is exactly what a blind
  // monitor looks like, and a FAIL from such a run cannot be attributed either.
  //
  // Exits 4 and 5 are the two exceptions and do NOT come through here. Neither
  // is a verdict the PROGRAM reached: an errcode is direct evidence that the
  // monitor fired, and trap-to-zero is a direct observation of the machine that
  // owes nothing to the monitor. Routing either through the silence gate would
  // replace a specific diagnosis with "the oracle was blind".
  //
  // For exit 5 that is not hypothetical, it is the trap-to-zero case itself.
  // Traps are detected and committed in DECODE while a retire happens
  // in writeback, so a fault in the first few instructions of `_start` raises
  // trap_to_zero several cycles before anything has retired. Measured: `ecall`
  // as the second instruction gives `RETIRES 0` and used to return 6, which
  // test/run_tests.sh labels MONITOR-SILENT — pointing at the monitor for a
  // program that faulted before installing `mtvec`, which is precisely the
  // misattribution the TRAP-TO-ZERO label exists to prevent. The same
  // `ecall` after six retiring instructions returned 5. A named failure path
  // that is unreachable in its own scenario is the defect this repo keeps
  // finding in its grading layer, one level down.
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
    if (cycle == 0)
      top.p_reset.set(false);
    top.p_clk.set<bool>(false);
    top.step();
    // Sampled here, between the two edges, because these are combinational: with
    // the clock low they hold the values that decide what the rising edge below
    // does. Read after the rising edge instead and every count belongs to the
    // next cycle. debug_eval() is what fills them in -- yosys leaves them
    // outlined, so their storage is stale until it is called.
    if (args.stalls) {
      top.debug_eval();
      counted_cycles++;
      if ((stall_any->curr[0] & 1) == 0) {
        issue_cycles++;
      } else {
        bool charged = false;
        for (const auto &[item, bucket] : stall_probes) {
          if ((item->curr[0] & 1) != 0) {
            stall_cycles[bucket]++;
            charged = true;
            break;
          }
        }
        if (!charged)
          unattributed_cycles++;
      }
    }
    sample(cycle * 2 + 0);
    top.p_clk.set<bool>(true);
    top.step();
    sample(cycle * 2 + 1);

    uint32_t errcode = monitor_errcode->curr[0] & 0xffff;
    if (errcode != 0) {
      std::fprintf(stderr, "RVFI monitor error %u at cycle %ld\n", errcode, cycle);
      report_counts();
      return 4;
    }

    if ((trap_to_zero->curr[0] & 1) != 0) {
      std::fprintf(stderr,
                    "trap taken with mtvec == 0 at cycle %ld -- the handler was "
                    "never installed and the program has restarted at _start "
                    "(ADR-0029)\n",
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
