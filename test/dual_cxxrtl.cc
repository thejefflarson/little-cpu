// The cxxrtl runner for the DUAL configuration: two harts, one text storage, one data RAM
// and one bus arbiter (test/dual_testbench.v).
#include <cxxrtl/cxxrtl_vcd.h>
#include "dual_rtl.cc"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// test/asm/sections.lds' `ram` region, the same copy test/cxxrtl.cc carries and
// test/memmap_test.sh compares against rtl/memory.v's `BASE`.
constexpr uint32_t kRamBase = 0x00010000;

constexpr int kHarts = 2;

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
                   "error: %s image address 0x%08x is outside the simulated %s "
                   "(%zu words at base 0x%08x)\n",
                   name.c_str(), addr * 4, name.c_str(), item.depth,
                   base_words * 4);
      return false;
    }
    data[addr - base_words] = word;
  }
  return true;
}

// EVERY WORD OF EVERY BANK IS WRITTEN BEFORE THE PROGRAM GOES IN, and that is not
// tidiness.
bool load_rom_banks(cxxrtl::debug_items &items, const HexImage &image) {
  static const char *kBankName[2] = {"dut imem rom_even", "dut imem rom_odd"};
  const cxxrtl::debug_item *bank[2];
  for (int b = 0; b < 2; ++b) {
    bank[b] = &items.at(kBankName[b]).at(0);
    if (bank[b]->type != cxxrtl::debug_item::MEMORY) {
      std::fprintf(stderr, "error: debug item %s is not a memory\n",
                   kBankName[b]);
      return false;
    }
    for (size_t i = 0; i < bank[b]->depth; ++i)
      bank[b]->curr[i] = 0;
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

struct Args {
  std::string rom_path;
  std::string ram_path;
  std::string vcd_path;
  long cycles = 0;
  bool hold_hart1 = false;
  // A word of RAM to print at the end of the run.
  bool report_word = false;
  uint32_t report_addr = 0;
  // Repeatable: one hart's report, or both.
  std::vector<uint32_t> console_addrs;
};

// test/cxxrtl.cc's helper, unchanged: walks `ram_data` from `addr` and writes what it
// finds to stdout, stopping at the first NUL or the end of RAM.
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
    } else if (arg == "--hold-hart1") {
      args.hold_hart1 = true;
    } else if (arg == "--report-word") {
      const char *v = next("--report-word");
      if (!v) return false;
      args.report_word = true;
      args.report_addr = (uint32_t)std::strtoul(v, nullptr, 0);
    } else if (arg == "--console") {
      const char *v = next("--console");
      if (!v) return false;
      args.console_addrs.push_back((uint32_t)std::strtoul(v, nullptr, 0));
    } else {
      std::fprintf(stderr, "error: unrecognized argument '%s'\n", arg.c_str());
      return false;
    }
  }
  if (args.rom_path.empty() || args.ram_path.empty() || args.cycles <= 0) {
    std::fprintf(stderr,
                 "usage: dual-sim --rom <hex> --ram <hex> --cycles N "
                 "[--vcd out.vcd] [--hold-hart1] [--report-word <addr>] "
                 "[--console <addr>]...\n");
    return false;
  }
  return true;
}

// Every item this runner reads is looked up the same way and a miss is a setup error
// rather than something to skip: a silently absent counter reports the exact false green
// the counters exist to prevent.
const cxxrtl::debug_item *item_or_null(cxxrtl::debug_items &items,
                                       const std::string &name) {
  try {
    return &items.at(name).at(0);
  } catch (const std::out_of_range &) {
    std::fprintf(stderr,
                 "error: '%s' is not a debug item of the simulated design. It "
                 "is a (* keep *) signal in test/dual_testbench.v; a rename "
                 "there means renaming it here.\n",
                 name.c_str());
    return nullptr;
  }
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

  cxxrtl_design::p_dual__testbench top;
  cxxrtl::debug_items items;
  top.debug_info(&items, nullptr, "");

  if (!load_rom_banks(items, rom_image))
    return 3;
  if (!load_image(items, "dut dmem ram", ram_image, kRamBase / 4))
    return 3;

  const cxxrtl::debug_item &memory_item = items.at("dut dmem ram").at(0);
  uint32_t *ram_data = memory_item.curr;
  const uint32_t tohost_index = 0;

  const cxxrtl::debug_item *errcode[kHarts];
  const cxxrtl::debug_item *trap_to_zero[kHarts];
  const cxxrtl::debug_item *retires[kHarts];
  const cxxrtl::debug_item *spec_retires[kHarts];
  for (int h = 0; h < kHarts; ++h) {
    const std::string n = std::to_string(h);
    errcode[h] = item_or_null(items, "monitor" + n + " errcode");
    trap_to_zero[h] = item_or_null(items, "trap_to_zero" + n);
    retires[h] = item_or_null(items, "rvfi_retires" + n);
    spec_retires[h] = item_or_null(items, "rvfi_spec_retires" + n);
    if (!errcode[h] || !trap_to_zero[h] || !retires[h] || !spec_retires[h])
      return 3;
  }
  const cxxrtl::debug_item *bus_conflict = item_or_null(items, "bus_conflict");
  if (!bus_conflict)
    return 3;

  // The cycle each hart last retired on, and -1 for a hart that never has.
  long last_retire[kHarts] = {-1, -1};
  uint32_t seen_retires[kHarts] = {0, 0};

  auto report_counts = [&]() {
    if (args.report_word) {
      if (args.report_addr < kRamBase ||
          (args.report_addr - kRamBase) / 4 >= memory_item.depth) {
        std::fprintf(stderr,
                     "error: --report-word address 0x%08x is outside the "
                     "simulated RAM\n",
                     args.report_addr);
      } else {
        std::printf("WORD 0x%08x %u\n", args.report_addr,
                    ram_data[(args.report_addr - kRamBase) / 4]);
      }
    }
    std::printf("HART0 RETIRES %u SPEC-CHECKED %u\n", retires[0]->curr[0],
                spec_retires[0]->curr[0]);
    std::printf("HART1 RETIRES %u SPEC-CHECKED %u\n", retires[1]->curr[0],
                spec_retires[1]->curr[0]);
    for (uint32_t addr : args.console_addrs)
      print_console(ram_data, memory_item.depth, addr);
  };

  // Silence outranks the run's own verdict, per hart.
  auto finish = [&](int code) {
    report_counts();
    for (int h = 0; h < kHarts; ++h) {
      uint32_t r = retires[h]->curr[0];
      uint32_t s = spec_retires[h]->curr[0];
      if (r == 0 || s == 0) {
        std::fprintf(stderr,
                     "hart %d's RVFI monitor observed nothing this run: %u "
                     "retires, %u of them spec-checked. This harness exists to "
                     "watch two harts, so a run that watched one has no verdict "
                     "to report (exit %d was the program's).\n",
                     h, r, s, code);
        return 6;
      }
    }
    return code;
  };

  std::unique_ptr<cxxrtl::vcd_writer> vcd;
  std::ofstream vcd_out;
  if (!args.vcd_path.empty()) {
    vcd = std::make_unique<cxxrtl::vcd_writer>();
    vcd->timescale(1, "us");
    vcd->add_without_memories(items);
    vcd_out.open(args.vcd_path);
  }
  auto sample = [&](int64_t time) {
    if (!vcd) return;
    vcd->sample(time);
    vcd_out << vcd->buffer;
    vcd->buffer.clear();
  };

  top.p_hold__hart1.set<bool>(args.hold_hart1);
  top.p_reset.set(true);
  top.step();

  for (long cycle = 0; cycle < args.cycles; ++cycle) {
    top.p_clk.set<bool>(false);
    top.step();
    sample(cycle * 2 + 0);
    top.p_clk.set<bool>(true);
    top.step();
    sample(cycle * 2 + 1);

    // Reset spans exactly one rising edge, the shape test/testbench.v drives and
    // rtl/littledualsoc.v's power-on counter generalizes.
    if (cycle == 0)
      top.p_reset.set(false);

    for (int h = 0; h < kHarts; ++h) {
      uint32_t r = retires[h]->curr[0];
      if (r != seen_retires[h]) {
        seen_retires[h] = r;
        last_retire[h] = cycle;
      }
    }

    if ((bus_conflict->curr[0] & 1) != 0) {
      std::fprintf(stderr,
                   "both harts drove the shared data bus at cycle %ld. "
                   "rtl/littledual.v joins the two harts' bus ports with an OR, "
                   "so the address the memories saw is neither hart's.\n",
                   cycle);
      report_counts();
      return 7;
    }

    for (int h = 0; h < kHarts; ++h) {
      uint32_t code = errcode[h]->curr[0] & 0xffff;
      if (code != 0) {
        std::fprintf(stderr, "hart %d: RVFI monitor error %u at cycle %ld\n", h,
                     code, cycle);
        report_counts();
        return 4;
      }
      if ((trap_to_zero[h]->curr[0] & 1) != 0) {
        std::fprintf(stderr,
                     "hart %d took a trap with mtvec == 0 at cycle %ld -- the "
                     "handler was never installed and that hart has restarted "
                     "at _start\n",
                     h, cycle);
        report_counts();
        return 5;
      }
    }

    uint32_t tohost = ram_data[tohost_index];
    if (tohost != 0) {
      if (tohost == 1) {
        std::printf("PASS\n");
        return finish(0);
      }
      std::printf("FAIL %u\n", tohost >> 1);
      return finish(1);
    }
  }

  std::printf("TIMEOUT\n");
  for (int h = 0; h < kHarts; ++h) {
    if (last_retire[h] < 0)
      std::fprintf(stderr, "hart %d never retired an instruction\n", h);
    else
      std::fprintf(stderr, "hart %d last retired at cycle %ld of %ld\n", h,
                   last_retire[h], args.cycles);
  }
  return finish(2);
}
