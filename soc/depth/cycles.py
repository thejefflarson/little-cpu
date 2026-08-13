#!/usr/bin/env python3
"""Count REDIRECTS on both workloads, beside the cycles and issues `make cycles`
already reports.

WHY THIS NUMBER. A register in the fetch loop is a fetch stage, and what a fetch
stage costs depends entirely on what refills it. If the next address is only
known after decode, every issue waits for the memory and the price is one cycle
per ISSUE. If fetch runs ahead sequentially and something discards the word it
guessed wrong, the price is one cycle per REDIRECT. Those two differ by close to
an order of magnitude on compiled code, and nothing here counted the second.

A redirect is a cycle the decoder ISSUES an instruction whose `next_pc` is not
`pc + pc_inc`: the five arms of rtl/decoder.v's next-PC chain below the stall
arm. Reset and stall are above it and are not redirects -- a stalled cycle
re-presents the same words and moves nothing.

HOW. The shipping runner is copied and patched at three anchors that are checked
to exist, rather than forked into a second copy that would drift from it. A
missing anchor stops this script; it does not fall back to a binary that counts
nothing. The counters ride on `--stalls`, so they are read on exactly the cycles
the stall accounting is read on, and each run appends its line to `$DEPTH_LOG`
because test/run_tests.sh keeps only the `STALLS` line off a run's stdout.

Usage: cycles.py [--runs N]     (writes into depth.out/)
"""

import argparse
import os
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]

ANCHORS = {
    "probes": "  uint64_t stall_cycles[kStallBuckets] = {};",
    "count": "      if ((stall_any->curr[0] & 1) == 0) {",
    # The last line of `report_counts`, which every path that simulated anything
    # goes through and which has already returned early without `--stalls`.
    "report": '    std::printf(" unattributed=%llu\\n", (unsigned long long)unattributed_cycles);',
}

PROBES = r"""
  static const char *kRedirectItems[] = {
      "uut decoder trap_taken", "uut decoder instr_mret", "uut decoder instr_jalr",
      "uut decoder instr_jal", "uut decoder branch_taken"};
  std::vector<const cxxrtl::debug_item *> redirect_probes;
  uint64_t redirect_cycles = 0;
  if (args.stalls) {
    try {
      for (const char *item : kRedirectItems)
        redirect_probes.push_back(&all_debug_items.at(item).at(0));
    } catch (const std::out_of_range &) {
      std::fprintf(stderr, "error: a next-PC arm is not a debug item\n");
      return 3;
    }
  }
"""

COUNT = r"""
        for (const cxxrtl::debug_item *item : redirect_probes)
          if ((item->curr[0] & 1) != 0) { redirect_cycles++; break; }
"""

# test/run_tests.sh keeps only the `STALLS` line off a run's stdout, so this goes
# to a file named in the environment instead of competing for it.
REPORT = r"""
    if (const char *path = std::getenv("DEPTH_LOG")) {
      if (std::FILE *f = std::fopen(path, "a")) {
        std::fprintf(f, "%s cycles=%llu issue=%llu redirect=%llu\n",
                     args.rom_path.c_str(), (unsigned long long)counted_cycles,
                     (unsigned long long)issue_cycles,
                     (unsigned long long)redirect_cycles);
        std::fclose(f);
      }
    }
"""


def patch(runner_source):
    text = runner_source.read_text()
    for name, anchor in ANCHORS.items():
        if anchor not in text:
            sys.exit(
                f"error: {runner_source} no longer contains the {name} anchor:\n"
                f"  {anchor}\n"
                "This spike patches that file rather than copying it, so an anchor "
                "that moved has to be re-found, not worked around."
            )
    text = text.replace(ANCHORS["probes"], ANCHORS["probes"] + PROBES, 1)
    text = text.replace(ANCHORS["count"], ANCHORS["count"] + COUNT, 1)
    text = text.replace(ANCHORS["report"], ANCHORS["report"] + REPORT, 1)
    return text


def totals(log):
    cycles = issues = redirects = 0
    programs = 0
    for line in log.read_text().splitlines():
        m = re.search(r"cycles=(\d+) issue=(\d+) redirect=(\d+)", line)
        if not m:
            continue
        programs += 1
        cycles += int(m.group(1))
        issues += int(m.group(2))
        redirects += int(m.group(3))
    return programs, cycles, issues, redirects


def report(label, log):
    programs, cycles, issues, redirects = totals(log)
    if cycles == 0:
        sys.exit(f"{label}: no counted cycles in {log}. That is a failed run, not a fast core.")
    print(f"\n== {label} ==")
    print(f"  programs        {programs}")
    print(f"  cycles          {cycles}")
    print(f"  issues          {issues}   CPI {cycles / issues:.3f}")
    print(f"  redirects       {redirects}   {100 * redirects / issues:.2f}% of issues, "
          f"{100 * redirects / cycles:.2f}% of cycles")
    for stages in (1, 2):
        per_issue = cycles + stages * issues
        per_redirect = cycles + stages * redirects
        print(f"  +{stages} fetch stage{'s' if stages > 1 else ' '}: "
              f"one bubble per issue {per_issue / cycles:.3f}x cycles; "
              f"one per redirect {per_redirect / cycles:.3f}x cycles")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=2000, help="Dhrystone iterations")
    args = parser.parse_args()

    os.chdir(ROOT)
    work = pathlib.Path(os.environ.get("DEPTH_WORK", "depth.out"))
    work.mkdir(exist_ok=True)
    source = work / "depth_cycles.cc"
    source.write_text(patch(pathlib.Path("test/cxxrtl.cc")))

    subprocess.run(["make", "-s", "sim"], check=True)
    datdir = subprocess.run(["yosys-config", "--datdir"], check=True,
                            capture_output=True, text=True).stdout.strip()
    binary = work / "depth-sim"
    subprocess.run(
        ["c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-I", "test",
         "-isystem", f"{datdir}/include/backends/cxxrtl/runtime",
         str(source), "-o", str(binary)], check=True)

    env = dict(os.environ, STALL_REPORT="1")

    suite_log = work / "suite.redirects"
    suite_log.unlink(missing_ok=True)
    subprocess.run(["./test/run_tests.sh", str(binary), "test/asm",
                    "test/EXPECTED_FAIL", "test/OBSERVED_FLOOR"],
                   env=dict(env, DEPTH_LOG=str(suite_log.resolve())),
                   stdout=(work / "suite.log").open("w"), stderr=subprocess.STDOUT)

    dhry_log = work / "dhry.redirects"
    dhry_log.unlink(missing_ok=True)
    dhry_flags = ("-march=rv32imac_zicsr_zifencei -mabi=ilp32 -O2 -std=c11 -ffreestanding "
                  "-fno-tree-loop-distribute-patterns -Wall -Wextra -Werror")
    subprocess.run(["./test/bench/run_dhrystone.sh", str(binary), str(args.runs),
                    "4000000", dhry_flags],
                   env=dict(env, DEPTH_LOG=str(dhry_log.resolve())),
                   stdout=(work / "dhry.log").open("w"), stderr=subprocess.STDOUT)

    report("the .S and .c suite", suite_log)
    report(f"Dhrystone, {args.runs} runs", dhry_log)


if __name__ == "__main__":
    main()
