#!/usr/bin/env python3
"""Count REDIRECTS and KILL (mispredict) cycles on the suite, Dhrystone and CoreMark,
base against the fetch-ahead-with-discard prototype (soc/fetch_ahead/apply.sh).

WHY A NEW BUCKET. `make cycles`' accounting sums `issue` and eight named `stall`
reasons to `counted_cycles`, and nothing between them describes "nothing issued,
but not because of a stall reason either." Under fetch-ahead-with-discard a
mispredicted word is discarded in exactly the cycle after the instruction that
redirected -- `rtl/decoder.v`'s `wrongpath` -- and it is neither an issue (`out`
is zeroed) nor one of the eight `stall` reasons (`stall` is unchanged and does
not gate it, on purpose, so a discard never waits on a hazard it cannot have).
Left unpatched, `test/cxxrtl.cc` would count every kill cycle as an ISSUE, the
same blind spot ADR-0078 recorded for its own one-deep kill.

HOW. `test/cxxrtl.cc`, read from the PROTOTYPE tree (soc/fetch_ahead/apply.sh's
output, never the checkout's own copy), is patched at checked anchors -- the same
shape as soc/depth/cycles.py -- to probe `wrongpath` and `is_redirect` and print
two more fields on the STALLS line. A missing anchor stops this script.

Usage: cycles.py <applied-tree-dir> [--dhry-runs N] [--coremark-iters N]
"""

import argparse
import os
import pathlib
import re
import subprocess
import sys

ANCHORS = {
    "decl": "  const cxxrtl::debug_item *stall_any = nullptr;",
    "lookup": '      stall_any = &all_debug_items.at("uut decoder stall").at(0);',
    "counters": "  uint64_t unattributed_cycles = 0;",
    "report": (
        '    std::printf(" unattributed=%llu lsissue=%u lsedge=%u lsbypass=%u\\n",\n'
        '                 (unsigned long long)unattributed_cycles, ls_issues->curr[0],\n'
        '                 ls_edges->curr[0], ls_bypasses->curr[0]);'),
    "branch": "      if ((stall_any->curr[0] & 1) == 0) {\n        issue_cycles++;\n      } else {",
}

DECL = """
  const cxxrtl::debug_item *wrongpath_item = nullptr;
  const cxxrtl::debug_item *is_redirect_item = nullptr;"""

LOOKUP = """
      wrongpath_item = &all_debug_items.at("uut decoder wrongpath").at(0);
      is_redirect_item = &all_debug_items.at("uut decoder is_redirect").at(0);"""

COUNTERS = """
  uint64_t kill_cycles = 0;
  uint64_t redirect_cycles = 0;"""

REPORT = """
    std::printf(" kill=%llu redirect=%llu", (unsigned long long)kill_cycles,
                 (unsigned long long)redirect_cycles);"""

# test/run_tests.sh keeps only the console STALLS line off a run's stdout, so the
# per-program totals this script needs go to a file named in the environment instead.
FILE_REPORT = """
    if (const char *path = std::getenv("DEPTH_LOG")) {
      if (std::FILE *f = std::fopen(path, "a")) {
        std::fprintf(f, "%s cycles=%llu issue=%llu kill=%llu redirect=%llu\\n",
                     args.rom_path.c_str(), (unsigned long long)counted_cycles,
                     (unsigned long long)issue_cycles,
                     (unsigned long long)kill_cycles,
                     (unsigned long long)redirect_cycles);
        std::fclose(f);
      }
    }
"""

BRANCH = """      if ((wrongpath_item->curr[0] & 1) != 0) {
        kill_cycles++;
      } else if ((stall_any->curr[0] & 1) == 0) {
        issue_cycles++;
        if ((is_redirect_item->curr[0] & 1) != 0) redirect_cycles++;
      } else {"""

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
    text = text.replace(ANCHORS["decl"], ANCHORS["decl"] + DECL, 1)
    text = text.replace(ANCHORS["lookup"], ANCHORS["lookup"] + LOOKUP, 1)
    text = text.replace(ANCHORS["counters"], ANCHORS["counters"] + COUNTERS, 1)
    # REPORT is inserted BEFORE the anchor's own printf, which is the one call on this
    # line that terminates it with `\n` -- every other call on the STALLS line, this one
    # included, does not, so the physical line stays one line. FILE_REPORT goes AFTER it,
    # still inside `report_counts`, where `kill_cycles`/`redirect_cycles` are in scope.
    text = text.replace(ANCHORS["report"], REPORT + "\n" + ANCHORS["report"] + FILE_REPORT, 1)
    text = text.replace(ANCHORS["branch"], BRANCH, 1)
    return text

def totals(log):
    cycles = issues = redirects = kills = 0
    programs = 0
    for line in log.read_text().splitlines():
        m = re.search(r"cycles=(\d+) issue=(\d+).*kill=(\d+) redirect=(\d+)", line)
        if not m:
            continue
        programs += 1
        cycles += int(m.group(1))
        issues += int(m.group(2))
        kills += int(m.group(3))
        redirects += int(m.group(4))
    return programs, cycles, issues, kills, redirects

def report(label, log):
    programs, cycles, issues, kills, redirects = totals(log)
    if cycles == 0:
        sys.exit(f"{label}: no counted cycles in {log}. That is a failed run, not a fast core.")
    print(f"\n== {label} ==")
    print(f"  programs        {programs}")
    print(f"  cycles          {cycles}")
    print(f"  issues          {issues}   CPI {cycles / issues:.3f}")
    print(f"  redirects       {redirects}   {100 * redirects / issues:.2f}% of issues")
    kill_line = f"  kill (bubble)   {kills}"
    if redirects:
        kill_line += (f"   {100 * kills / cycles:.2f}% of cycles, "
                       f"{100 * kills / redirects:.2f}% of redirects")
    print(kill_line)

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("applied", help="soc/fetch_ahead/apply.sh's output directory")
    parser.add_argument("--dhry-runs", type=int, default=2000)
    parser.add_argument("--coremark-iters", type=int, default=400)
    args = parser.parse_args()

    root = pathlib.Path(args.applied).resolve()
    os.chdir(root)
    work = pathlib.Path("fetch_ahead_cycles.out")
    work.mkdir(exist_ok=True)
    source = work / "cycles_sim.cc"
    source.write_text(patch(pathlib.Path("test/cxxrtl.cc")))

    subprocess.run(["make", "-s", "sim"], check=True)
    datdir = subprocess.run(["yosys-config", "--datdir"], check=True,
                             capture_output=True, text=True).stdout.strip()
    binary = work / "cycles-sim"
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
    dhry_flags = ("-march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -O2 -std=c11 -ffreestanding "
                  "-fno-tree-loop-distribute-patterns -Wall -Wextra -Werror")
    subprocess.run(["./test/bench/run_dhrystone.sh", str(binary), str(args.dhry_runs),
                     "4000000", dhry_flags],
                    env=dict(env, DEPTH_LOG=str(dhry_log.resolve())),
                    stdout=(work / "dhry.log").open("w"), stderr=subprocess.STDOUT)

    coremark_log = work / "coremark.redirects"
    coremark_log.unlink(missing_ok=True)
    coremark_flags = ("-march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -O2 -std=c11 "
                       "-ffreestanding -fno-tree-loop-distribute-patterns "
                       "-Wall -Wextra -Werror")
    subprocess.run(["./test/bench/run_coremark.sh", str(binary), str(args.coremark_iters),
                     "200000000", coremark_flags],
                    env=dict(env, DEPTH_LOG=str(coremark_log.resolve())),
                    stdout=(work / "coremark.log").open("w"), stderr=subprocess.STDOUT)

    report("the .S and .c suite", suite_log)
    report(f"Dhrystone, {args.dhry_runs} runs", dhry_log)
    report(f"CoreMark, {args.coremark_iters} iterations", coremark_log)

if __name__ == "__main__":
    main()
