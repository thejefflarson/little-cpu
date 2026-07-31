#!/usr/bin/env python3
"""Lockstep co-simulation of this core against the Sail RISC-V model.

A spike, not a fourth verification leg: `make test` neither builds nor runs
this, and CI does not gate on it. See docs/adr/0031 for what it is for and
what it costs.

WHAT IS ACTUALLY COMPARED
-------------------------
The core's REAL architectural register file -- `uut regfile regs`, read
through cxxrtl `debug_items` by test/cosim.cc -- against the register file of
RISC-V International's own executable specification.  No rvfi_* signal is read
on either side.  That is the whole point: test/monitor.v and the riscv-formal
ladder both check the core's SELF-REPORT (rvfi_rd_wdata against a spec model
evaluated on rvfi_rs1_rdata / rvfi_rs2_rdata / rvfi_insn), so a core that
mis-reports a value and computes with that same mis-reported value satisfies
both.  This compares state that neither side got to describe.

Both sides are reduced to the same canonical form: the sequence of DISTINCT
architectural register-file states.  A write of a value the register already
holds is architecturally invisible; Sail traces it and a state snapshot does
not, so comparing states rather than write events makes the two agree on what
counts as an event without special-casing.  Writes to x0 are dropped on the
Sail side for the same reason -- rtl/regfile.v guards them and the ISA makes
them unobservable.

HTIF HAS TO BE DISABLED, AND THAT IS A REAL FINDING
---------------------------------------------------
Sail auto-detects HTIF from the ELF's `tohost` symbol and then claims the
whole DOUBLEWORD at that address as an IO window.  ADR-0008 puts `tohost` at
the base of the RAM region and test/asm/riscv_test.h emits it as a 32-bit
`.word`, so `.data` -- every load/store test's TEST_DATA -- starts four bytes
later, INSIDE that window.  Left alone, Sail answers every `lw` from 0x10004
with zero (`--trace-mem` shows `htif[0x000010004] -> 0x0` ahead of the read)
and eight of this suite's programs "diverge" for a reason that is entirely
the harness's fault.

It also does not help on the other side: HTIF acts on a 64-bit doubleword
write, and RVTEST_PASS stores with `sw`, so the run does not terminate on the
verdict either (`--trace-htif` logs the write and keeps going).

So the ELF handed to Sail gets its `tohost` symbol stripped -- a harness-side
`objcopy` on a throwaway copy, changing nothing about what either existing
sim leg builds or how ADR-0008's protocol works.  The reference run is then
bounded with `--inst-limit` and confirmed to have reached the pass/fail spin
loop by checking that its last instructions are a self-loop at one PC.

The alternative -- padding `tohost` to a full doubleword in riscv_test.h so
it stops overlapping test data, which is what upstream riscv-tests does -- is
the better long-term fix and would buy a clean HTIF exit as well.  It is a
change to shared test infrastructure that both existing legs read, so it
belongs in the integration ticket, not in a spike.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASM_DIR = os.path.join(REPO, "test", "asm")
MEMORY_MAP = os.path.join(REPO, "test", "sail", "memory-map.json")

# `[123] [M]: 0x00000006 (0x00208733) add x14, x1, x2      test_2+4`
INSN_RE = re.compile(
    r"^\[(?P<idx>\d+)\]\s+\[\w+\]:\s+0x(?P<pc>[0-9A-Fa-f]+)\s+"
    r"\((?P<insn>0x[0-9A-Fa-f]+)\)\s+(?P<disasm>.*?)\s*$"
)
# `x14 <- 0x0000000A`
GPR_RE = re.compile(r"^x(?P<reg>\d+)\s+<-\s+0x(?P<val>[0-9A-Fa-f]+)\s*$")
# `CS 0 41 x14=0000000a @pc=00000012`. The `CS ` prefix separates the runner's
# records from test/testbench.v's per-cycle $display chatter, which cxxrtl
# emits onto the same stdout; see test/cosim.cc.
DUT_RE = re.compile(
    r"^CS\s+(?P<idx>\d+)\s+(?P<cycle>\d+)\s+(?P<writes>(?:x\d+=[0-9a-f]{8}\s+)+)"
    r"@pc=(?P<pc>[0-9a-f]{8})\s*$"
)


class Fatal(Exception):
    pass


def find_cross_compiler():
    for cc in ("riscv64-elf-gcc", "riscv64-unknown-elf-gcc"):
        if shutil.which(cc):
            return cc
    raise Fatal(
        "no RISC-V cross compiler found (tried riscv64-elf-gcc, "
        "riscv64-unknown-elf-gcc). Run 'make setup'."
    )


def find_sail(explicit):
    """Locate sail_riscv_sim: --sail, $SAIL_RISCV_SIM, tools/sail, then PATH.

    Opt-in by construction (the ticket's constraint, and ADR-0031's): nothing
    in `make test` reaches this function, and when the binary is absent the
    message names the target that installs it rather than a build failure.
    """
    for candidate in (
        explicit,
        os.environ.get("SAIL_RISCV_SIM"),
        os.path.join(REPO, "tools", "sail", "bin", "sail_riscv_sim"),
    ):
        if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    found = shutil.which("sail_riscv_sim")
    if found:
        return found
    raise Fatal(
        "sail_riscv_sim not found. Run 'make sail-setup' to install the pinned "
        "release into tools/sail/, or set SAIL_RISCV_SIM to an existing build."
    )


def assemble(cc, src, outdir):
    """Assemble one test/asm/*.S exactly as test/run_tests.sh does, and emit
    the ELF (for Sail) plus the two objcopy images (for the cxxrtl runner)."""
    base = os.path.splitext(os.path.basename(src))[0]
    elf = os.path.join(outdir, base + ".elf")
    rom = os.path.join(outdir, base + ".rom.hex")
    ram = os.path.join(outdir, base + ".ram.hex")
    objcopy = cc[: -len("gcc")] + "objcopy"
    log = subprocess.run(
        [cc, "-march=rv32imc_zicsr", "-mabi=ilp32", "-nostdlib", "-I", ASM_DIR,
         "-T", os.path.join(ASM_DIR, "sections.lds"), src, "-o", elf],
        capture_output=True, text=True,
    )
    if log.returncode != 0:
        raise Fatal(f"assembling {src} failed:\n{log.stdout}{log.stderr}")
    for args, out in (
        (["--only-section=.text"], rom),
        (["--remove-section=.text"], ram),
    ):
        subprocess.run(
            [objcopy, "-O", "verilog", "--verilog-data-width=4", *args, elf, out],
            check=True, capture_output=True,
        )
    # A throwaway copy with `tohost` removed from the symbol table, so Sail
    # does not auto-detect HTIF and shadow the test data that ADR-0008's
    # memory map places immediately after it. See the module docstring.
    sail_elf = os.path.join(outdir, base + ".sail.elf")
    subprocess.run([objcopy, "--strip-symbol=tohost", elf, sail_elf],
                   check=True, capture_output=True)
    return sail_elf, rom, ram


def run_sail(sail, elf, inst_limit, outdir):
    """Run the reference model and return its list of distinct register-file
    states, each tagged with the instruction that produced it."""
    trace = os.path.join(outdir, "sail.trace")
    proc = subprocess.run(
        [sail, "--rv32", "--config-override", MEMORY_MAP,
         "--inst-limit", str(inst_limit), "--trace-instr", "--trace-gpr",
         "--trace-output", trace, elf],
        capture_output=True, text=True,
    )
    if not os.path.exists(trace):
        raise Fatal(f"sail produced no trace:\n{proc.stdout}{proc.stderr}")

    regs = [0] * 32
    records = []       # (change_index, sail_idx, pc, disasm, {reg: val})
    tail_pcs = []
    current = None     # (idx, pc, disasm)
    pending = {}

    def flush():
        nonlocal current, pending
        if current is None:
            return
        changed = {r: v for r, v in pending.items() if regs[r] != v}
        for r, v in changed.items():
            regs[r] = v
        if changed:
            records.append((len(records), current[0], current[1], current[2], changed))
        current, pending = None, {}

    with open(trace) as fh:
        for line in fh:
            m = INSN_RE.match(line)
            if m:
                flush()
                current = (int(m.group("idx")), int(m.group("pc"), 16),
                           m.group("disasm"))
                tail_pcs.append(current[1])
                if len(tail_pcs) > 8:
                    tail_pcs.pop(0)
                continue
            m = GPR_RE.match(line)
            if m and current is not None:
                reg = int(m.group("reg"))
                if reg != 0:  # x0 is hardwired; rtl/regfile.v guards it too
                    pending[reg] = int(m.group("val"), 16)
    flush()

    if not tail_pcs:
        raise Fatal(f"sail traced no instructions:\n{proc.stdout}{proc.stderr}")
    # riscv_test.h's RVTEST_PASS/RVTEST_FAIL both end in `1: j 1b`, so a run
    # that reached a verdict is parked on one PC. If it is not, --inst-limit
    # cut the reference short and any length mismatch below would be an
    # artefact of this harness rather than a finding.
    converged = len(set(tail_pcs)) == 1 and len(tail_pcs) == 8
    return records, converged, tail_pcs[-1]


def run_dut(binary, rom, ram, cycles):
    proc = subprocess.run(
        [binary, "--rom", rom, "--ram", ram, "--cycles", str(cycles)],
        capture_output=True, text=True,
    )
    records = []   # (change_index, cycle, {reg: val}, pc)
    verdict = None
    for line in proc.stdout.splitlines():
        if not line.startswith("CS "):
            continue  # test/testbench.v's $display chatter
        if line.startswith("CS END "):
            verdict = line[len("CS END "):].strip()
            continue
        m = DUT_RE.match(line)
        if not m:
            raise Fatal(f"unparseable cosim record line: {line!r}")
        writes = {}
        for w in m.group("writes").split():
            reg, val = w.split("=")
            writes[int(reg[1:])] = int(val, 16)
        records.append((int(m.group("idx")), int(m.group("cycle")), writes,
                        int(m.group("pc"), 16)))
    if verdict is None:
        raise Fatal(
            f"cosim produced no `CS END` line (exit {proc.returncode}):\n{proc.stderr}"
        )
    return records, verdict


def fmt(writes):
    return " ".join(f"x{r}=0x{v:08x}" for r, v in sorted(writes.items()))


def compare(sail_records, dut_records):
    """Return (ok, list_of_report_lines). Reports the FIRST divergence with
    instruction number, PC and both values, per the spike's acceptance
    criteria."""
    out = []
    for i in range(min(len(sail_records), len(dut_records))):
        _, sidx, spc, sdis, swrites = sail_records[i]
        _, cycle, dwrites, dpc = dut_records[i]
        if swrites != dwrites:
            out.append(f"DIVERGENCE at architectural change #{i}")
            out.append(f"  sail instruction #{sidx}  pc=0x{spc:08x}  {sdis}")
            out.append(f"  sail : {fmt(swrites)}")
            out.append(f"  core : {fmt(dwrites)}   (cycle {cycle}, "
                       f"decode pc=0x{dpc:08x})")
            return False, out
    if len(sail_records) != len(dut_records):
        n = min(len(sail_records), len(dut_records))
        out.append(
            f"DIVERGENCE in length: sail made {len(sail_records)} "
            f"architectural register-file changes, the core made "
            f"{len(dut_records)}"
        )
        longer, label = ((sail_records, "sail") if len(sail_records) > n
                         else (dut_records, "core"))
        extra = longer[n]
        if label == "sail":
            out.append(f"  first unmatched ({label}): instruction #{extra[1]} "
                       f"pc=0x{extra[2]:08x} {extra[3]} -> {fmt(extra[4])}")
        else:
            out.append(f"  first unmatched ({label}): cycle {extra[1]} "
                       f"-> {fmt(extra[2])} (decode pc=0x{extra[3]:08x})")
        return False, out
    return True, out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("program", nargs="?", default="add.S",
                    help="a test/asm/*.S file name (default: add.S)")
    ap.add_argument("--cosim-binary", default=os.path.join(REPO, "cosim"))
    ap.add_argument("--sail", default=None, help="path to sail_riscv_sim")
    ap.add_argument("--cycles", type=int, default=5000,
                    help="cxxrtl cycle budget (test/run_tests.sh uses 5000)")
    ap.add_argument("--inst-limit", type=int, default=20000,
                    help="sail instruction budget")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    try:
        cc = find_cross_compiler()
        sail = find_sail(args.sail)
        if not os.path.isfile(args.cosim_binary):
            raise Fatal(f"{args.cosim_binary} not built. Run 'make cosim'.")
        src = args.program
        if not os.path.isabs(src):
            src = os.path.join(ASM_DIR, os.path.basename(src))
        if not os.path.isfile(src):
            raise Fatal(f"no such test program: {src}")

        with tempfile.TemporaryDirectory(prefix="littlecpu-cosim.") as outdir:
            elf, rom, ram = assemble(cc, src, outdir)
            sail_records, converged, last_pc = run_sail(
                sail, elf, args.inst_limit, outdir)
            dut_records, verdict = run_dut(
                args.cosim_binary, rom, ram, args.cycles)
    except Fatal as e:
        print(f"error: {e}", file=sys.stderr)
        return 3

    name = os.path.basename(src)
    if not args.quiet:
        print(f"program            : {name}")
        print(f"sail               : {sail}")
        print(f"sail changes       : {len(sail_records)}  "
              f"(spin-loop reached: {converged}, last pc=0x{last_pc:08x})")
        print(f"core changes       : {len(dut_records)}   (tohost: {verdict})")

    if not converged:
        print(f"INCONCLUSIVE {name}: sail hit --inst-limit "
              f"{args.inst_limit} without reaching the pass/fail spin loop; "
              f"raise it.", file=sys.stderr)
        return 3
    if verdict.startswith("TIMEOUT"):
        print(f"INCONCLUSIVE {name}: the core hit its {args.cycles}-cycle "
              f"budget without a tohost verdict.", file=sys.stderr)
        return 3

    ok, report = compare(sail_records, dut_records)
    if ok:
        print(f"AGREE {name}: {len(sail_records)} architectural register-file "
              f"changes, identical in order and value.")
        return 0
    print(f"DISAGREE {name}")
    for line in report:
        print(line)
    return 1


if __name__ == "__main__":
    sys.exit(main())
