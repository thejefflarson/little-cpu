#!/usr/bin/env python3
"""Lockstep co-simulation of this core against the Sail RISC-V model.

One program per invocation; test/run_cosim.sh runs the suite and grades it
against test/COSIM_EXPECTED_FAIL. Opt-in, and it stays that way: `make test`
neither builds nor runs any of this and CI does not gate on it. See
docs/adr/0032 for what it is for and what it costs, and docs/adr/0039 for
what suite-wide integration changed.

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

HTIF IS SPOKEN, NOT WORKED AROUND
--------------------------------
Sail auto-detects HTIF from the ELF's `tohost` symbol and claims the whole
DOUBLEWORD at that address as an IO window.  `tohost` used to be a 32-bit
`.word` at the base of RAM, so `.data` -- every load/store test's TEST_DATA,
which ADR-0008 places immediately after -- began four bytes INSIDE that
window: Sail answered every `lw` from 0x10004 with zero and eight programs
"diverged" for a reason that was entirely this repo's fault.  The spike
(ADR-0032) worked around it by stripping the symbol from a throwaway ELF copy
and then bounding the reference run with `--inst-limit`, checking it had
reached the pass/fail spin loop by looking for a self-loop at one PC.

Both workarounds are gone.  test/asm/riscv_test.h now emits `tohost` as an
8-byte-aligned `.dword` (ADR-0039), which is what the HTIF protocol always
specified, so the IO window covers only `tohost` itself and Sail reads test
data out of real memory.  The verdict macros write the doubleword's upper
word first and the verdict last, so the reference model TERMINATES on the
same store the cxxrtl runners stop on, and `--inst-limit` goes back to being
what it should always have been: a runaway bound, not a convergence
criterion.  A run that hits it without an HTIF verdict is INCONCLUSIVE and
says so, rather than being compared and reported as a finding.

Both sides' verdicts are compared as well as their register traces, so a run
where the reference model passes the program and the core fails it (or the
reverse) is a divergence even in the impossible case that the register
sequences matched.
"""

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASM_DIR = os.path.join(REPO, "test", "asm")
MEMORY_MAP = os.path.join(REPO, "test", "sail", "memory-map.json")
SAIL_DIR = os.path.join(REPO, "tools", "sail")
SAIL_BIN = os.path.join(SAIL_DIR, "bin", "sail_riscv_sim")
# Written by `make sail-setup` after it verifies the release tarball's SHA-256:
# line 1 is the pin (version, asset, tarball digest), line 2 is the digest of
# the unpacked sail_riscv_sim. See the Makefile's sail-setup target.
SAIL_STAMP = os.path.join(SAIL_DIR, ".sail-pin")

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


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_pin():
    """(pin_line, binary_sha256) from tools/sail/.sail-pin, or (None, None)."""
    try:
        with open(SAIL_STAMP) as fh:
            lines = [ln.strip() for ln in fh.read().splitlines() if ln.strip()]
    except OSError:
        return None, None
    if len(lines) < 2:
        return None, None
    return lines[0], lines[1].split()[0]


def find_sail(explicit):
    """Locate sail_riscv_sim and check it against the pin before running it.

    Returns (path, provenance). Two sources, in order:

    1. `--sail` / `$SAIL_RISCV_SIM` -- a build a human named on purpose. This
       is the documented escape hatch for a host with no prebuilt release
       asset (the Makefile's sail-setup names it), so it is honoured even when
       it does not match the pin -- but never silently: an unmatched binary is
       announced on stderr with its digest.
    2. `tools/sail/bin/sail_riscv_sim` -- must match the SHA-256 that
       `make sail-setup` recorded in tools/sail/.sail-pin after verifying the
       release tarball. A mismatch is fatal, not a warning: nobody named this
       path, so an unexpected binary here is a substituted one, and tools/sail
       is gitignored so it never shows up in `git status` or in review.

    There is deliberately no bare PATH fallback. It executed whatever
    `sail_riscv_sim` happened to be first on PATH, with no version or digest
    check and nobody having asked for that particular build; naming it in
    SAIL_RISCV_SIM is one environment variable away and is then a choice.

    What this does NOT establish: the stamp is checked against the binary, not
    against the Makefile. A whole tools/sail tree substituted along with its
    stamp passes here. `make cosim-run` catches that -- the Makefile compares
    line 1 of the stamp to its own `override` pin before running anything --
    so the two checks are complementary and a direct ./test/cosim.py
    invocation only gets this one.

    Opt-in by construction (ADR-0032): nothing in `make test` reaches this
    function, and when the binary is absent the message names the target that
    installs it rather than a build failure.
    """
    pin, want = read_pin()

    named = explicit or os.environ.get("SAIL_RISCV_SIM")
    if named:
        if not (os.path.isfile(named) and os.access(named, os.X_OK)):
            raise Fatal(f"{named} is not an executable file")
        got = sha256(named)
        if want is not None and got == want:
            return named, f"pinned: {pin}"
        print(f"warning: {named} was named explicitly and does not match the "
              f"pin recorded in {SAIL_STAMP}; running it unverified "
              f"(sha256 {got})", file=sys.stderr)
        return named, f"unverified, named explicitly (sha256 {got})"

    if os.path.isfile(SAIL_BIN) and os.access(SAIL_BIN, os.X_OK):
        if want is None:
            raise Fatal(
                f"{SAIL_BIN} exists but {SAIL_STAMP} does not record the "
                "digest it was verified against, so it was not installed by "
                "the current 'make sail-setup'. Re-run it: "
                f"rm -rf {SAIL_DIR} && make sail-setup"
            )
        got = sha256(SAIL_BIN)
        if got != want:
            raise Fatal(
                f"{SAIL_BIN} does not match the digest recorded when it was "
                f"verified.\n  recorded : {want}\n  on disk  : {got}\n"
                "Refusing to execute it. Start over with: "
                f"rm -rf {SAIL_DIR} && make sail-setup"
            )
        return SAIL_BIN, f"pinned: {pin}"

    raise Fatal(
        "sail_riscv_sim not found. Run 'make sail-setup' to install the pinned "
        "release into tools/sail/, or set SAIL_RISCV_SIM to an existing build."
    )


def assemble(cc, src, outdir):
    """Assemble one test/asm/*.S exactly as test/run_tests.sh does, and emit
    the ELF (for Sail) plus the two objcopy images (for the cxxrtl runner).

    The reference model gets the SAME ELF, unmodified -- no stripped symbol,
    no throwaway copy (ADR-0039). If the two legs ever stop building from
    identical bytes, this is the function that would have to say so."""
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
    return elf, rom, ram


def sail_verdict(output):
    """The reference model's HTIF verdict, in the vocabulary test/cosim.cc
    prints for the core, or None if the run did not reach one.

    Sail terminates on the doubleword `tohost` write (ADR-0039) and announces
    it: `SUCCESS` for the riscv-tests pass encoding, `FAILURE: <n>` for
    `(testnum << 1) | 1`, where n is the test number. There is deliberately no
    fallback to the process exit status -- it is 0 both for SUCCESS and for
    hitting `--inst-limit`, so reading it would turn "the reference model never
    reached the program's verdict" into "the program passed".
    """
    for line in output.splitlines():
        line = line.strip()
        if line == "SUCCESS":
            return "PASS"
        m = re.match(r"^FAILURE:\s+(\d+)", line)
        if m:
            return f"FAIL {m.group(1)}"
    return None


def run_sail(sail, elf, inst_limit, outdir):
    """Run the reference model and return its list of distinct register-file
    states, each tagged with the instruction that produced it, plus the HTIF
    verdict it terminated on (None if it never reached one)."""
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
    saw_insn = False
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
                saw_insn = True
                continue
            m = GPR_RE.match(line)
            if m and current is not None:
                reg = int(m.group("reg"))
                if reg != 0:  # x0 is hardwired; rtl/regfile.v guards it too
                    pending[reg] = int(m.group("val"), 16)
    flush()

    if not saw_insn:
        raise Fatal(f"sail traced no instructions:\n{proc.stdout}{proc.stderr}")
    return records, sail_verdict(proc.stdout + proc.stderr)


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
    """Return (status, list_of_report_lines). Reports the FIRST divergence with
    instruction number, PC and both values, per the spike's acceptance
    criteria.

    `status` is "AGREE" or one of the DISAGREE labels test/run_cosim.sh
    baselines against test/COSIM_EXPECTED_FAIL. The labels distinguish HOW the
    two disagreed, so a baselined program that starts diverging somewhere else
    is a red gate rather than a match (ADR-0035)."""
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
            return f"DISAGREE AT {i}", out
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
        return "DISAGREE LENGTH", out
    return "AGREE", out


def core_verdict(raw):
    """test/cosim.cc's terminator line reduced to sail_verdict()'s vocabulary.

    `PASS <cycle>` -> "PASS", `FAIL <n> <cycle>` -> "FAIL <n>", `TIMEOUT` ->
    None. The cycle number is dropped on purpose: it is the one part of the
    verdict the reference model has no opinion about."""
    fields = raw.split()
    if not fields or fields[0] == "TIMEOUT":
        return None
    if fields[0] == "FAIL" and len(fields) >= 2:
        return f"FAIL {fields[1]}"
    return fields[0]


# The one line test/run_cosim.sh reads. Everything else this script prints is
# for a human; this is the machine-readable verdict, and it is emitted for
# every outcome that got as far as running both sides. Its absence means the
# run did not get that far, which the suite runner reports as its own label
# rather than as a verdict about the core.
STATUS_PREFIX = "COSIM-STATUS"


def emit(status):
    print(f"{STATUS_PREFIX} {status}")


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
    ap.add_argument("--check-setup", action="store_true",
                    help="probe the toolchain, the pinned sail binary and the "
                         "cosim runner, then exit; runs no program")
    args = ap.parse_args()

    try:
        cc = find_cross_compiler()
        sail, sail_provenance = find_sail(args.sail)
        if not os.path.isfile(args.cosim_binary):
            raise Fatal(f"{args.cosim_binary} not built. Run 'make cosim'.")
        # test/run_cosim.sh calls this once before the suite, so a missing
        # toolchain or an unverified sail binary says so once, up front,
        # instead of as 52 identical per-program errors.
        if args.check_setup:
            print(f"cross compiler     : {cc}")
            print(f"sail               : {sail}")
            print(f"sail provenance    : {sail_provenance}")
            print(f"cosim runner       : {args.cosim_binary}")
            return 0
        src = args.program
        if not os.path.isabs(src):
            src = os.path.join(ASM_DIR, os.path.basename(src))
        if not os.path.isfile(src):
            raise Fatal(f"no such test program: {src}")

        with tempfile.TemporaryDirectory(prefix="littlecpu-cosim.") as outdir:
            elf, rom, ram = assemble(cc, src, outdir)
            sail_records, sail_end = run_sail(
                sail, elf, args.inst_limit, outdir)
            dut_records, dut_end_raw = run_dut(
                args.cosim_binary, rom, ram, args.cycles)
    except Fatal as e:
        print(f"error: {e}", file=sys.stderr)
        return 3

    name = os.path.basename(src)
    dut_end = core_verdict(dut_end_raw)
    if not args.quiet:
        print(f"program            : {name}")
        print(f"sail               : {sail}")
        print(f"sail provenance    : {sail_provenance}")
        print(f"sail changes       : {len(sail_records)}  "
              f"(htif verdict: {sail_end})")
        print(f"core changes       : {len(dut_records)}   "
              f"(tohost: {dut_end_raw})")

    # Neither budget is a finding. A run that ran out of one was never
    # compared against a complete reference, so reporting it as agreement or
    # as a divergence would both be lies about what was measured.
    if sail_end is None:
        print(f"INCONCLUSIVE {name}: sail hit --inst-limit "
              f"{args.inst_limit} without an HTIF verdict; raise it.",
              file=sys.stderr)
        emit("INCONCLUSIVE SAIL-LIMIT")
        return 2
    if dut_end is None:
        print(f"INCONCLUSIVE {name}: the core hit its {args.cycles}-cycle "
              f"budget without a tohost verdict.", file=sys.stderr)
        emit("INCONCLUSIVE CORE-TIMEOUT")
        return 2

    status, report = compare(sail_records, dut_records)
    if status == "AGREE" and sail_end != dut_end:
        # Unreachable by construction if the register comparison is doing its
        # job -- the verdict is a store of a value the program computed into a
        # register first -- which is exactly why it is worth asserting: if it
        # ever fires, the register comparison stopped comparing.
        status = "DISAGREE VERDICT"
        report = [f"DIVERGENCE in verdict: sail ran the program to {sail_end}, "
                  f"the core to {dut_end}, with identical register traces"]

    if status == "AGREE":
        print(f"AGREE {name}: {len(sail_records)} architectural register-file "
              f"changes, identical in order and value; both ran to {sail_end}.")
        emit(status)
        return 0
    print(f"DISAGREE {name}")
    for line in report:
        print(line)
    emit(status)
    return 1


if __name__ == "__main__":
    sys.exit(main())
