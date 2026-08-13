#!/usr/bin/env python3
"""Lockstep co-simulation of this core against the Sail RISC-V model.

One program per invocation; test/run_cosim.sh runs the suite and grades it
against test/COSIM_EXPECTED_FAIL. `make test` neither builds nor runs any of
this, and it stays that way: CI gates on it in a job of its own, which fetches
Sail at a verified digest before running anything. See docs/adr/0032 for what
it is for and what it costs, and docs/adr/0039 for what suite-wide integration
changed.

WHAT IS ACTUALLY COMPARED
-------------------------
The core's REAL architectural register file -- `uut regfile regs`, read
through cxxrtl `debug_items` by test/cosim.cc -- against the register file of
RISC-V International's own executable specification.  No rvfi_* signal is read
on either side.  That is the whole point: test/monitor.v and the riscv-formal
check set both check the core's SELF-REPORT (rvfi_rd_wdata against a spec model
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
which sections.lds places immediately after -- began four bytes INSIDE that
window: Sail answered every `lw` from 0x10004 with zero and eight programs
"diverged" for a reason that was entirely this repo's fault.  The spike
worked around it by stripping the symbol from a throwaway ELF copy
and then bounding the reference run with `--inst-limit`, checking it had
reached the pass/fail spin loop by looking for a self-loop at one PC.

Both workarounds are gone.  test/asm/riscv_test.h now emits `tohost` as an
8-byte-aligned `.dword`, which is what the HTIF protocol always
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

THE REFERENCE MODEL IS CONFIGURED AS THIS CORE, NOT AS A DEFAULT MACHINE
-----------------------------------------------------------------------
test/sail/rv32imc_zicsr.json is a COMPLETE `--config`, not a
`--config-override` on the model's default RV32 machine.  An override inherits
everything it does not mention, and that is how a reference model with
atomics, bit-manipulation, float, supervisor mode, user mode and vectors
became the thing this core was cross-checked against.  `--config` is rejected
outright if a key is missing, so nothing is inherited silently.  See
docs/adr/0043 and the header of the config itself.

WHAT REMAINS NOT COMPARABLE, AND WHY THAT IS A SHORT EXPLICIT LIST
------------------------------------------------------------------
Three machine CSRs hold values that are implementation-defined AND that
sail-riscv 0.13.1 has no knob for, so no configuration makes the two sides
agree.  Reading one of them into a GPR parks such a value in an architectural
register at a comparison point.  NONCOMPARABLE_CSRS below names them, one
reason each, and the comparison skips THE VALUE of exactly the one register
such a read writes -- never the register's identity, never the position of the
change in the sequence, and never anything computed from it afterwards.  A
core that failed to write the register, wrote a different one, or wrote extra
ones still diverges.  Both the count and every skipped change are printed, so
this can never quietly become "compared nothing".
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
SAIL_CONFIG = os.path.join(REPO, "test", "sail", "rv32imc_zicsr.json")
# `make sail-setup` unpacks the release here, outside any checkout. The install
# used to live in the gitignored tools/, and a git worktree gets tracked files
# only, so co-simulation was unavailable from every worktree while the main
# checkout ran it fine. The Makefile computes this same path from TOOL_CACHE;
# test/tool_cache_test.sh is what says the two still agree.
TOOL_CACHE = os.path.join(
    os.environ.get("XDG_CACHE_HOME")
    or os.path.join(os.path.expanduser("~"), ".cache"),
    "little-cpu")
SAIL_DIR = os.path.join(TOOL_CACHE, "sail")
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

# CSRs whose VALUE this core and the reference model are both entitled to
# answer differently, and which sail-riscv 0.13.1 gives no way to configure.
# The whole of the rest of the disagreement between the two machines lives in
# test/sail/rv32imc_zicsr.json; this is what is left over after that file is as
# faithful as the schema allows.
#
# Each entry costs exactly one register's VALUE at the change the read
# produces. Which register, and where in the sequence, are still compared.
#
# ADDING AN ENTRY IS TWO CLAIMS, not one, and the second is the one that gets
# forgotten: (1) no configuration can close the gap -- check
# `--print-config-schema` first; (2) every program that reads the CSR still
# takes the SAME BRANCHES on both machines. An exemption relaxes a value, and
# a value nothing branches on is all it can relax. `mie` and `mip` are the
# worked example and are deliberately NOT here: this core reads them as zero
# and the model reads mip.MTIP set, so a program asserting they are zero runs
# to `fail` on one side and `pass` on the other. That is a different program,
# not a different value, and it belongs in a bench with no reference model in
# it -- test/csr_tb.v, where it is (docs/adr/0043).
NONCOMPARABLE_CSRS = {
    0xB00: "mcycle counts CYCLES; an ISA model has no pipeline. No setting "
           "makes these agree, and one that did would mean this core retires "
           "one instruction per cycle. test/asm/minstret.S asserts only "
           "monotonicity and a bound, which both machines satisfy",
    0xB80: "mcycleh -- the upper half of the same counter. It reads zero on "
           "both sides for every program in this suite today; it is here so "
           "that stops being load-bearing",
}

SYSTEM_OPCODE = 0x73
# funct3 for csrrw/csrrs/csrrc and their immediate forms. funct3 == 0 is
# ecall/ebreak/mret/wfi, which read no CSR into a register.
CSR_FUNCT3 = {0b001, 0b010, 0b011, 0b101, 0b110, 0b111}


def noncomparable_csr_rd(insn):
    """rd, if `insn` reads one of NONCOMPARABLE_CSRS into a real register.

    Returns None otherwise -- including for the `csrw`/`csrrw x0, ...` forms,
    which write the CSR and leave no architectural register holding its value,
    and for every compressed encoding (the low two bits are never 0b11, and no
    compressed encoding is a CSR access).
    """
    if (insn & 0x7F) != SYSTEM_OPCODE:
        return None
    if ((insn >> 12) & 0x7) not in CSR_FUNCT3:
        return None
    if ((insn >> 20) & 0xFFF) not in NONCOMPARABLE_CSRS:
        return None
    rd = (insn >> 7) & 0x1F
    return rd if rd != 0 else None


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
    """(pin_line, binary_sha256) from the install's .sail-pin, or (None, None)."""
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
    2. SAIL_BIN in the shared tool cache -- must match the SHA-256 that
       `make sail-setup` recorded in SAIL_STAMP after verifying the release
       tarball. A mismatch is fatal, not a warning: nobody named this path, so
       an unexpected binary here is a substituted one, and it sits outside
       every checkout so it never shows up in `git status` or in review.

    There is deliberately no bare PATH fallback. It executed whatever
    `sail_riscv_sim` happened to be first on PATH, with no version or digest
    check and nobody having asked for that particular build; naming it in
    SAIL_RISCV_SIM is one environment variable away and is then a choice.

    What this does NOT establish: the stamp is checked against the binary, not
    against the Makefile. A whole install tree substituted along with its stamp
    passes here. `make cosim-run` catches that -- the Makefile compares
    line 1 of the stamp to its own `override` pin before running anything --
    so the two checks are complementary and a direct ./test/cosim.py
    invocation only gets this one.

    Opt-in by construction: nothing in `make test` reaches this
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
        f"release into {SAIL_DIR}/, or set SAIL_RISCV_SIM to an existing build."
    )


def assemble(cc, src, outdir):
    """Build one program from test/asm exactly as test/run_tests.sh does, and
    emit the ELF (for Sail) plus the two objcopy images (for the cxxrtl runner).

    A `.c` program is linked against test/asm/boot.lds and test/crt0.S, which
    puts `.data`'s load address in ROM and copies it into RAM before main; a
    `.S` program is freestanding and has its `.data` poked straight into the
    simulated RAM. Sail is a real ELF loader, so it honours the load addresses
    and needs no help with either shape.

    The reference model gets the SAME ELF, unmodified -- no stripped symbol,
    no throwaway copy. If the two legs ever stop building from
    identical bytes, this is the function that would have to say so."""
    base = os.path.splitext(os.path.basename(src))[0]
    elf = os.path.join(outdir, base + ".elf")
    rom = os.path.join(outdir, base + ".rom.hex")
    ram = os.path.join(outdir, base + ".ram.hex")
    objcopy = cc[: -len("gcc")] + "objcopy"
    if src.endswith(".c"):
        build = [cc, "-march=rv32imac_zicsr_zifencei", "-mabi=ilp32", "-nostdlib",
                 "-Os", "-std=c11", "-ffreestanding",
                 "-fno-tree-loop-distribute-patterns",
                 "-Wall", "-Wextra", "-Werror", "-I", ASM_DIR,
                 "-T", os.path.join(ASM_DIR, "boot.lds"),
                 os.path.join(REPO, "test", "crt0.S"), src, "-o", elf]
        rom_args = ["--only-section=.text", "--only-section=.data"]
        ram_args = ["--only-section=.tohost"]
    else:
        build = [cc, "-march=rv32imac_zicsr_zifencei", "-mabi=ilp32", "-nostdlib",
                 "-I", ASM_DIR,
                 "-T", os.path.join(ASM_DIR, "sections.lds"), src, "-o", elf]
        rom_args = ["--only-section=.text"]
        ram_args = ["--remove-section=.text"]
    log = subprocess.run(build, capture_output=True, text=True)
    if log.returncode != 0:
        raise Fatal(f"assembling {src} failed:\n{log.stdout}{log.stderr}")
    for args, out in ((rom_args, rom), (ram_args, ram)):
        subprocess.run(
            [objcopy, "-O", "verilog", "--verilog-data-width=4", *args, elf, out],
            check=True, capture_output=True,
        )
    return elf, rom, ram


def sail_verdict(output):
    """The reference model's HTIF verdict, in the vocabulary test/cosim.cc
    prints for the core, or None if the run did not reach one.

    Sail terminates on the doubleword `tohost` write and announces
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
        [sail, "--config", SAIL_CONFIG,
         "--inst-limit", str(inst_limit), "--trace-instr", "--trace-gpr",
         "--trace-output", trace, elf],
        capture_output=True, text=True,
    )
    if not os.path.exists(trace):
        raise Fatal(f"sail produced no trace:\n{proc.stdout}{proc.stderr}")

    regs = [0] * 32
    records = []       # (change_index, sail_idx, pc, disasm, {reg: val}, nc_rd)
    saw_insn = False
    current = None     # (idx, pc, disasm, nc_rd)
    pending = {}

    def flush():
        nonlocal current, pending
        if current is None:
            return
        changed = {r: v for r, v in pending.items() if regs[r] != v}
        for r, v in changed.items():
            regs[r] = v
        # A non-comparable CSR read is recorded whether or not it changed a
        # register on THIS side. It has to be: the two sides read different
        # values, so one of them can land on the value the register already
        # holds -- invisible to a distinct-state reduction -- while the other
        # does not. Emitting the event unconditionally is what lets compare()
        # stay aligned across that asymmetry instead of reporting it as a
        # length divergence.
        if changed or current[3] is not None:
            records.append((len(records), current[0], current[1], current[2],
                            changed, current[3]))
        current, pending = None, {}

    with open(trace) as fh:
        for line in fh:
            m = INSN_RE.match(line)
            if m:
                flush()
                current = (int(m.group("idx")), int(m.group("pc"), 16),
                           m.group("disasm"),
                           noncomparable_csr_rd(int(m.group("insn"), 16)))
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
    """Return (status, report_lines, skipped). Reports the FIRST divergence
    with instruction number, PC and both values, per the spike's acceptance
    criteria.

    `status` is "AGREE" or one of the DISAGREE labels test/run_cosim.sh
    baselines against test/COSIM_EXPECTED_FAIL. The labels distinguish HOW the
    two disagreed, so a baselined program that starts diverging somewhere else
    is a red gate rather than a match.

    `skipped` lists every change whose VALUE was not comparable -- a read of a
    NONCOMPARABLE_CSRS entry into a register. The caller prints it. Two cursors
    rather than one index because such a read may produce a change on one side
    and not the other: it lands on a different value on each, and one of those
    values can be the one the register already held.

    What is NOT relaxed: the skipped change still has to be a change to
    EXACTLY that register on the core side to be consumed, everything before
    and after it is compared normally, and a value computed from the register
    afterwards is compared like anything else. A core that wrote the wrong
    register, wrote extra registers, or drifted by a change still diverges."""
    out = []
    skipped = []
    i = j = 0
    while i < len(sail_records) and j < len(dut_records):
        _, sidx, spc, sdis, swrites, nc_rd = sail_records[i]
        _, cycle, dwrites, dpc = dut_records[j]
        if nc_rd is not None:
            note = (f"  #{i}: sail instruction #{sidx} pc=0x{spc:08x} {sdis}"
                    f"  -- x{nc_rd}'s value is not comparable")
            if set(dwrites) == {nc_rd}:
                note += (f" (sail 0x{swrites.get(nc_rd, 0):08x}, "
                         f"core 0x{dwrites[nc_rd]:08x})")
                j += 1
            else:
                note += " (no matching change on the core side)"
            skipped.append(note)
            i += 1
            continue
        if swrites != dwrites:
            out.append(f"DIVERGENCE at architectural change #{i}")
            out.append(f"  sail instruction #{sidx}  pc=0x{spc:08x}  {sdis}")
            out.append(f"  sail : {fmt(swrites)}")
            out.append(f"  core : {fmt(dwrites)}   (cycle {cycle}, "
                       f"decode pc=0x{dpc:08x})")
            return f"DISAGREE AT {i}", out, skipped
        i += 1
        j += 1

    # A trailing run of non-comparable reads on the sail side that the core
    # never turned into a change is not a length divergence.
    while i < len(sail_records) and sail_records[i][5] is not None:
        skipped.append(f"  #{i}: sail instruction #{sail_records[i][1]} "
                       f"{sail_records[i][3]}  -- trailing, not comparable")
        i += 1

    if i != len(sail_records) or j != len(dut_records):
        out.append(
            f"DIVERGENCE in length: sail made {len(sail_records) - i} "
            f"comparable architectural register-file changes more than the "
            f"core, or the core made {len(dut_records) - j} more than sail "
            f"(sail {len(sail_records)} records, core {len(dut_records)})"
        )
        if i < len(sail_records):
            extra = sail_records[i]
            out.append(f"  first unmatched (sail): instruction #{extra[1]} "
                       f"pc=0x{extra[2]:08x} {extra[3]} -> {fmt(extra[4])}")
        else:
            extra = dut_records[j]
            out.append(f"  first unmatched (core): cycle {extra[1]} "
                       f"-> {fmt(extra[2])} (decode pc=0x{extra[3]:08x})")
        return "DISAGREE LENGTH", out, skipped
    return "AGREE", out, skipped


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
                    help="a test/asm/*.S or *.c file name (default: add.S)")
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

    status, report, skipped = compare(sail_records, dut_records)
    if status == "AGREE" and sail_end != dut_end:
        # Unreachable by construction if the register comparison is doing its
        # job -- the verdict is a store of a value the program computed into a
        # register first -- which is exactly why it is worth asserting: if it
        # ever fires, the register comparison stopped comparing.
        status = "DISAGREE VERDICT"
        report = [f"DIVERGENCE in verdict: sail ran the program to {sail_end}, "
                  f"the core to {dut_end}, with identical register traces"]

    # Printed for every outcome, and unconditionally -- not behind --quiet.
    # Skipping a value is the one thing here that makes the comparison weaker,
    # so it is the one thing that must never be invisible in a log. The empty
    # case prints nothing.
    if skipped:
        print(f"NOT COMPARED BY VALUE ({len(skipped)}, "
              f"see NONCOMPARABLE_CSRS in test/cosim.py):")
        for line in skipped:
            print(line)

    if status == "AGREE":
        print(f"AGREE {name}: {len(sail_records) - len(skipped)} architectural "
              f"register-file changes, identical in order and value "
              f"({len(skipped)} not comparable by value); "
              f"both ran to {sail_end}.")
        emit(status)
        return 0
    print(f"DISAGREE {name}")
    for line in report:
        print(line)
    emit(status)
    return 1


if __name__ == "__main__":
    sys.exit(main())
