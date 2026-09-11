#!/usr/bin/env python3
"""Fail if nano/formal/rvfi_insn_check.sv has drifted from the pinned riscv-formal.

rvfi_insn_check.sv is a fork, not a vendored copy: riscv-formal ships no RV32E
spec model, so an E core's generated per-instruction checks need one extra
`ifdef` block bounding the spec model's own decoded register addresses (see
that file's header). Exactly one thing may differ from the pin: that block.

Modelled directly on formal/check-genchecks.py, and for the same reason that
file gives for not diffing and eyeballing the result: this UNDOES the
documented edit and then requires byte equality with the clone. A residual
diff is drift by construction, and it is printed.

Usage: check-rvfi-insn-check.py <upstream rvfi_insn_check.sv> <forked rvfi_insn_check.sv>
"""

import difflib
import sys

# The one edit this fork carries, as it appears in the file.
E_ASSUMPTION = """			assume(spec_valid);

`ifdef RISCV_FORMAL_E
				assume(spec_rs1_addr < 16 && spec_rs2_addr < 16 && spec_rd_addr < 16);
`endif

				if (!`rvformal_addr_valid(pc_rdata) || !insn_pma_x || mem_access_fault) begin"""

E_ASSUMPTION_UPSTREAM = """			assume(spec_valid);

				if (!`rvformal_addr_valid(pc_rdata) || !insn_pma_x || mem_access_fault) begin"""

# Boundary markers for the fork's own added paragraph.
HEADER_START = "// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.\n"
HEADER_END = "module rvfi_insn_check ("


def strip_fork_header(text, path):
    """Remove this fork's added header paragraph, returning upstream-shaped text."""
    start = text.find(HEADER_START)
    if start < 0:
        raise SystemExit(
            f"{path}: does not contain upstream's closing banner line, so the\n"
            f"fork's own header paragraph cannot be located. Re-sync from the\n"
            f"pin per this file's docstring."
        )
    start += len(HEADER_START)
    end = text.find(HEADER_END, start)
    if end < 0:
        raise SystemExit(
            f"{path}: no {HEADER_END!r} line found after the banner. This is\n"
            f"upstream's own module declaration; its absence means the file is\n"
            f"not a fork of checks/rvfi_insn_check.sv at all."
        )
    added = text[start:end]
    # Guard the delete: only comment lines and blank lines may be removed this way.
    stray = [
        line
        for line in added.splitlines()
        if line.strip() and not line.startswith("//")
    ]
    if stray:
        raise SystemExit(
            f"{path}: the header paragraph above `module rvfi_insn_check (` "
            f"contains\nnon-comment lines, which this check will not silently "
            f"drop:\n" + "\n".join(f"    {line}" for line in stray)
        )
    # Restore upstream's one-blank-line separator, not the fork's own spacing.
    return text[:start] + "\n" + text[end:]


def main(argv):
    if len(argv) != 3:
        sys.stderr.write(f"usage: {argv[0]} <upstream> <forked>\n")
        return 2

    upstream_path, forked_path = argv[1], argv[2]
    with open(upstream_path) as handle:
        upstream = handle.read()
    with open(forked_path) as handle:
        forked = handle.read()

    normalized = strip_fork_header(forked, forked_path)

    count = normalized.count(E_ASSUMPTION)
    if count != 1:
        sys.stderr.write(
            f"{argv[0]}: {forked_path}: the documented RISCV_FORMAL_E assumption\n"
            f"block appears {count} time(s), expected 1. Re-apply it exactly as\n"
            f"this file's own header states, or -- if upstream has restructured\n"
            f"this code -- update E_ASSUMPTION in {argv[0]} and say so in the ADR.\n"
        )
        return 1
    normalized = normalized.replace(E_ASSUMPTION, E_ASSUMPTION_UPSTREAM)

    if normalized == upstream:
        print(
            f"check-rvfi-insn-check: {forked_path} matches {upstream_path} at the "
            f"pin (header and the RISCV_FORMAL_E block only)"
        )
        return 0

    sys.stderr.write(
        f"{argv[0]}: {forked_path} has DRIFTED from {upstream_path}.\n"
        f"Exactly two differences are permitted -- this fork's header paragraph\n"
        f"and the RISCV_FORMAL_E assumption block -- and both are undone before\n"
        f"the comparison below, so everything shown is drift. Re-sync per the\n"
        f"recipe in the forked file's header; do not hand-patch upstream code in\n"
        f"place.\n\n"
    )
    sys.stderr.writelines(
        difflib.unified_diff(
            upstream.splitlines(keepends=True),
            normalized.splitlines(keepends=True),
            fromfile=f"{upstream_path} (at the pin)",
            tofile=f"{forked_path} (header and RISCV_FORMAL_E normalized away)",
        )
    )
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
