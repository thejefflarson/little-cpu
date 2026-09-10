#!/usr/bin/env python3
"""Re-points the generated insn_* checks at `btor pono` with no `skip`.

Measured on the runner pod, per check: insn_add 25.6s -> 13.5s, insn_mul
22.8s -> 10.1s, insn_div 25.3s -> 15.1s, and peak RSS 352 MB -> 253 MB.
btormc still wins the other families -- causal 3.7s against 6.0s, liveness
5.5s against 12.4s -- so only insn_* moves.

Dropping `skip` widens the search rather than narrowing it. genchecks emits
`depth = N+1, skip = N`, so the bound is unchanged and only the shallower
bounds are added; the [depth] table, its `#derive` lines and
genchecks-audit.py's floors all still describe exactly the bound that runs.
`skip` is also why the engine could not simply be swapped: sby accepts it for
btormc alone.

Usage: retune-checks.py <checks-dir>
"""
import pathlib
import re
import shutil
import sys

ENGINE_WAS = "btor btormc"
ENGINE_NOW = "btor pono"
SKIP_RE = re.compile(r"^skip \d+\n", re.M)


def refuse(*lines):
    for line in lines:
        print(f"error: {line}" if line is lines[0] else line, file=sys.stderr)
    sys.exit(1)


def main():
    if len(sys.argv) != 2:
        refuse("usage: retune-checks.py <checks-dir>")
    root = pathlib.Path(sys.argv[1])
    if not root.is_dir():
        refuse(f"{root}: not a directory, so nothing was generated to retune.")

    solver = ENGINE_NOW.split()[-1]
    if shutil.which(solver) is None:
        refuse(f"{solver} is not on PATH, so the checks cannot be retuned to it.",
               "The OSS CAD Suite is the one tool this repo does not pin, so a",
               "release that drops the solver must stop here rather than write",
               "an engine sby will only fail on once every check is running.")

    every = sorted(root.glob("*.sby"))
    if not every:
        refuse(f"{root} holds no .sby files.",
               "Retuning nothing and reporting success would hide a generation",
               "that produced no checks at all.")

    insn = [p for p in every if p.name.startswith("insn_")]
    if not insn:
        refuse(f"{root} holds {len(every)} checks and none is named insn_*.",
               "The family this retunes is gone or renamed upstream. Retuning",
               "nothing silently would leave the whole set on the slower engine",
               "with this script still reporting success.")

    for path in insn:
        text = path.read_text()
        if text.count(ENGINE_WAS) != 1:
            refuse(f"{path.name} names `{ENGINE_WAS}` {text.count(ENGINE_WAS)} times, not once.",
                   "genchecks no longer emits the engine this rewrites.")
        if len(SKIP_RE.findall(text)) != 1:
            refuse(f"{path.name} has no single `skip` line.",
                   "genchecks no longer emits the skip this removes, so pono",
                   "would be handed an option only btormc accepts.")
        path.write_text(SKIP_RE.sub("", text).replace(ENGINE_WAS, ENGINE_NOW, 1))

    print(f"retuned {len(insn)} insn_* checks to `{ENGINE_NOW}`, "
          f"{len(every) - len(insn)} others left on `{ENGINE_WAS}`")


if __name__ == "__main__":
    main()
