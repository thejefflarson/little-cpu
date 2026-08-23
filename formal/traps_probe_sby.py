#!/usr/bin/env python3
"""The `traps` task's sby, read out of formal/components.sby rather than copied.

Both traps probes build a core one line of rtl/decoder.v from the shipping one
and require it to go red at a named line. That is evidence about the shipping
design only while the probe reads the shipping SCRIPT -- and the script is where
this task states which files carry proof content, which is not a detail: the
decoder is read with `-formal -noassume` there so its assertions come in and its
standalone environment model does not, and formal/traps.sv's induction needs one
of those assertions.

So the block is read here instead of restated. Two texts of one script drift,
and the drift is silent in the direction that matters: a probe on a stale script
proves a design the shipping task does not build, and reports a green control
about it. This happened -- the copies said `read -sv` for as long as it took one
run to notice, and only the probe with a control noticed at all.

THE SCRIPT IS READ AND THE ENGINE IS NOT, deliberately. components.sby gives
`pcloop` boolector and `busarbiter` `--keep-going`; a probe that inherited an
engine choice would inherit one made for some other task's runtime, and both
files here pin their answer to the FIRST assertion a failing step breaks.
"""

import re

# sby's own task syntax: a label alone on a line opens a block, the next label
# or the section's `--` closes it.
LABEL = re.compile(r"^([A-Za-z_][A-Za-z0-9_ ]*):\s*$")
SECTION = re.compile(r"^\[(\w+)\]\s*$")

# The files the script names. The probe copies its own mutated tree into `src/`,
# so this section is written here rather than read: the paths differ by
# construction.
SOURCES = ("structs.v", "fetcher.v", "decoder.v", "regsel.v", "csrs.v")

TEMPLATE = """[options]
mode prove

[engines]
smtbmc

[script]
{script}

[files]
{files}
"""


def script_block(components_sby, task="traps"):
    """The lines of one task's `[script]` block, or None if it has none."""
    section, label, out = None, None, None
    for line in components_sby.splitlines():
        found = SECTION.match(line)
        if found:
            section, label = found.group(1), None
            continue
        if section != "script":
            continue
        if line.strip() == "--":
            break
        found = LABEL.match(line)
        if found:
            label = found.group(1)
            if label == task:
                out = []
            continue
        if label == task:
            out.append(line)
    return out


def probe_sby(repo, stop, task="traps"):
    """The probe's sby text: components.sby's script over the probe's own tree."""
    path = repo / "formal" / "components.sby"
    if not path.is_file():
        stop(
            f"{path} is missing, and this probe reads the `{task}` task's script out\n"
            "of it rather than keeping a copy. Without it there is no shipping\n"
            "script to build the mutated cores against, and a control that proved\n"
            "would be a statement about some other design."
        )
    block = script_block(path.read_text(), task)
    if not block:
        stop(
            f"formal/components.sby states no `{task}:` block under [script], so this\n"
            "probe cannot read the script it is meant to build against. Teach it the\n"
            "new task name rather than restoring a copy here."
        )
    return TEMPLATE.format(
        script="\n".join(block).strip("\n"),
        files="\n".join(f"src/{name}" for name in SOURCES + ("traps.sv",)),
    )
