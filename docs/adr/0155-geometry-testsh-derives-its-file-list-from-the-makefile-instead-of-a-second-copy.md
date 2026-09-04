# ADR-0155: `geometry_test.sh` derives its file list from the Makefile instead of a second copy

**Status:** Accepted · 2026-09-01

## Context

`soc/compare/geometry_test.sh` compares the comparison harness's ROM/RAM sizes across the Makefile,
each core's own module-parameter defaults, the linker script and the RTL's reset base, and stops
the harness from silently measuring three cores at three different sizes. Its file list was a
literal six-entry existence check: `for f in Makefile soc/compare/bench_littlecpu.v
soc/compare/bench_vexriscv.v soc/compare/bench_hazard3.v soc/compare/bench.lds
soc/compare/bench.S rtl/memory.v`, and the three cores it cross-checked were a second literal,
`for top in bench_littlecpu bench_vexriscv bench_hazard3`.

That is the same failure mode ADR-0139 (`bench_hazard3` joining the loop) had just been fixed by
hand: a name only gets added when a person remembers to add it. PR #243, open at the time this
ticket was opened, adds `soc/compare/coremark.lds` and `soc/compare/coremark_tb.v` — new files
under this directory — and neither literal list would so much as glance at them, because nothing
enumerates `soc/compare/*.lds` or `soc/compare/bench_*.v` and requires each to appear. A fourth
comparison core, or a fifth `.lds`, could land the same way `bench_hazard3` almost did: present on
disk, absent from every comparison.

## The hard part

A directory-enumerating check must not go red on a file that legitimately states no geometry.
`soc/compare/bench_tb.v` matches `bench_*.v` and is a testbench with no `ROM_WORDS` of its own;
`soc/compare/dhry.lds` matches `*.lds` and states a real, different, already-checked geometry
(`soc/compare/dhry_fit.py` compares it against `soc/compare/dhry_tb.v`, deliberately sized 8 KB/16 KB
because Dhrystone does not fit the 4 KB/2 KB placed harness). Demanding a declaration — or agreement
with this one geometry — from every file matching either glob is red by default, which this repo has
already learned gets ignored rather than fixed.

## Options considered

**A. An explicit opt-out list** (`EXEMPT = (bench_tb.v dhry.lds)`). Honest about what is exempt, but
it is the literal list this ticket exists to remove, one level up: a new testbench or a new
differently-sized `.lds` would need someone to remember to add it to the *exemption*, the same
memory the ROM/RAM literal list already depended on and lost.

**B. A naming convention** (`bench_*.v` declares, everything else does not). Fails on contact with
the files that exist today: `soc/compare/bench_tb.v` matches the glob and declares nothing, so a
convention keyed on the filename alone cannot tell it apart from a real core.

**C. Detect the declaration itself, compare only where one exists.** The ticket's own objection to
this shape is real: a file that *had* a declaration and quietly lost it — demoted from a real
parameter to a fixed constant — would stop being compared instead of failing. Taken alone, this is
the weakest of the three.

**Chosen: (C), with the declaration's target derived from the Makefile instead of invented by this
script.** The distinguishing test for a `.v` file is not the filename but whether it declares
`parameter integer ROM_WORDS` — the module-parameter form `chparam -set ROM_WORDS ...
$(COMPARE_TOP)` can reach, as opposed to a `localparam` (the shape `soc/compare/dhry_tb.v` uses),
which is fixed at authoring time and chparam cannot touch at all. That is a fact about the Verilog,
not a guess about the author's intent, and it is exactly the property that makes a file's ROM_WORDS
*the Makefile's* ROM_WORDS: a `localparam` is never subject to `COMPARE_ROM_WORDS` regardless of
what number it holds, so it has nothing to agree or disagree with.

That answers shape (C)'s stated weakness for the case that actually matters here: a core
demoting its own `parameter integer ROM_WORDS` to a `localparam` is exactly demoting itself out of
the Makefile's `chparam` reach — the file stops being placed at the Makefile's size in truth, not
only in this check, so there is no silent gap between what the RTL does and what the check reports.
A demotion the check cannot see is a demotion `compare.$(COMPARE_CORE).json`'s `chparam -set
ROM_WORDS ... $(COMPARE_TOP)` line cannot see either, and that line would then silently stop sizing
the design at all — the Makefile recipe itself is the second, independent witness this check leans
on, not something this script invents on top of a text scan.

The list of names a declaring file is allowed to be is not hand-kept a second time either: it is
read from the Makefile's own `COMPARE_TOP := <name>` lines (three today, one per `ifeq
($(COMPARE_CORE),...)` branch) — the actual mechanism that ties a `.v` file's `ROM_WORDS` to
`COMPARE_ROM_WORDS` in the first place. A `soc/compare/bench_*.v` file that declares the
chparam'able parameter but names no matching `COMPARE_TOP` line is red: it is measured by nothing,
however faithfully its own default agrees with a number nobody chparams it to. This is what would
have caught `bench_hazard3` arriving with its `Makefile` wiring dropped — the failure ADR-0139
fixed by hand.

The same argument covers the one `.lds` file this geometry's numeric comparison reads. The Makefile
links it directly, `-T soc/compare/<name>.lds`, in the one recipe (`compare-rom`) that also chparams
the ROM/RAM size — that is what ties a linker script to *this* geometry rather than to a
differently-sized simulation of its own. Every other `soc/compare/*.lds` must be named by some other
`soc/compare/*.sh` or `*.py` (`soc/compare/dhry.lds` is, by `soc/compare/run_dhrystone.sh`); one
named by neither is a geometry graded by nothing, which is precisely the class PR #243's
`coremark.lds` would have landed as if `soc/compare/run_coremark.sh` — or whatever names it — is
never written. **This PR does not assume PR #243 has landed**: `soc/compare/coremark.lds` and
`coremark_tb.v` do not exist on `main` yet, so neither glob currently matches them; once #243 lands,
this check starts grading them for free, and if #243's own reference script is missing, this check
is what says so.

## Forced red direction

`test/probe_gates.sh` gained four probes, appended to the existing `soc/compare/geometry_test.sh`
group and named in `test/PROBES_EXPECTED`:

- a `bench_fourth.v` fixture declaring `parameter integer ROM_WORDS` with no `COMPARE_TOP :=
  bench_fourth` line in a copied Makefile — red, "declares parameter integer ROM_WORDS but no
  ... reaches it".
- the same fixture with no `ROM_WORDS` parameter at all — green, demonstrating the exemption is by
  declaration and not by name.
- an `orphan.lds` fixture the Makefile does not `-T` and no `soc/compare/*.sh`/`*.py` names — red,
  "is linked by nothing".
- the same fixture plus a stub script naming `orphan.lds` — green.

All existing probes in the group were re-pointed at the new, derived code path and still pass: the
existing eleven probes (ROM/RAM drift on each of the three shipping cores, the linker script's two
regions, the program's RAM-base literal, the linker's `ram` `ORIGIN`, a moved file, an unreadable
declaration) exercise the same failure paths through the Makefile-derived lists that they exercised
through the old literal ones.

## Consequences

- `soc/compare/geometry_test.sh` no longer needs an edit when a fourth comparison core or a second
  `.lds` this geometry's numeric comparison must include is added — wiring the Makefile (which is
  already required for the core to be selectable and buildable at all) is sufficient, and *not*
  wiring it is what goes red.
- The script avoids bash arrays entirely (`for f in "$REPO"/soc/compare/bench_*.v; do ... done`
  under `shopt -s nullglob`, rather than `files=(...)`). `/bin/bash` on macOS is 3.2.57, whose
  `set -u` treats `"${arr[@]}"` on a zero-element array as an unbound variable — a real crash this
  script hit while under development, on the very case (`bench_fourth.v` absent, nothing to glob)
  its own probes are built to exercise. `test/check_suite_shape.sh` sidesteps the same trap by
  checking `${#arr[@]}` before ever expanding `[@]`; this script sidesteps it by never building the
  array at all.
- `soc/compare/dhry.lds` and (once PR #243 lands) `soc/compare/coremark.lds` remain outside this
  script's numeric comparison, on purpose — they are checked by `soc/compare/dhry_fit.py` and
  whatever PR #243 adds for CoreMark, against their own, deliberately different, sizes. This ADR
  does not unify those into one geometry; it only guarantees neither is silently unchecked by
  everything.
- No RTL changed; `make netlist-digest` is unaffected.
