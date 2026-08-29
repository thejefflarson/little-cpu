# 0142 — `FIT_MAX_LC` is a derived budget, and the derivation lives here

Status: Accepted · 2026-08-29

## Context

`FIT_MAX_LC` is `make fit`'s ratchet on `rtl/`'s cell count. It is not a round number: it is a sum
of three measured terms, and each term is what makes a future raise of the ratchet auditable rather
than a number someone picked to make CI pass. That derivation used to live as a 36-line comment
directly above `FIT_MAX_LC := 4219` in the Makefile — the largest of several such blocks, in a file
CLAUDE.md holds to "one or two plain sentences" per comment. This ADR is that derivation, moved
out; the Makefile keeps the number, a one-line pointer to this ADR, and the tripwire sentence that
was already there: *"If this goes red, find out what grew; raising it to pass defeats the point."*

The text below is carried over verbatim from the comment it replaces. It is a measurement with a
date on it — the specific commit SHAs and toolchain readings it cites are not re-verified here, and
should not be silently updated if a later tree disagrees; a fresh derivation is a new ADR.

## The derivation

THE INSTRUMENT IS THE `fit` CI JOB, not a local run. Three yosys builds of one version read this
design three ways, so a local `make fit` is the sanity check and the job's count is the figure this
budget is derived from and graded against.

4219 = 4097 + 68 + 54:

- **4097** the `fit` job's count on this tree, run 32450082354. The tree before mtval read 3937 in
  the same job, so +160 of that count is this change: a 32-bit register in `rtl/csrs.v`, the write
  mux that chooses between a trap's value and a software write, and the four-arm mux in
  `rtl/decoder.v` that builds it. That is what this raise bought, and none of the headroom below is
  a budget for the next change.
- **+68** the churn band, measured rather than quoted at ±50. Setting one further bit of the
  read-only `misa` constant spans 68 cells on `64759da` (3988, 3979 and 3920 against a base of 3935)
  and 63 on `2007d9d` (3958, 3971, 3987, 3925 and 3980 against 3988) — edits that change no logic at
  all. Budget the whole span and not just its upward half: every probe on the second tree came out
  BELOW the base, so a count can sit anywhere in that window, including at the bottom with the whole
  of it still to come.
- **+54** the widest gap measured between two toolchains on one tree, which is `2007d9d`: the job
  read 3934 where both a local Homebrew yosys and a cached OSS CAD Suite read 3988. Other trees read
  32 (3543 job, 3575 local on `d3a9556`) and 3 twice (3938 against 3935 on `64759da`, 3966 against
  3969 here, the sign not the same either time), so the gap is re-mapping too and has no fixed size
  or sign. CI resolves the suite rather than pinning it, so a release moves the count with nothing
  committed against it.

The budget clears the higher of this tree's pair by more than a band — 4084 local against 4097 in
the job. Preserve that when this is next re-derived.

A raise names what it bought, and the two kinds read differently: 4000 → 4088 bought band clearance
with no cells spent for it, and 3625 → 4000 was +452 measured cells for the eleven A instructions.
This one is the second kind.

## Consequences

- `Makefile`'s `FIT_MAX_LC := 4219` keeps a one-line pointer to this ADR and the tripwire sentence;
  the arithmetic and the measurements behind it live only here.
- The next raise re-derives its own three terms against the tree it is taken on rather than editing
  these numbers — the same rule `docs/adr/` already holds every measurement to.
