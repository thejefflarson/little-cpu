# The comment-density budget

`test/comment_density_test.py` is the grader and `THRESHOLD_PCT` in that file is the one
place the number is declared, the way `soc/bands.py` is the one place a placement figure
is. This page is the derivation. If the two ever disagree, the script is what runs.

**No file may be more than 5% comment lines.** The rule is per file, not an average: an
average lets one 40% file hide behind twenty 2% ones, and the 40% file is the one a reader
actually has to wade through.

## Where the tree stands

Every tracked code file is at or under the budget. Before the pass the tree was 22.5%
comments with 258 of 300 files over 5%; `.github/workflows/ci.yml` was the worst at 37.1%
and no grader could see it, because the gate had no YAML style at all.

## What counts as a comment, and what only looks like one

Getting this wrong in either direction makes the number meaningless, so the classifier is
specific about seven cases, each of which broke something real when it was got wrong:

- **A Python shebang is a `COMMENT` token to `tokenize`.** Counting it stripped the shebang
  from 49 files, 20 of them executable. Python is classified through `tokenize` rather than
  by line prefix, which also means a `#` inside a string is never mistaken for a comment.
- **`#derive`, `#floor` and `#omit` in `formal/checks.cfg`** are parsed by
  `formal/depth_rules.py` and `formal/genchecks-audit.py`. They are data.
- **`// EXCLUDE` in `formal/complete.sv`** is parsed by
  `formal/check-complete-exclusions.py`, which additionally requires 40 characters of prose
  underneath each one. That prose is a required field, not narration.
- **A `#` inside a shell heredoc** belongs to the heredoc. `test/march_test.sh` and
  `test/lut4_site_test.sh` are mostly heredoc, and counting theirs mis-measured both.
- **A `#` inside a YAML block scalar** is part of the script the scalar holds.
- **A `#` comment in a `.S` file whose first word is `else`** is read by the C preprocessor
  as `#else`. Assembly carries both `#` and `//` comments; both are counted.
- **Linter and licence directives** — `shellcheck`, `noqa`, `SPDX`, `See LICENSE` — are
  instructions to a tool or a legal notice, not prose.

## What is out of scope, and why

Vendored and generated files, because editing them is either forbidden or pointless:
CoreMark (`test/bench/coremark/`, pinned by SHA-256), Dhrystone 2.1's three published
sources, the generated `soc/compare/vexriscv/`, `test/monitor.v` (generated at the
riscv-formal pin), and `formal/genchecks-local.py` (must differ from the pin by header and
basedir only).

## Where prose goes instead of away

The budget is a claim about density, not about whether a thing is worth writing down. Three
kinds of prose moved rather than died:

- **The data manifests** — `test/EXPECTED_FAIL`, `test/OBSERVED_FLOOR`,
  `formal/COMPLETE_EXCLUSIONS` and their siblings — keep one pointer line each, and their
  format and reasoning live under `docs/manifests/`.
- **The pin-constraint files** are five to seventeen lines of `set_io` under a header of
  measured fact. A 5% budget there is zero lines, so those headers are in
  `docs/pin-constraints.md`.
- **A Python docstring is a string, not a comment**, and does not count. `soc/bram_reset_check.py`
  carries the whole DP16KD output-reset argument that way, and `rtl/memory.v` points at it.

## `MIN_COMMENT_LINES`

A file with two comment lines or fewer is never reported, whatever its ratio. That is what
lets a manifest be one pointer line above its data without failing at 100%.
