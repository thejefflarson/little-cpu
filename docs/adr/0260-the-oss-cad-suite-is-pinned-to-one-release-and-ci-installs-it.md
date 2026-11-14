# ADR-0260: The OSS CAD Suite is pinned to one release, and CI installs it

**Status:** Accepted · 2026-11-13

## What was true

The OSS CAD Suite (yosys, nextpnr, icetime, sby, iverilog) was the one tool this repo let float.
`.github/actions/setup-oss-cad-suite` used `/opt/oss-cad-suite` when the runner image had one
and otherwise downloaded the latest release, so CI ran whatever the image's last build
contained. A local machine ran whatever its owner last unpacked into
`~/.cache/little-cpu/oss-cad-suite`. Every other downloaded tool was pinned (riscv-formal,
sail-riscv, svlint, the RISC-V gcc, Hazard3, CoreMark).

The two disagreed on real numbers. On 2026-11-13 the CI jobs read yosys 0.70+360
(`5e740962d`), nextpnr 0.11.1-89 and an image `VERSION` of 20261109; the local cache read yosys
0.68+48 (`ff5817c34`), nextpnr 0.11-1 and `VERSION` 20260811. The SPRAM design mapped to 5,367
cells on CI and 5,279 locally, and `make netlist-determinism` was red locally on `rtl/csrs.v`
with no reading on CI's version. The image's copy also moved without a change in this repo (0.68+40
one day, 0.68+71 eight days later), so a CI number could shift under an unchanged tree.

## The decision

**One release, written down once, fetched and checked the way the gcc is, and installed by every
job.**

- **What is pinned**: release `2026-11-09` (`VERSION` 20261109), the release the CI pool's image
  carried on 2026-11-13. The jobs' own logs identify it: `make soc-timing` printed `Yosys 0.70+360
  (git sha1 5e740962d)`, `nextpnr-0.11.1-89-g445e2a65` and `icetime: oss-cad-suite 20261109`, and
  yosys commit `5e740962d` is dated 2026-11-06, which fits no release before 2026-11-07. Installing
  the 2026-11-09 asset reproduces exactly that yosys line and that `VERSION`. Pinning what CI used
  means CI's numbers do not move with this change.
- **Where**: `mk/oss-cad-pin.mk` holds the tag, the first `yosys -V` text of the release and a
  SHA-256 per platform asset, behind the same `override` and command-line refusal as
  `RISCV_GCC_VERSION`. The digests are of the real assets, downloaded and hashed here, and
  equal the digests GitHub publishes beside them: darwin-arm64 (local machines), linux-arm64 (the
  runner pods) and linux-x64 (the `ubuntu-latest` arm of the fork-PR expression, which never runs
  on the pool). A platform with no digest is refused rather than fetched. The yosys text stops at
  the commit id, because the trailing `-dirty` and compiler text differ between one release's
  platform builds; each digest variable's name embeds the release, so a tag bumped without new
  digests finds none and is refused.
- **How**: `make cad-setup` downloads the asset into the tool cache's `download/`, checks its
  SHA-256 before extracting, rejects any member outside `oss-cad-suite/` or with a `..`, and
  unpacks into `~/.cache/little-cpu/oss-cad-suite-<tag>`, a directory named for the release
  so two releases can sit side by side. A stamp of the pin and `yosys`'s digest makes a second
  run a no-op and a changed binary a refusal. `mk/toolchain.mk` puts that directory's `bin/`
  first on `PATH` once it exists, and no longer looks at the old unversioned directory.
- **CI**: the composite action runs `make cad-setup` on every job, with the tarball cached by the
  pin (`make cad-pin` prints the key, mirroring `make riscv-gcc-pin`), and adds the pinned `bin/`
  and `py3bin/` to `PATH`. It never uses `/opt/oss-cad-suite` and never asks GitHub for the
  latest release. The runner image (the separate `cluster` repo) may still bake a suite in; the
  action installs on top and ignores it, so a new image changes nothing here. Every job keeps
  its `runs-on` expression unchanged.
- **Graded**: `make doctor` reports whether yosys, nextpnr, icetime and sby on `PATH` are the
  pinned release. `test/oss_cad_pin_test.sh`, on `make test`'s path as `oss-cad-pin-test`, fails
  when a tool sits in a suite whose `VERSION` is not the pin's, or when yosys prints another
  version. Four `test/PROBES_EXPECTED` labels force it: the pinned suite is green, another
  release is red even with the pinned yosys text, another yosys under the pinned `VERSION` is
  red, and no yosys is red.

## What a move costs now

A newer release is a change to `mk/oss-cad-pin.mk` and its digests, with a fresh sixteen-seed
`make soc-timing` and `make fit` sweep and an ADR, never a side effect of a new day. The pin
bumps on purpose, which is also what makes `soc/pin.json`, `soc/board_pin.json` and the
`FIT_MAX_LC` span comparable between a laptop and CI from now on.

CI pays a download per job in place of an image copy already on disk: a 688 MB tarball restored
from the cache service on linux-arm64, then unpacked. The earlier action took the image's copy
to avoid exactly that restore (measured 129 s mean per job); that cost is the price of a
toolchain the repo names. A job could skip the install when the image's `VERSION` equals the pin,
at the price of trusting a copy this repo cannot hash; it is not done here.

## Netlist determinism on the pinned release

Run locally on macOS arm64 with the pinned suite (yosys 0.70+360, nextpnr 0.11.1-89), using the
control scripts from the open fix for the mutant tree's missing ABC9 script (without it the mutant
cannot synthesise on any release):

- Stage A (two placements of one netlist agree byte for byte) and the first half of stage B
  (the mutant's shipping netlist differs from this tree's) pass.
- The control then stops at its dead-net precondition: the injected `netlist_control_dead` wire
  does not appear in the mapped netlist, so the dead-net class is not exercised and the control
  refuses before it reaches the `rtl/csrs.v` comment class. The cause is not examined here.
- The same scripts under the previous local suite (yosys 0.68+48) pass that precondition and go red
  at the `rtl/csrs.v` comment class, which moves the mapped netlist by +68 `SB_LUT4`.

So `make netlist-determinism` is red on the pinned release too, at an earlier check, and the digest
gate it controls is void on this release until that is looked at. CI does not run it.
