# ADR-0193: The pre-restructure dual baseline is taken, and it passes

**Status:** Accepted · 2026-09-18

## Why this is a re-take worth dating

`rtl/littledual.v` is the only integrator that drives `bus_wait`, and `make dual-smoke` is the only
functional grader of two harts sharing one text storage and one arbiter. Both `make dual-smoke` and
`make dual-ecp5-timing` are off `make test`'s path and off CI, so nobody currently knew whether
either passed on today's tree. A pipeline restructure is coming in two stages
(`docs/ideas/the-fetch-address-reads-registers.md`); both stages change what the fetch port does
and what the data port publishes when idle. ADR-0125 already recorded the exact failure shape this
invites — `rtl/accessor.v` publishes `rs2` on `mem_wdata` for every issuing instruction, so ORed
across two harts it lost most of a smoke program's counted increments, invisibly to a bus-exclusivity
check. Without a baseline taken now, a dual failure found after the restructure lands cannot be
attributed to it.

## What was run, and on what

Tree: `6282b42635f647212a7ee8860108a4d60b17a61e` (`origin/main`), clean, no local changes.
Toolchain, from `make doctor`: `riscv-none-elf-gcc` 15.2.0 (xPack, arm64) at the pinned
`RISCV_GCC_VERSION`; `yosys` 0.68+48 (git sha1 `ff5817c34-dirty`, Clang 21.1.8); `nextpnr-ice40` and
`nextpnr-ecp5` both `0.11-1-g62e659ed`; `icetime` from `oss-cad-suite 20260811`; the ECP5 flow's
Trellis device database `sha256:5a3869c1b6fe7ea1`. Date: 2026-09-18.

## `make dual-smoke`: both shapes, both PASS

**Both harts:**

```
== both (exit 0) ==
PASS
WORD 0x00010810 32
HART0 RETIRES 116 SPEC-CHECKED 115
HART1 RETIRES 142 SPEC-CHECKED 141
```

`test/dual/smoke.S`'s shared counter reads 32 = 2 × `ITERS` (16), the shape the harness requires
when both harts run: each adds `ITERS` to its own counter, hart 0 sums both and compares against
`2 * ITERS`. Both harts retired a nonzero, RVFI-spec-checked instruction count.

**Hart 1 held in reset:**

```
== held (exit 6) ==
FAIL 2
WORD 0x00010810 16
HART0 RETIRES 1135 SPEC-CHECKED 1134
HART1 RETIRES 0 SPEC-CHECKED 0
hart 1's RVFI monitor observed nothing this run: 0 retires, 0 of them spec-checked. This harness
exists to watch two harts, so a run that watched one has no verdict to report (exit 1 was the
program's).

dual-smoke: OK -- two harts counted 32, one hart counted 16
```

This shape is designed to report `FAIL` and exit 6, not `PASS`: with hart 1 held, `done1` never
gets set, hart 0's poll loop (`POLL_LIMIT` = 256) times out, and hart 0 sums `count0 + count1` =
`ITERS + 0`, which fails the program's own `2 * ITERS` check — `test/dual_smoke.sh` asserts exactly
this exit code (6, the harness's per-hart silence gate) and that the counter reads `ITERS` = 16, not
`2 * ITERS`. Hart 0 retired 1135 instructions spinning through the poll loop; hart 1 retired zero,
confirming it never left reset. `test/dual_smoke.sh`'s own final line, `dual-smoke: OK`, and the
overall exit code of 0, are the grader's verdict: **both shapes pass on this tree.**

## `make dual-ecp5-timing`: all three censuses pass

Single default-seed placement (`ECP5_SEED` unset), `LFE5U-25F-6CABGA381`:

```
DP16KD: 40, as declared
TRELLIS_DPR16X4: 64, as declared
MULT18X18D: 8, as declared
block RAM resets: 40 DP16KD / PDPW16KD, none driven by logic

Fmax          : 33.71 MHz  (29.66 ns)
  constraint  : 200.00 MHz -- an INPUT to the placer, not a threshold
  logic       :  10.98 ns  37.0%
  routing     :  18.69 ns  63.0%
```

All three gating censuses read exactly their declared figures (`DUAL_EXPECT_DP16KD` = 40,
`DUAL_EXPECT_LUTRAM` = 64, `DUAL_EXPECT_DSP` = 8) — two fetch windows are two copies of the banked
ROM, one register file per hart is its own distributed RAM, one multiplier per hart is its own DSP
block — and `soc/bram_reset_check.py` finds no block RAM reset driven by logic. The 33.71 MHz figure
is one placement, a sample rather than a swept measurement (this repo's own convention: "one
placement is a sample, not a measurement"), recorded here as the number this baseline compares
against, not as a ratchet — `make dual-ecp5-timing` publishes with no ratchet by design, and no
up5k number describes this design at all.

## Verdict

**Nothing was broken.** Both `make dual-smoke` shapes pass and all three `make dual-ecp5-timing`
censuses gate clean on `6282b42`, so the two-stage fetch-address restructure has a dated baseline to
be checked against: dual-smoke's two shapes (32/16 total, per-hart retire counts above, exit codes
0/6) and dual-ecp5-timing's censuses (40/64/8, Fmax 33.71 MHz on this one placement) as measured on
this tree with this toolchain. A restructure that changes what the fetch or data port publishes when
idle and moves either shape's counted total, either hart's zero-retire guarantee, or any of the
three census counts should be checked against the figures above before either stage is judged to
have broken the dual configuration — and, per ADR-0125's rule, read the producer's idle behaviour
(what each port publishes when its hart is not issuing) before joining two of anything the
restructure touches.

## Alternatives considered

- **Skip the ADR and just note the PASS in the ticket.** Rejected: CLAUDE.md requires a measurement
  with a date to live in an ADR, specifically so the two restructure stages have something concrete
  to diff against rather than an inherited "it was fine before" with no cycle counts attached.
- **Sweep `dual-ecp5-timing` across sixteen seeds for the baseline.** Not done: the ticket asked to
  confirm the three censuses pass and record the figures, and this design has no derived ECP5 band
  (`soc/bands.py` refuses to answer for the part) for a sweep's spread to be judged against; a single
  placement, clearly labeled as such, is what the tool itself hands back with no ratchet attached.
