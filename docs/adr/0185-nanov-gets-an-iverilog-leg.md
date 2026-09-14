# ADR-0185: nano.v gets an iverilog leg

**Status:** Accepted · 2026-09-13

## Context

ADR-0180 shipped nano's simulation harness cxxrtl-only and filed the iverilog leg as a
follow-up, recording the two reasons `nano.v` would not elaborate: `regs[]` and most decode
flags are read in continuous assigns before their own declarations, and `` `RVFI_OUTPUTS ``
declares the RVFI ports with no net/variable keyword, which iverilog resolves as `wire`
while nano's own `always_ff` block drives them procedurally. cxxrtl is two-state, so it
cannot show that gap; CLAUDE.md's Verification section already states the same divergence
for littlecpu's two sim legs -- an undefined word is green under cxxrtl and only iverilog
can see the X.

## Declaration order

Every bare `logic` declaration in `nano.v` moves to one block right after the port list;
every `assign`, `always_comb` and `always_ff` stays exactly where it was, in the same
relative order. Declaration position is the only thing iverilog's parser needs settled
(confirmed against a minimal repro: a continuous assign that reads a `logic` declared later
in the file fails to bind under `-g2012` for both a scalar and a memory, contrary to the
donor brief's note that a scalar forward reference "merely warns" -- this iverilog build
errors on both). Moving only declarations, never an assign or an always block, is what
keeps this a no-op: nothing about what drives what, or when, changes.

## The RVFI ports

`` `RVFI_OUTPUTS `` is untouched -- littlecpu's own formal wrapper needs the exact same
macro, and the conditional CSR-extension ports two of nano's own generated checks
(`csrw_mcycle_ch0`, `csrw_minstret_ch0`) compile against depend on it staying macro-driven.
`rtl/littlecpu.v` solves the same wire-vs-variable problem by hand-declaring every RVFI
port as `output logic` instead, including the CSR-extension ones, because it implements
`mcycle`/`minstret` for real and drives them. nano does not yet -- ADR-0177 keeps the
counters as a feature, not a cut, and a later ticket (CSRs, traps and `MEIP`) is what
actually wires `mcycle`/`minstret` up as real 64-bit writable CSRs, which is what will flip
`csrw_mcycle_ch0` and `csrw_minstret_ch0` from `EXPECTED_FAIL` to passing. So this is not a
permanent gap this file's port list can be shaped around once; it is a port set that grows
on nano's own roadmap.

Instead, a local `` `RVFI_SHADOW(name) `` macro (`` `undef `` immediately after its twenty
invocations) declares one `logic` shadow register per RVFI output nano.v actually drives,
sized by `$bits(name)` so it tracks `` `RVFI_OUTPUTS `` rather than restating its
arithmetic, assigns each shadow from the `always_ff` block exactly as before, and connects
it out to its macro-declared port with a continuous assign. The reason to keep this shape
is the same reason it happens to also survive the CSR ticket: the module's own port list
never names an RVFI signal -- it is always whatever `` `RVFI_OUTPUTS `` expands to for
whatever macros are defined -- so nothing about `nano.v`'s header, or `nano/formal/wrapper.v`'s
`` `RVFI_WIRES ``/`` `RVFI_CONN ``, has to move when the driven set changes. Landing real
`mcycle`/`minstret` only costs that ticket eight more `` `RVFI_SHADOW(...) `` lines --
`rvfi_csr_mcycle_rmask`/`wmask`/`rdata`/`wdata` and the `minstret` equivalents -- each
assigned from wherever the new CSR write-back logic lives; nothing else in this file, and
nothing in the wrapper, needs to change to make that pass. Until then, any CSR-extension
port this file has never driven stays exactly as undriven as it was -- `make -C nano/formal
check` reproduces 82 checks, 80 pass and the same 2 known-fail (`csrw_mcycle_ch0`,
`csrw_minstret_ch0`) EXPECTED_FAIL entries, unmoved.

## No functional change, measured

`make nano-area` builds with no `RISCV_FORMAL` defined, so it reads 61411.4 of 61412.0 um2
and the full `yosys stat -liberty -json` output byte-identical both before and after -- but
that build never compiles the `` `ifdef RISCV_FORMAL `` block at all, so it is evidence for
the declaration reorder alone, not for `` `RVFI_SHADOW ``. That block's own evidence is
`make -C nano/formal check`, which does compile it: 82 checks, 80 pass, the same 2
known-fail (`csrw_mcycle_ch0`, `csrw_minstret_ch0`) EXPECTED_FAIL entries, unmoved. The
second half is `make nano-test`'s dual-leg comparison -- two separate elaborations of the
same `` `RVFI_SHADOW ``-based `nano.v`, cxxrtl and iverilog, retiring the same six programs
to the same PASS/FAIL verdict and the same retire count each. Neither half is a claim about
`nano.v`'s own semantics changing; both are that this RTL change did not change them.

## The iverilog leg itself

`nano/tb/nano_icarus.vvp` compiles the same `NANO_SIM_RTL_SRCS`/`NANO_SIM_TB_SRCS` the
cxxrtl build reads, with `-DICARUS` alongside the same `NANO_RISCV_FORMAL_MACROS`.
`nano_testbench.v`'s `` `ifdef ICARUS `` block grew from a single baked-in-program dumper
(the shape `test/testbench.v`'s own `make waves` still has) into a general runner: `+ROM=`/
`+RAM=` name the same objcopy-verilog hex images `nano_cxxrtl.cc` loads by argv, `+CYCLES=`
bounds the run, and a `finish_run` task prints the identical `RETIRES`/`PASS`/`FAIL
<n>`/`TIMEOUT`/`trap taken`/`RVFI monitor error` protocol the cxxrtl runner does from every
exit path, plus this leg's own `X reached a retiring instruction...` on exit 7 -- a code of
its own rather than sharing 4 with a real monitor error, which would otherwise read as a
bare `MONITOR-ERROR` once the run log itself is gone. Its own reset timing folds the
reset-deassert edge into cycle 0, inside the counted loop, matching `nano_cxxrtl.cc`'s
`if (cycle == 0) top.p_reset.set(false);`: the earlier shape ran that edge before the loop
started, so the same `--cycles`/`+CYCLES=` value gave the iverilog leg one more post-reset
edge than the cxxrtl leg got, and any diagnostic "at cycle N" the two legs printed for the
same real edge would not have agreed. `nano/tb/nano_sim_icarus.sh` translates the protocol
into `nano-sim`'s own `--rom`/`--ram`/`--cycles` CLI and 0-6 exit codes plus its own 7, so
`nano/asm/run_nano_tests.sh` (whose own `case` gained a `7) status="X-REACHED"` arm) drives
either leg with no change of its own.

`make nano-test` now runs `nano/tb/nano_dual_leg_test.sh`, which runs both legs' suites in
parallel (independent simulators, nothing shared until the diff), grades each against
`nano/asm/EXPECTED_FAIL`/`OBSERVED_FLOOR`, and then diffs their two per-program result
tables (status and retire count), requiring an exact match. All six programs agree, digit
for digit, on both legs today. Its own baseline-status check compares as a string (`!=
"0"`), not `-ne 0`, because `[ "$(cat empty-file)" -ne 0 ]` reads an empty status file as
success rather than erroring; and its result-table `grep` is wrapped in `{ ... || true; }`
so a runner that passes but prints nothing parseable reaches the empty-table guard instead
of tripping `set -e` first, dead code no probe had ever exercised.

## The iverilog leg is the only one that can see an X

`nano_testbench.v`'s memory zeroing loop (present since ADR-0180's `nano_cxxrtl.cc`, which
memsets the same array before either image is poked in) has never had a demonstrated red
direction. It has one now: after every retire, the ICARUS block reduction-XORs
`rvfi_insn`/`rvfi_pc_rdata`/`rvfi_pc_wdata`/`rvfi_rs1_rdata`/`rvfi_rs2_rdata`/`rvfi_rd_wdata`/
`rvfi_mem_addr` against `1'bx`, plus `rvfi_mem_wdata` a byte at a time under
`rvfi_mem_wmask`'s own bits (an unwritten byte lane owes nothing), and reports if any bit
is unknown -- gated on `rvfi_valid_observed === 1'b1 || === 1'bx` rather than a plain
`&&`, since a validity signal that itself reads X would otherwise skip the cycle silently
instead of being read as suspect too. `nano.v` never resets `regs[1]`-`regs[15]` (only
`regs[0]`, every `fetch_instr` cycle), so widening the reduction to the register-read
fields could have made a program that reads a register before writing it a false positive;
`make nano-test`'s own six programs stay clean under the wider check, so none of them do.

`nano/tb/nano_x_probe.sh` proves the check both ways, on both paths it now covers. The
load path: a tiny program (`li x1, 0x00013000; lw x6, 0(x1)`, an address neither its
`.text` nor its `.data`/`.bss` reaches) retires clean on the shipping harness and reports
the X at its second retire on a copy of `nano_testbench.v` with the zeroing loop's `for`
replaced by `if (0)`. The store path needs no testbench mutation at all: a program that
stores `x1` after writing it first (`store_clean.S`) stays clean on the shipping harness,
and one that stores `x1` without ever writing it (`store_x.S`) reports the X there directly,
since `regs[1]` is real hardware state this file never resets -- proving the write-mask-
masked `mem_wdata` term is reached is a fact about the shipping core, not a fixture. This
part of the probe is NOT hermetic -- it runs the real cross compiler and the real iverilog,
compiling the shipping harness once and the zeroing-loop mutant once, and running three
programs (the load probe, `store_clean.S`, `store_x.S`) against whichever image each needs
-- and is wired as a prerequisite of `make nano-test`, the same standing
`nano/formal/ill-e-probe.py` has for `make -C nano/formal ill_e`. It takes `nano/tb.mk`'s
`NANO_SIM_RTL_SRCS` and `NANO_RISCV_FORMAL_MACROS` as arguments rather than restating them,
and reads `rvfi_macros.vh`/`test/monitor.sim.v` as `nano-x-probe`'s own Make prerequisites
rather than regenerating them -- one source of truth for both, the same one `nano-sim`'s
build already reads. `test/probe_gates.sh` covers its own orchestration (argument checks,
the stale-pattern guard, every failure message on both paths) against stub `iverilog`/`vvp`
binaries that read both the image name and the program name off `vvp`'s own arguments,
`nano_dual_leg_test.sh`'s own agree/disagree logic against two independently controllable
fake sims, and `nano_sim_icarus.sh`'s own verdict-to-exit mapping (all eight cases,
including the shared-then-split 4/7 pair and the retires-observed-nothing override to 6)
against a stub `vvp` that plays back a fixed transcript per case.

## Consequence

`make nano-test` costs two builds and two six-program runs instead of one of each, the runs
in parallel; on this machine that is a few seconds, not the difference between a fast and a
slow `make test`. The two grep patterns in this change that used GNU-only syntax (`\|`
alternation, `\S`) are now `-E 'a|b'` and `[^[:space:]]`, portable to a BSD grep that reads
`\|`/`\S` literally. Nothing here changes nano's ISA, its memory model, or its benchmarks --
both still-open follow-ups from ADR-0180 (a DMIPS/MHz figure, the flash front end) are
untouched.
