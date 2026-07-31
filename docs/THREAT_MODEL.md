# Threat model

**Status:** Living document. Read this before running a security review over this repo, and
update it when the answer to "what is actually at risk here" changes.

This exists because automated security tooling defaults to assuming it is looking at a web
service. This repo is not one, and a review that reports missing CSRF tokens on a CPU core is
worse than no review — it trains the reader to skim. The point of this file is to say plainly
what is at risk, so a reviewer spends its effort on the things that can really go wrong here.

## What this is

A hobby RV32IMC_Zicsr CPU core in SystemVerilog, plus the verification harness that checks it.
It builds and runs in exactly two places:

| | |
|---|---|
| **A developer laptop** | `make` driving yosys, iverilog, SymbiYosys, clang++, and a riscv64 cross-compiler |
| **GitHub Actions CI** | `pull_request` and `push` to main, `permissions: contents: read`, SHA-pinned actions |

There is **no service, no server, no deployed artifact, no authentication, no user data, no
personal data, and no cryptography.** Nothing here listens on a socket. The eventual target is an
ice40 up5k FPGA and possibly a Tiny Tapeout shuttle — silicon, not a fleet.

The actors are the maintainer, and whoever opens a pull request.

## The asset is assurance, not data

This is the part that matters, and it is the part a generic reviewer gets wrong.

There is nothing here to steal. The blast radius of a total compromise is one laptop and one CI
runner — no customer data, no credentials worth having, no production anything. By the ordinary
measure, the security stakes are close to zero.

**What can actually be damaged is the trustworthiness of the verification result.** This project
went from formally verified to unverified during a rewrite, and getting back is the entire plan
(see `CLAUDE.md`). The milestone ladder is defined against specific gates: `make test` passing,
the riscv-formal ladder's failure set matching `formal/EXPECTED_FAIL`, that baseline reaching
empty as the M2 signal. Every one of those is a claim about correctness that a person will act
on.

So the failure mode with real consequences is **a gate that reports green while it is not
actually checking what it claims to check.** A check silently not generated. An oracle silently
weakened. A test that passes because its data image was empty. That is the security property
this repo has, and it is worth defending with the same seriousness a web service defends a
session cookie.

**Treat silent loss of verification coverage as a first-class finding.** It is not a stretch or a
category error. It is the main event.

## Trust boundaries

### Trusted

Everything the maintainer wrote and committed, and the pins that control what else gets pulled in:

- `rtl/` and its structs and headers
- test fixtures: `test/asm/*.S`, `riscv_test.h`, `sections.lds`, the testbenches
- build orchestration: the Makefiles, `test/run_tests.sh`, `test/cosim.py`,
  `test/sanitize_monitor.py`, `test/cxxrtl.cc`, `test/cosim.cc`
- pins and baselines: `formal/pin.mk`, `ci.yml`'s `OSS_CAD_SUITE_SHA256`, `test/EXPECTED_FAIL`,
  `formal/EXPECTED_FAIL`, `formal/checks.cfg`
- `test/monitor.v` — generated, but tracked and reviewed (invariant 7), freshness re-checked in CI
- `CLAUDE.md`, `docs/`

**Command-line arguments, environment variables, and config files are maintainer input, not
attacker input.** A finding whose exploit requires the maintainer to pass a hostile `argv[1]` to
their own build script is describing a typo, not an attack. Report it as robustness if it is
worth reporting at all, and rate it accordingly.

### Untrusted

Code and data this repo executes or believes, that it did not write:

- **Anything downloaded and then run.** The oss-cad-suite tarball, the riscv-formal clone, the
  Sail emulator, distro-installed toolchains.
- **Vendored third-party code.** `formal/genchecks-local.py` is upstream Python that the build
  executes; it is not covered by `pin.mk`'s SHA verification.
- **Generated code that feeds an oracle.** riscv-formal's monitor generator produces the Verilog
  that `test/sanitize_monitor.py` rewrites into what both sim legs check against. A change in
  generator output shape can change what "passing" means.
- **Output of third-party executables parsed as structured data** — Sail's trace file, sby's
  status files.
- **CI runner state that outlives a job**, particularly the actions/cache entry.
- **Pull-request-authored build tooling**, which CI executes by running `make test` on the PR
  branch.

## The in-repo standard for third-party code

`formal/pin.mk` is the reference implementation and new code should be measured against it. It:

- pins to a **40-hex commit SHA**, not a tag or branch, and validates the format
- uses make `override` so the pin **cannot be defeated** with `make VAR=... ` from a script or CI job
- clones atomically into `.tmp` and renames only after verifying `HEAD` equals the pin
- **fails closed** when a pre-existing clone is at the wrong SHA, because make will not re-run a
  recipe for a directory that already exists — "without this check the pin reads as a control
  while providing no protection"

`.github/actions/setup-oss-cad-suite` applies the same idea to a tarball with a SHA-256.

**A new dependency that downloads and then executes something, without an equivalent control, is
a real finding** — not because the risk is large in absolute terms, but because the repo
demonstrably knows how to do this and the deviation is unexplained. Say so, and name `pin.mk`.

A related and separately reportable defect: a **prose-only guard** — a comment or ADR asserting a
safety property that the adjacent code does not actually have. Those are worse than no comment,
because they stop the next reader from checking.

## Out of scope

Not because they are unimportant in general, but because this repo has no instance of them.
**Do not report these.** If a review returns findings in these categories, the review was
mis-scoped, not thorough:

- Web application security of every kind — XSS, CSRF, SSRF, open redirect, clickjacking, CORS,
  security headers, cookies, sessions
- Authentication, authorization, access control, multi-tenancy, IDOR
- SQL and NoSQL injection — there is no database
- PII, GDPR, data retention, encryption at rest or in transit
- Cryptographic primitives and key management
- Denial of service and rate limiting
- LLM and agent security categories

## Two false-positive classes, named

These have both been reported by tooling and are both wrong. They are written down so the next
review does not spend effort rediscovering them:

**1. The simulated machine's memory is not the host's memory.** `test/sail/memory-map.json`
declares a readable, writable, *executable* region, and `sections.lds` maps `.text` at address 0.
Those describe the address space of an emulated RISC-V machine inside a reference model. They are
not a W^X violation, not a missing ASLR, and not a host memory-protection weakness. There is no
host memory protection involved at all.

**2. RTL and formal-model correctness are functional concerns, not vulnerabilities.** A wrong
immediate in the decoder, a divider that mis-signs, an ALU that fails to mask a shift amount —
these are bugs, they are tracked as bugs, and the whole repo exists to find them. They are not
security findings and reporting them as such inflates the count without adding information.

The distinction that *does* matter: a defect in the machinery that decides whether such bugs are
*caught* is in scope, because that is assurance. `test/sanitize_monitor.py` rewriting the oracle
is in scope. The oracle being wrong about `DIV` is a bug (ADR-0019). The sanitizer silently
failing to apply its fix is a finding.

## What a good review of this repo looks like

Ranked by how much it is worth:

1. **A gate that can report success without having checked anything.** Empty baselines, skipped
   checks, vacuous assertions, comparisons over empty sets, a suppressed exit code.
2. **A downloaded artifact that is executed without integrity verification**, or one that is
   never re-validated once on disk.
3. **A prose-only guard** — a comment claiming a property the code does not enforce.
4. **An error path that produces a wrong verdict rather than an error** — an unchecked
   subprocess status, a parse failure that yields a plausible zero, a missing file that becomes
   an empty list.
5. Ordinary robustness in the build and test tooling: unquoted expansions, unchecked `mktemp`,
   temp-file races.

Findings in category 1 should be reported even when no attacker is involved. "A routine edit
silently deletes a check and the nightly stays green" is the highest-value thing a review of this
repo can find.
