# 0138 — The per-encoding exclusion predicate is built

Status: accepted · builds the mechanism ADR-0132 decided but left as a ticket of its own. Amends
`formal/complete.sv` and `formal/check-complete-exclusions.py`; supersedes nothing, because every
exclusion standing before this record stays legal under the widened shape.

## Context

ADR-0132 decided that `formal/complete.sv`'s exclusion predicate may key on opcode plus funct3 and
funct7, not only on the seven-bit opcode class, and named three properties any implementation must
keep. Nobody had built it: `make -C formal complete` still failed for any instruction sharing a
major opcode with an already-modelled one, which by ADR-0131's own account is every extension this
repo's ISA-target section names.

## Decision

**Built.** `formal/complete.sv` gains `insn_funct3` and `insn_funct7`, defined the same literal way
`insn_opcode` is. `formal/check-complete-exclusions.py`'s `WIRE_RE` now accepts three predicate
shapes — opcode alone, opcode+funct3, or opcode+funct3+funct7 — never funct7 without funct3, which
RISC-V's own encoding never discriminates on. `formal/COMPLETE_EXCLUSIONS`'s format grows the same
way: `<opcode>[/<funct3>[/<funct7>]]`, one token, so today's three whole-class lines parse unchanged.

The three properties ADR-0132 required:

1. **The predicate reads `rvfi_insn` and nothing else.** `insn_funct3`/`insn_funct7` are pinned in
   `REQUIRED_DEFS` the same way `insn_uncompressed`/`insn_opcode` already were — an exact string
   match, not a pattern, so a redefinition off anything but `rvfi_insn` fails before the predicate
   shape is even read.
2. **The checker re-derives "no spec model" per encoding, and fails closed on what it cannot
   resolve.** This is where the ADR's own scoping shifted the design. The obvious oracle —
   `insns/insn_<mnemonic>.v` existing anywhere under the pinned clone — is wrong: riscv-formal
   vendors a model for `sh2add` at the pin (`insns/insn_sh2add.v`, under Zba's own
   `isa_rv32iZba*.txt`), and that file is never reachable from `formal/complete.sv`'s `isa_spec`
   instance, which is built from `insns/isa_rv32imc.txt` alone (`insns/isa_rv32imc.v`, tracked at
   the pin, statically instantiates only that list's members). A checker that flagged `sh2add` as
   "has a model" on file existence would have been wrong in exactly the direction that matters: it
   would refuse the one exclusion ADR-0131 needed, for a model this core's oracle can never see. So
   `check-complete-exclusions.py` re-derives its whole index from `insns/isa_rv32imc.txt`, reads
   each named mnemonic's own `insns/insn_<mnemonic>.v` for its `spec_valid` line, and extracts the
   opcode (mandatory), funct3 and funct7 (or funct6, for the shift-immediate family, represented as
   a funct7 pattern with its low bit a wildcard) that line tests. A mnemonic the list names but whose
   model file is missing or unparseable is an error, not a skip — fail closed. A compressed
   mnemonic's own module redeclares `insn_opcode` as a two-bit quadrant local to that file; it is
   recognised and excluded from the index rather than reported as unresolvable, because no exclusion
   here is ever `insn_uncompressed`-gated false.
3. **The set stays a two-way tracked diff.** Unchanged in shape: `complete.sv`'s declared exclusions
   and `COMPLETE_EXCLUSIONS`'s baseline are compared on `(class, opcode, funct3, funct7, mnemonics)`
   tuples, both directions, the same as before with two more fields in the tuple.

**A fourth clause was added, not merely implied: a predicate must be as narrow as the pin's own model
lets it be.** The re-derived index is not only clause 5's oracle for a *named* mnemonic — it is
checked for overlap against *every* declared exclusion's own (opcode, funct3, funct7), regardless of
what mnemonics the entry names. A class-wide predicate whose opcode any modelled instruction shares
is rejected the same way a stale mnemonic is, and a funct3-only predicate is rejected too if two
modelled instructions still share that funct3 at different funct7 values. This is what makes "as
narrow as possible" a property of the mechanism rather than of the author: excluding OP wholesale for
`sh2add` is refused with `add, and, div, ..., which rvfi_isa_rv32imc models`; narrowing to
`0110011/100` (opcode+funct3 alone) is still refused, because XOR and DIV share that funct3; only
`0110011/100/0010000` — the exact row — is accepted.

## The acceptance test: sh2add, re-applied and reverted

ADR-0131's `sh2add` probe (`41f48e4`, reverted at `ed675e0`) was cherry-picked back, unmodified, and
`formal/complete.sv` gained one entry:

```
wire exclude_op_sh2add = insn_uncompressed && insn_opcode == 7'b0110011 &&
                          insn_funct3 == 3'b100 && insn_funct7 == 7'b0010000;
```

With the exclusion **absent**, `make -C formal complete` reproduces ADR-0131's exact failure —
`Assert failed in rvfi_testbench: complete.sv:231` (this tree's line number for the same
`assert(spec_valid && !spec_trap)`), on the sh2add retire at step 5. With it present,
`check-complete-exclusions.py` passes (4 entries, both directions), and both `make -C formal
complete` and `complete_cover` pass — the OP-major-opcode wall ADR-0131 hit is down for exactly the
one row that needs it, with ADD, SUB and the eight M instructions still under the assertion.

**The RTL and the exclusion both come back out**, per the convention ADR-0100/0101/0113 set: the
probe is a measurement, not a feature, and an exclusion with no instruction behind it is a dangling
entry rather than a recorded restriction. `rtl/decoder.v`, `rtl/executor.v`, `rtl/structs.v`,
`test/decoder_tb.v`, `test/OBSERVED_FLOOR` and `test/asm/sh2add.S` are reverted; `formal/complete.sv`
and `formal/COMPLETE_EXCLUSIONS` carry no `sh2add` line. What survives is the mechanism and this
record of what it did with a real instruction.

## What this does not claim

Same caveat ADR-0132 stated: an exclusion is still a hole, narrower now but still a hole, and an
instruction excluded this way has no oracle from `complete`. Nothing here ships an extension —
`sh2add` is out of the tree again — and nothing here changes the area or period costs ADR-0131
measured, which remain the open question for whichever ticket next proposes shipping it.
