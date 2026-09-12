# test/PROBES_EXPECTED

The manifest `test/probe_gates.sh`'s own coverage ratchet is checked against: every
probe's label, one per line, sorted. `make probe-gates` collects the label passed to every
`probe` call that actually ran and compares that list to this file under set equality in
both directions, the same rule `test/EXPECTED_FAIL` and `test/OBSERVED_FLOOR` use. A probe
this file has and the run does not is as red as one the run has and this file does not, so
a probe that is deleted, skipped, or stranded behind an early `return` is caught by name
rather than by a total going out of sync with a different total.

Two identical labels are two lines, not one. A handful of probes across different fixtures
share the same English description on purpose (the same usage-error message, checked
against a different script); the comparison is a multiset, so losing one occurrence of a
repeated label is still a mismatch even though the label survives elsewhere in the file.

Edit it by hand, in the same commit that adds, removes, or renames a `probe` call. Never
regenerate it wholesale from a run: that launders a dropped probe into the baseline the
same way regenerating `test/EXPECTED_FAIL` would launder a regression. To add a line, copy
the label literally out of the `probe "..."` call and insert it in `LC_ALL=C` order, which
is how `make probe-gates` reads the file.

Sorted rather than left in run order: two commits adding unrelated probes almost never
claim the same alphabetical neighbourhood, so most additions land as independent line
insertions a merge or a rebase applies cleanly, rather than two commits both rewriting the
one line a bare counter used to be.

## The header

The file's own comment header keeps the two tripwires above -- duplicates are deliberate,
never regenerate -- and a pointer here. `make probes-header-test` refuses a header shorter
than two lines, or one whose lines are in `LC_ALL=C` order. A whole-file `sort` reorders the
header in place rather than interleaving it with the labels, because `#` sorts below every
label, so a header in C order is how a sorted file shows itself. Write the header so its
lines are not in that order.
