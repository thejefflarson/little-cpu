#!/bin/bash
# Asserts that a word this repo retired has not come back anywhere it was
# retired from.
#
# Usage: retired_term_test.sh [repo-root]     # defaults to this script's parent
#
# WHY THIS EXISTS. "Ladder" was this repo's name for the generated riscv-formal
# check set, and it was retired: one sweep took out 59 occurrences across 23
# files, including probe labels and the diagnostics a gate prints when it refuses
# to grade. The sweep added no guard, and the word was back on main within hours
# -- in a comment written on a branch that predated the sweep and merged after
# it, with nothing anywhere to object. That merge order is the reintroduction
# path, and no amount of care during a sweep closes it.
#
# THE SECOND TERM IS THE SAME STORY WITH A SHORTER FUSE. The core's ISA name
# widened, and five prose sites went on stating the old one. A sweep corrected
# two of them believing that was all; the other three were found by a grep
# afterwards, and that grep found three more the sweep had not looked for. So
# this file is a loop over a table now rather than one string, because the
# second term arrived before the first one's guard was a year old.
#
# WHY THE ALLOW-LIST IS PROSE. A blind grep for either word would have been wrong
# several times in this repo already: test/probe_gates.sh used "ladder" for a
# hierarchy of exit codes, test/check_suite_shape.sh for a set-equality property,
# and docs/THREAT_MODEL.md for a stepwise sequence of milestones. All three are a
# different word that reads the same, and the sweep told them apart only because
# a person read them. So each entry below states WHICH SENSE it carries and why
# it stays; an entry that is only a path teaches the next reader nothing and gets
# deleted by the next person tidying.
#
# The comparison runs BOTH WAYS, like every other table in this repo: an entry
# naming a site the word has left is as red as the word appearing at a site no
# entry names. An exemption that outlives its reason is how the next one gets
# waved through. Each term carries its own list, so an entry earns its exemption
# for the word it was written about and not for the other one.
#
# Hermetic: git, grep and sed. No toolchain, no simulator, no yosys, so this runs
# inside `make test` anywhere.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=${1:-$(cd "$HERE/.." && pwd)}

if [ ! -d "$REPO" ]; then
  echo "error: '$REPO' is not a directory, so there is nothing to scan." >&2
  exit 1
fi

# A table line is a path or a term followed by a keyword; `#` starts a comment.
strip_comments() { sed -e 's/#.*//' -e 's/[[:space:]]*$//' -e '/^$/d'; }

# THE TERMS, each with how its match is spelled. Both are matched as substrings
# rather than whole words, so a pluralised or suffixed spelling cannot walk past.
#
# `any-case` is the default a retired word wants: it cannot be written around by
# capitalising it. `exact-case` exists for exactly one situation -- a retired
# spelling whose lower-case twin is a live identifier somewhere -- and it is a
# narrower guard, so it has to earn its narrowness on the line that asks for it.
retired_terms() {
  strip_comments <<'TERMS'
# This repo's retired name for the generated riscv-formal check set. Nothing in
# the tree spells it in any other sense as an identifier, so it is caught
# however it is capitalised.
ladder any-case

# The ISA name this core outgrew. It claims RV32IMAC_Zicsr_Zifencei: `misa`
# reads 0x4000_1105, the eleven A encodings are decoded and executed, and the
# suite builds at -march=rv32imac_zicsr_zifencei.
#
# CASE-SENSITIVE, AND THE REASON IS THE WHOLE DESIGN OF THIS ENTRY. The prose
# name of an ISA is written in capitals; the lower-case `rv32imc` is a live and
# correct identifier in four places that must NOT move with it --
# formal/checks.cfg's `isa rv32imc`, the Makefile's `MONITOR_GEN -i rv32imc`,
# riscv-formal's own insns/isa_rv32imc.txt, and the `rvfi_isa_rv32imc` module
# generated from it. They name the instruction set riscv-formal has a spec model
# for, and the pinned clone has none for any A encoding, so widening any of them
# generates nothing. Matching without regard to case would land on all four and
# the only way back to green would be a pattern this file quietly skips -- which
# is the shape of exemption this file was written to refuse. Matching on case
# separates them on the property that actually tells them apart: one is prose,
# the others are arguments to a tool. RV32IMAC does not contain RV32IMC, so the
# name that replaced it is not self-matching.
#
# What that trades away, stated rather than discovered later: a lower-case
# `rv32imc` written as PROSE would not be caught here. test/march_test.sh holds
# the other side of it -- it requires those two live spellings to still be there
# and grades every compiler ISA flag in the tree against one declared string.
RV32IMC exact-case
TERMS
}

# The allow-list, per term. One path per line -- a file, or a directory ending
# in `/` -- with the sense the word carries there written above it.
allow_paths() {  # $1 = term
  case "$1" in
  ladder)
    strip_comments <<'PATHS'
# The link text of Mozilla's published "code of conduct enforcement ladder",
# inside the URL that cites it. Someone else's document title: rewriting it
# would misquote the source rather than tidy this repo's vocabulary.
CODE_OF_CONDUCT.md

# "The milestone ladder" -- a stepwise sequence of milestones, which is a
# different word that reads the same. The file says so at the sentence itself,
# because a reader who finds only this entry would still have to guess.
docs/THREAT_MODEL.md

# Dated decision records, and dated proposals. Both are history: several of the
# filenames are the word, and the whole value of a record is that it says what
# was decided in the vocabulary of the day. Editing them would rewrite the
# record to agree with a later decision.
docs/adr/
docs/ideas/

# The probes that force this check red spell the word in their fixtures and
# their labels. A probe that cannot name what it is planting is not a probe.
test/probe_gates.sh

# This file: the declaration above, the diagnostic below and this list. The
# check has to be able to say what it is looking for.
test/retired_term_test.sh
PATHS
    ;;
  RV32IMC)
    strip_comments <<'PATHS'
# Dated decision records, and dated proposals. ADR-0002 IS the decision that the
# target was this ISA, and ADR-0108 is the record of moving off it; the briefs
# proposed it under that name. Several of the filenames carry it. Rewriting
# either directory would make the history agree with a later decision, which is
# vandalism rather than a sweep -- the record's whole value is that it says what
# was decided in the vocabulary of the day.
docs/adr/
docs/ideas/

# The probes that force this check red plant the string in their fixtures and
# quote it in their labels. A probe that cannot name what it is planting is not
# a probe.
test/probe_gates.sh

# This file: the term table above, this list, and the diagnostic below. The
# check has to be able to say what it is looking for.
test/retired_term_test.sh
PATHS
    ;;
  *)
    echo "error: no allow-list is written for the term '$1'. A term added to" >&2
    echo "the table above needs a list here and a diagnostic below, or this" >&2
    echo "check would grade it against nothing." >&2
    exit 1
    ;;
  esac
}

# What to write instead, per term, and when an allow-list entry is the right
# answer instead. A guard that says only "this is wrong" leaves the next reader
# to guess, and guessing is how a pattern skip gets added.
explain_term() {  # $1 = term; writes to stderr
  case "$1" in
  ladder)
    echo "It was this repo's name for the generated riscv-formal check set and is" >&2
    echo 'retired. Write "the generated riscv-formal checks", or "these checks"' >&2
    echo "where the context is already a formal one." >&2
    echo >&2
    echo "If the use above is genuinely a DIFFERENT word -- a hierarchy of exit" >&2
    echo "codes, a set-equality property, a stepwise sequence of milestones, another" >&2
    echo "project's document title -- add its path to the allow-list in" >&2
    echo "test/retired_term_test.sh, with the sense it carries and why it stays. An" >&2
    echo "entry that is only a path is one the next person tidying will delete." >&2
    ;;
  RV32IMC)
    echo "That is the ISA this core used to target. It claims" >&2
    echo "RV32IMAC_Zicsr_Zifencei now: misa reads 0x4000_1105, the eleven A" >&2
    echo "encodings are decoded and executed, and the suite builds at" >&2
    echo "-march=rv32imac_zicsr_zifencei. Write RV32IMAC, with whichever" >&2
    echo "Z-extensions the sentence was already naming." >&2
    echo >&2
    echo "If what you meant is the instruction set riscv-formal GENERATES a spec" >&2
    echo "model for, that is a different thing and it is spelled in lower case at" >&2
    echo "every live site -- formal/checks.cfg's \`isa rv32imc\`, the Makefile's" >&2
    echo "\`MONITOR_GEN -i rv32imc\`, insns/isa_rv32imc.txt. Those must not move:" >&2
    echo "the pinned clone models no A encoding, so widening one generates" >&2
    echo "nothing. This check is case-sensitive so that they are not caught, and" >&2
    echo "test/march_test.sh requires them to stay exactly as they are." >&2
    ;;
  *)
    echo "error: no diagnostic is written for the term '$1', so the error above" >&2
    echo "says a word is wrong without saying what to write instead. That is the" >&2
    echo "advice a pattern skip gets added in place of." >&2
    exit 1
    ;;
  esac
}

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-retired.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
trap 'rm -rf "$tmp"' EXIT

retired_terms > "$tmp/terms"

if [ ! -s "$tmp/terms" ]; then
  echo "error: the term table is empty, so this check would scan for nothing" >&2
  echo "and report green over whatever came back." >&2
  exit 1
fi

# Tracked files only, and the enumeration is git's rather than a `find` with a
# prune list. What this guard defends against is the word arriving in a commit,
# and a checkout carries build artifacts, downloaded tools and -- in the primary
# checkout here -- whole worktrees of the repo under `.claude/`, none of which a
# merge can reintroduce anything through. A prune list would have to grow every
# time one of those appeared.
if ! git -C "$REPO" ls-files -z > "$tmp/files" 2>/dev/null || [ ! -s "$tmp/files" ]; then
  echo "error: cannot enumerate any tracked files under $REPO. This check reads" >&2
  echo "git's index, because what it guards is the word arriving in a commit; a" >&2
  echo "tree git cannot list is a scan of nothing reporting green." >&2
  exit 1
fi

# Which allow-list entry covers a path, or nothing. A directory entry matches on
# the `/` it ends with, so `docs/adr/` cannot quietly cover a `docs/adrenaline.md`
# that nobody meant to exempt.
match_entry() {  # $1 = path
  local path=$1 entry
  while IFS= read -r entry; do
    case "$entry" in
      */) case "$path" in "$entry"*) printf '%s' "$entry"; return 0 ;; esac ;;
      *)  if [ "$path" = "$entry" ]; then printf '%s' "$entry"; return 0; fi ;;
    esac
  done < "$tmp/allow"
  return 0
}

rc=0
: > "$tmp/summary"

while read -r term casing; do
  case "$casing" in
    any-case)   flags='-inI' ;;
    exact-case) flags='-nI'  ;;
    *)
      echo "error: the term '$term' asks for '$casing', which is neither" >&2
      echo "any-case nor exact-case. A spelling this script cannot read is a" >&2
      echo "term it would go on to scan for under the wrong one." >&2
      exit 1
      ;;
  esac

  allow_paths "$term" > "$tmp/allow"

  # `/dev/null` as a first argument so grep always prefixes the filename, even
  # when xargs hands it a single file. No match at all leaves grep with status 1
  # and xargs with 123, which is not an error here.
  hits=$( (cd "$REPO" && xargs -0 grep "$flags" -e "$term" -- /dev/null < "$tmp/files") || true)

  : > "$tmp/unexpected"
  : > "$tmp/covered"

  while IFS= read -r hit; do
    [ -n "$hit" ] || continue
    path=${hit%%:*}
    entry=$(match_entry "$path")
    if [ -n "$entry" ]; then
      printf '%s\n' "$entry" >> "$tmp/covered"
    else
      printf '%s\n' "$hit" >> "$tmp/unexpected"
    fi
  done <<< "$hits"

  if [ -s "$tmp/unexpected" ]; then
    rc=1
    echo "error: the retired term '$term' appears where nothing allows it:" >&2
    sed -e 's|^|  |' "$tmp/unexpected" >&2
    echo >&2
    explain_term "$term"
  fi

  while IFS= read -r entry; do
    if ! grep -qxF -- "$entry" "$tmp/covered"; then
      rc=1
      echo >&2
      echo "error: the allow-list exempts $entry, and '$term' does not" >&2
      echo "appear there any more. Delete the entry -- or if that use moved, move" >&2
      echo "the entry with it. An exemption kept past its reason is how the next one" >&2
      echo "gets waved through, and it is the reason this comparison runs both ways." >&2
    fi
  done < "$tmp/allow"

  entries=$(wc -l < "$tmp/allow" | tr -d ' ')
  printf "Retired term '%s' confined to its %s allowed sites\n" "$term" "$entries" \
    >> "$tmp/summary"
done < "$tmp/terms"

if [ "$rc" -ne 0 ]; then
  exit 1
fi

cat "$tmp/summary"
terms=$(wc -l < "$tmp/terms" | tr -d ' ')
files=$(tr -cd '\0' < "$tmp/files" | wc -c | tr -d ' ')
echo "$terms retired terms checked over $files tracked files"
