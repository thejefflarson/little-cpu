#!/bin/sh
# Prints the toolchain a measurement was taken with: one line per tool, its
# resolved path and the version it answers with, refusing rather than guessing
# when a tool cannot be asked.
#
#   soc/print_toolchain.sh yosys nextpnr-ice40 icetime
#   make print-toolchain TOOLS='yosys nextpnr-ice40'
#
# `make fit` and `make soc-timing` are graded against FIT_MAX_LC and
# SOC_MIN_MHZ, and both numbers move with a toolchain CI resolves rather than
# pins -- 54 cells between two builds on one tree, with no fixed sign either
# time. Without this block the first question a tripped ratchet raises, "did the
# tool move?", is unanswerable from the run that failed, which is the whole
# reason the repo's answer to a moving toolchain is a budgeted band PLUS a
# stamped comparison rather than the band alone.
#
# The lines are soc/baseline_sweep.sh's provenance lines verbatim, so a sweep
# and a CI job stamp from one source and soc/baseline_summary.py's field names
# are these. A second copy of "which toolchain" that can disagree with this one
# is worse than none, because both look authoritative.
set -eu

if [ "$#" -eq 0 ]; then
  echo "usage: soc/print_toolchain.sh <tool>..." >&2
  echo "*** soc/print_toolchain.sh: no tools named, so there is nothing to" >&2
  echo "*** stamp a measurement with." >&2
  exit 2
fi

digest() {
  if command -v shasum > /dev/null 2>&1; then
    shasum -a 256 "$1" | cut -c1-16
  elif command -v sha256sum > /dev/null 2>&1; then
    sha256sum "$1" | cut -c1-16
  else
    printf 'no-digest-tool'
  fi
}

# icetime publishes no version string -- every flag it does not recognise gets
# the same usage message -- so the suite's own VERSION file beside the binary and
# a digest of the binary stand in for one. Two installs that both stamped
# "unknown" would compare equal, which is the mismatch a stamp exists to catch.
icetime_version() {
  version_file=$(dirname "$1")/../VERSION
  if [ -r "$version_file" ]; then
    printf 'oss-cad-suite %s sha256:%s' "$(sed -n '1p' "$version_file")" "$(digest "$1")"
  else
    printf 'no version string sha256:%s' "$(digest "$1")"
  fi
}

# Graded on the tool's own status and not merely on whether it printed: the
# local nextpnr on one machine here answers `--version` with a dynamic-linker
# error, which is a non-empty first line and would otherwise be stamped as the
# toolchain that produced the number.
first_line() {
  if ! said=$("$@" 2>&1); then
    echo "*** soc/print_toolchain.sh: '$1' could not be asked for its version:" >&2
    printf '%s\n' "$said" >&2
    exit 1
  fi
  line=$(printf '%s\n' "$said" | sed -n '1p')
  if [ -z "$line" ]; then
    echo "*** soc/print_toolchain.sh: '$1' printed no version string." >&2
    exit 1
  fi
  printf '%s' "$line"
}

# Collected and printed at the end rather than a line at a time: a refusal
# halfway down would otherwise leave a short block on stdout that reads exactly
# like a complete one, and the callers splice this into a CSV header and a step
# summary where nobody counts the lines.
nl='
'
block=
for tool in "$@"; do
  path=$(command -v "$tool") || {
    echo "*** soc/print_toolchain.sh: no $tool on PATH, so there is nothing to" >&2
    echo "*** stamp this measurement with." >&2
    exit 1
  }
  # The three ways a tool here answers the question, in one table.
  case $tool in
    icetime)        version=$(icetime_version "$path") ;;
    yosys|iverilog) version=$(first_line "$tool" -V) ;;
    *)              version=$(first_line "$tool" --version) ;;
  esac
  block=${block:+$block$nl}"# $tool: $version [$path]"
done
printf '%s\n' "$block"
