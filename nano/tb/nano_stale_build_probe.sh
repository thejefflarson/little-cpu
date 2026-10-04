#!/bin/bash
# Grades nano/stamp.mk both ways, hermetically; docs/nano-stale-builds.md describes the scenarios
# and their red directions.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-stale-build.XXXXXX") || exit 1
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

mkdir -p "$tmp/nano"
cp "$REPO/nano/stamp.mk" "$tmp/nano/stamp.mk" || exit 1

cat > "$tmp/Makefile" <<MK
BUILD := b
DEFS  ?=
include nano/stamp.mk
\$(eval \$(call nano_stamp,S,in.v,\$(DEFS)))
out: in.v \$(S)
	echo "\$(DEFS)" > out; cat in.v >> out
out-nostamp: in.v
	echo "\$(DEFS)" > out-nostamp; cat in.v >> out-nostamp
MK

# $1 = target, $2 = defines, $3 = input text the image must reflect.
reflects() {
  [ "$(cat "$tmp/$1")" = "$(printf '%s\n%s' "$2" "$3")" ]
}

# Returns 0 when the image tracked every change, 1 when it went stale.
tracks_changes() {
  local target=$1
  rm -rf "$tmp/b" "$tmp/out" "$tmp/out-nostamp"
  printf 'old\n' > "$tmp/in.v"
  make -s -C "$tmp" "$target" DEFS=-DA >/dev/null || exit 1
  reflects "$target" -DA old || return 1

  printf 'new\n' > "$tmp/in.v"
  touch -t 200001010000 "$tmp/in.v" || exit 1
  make -s -C "$tmp" "$target" DEFS=-DA >/dev/null || exit 1
  reflects "$target" -DA new || return 1

  if [ "$target" = out ]; then
    make -s -C "$tmp" "$target" DEFS=-DB >/dev/null || exit 1
    reflects "$target" -DB new || return 1

    printf 'tie\n' > "$tmp/in.v"
    make -s -C "$tmp" "$target" DEFS=-DB >/dev/null || exit 1
    reflects "$target" -DB tie || return 1
  fi
  return 0
}

failed=0

if tracks_changes out; then
  echo "ok: a changed input with an older mtime, and a changed define, both rebuild the stamped image"
else
  echo "FAIL: the stamped image went stale" >&2
  failed=1
fi

if tracks_changes out-nostamp; then
  echo "FAIL: the unstamped control tracked the change, so the scenario no longer reproduces the defect" >&2
  failed=1
else
  echo "ok: the unstamped control goes stale, so the scenario can fail"
fi

IMAGES="nano/tb/nano_rtl.cc nano-sim nano/tb/nano_icarus.vvp nano/tb/nano_qspi_pins_rtl.cc
nano-qspi-pins-sim nano/tb/nano_icarus_qspi_pins.vvp nano/tb/nano_qspi_resume.vvp nano/tb/nano_qspi_latency.vvp"

# $1 = the tb.mk to read. Prints each image whose prerequisites name no .stamp.
unstamped() {
  local img rule db
  rm -rf "$tmp/scan"
  mkdir -p "$tmp/scan/nano"
  cp "$REPO/nano/stamp.mk" "$tmp/scan/nano/stamp.mk" || exit 1
  cp "$1" "$tmp/scan/nano/tb.mk" || exit 1
  printf 'BUILD := build\nall:\ninclude nano/tb.mk\n' > "$tmp/scan/Makefile"
  db=$(make -C "$tmp/scan" -qp 2>/dev/null || true)
  if ! grep -q '^nano-sim:' <<< "$db"; then
    echo "error: make -qp read no nano-sim rule from $1" >&2
    exit 1
  fi
  for img in $IMAGES; do
    rule=$(printf '%s\n' "$db" | grep -F "$img:" | head -1 || true)
    case "$rule" in
      *.stamp*) ;;
      *) echo "$img" ;;
    esac
  done
}

bare=$(unstamped "$REPO/nano/tb.mk")
if [ -n "$bare" ]; then
  echo "FAIL: not keyed on a stamp: $bare" >&2
  failed=1
else
  echo "ok: every nano simulator image is keyed on a stamp"
fi

sed 's/ \$(NANO_QSPI_PINS_STAMP)$//' "$REPO/nano/tb.mk" > "$tmp/tb.mutant.mk" || exit 1
if cmp -s "$REPO/nano/tb.mk" "$tmp/tb.mutant.mk"; then
  echo "error: the structural mutation matched nothing; re-anchor it on tb.mk's spelling." >&2
  exit 1
fi
mutant_bare=$(unstamped "$tmp/tb.mutant.mk")
if [ -z "$mutant_bare" ]; then
  echo "FAIL: dropping a stamp from a prerequisite list went unnoticed" >&2
  failed=1
else
  echo "ok: the structural check catches an image with its stamp removed"
fi

exit "$failed"
