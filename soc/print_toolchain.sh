#!/bin/sh
# Prints the toolchain a measurement was taken with: one line per tool, its resolved path
# and the version it answers with, refusing rather than guessing when a tool cannot be
# asked.
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

icetime_probe() {
  asc=$(mktemp "${TMPDIR:-/tmp}/icetime-chipdb-probe.XXXXXX")
  printf '.comment icetime chipdb probe\n.device 5k\n' > "$asc"
  said=$("$1" -d up5k "$asc" 2>&1) || true
  rm -f "$asc"
  # A positive marker, not a denylist of known failure text: icetime prints "Reading ...
  case "$said" in
    *"Creating timing netlist"*) return 0 ;;
  esac
  echo "*** soc/print_toolchain.sh: icetime did not reach its timing stage, so" >&2
  echo "*** it could not resolve its up5k chip database. That breaks the timing" >&2
  echo "*** half of \`make soc-timing\` and every sweep built on it:" >&2
  printf '%s\n' "$said" | sed -e 's/^/*** /' >&2
  echo "*** Put the OSS CAD Suite's icetime ahead of this one on PATH" >&2
  echo "*** (\$XDG_CACHE_HOME/little-cpu/oss-cad-suite/bin, or" >&2
  echo "*** ~/.cache/little-cpu/oss-cad-suite/bin), or reinstall icestorm." >&2
  return 1
}

icetime_version() {
  version_file=$(dirname "$1")/../VERSION
  if [ -r "$version_file" ]; then
    printf 'oss-cad-suite %s sha256:%s' "$(sed -n '1p' "$version_file")" "$(digest "$1")"
  else
    printf 'no version string sha256:%s' "$(digest "$1")"
  fi
}

trellis_db() {
  if [ -n "${TRELLIS_DB:-}" ]; then
    db=$TRELLIS_DB
  else
    pnr=$(command -v nextpnr-ecp5) || {
      echo "*** soc/print_toolchain.sh: no nextpnr-ecp5 on PATH, so the Trellis" >&2
      echo "*** database it places against cannot be located either." >&2
      exit 1
    }
    db=$(dirname "$pnr")/../share/trellis/database
  fi
  devices=$db/devices.json
  if [ ! -r "$devices" ]; then
    echo "*** soc/print_toolchain.sh: no readable $devices, so there is no" >&2
    echo "*** Trellis database to stamp this measurement with. Set TRELLIS_DB" >&2
    echo "*** to the database directory rather than leaving it unrecorded." >&2
    exit 1
  fi
  printf 'devices.json sha256:%s [%s]' "$(digest "$devices")" "$db"
}

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

nl='
'
block=
for tool in "$@"; do
  if [ "$tool" = trellis-db ]; then
    version=$(trellis_db) || exit 1
    block=${block:+$block$nl}"# trellis-db: $version"
    continue
  fi
  path=$(command -v "$tool") || {
    echo "*** soc/print_toolchain.sh: no $tool on PATH, so there is nothing to" >&2
    echo "*** stamp this measurement with." >&2
    exit 1
  }
  # The three ways a tool here answers the question, in one table.
  case $tool in
    icetime)        icetime_probe "$path" || exit 1
                     version=$(icetime_version "$path") ;;
    yosys|iverilog) version=$(first_line "$tool" -V) ;;
    *)              version=$(first_line "$tool" --version) ;;
  esac
  block=${block:+$block$nl}"# $tool: $version [$path]"
done
printf '%s\n' "$block"
