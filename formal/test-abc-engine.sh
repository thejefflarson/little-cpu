#!/bin/bash
# Drives formal/check-abc-engine.sh against a stub yosys and a stub sby, and pins both
# directions: it goes red naming the OSS CAD Suite on a yosys that refuses sby's abc
# call, and it stays silently green on one that accepts it.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
SCRIPT="$HERE/check-abc-engine.sh"

tmp=$(mktemp -d "${TMPDIR:-/tmp}/abc-engine.XXXXXX") || {
  echo "error: could not create a temporary directory." >&2
  exit 1
}
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

# The stub yosys accepts exactly the call in $STUB_ACCEPTS and answers anything else the
# way a yosys that dropped a flag does, so a probe of the wrong string is visible here
# rather than only on somebody's laptop.
mkdir -p "$tmp/bin"
cat > "$tmp/bin/yosys" <<'STUB'
#!/bin/bash
if [ "${1:-}" = "-V" ]; then
  echo "Yosys 9.99 (stub)"
  exit 0
fi
if [ "${1:-}" = "-qp" ] && [ "${2:-}" = "$STUB_ACCEPTS" ]; then
  exit 0
fi
if [ -n "${STUB_OTHER_ERROR:-}" ]; then
  echo "dyld: Library not loaded: libyosys.so" >&2
  exit 1
fi
echo "ERROR: Command syntax error: Unknown option or option in arguments." >&2
exit 1
STUB
chmod +x "$tmp/bin/yosys"

cat > "$tmp/bin/sby" <<'STUB'
#!/bin/bash
echo "stub sby: not meant to run" >&2
exit 9
STUB
chmod +x "$tmp/bin/sby"

mkdir -p "$tmp/share/yosys/python3"
write_sby_core() {  # write_sby_core <abc call>
  printf '                print("%s", file=f)\n' "$1" > \
    "$tmp/share/yosys/python3/sby_core.py"
}

cases=0
failed=0

check() {
  local label=$1 want_exit=$2 want_text=$3 snippet=$4
  cases=$((cases + 1))
  local out rc why=""
  set +e
  out=$(eval "$snippet" 2>&1)
  rc=$?
  set -e
  if [ "$rc" -ne "$want_exit" ]; then
    why="exited $rc, expected $want_exit"
  elif [ -n "$want_text" ] && ! grep -qF -- "$want_text" <<< "$out"; then
    why="output does not contain $want_text"
  elif [ -z "$want_text" ] && [ -n "$out" ]; then
    why="expected no output, got some"
  fi
  if [ -z "$why" ]; then
    printf '  ok   %s\n' "$label"
  else
    printf '  RED  %s\n     -> %s\n' "$label" "$why" >&2
    printf '%s\n' "$out" | sed -e 's|^|        |' >&2
    failed=1
  fi
}

run() {  # run <accepted call> -- runs the script under test
  env PATH="$tmp/bin:$PATH" STUB_ACCEPTS="$1" "$SCRIPT"
}

echo "== a yosys that refuses sby's abc call"
write_sby_core "abc -g AND -fast"

check "it fails" 1 "cannot run" \
  'run "abc -g AND"'
check "it names the toolchain to install" 1 "OSS CAD Suite" \
  'run "abc -g AND"'
check "it says the check is not skipped" 1 "The check is NOT being skipped." \
  'run "abc -g AND"'
check "it says CI still runs it" 1 "It runs on CI in the formal job" \
  'run "abc -g AND"'
check "it quotes the call that was refused" 1 "abc -g AND -fast" \
  'run "abc -g AND"'
check "it names the sby that asked for it" 1 "asked for by $tmp/bin/sby" \
  'run "abc -g AND"'
check "it names the yosys that refused it" 1 "refused by   $tmp/bin/yosys" \
  'run "abc -g AND"'
check "it quotes that yosys' version" 1 "Yosys 9.99 (stub)" \
  'run "abc -g AND"'

echo
echo "== a yosys that accepts it"
check "it is green and silent" 0 "" \
  'run "abc -g AND -fast"'

echo
echo "== the call is read from sby, not hardcoded"
write_sby_core "abc -g AND,NAND -quick"
check "a different call is the one probed" 1 "abc -g AND,NAND -quick" \
  'run "abc -g AND -fast"'
check "a pairing that works is not blocked" 0 "" \
  'run "abc -g AND,NAND -quick"'

echo
echo "== no evidence, no verdict"
rm -f "$tmp/share/yosys/python3/sby_core.py"
check "an sby whose source is unreadable is green" 0 "" \
  'run "abc -g AND"'
write_sby_core "abc -g AND -fast"
check "no yosys at all is green" 0 "" \
  'env PATH="$tmp/bin:$PATH" STUB_ACCEPTS= YOSYS=no-such-yosys "$SCRIPT"'
check "a yosys that failed some other way is green" 0 "" \
  'env PATH="$tmp/bin:$PATH" STUB_ACCEPTS= STUB_OTHER_ERROR=1 "$SCRIPT"'

echo
if [ "$failed" -ne 0 ]; then
  echo "$cases checks, at least one RED." >&2
  exit 1
fi
echo "$cases checks, all green."
