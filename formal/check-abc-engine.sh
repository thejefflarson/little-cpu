#!/bin/bash
# `complete` is the only target here whose engine is `abc bmc3`, and sby builds
# that engine's AIG through a yosys script it writes itself. Where yosys and sby
# were installed separately they can disagree about one line of that script --
# an `abc` call carrying `-fast` -- and the target then dies on a command syntax
# error inside a pass nobody in this repo wrote, which reads like a broken
# branch. Run the same call first and name the toolchain instead.
#
# This can only ADD a failure. It exits 0 whenever it has no positive evidence,
# so a working toolchain reaches sby exactly as before, and a toolchain that
# defeats the probe still fails the way it does today.
#
# Usage: formal/check-abc-engine.sh
set -euo pipefail

yosys_bin=$(command -v "${YOSYS:-yosys}") || exit 0
sby_bin=$(command -v sby) || exit 0

# Read the call out of the sby that will run rather than hardcoding a copy of
# it. A copy would go stale in the one direction that matters: refusing to start
# on a yosys/sby pairing where the real invocation works. These are the two
# directories sby's own launcher puts on sys.path.
sby_dir=$(dirname "$sby_bin")
abc_call=""
for lib in "$sby_dir/share/python3" "$sby_dir/../share/yosys/python3"; do
  [ -r "$lib/sby_core.py" ] || continue
  abc_call=$(awk -F'"' '/print\("abc /{print $2; exit}' "$lib/sby_core.py")
  break
done
[ -n "$abc_call" ] || exit 0

probe=$("$yosys_bin" -qp "$abc_call" 2>&1) && exit 0

# A yosys that fell over for some other reason -- a missing library, a suite
# whose environment was not sourced -- is not evidence about the call, and
# refusing to start on it would block a toolchain that works. Only yosys' own
# words for a rejected argument list count.
case "$probe" in
  *"Command syntax error"*) ;;
  *) exit 0 ;;
esac

version=$("$yosys_bin" -V 2>/dev/null | head -1)

cat >&2 <<MSG
error: the yosys and sby on this PATH cannot run \`make -C formal complete\`.

It is the only target here whose engine is \`abc bmc3\`, and sby builds that
engine's AIG with a yosys call this yosys rejects as a command syntax error:

    $abc_call
        asked for by $sby_bin
        refused by   $yosys_bin
        $version

That is the toolchain, not this tree: the same call fails on unmodified main.
The call is sby's own and is deliberately not adapted to whichever abc is on
this machine -- doing that would grade something other than what CI grades.

The check is NOT being skipped. It runs on CI in the formal job, on the YosysHQ
OSS CAD Suite, where it passes: that suite ships yosys and sby together as a
matching pair, which is what this PATH does not have. To run it here, put its
bin directory ahead of both of the above.
MSG
exit 1
