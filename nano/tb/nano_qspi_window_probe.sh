#!/bin/sh
# Forces nano_qspi_ctrl's FLASH_WINDOW_BYTES elaboration check red for its own reason,
# the way test/window_test.sh does for littlecpu's rtl/imemory.v: a window too wide for
# FLASH_TAG_BITS parcel tags must refuse to elaborate, in both frontends.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

for tool in iverilog yosys; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "error: $tool is not on PATH, so nano_qspi_ctrl's FLASH_WINDOW_BYTES check" >&2
    echo "cannot be forced to fire. Skipping would report a green run for a check" >&2
    echo "that was never executed." >&2
    exit 1
  fi
done

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-qspi-window.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
cd "$tmp"
cp "$REPO/nano/qspi.v" dut.v

run_case() {  # $1 = label, $2 = params, $3 = reject|accept
  label=$1
  params=$2
  expect=$3
  printf '`default_nettype none\nmodule window_probe;\n  nano_qspi_ctrl #(%s) dut (\n    .clk(1'"'"'b0), .reset(1'"'"'b1), .mem_valid(1'"'"'b0), .mem_instr(1'"'"'b0),\n    .mem_addr(32'"'"'b0), .mem_wdata(32'"'"'b0), .mem_wstrb(4'"'"'b0), .sio_in(4'"'"'b0));\nendmodule\n' \
    "$params" > probe.v

  iv_out=$(iverilog -g2012 -o /dev/null dut.v probe.v 2>&1) && iv_rc=0 || iv_rc=$?
  ys_out=$(yosys -p "read_verilog -sv dut.v probe.v; hierarchy -top window_probe" 2>&1) && ys_rc=0 || ys_rc=$?

  if [ "$expect" = reject ]; then
    if [ "$iv_rc" -eq 0 ] || [ "$ys_rc" -eq 0 ]; then
      echo "FAIL $label: elaborated cleanly (iverilog rc=$iv_rc, yosys rc=$ys_rc); the" >&2
      echo "check did not fire." >&2
      return 1
    fi
    case $iv_out in
      *FLASH_WINDOW_BYTES*) ;;
      *) echo "FAIL $label: iverilog rejected it, but not for FLASH_WINDOW_BYTES:" >&2
         printf '%s\n' "$iv_out" | sed 's/^/    /' >&2; return 1 ;;
    esac
    case $ys_out in
      *FATAL*) ;;
      *) echo "FAIL $label: yosys did not report a FATAL:" >&2
         printf '%s\n' "$ys_out" | sed 's/^/    /' >&2; return 1 ;;
    esac
    echo "ok   $label rejected in both frontends"
  else
    if [ "$iv_rc" -ne 0 ] || [ "$ys_rc" -ne 0 ]; then
      echo "FAIL $label: the shipping window was rejected (iverilog rc=$iv_rc, yosys" >&2
      echo "rc=$ys_rc):" >&2
      printf '%s\n%s\n' "$iv_out" "$ys_out" | sed 's/^/    /' >&2
      return 1
    fi
    echo "ok   $label accepted in both frontends"
  fi
}

failed=0
run_case "shipping FLASH_WINDOW_BYTES (16 MiB)" "" accept || failed=1
run_case "a 32 MiB window against 23-bit tags" ".FLASH_WINDOW_BYTES(32'h0200_0000)" reject \
  || failed=1

if [ "$failed" -ne 0 ]; then
  echo "*** FLASH_WINDOW_BYTES's elaboration check did not behave as required." >&2
  exit 1
fi
echo "shipping accepted, an oversized window rejected in both frontends."
