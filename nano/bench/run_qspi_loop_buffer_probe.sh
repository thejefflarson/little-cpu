#!/bin/bash
# The forced-red direction for run_qspi_loop_buffer_test.sh: reintroduces a real loop-buffer
# defect into a scratch copy of nano_qspi_memory.v and requires the reported FAIL, then checks
# that the model refuses to elaborate a tagged block too small to index.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: run_qspi_loop_buffer_probe.sh <nano-march-cflags>" >&2
  exit 2
fi
CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-qspi-loop-probe.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

# A build holds about 600 MB, so NANO_PROBE_JOBS bounds the mutations run at once.
jobs=${NANO_PROBE_JOBS:-1}
launched=0

# The compile is nearly all the cost, so a mutation builds only the one shape that catches it.
none_sim="$tmp/nano-qspi-sim.none"
if ! NANO_LOOP_SHAPES=none NANO_LOOP_SAVE_NONE="$none_sim" "$HERE/run_qspi_loop_buffer_test.sh" "$CFLAGS" \
  "$REPO/nano/tb/nano_qspi_memory.v"; then
  echo "error: the shipping model's no-loop-buffer simulator did not build" >&2
  exit 1
fi

run_mutation() {  # <label> <shapes> <sed-expr> <expected-text>
  local label=$1 shapes=${NANO_PROBE_SHAPES:-$2} expr=$3 want=$4
  local mutant="$tmp/$label.v"
  cp "$REPO/nano/tb/nano_qspi_memory.v" "$mutant"
  sed -i.bak -e "$expr" "$mutant"
  if cmp -s "$mutant" "$mutant.bak"; then
    echo "error: mutation '$label' matches nothing in nano_qspi_memory.v -- probe is stale" >&2
    exit 1
  fi
  rm -f "$mutant.bak"

  local out rc
  set +e
  out=$(NANO_LOOP_SHAPES="none $shapes" NANO_LOOP_NONE_SIM="$none_sim" \
    "$HERE/run_qspi_loop_buffer_test.sh" "$CFLAGS" "$mutant" 2>&1)
  rc=$?
  set -e
  if [ "$rc" -eq 0 ] || ! grep -qF -- "$want" <<< "$out"; then
    echo "RED PROBE FAILED: mutation '$label' did not produce '$want' (exit $rc)" >&2
    printf '%s\n' "$out" | sed -e 's/^/    /' >&2
    return 1
  fi
  echo "ok: mutation '$label' caught"
}

launch() {
  launched=$((launched + 1))
  { rc=0; ( run_mutation "$@" ) > "$tmp/out.$launched" 2>&1 || rc=$?; echo "$rc" > "$tmp/status.$launched"; } &
  if [ $((launched % jobs)) -eq 0 ]; then wait; fi
}

report_mutations() {
  wait
  local i
  for ((i = 1; i <= launched; i++)); do
    cat "$tmp/out.$i"
    [ "$(cat "$tmp/status.$i")" = 0 ] || status=1
  done
}

status=0
launch "loop-hit-gated-on-xfer-active" "tagged" \
  "s/loop_hit_now ? 1'b1 :/loop_hit_now ? xfer_active :/" \
  "FETCH/RETIRE MISMATCH"
launch "one-of-two-hits-a-cycle-late" "tagged" \
  "s/loop_hit_now ? 1'b1 :/loop_hit_now ? (xfer_active || !mem_addr[1]) :/" \
  "FETCH/RETIRE MISMATCH"
launch "one-of-two-hits-booked-as-a-handshake" "tagged" \
  "s/assign reason_loop_hit = mem_valid \&\& mem_instr \&\& loop_hit_now;/assign reason_loop_hit = mem_valid \&\& mem_instr \&\& loop_hit_now \&\& !mem_addr[1];/;s/assign reason_handshake = mem_valid \&\& mem_instr \&\& !loop_hit_now \&\& !preamble_active_now \&\& mem_ready;/assign reason_handshake = mem_valid \&\& mem_instr \&\& (!loop_hit_now || mem_addr[1]) \&\& !preamble_active_now \&\& mem_ready;/" \
  "loop-buffer hits over"
launch "cam-lookup-never-hits" "cam" \
  "s/if (cam_valid\[i\] && cam_idx\[i\] == target_index) cam_has0 = 1'b1;/if (1'b0) cam_has0 = 1'b1;/" \
  "still pays a marginal preamble/wait cost"
launch "queue-head-follows-every-hit" "tagged" \
  "s/if (target_index == fifo_head) fifo_head <= target_index + target_len;/fifo_head <= target_index + target_len;/" \
  "already handed over"
launch "fetch-ready-before-its-last-parcel-arrives" "tagged" \
  "s/aimed_now \&\& arrived_valid \&\& arrived_index >= target_last);/aimed_now \&\& arrived_valid);/" \
  "never fetched in its current run"
launch "queue-lead-unsigned" "cam" \
  "s/^  int produce_lead;/  int unsigned produce_lead;/" \
  "TIMEOUT"
launch "queued-parcel-counted-as-hit" "tagged" \
  "s/assign loop_hit_full = loop_hit_lookup \&\& !fifo_serves;/assign loop_hit_full = loop_hit_lookup;/" \
  "must be a handshake, not a hit"
launch "aim-ignores-queue-head" "tagged" \
  "s/(stream_open || preamble_pending) \&\& fifo_head == target_index;/(stream_open || preamble_pending) \&\& target_index >= preamble_target;/" \
  "already handed over"
launch "tagged-hit-skips-second-parcel-valid-bit" "tagged" \
  "s/tag_window_bits\[target_index_p1\[SLOTBITS-1:0\]\];/1'b1;/" \
  "never entered the buffer"
launch "cam-lookup-ignores-valid-bit" "cam" \
  "s/cam_valid\[i\] \&\& cam_idx\[i\] == target_index) cam_has0/cam_idx[i] == target_index) cam_has0/;s/cam_valid\[i\] \&\& cam_idx\[i\] == target_index + 1) cam_has1/cam_idx[i] == target_index + 1) cam_has1/" \
  "never entered the buffer"
launch "tag-compare-dropped" "tagged" \
  "s/target_index\[31:SLOTBITS\] == tag_window_tag \&\&//;s/target_index_p1\[31:SLOTBITS\] == tag_window_tag \&\&//" \
  "never entered the buffer"
launch "cam-holds-twice-its-window" "cam" \
  "s/localparam int CAM_SLOTS = SLOTS;/localparam int CAM_SLOTS = 2 * SLOTS;/" \
  "already evicted"
launch "resync-first-cycle-charged-to-parcel-wait" "tagged" \
  "s/(preamble_pending || (redirect_now \&\& !loop_hit_full))/(redirect_now \&\& !loop_hit_full)/" \
  "still in its address phase"

report_mutations

elaborate() {  # <loop kind> <loop window> -> yosys's output; sets rc
  set +e
  elab_out=$(yosys -p "read_verilog -sv $REPO/nano/tb/nano_qspi_memory.v; hierarchy -top nano_qspi_memory -chparam LOOP_KIND $1 -chparam LOOP_WINDOW $2" 2>&1)
  elab_rc=$?
  set -e
}
# yosys cannot print a $fatal's message, only name the line it sits on, so the probe names the line.
fatal_line=$(grep -nF 'LOOP_WINDOW must be at least 2' "$REPO/nano/tb/nano_qspi_memory.v" | cut -d: -f1) || fatal_line=""
if [ -z "$fatal_line" ]; then
  echo "error: the one-slot tagged block check is gone from nano_qspi_memory.v -- probe is stale" >&2
  exit 1
fi
elaborate 1 1
if [ "$elab_rc" -eq 0 ] || ! grep -qF -- "nano_qspi_memory.v:$fatal_line: ERROR" <<< "$elab_out"; then
  echo "RED PROBE FAILED: a one-slot tagged block was not refused at line $fatal_line (exit $elab_rc)" >&2
  printf '%s\n' "$elab_out" | sed -e 's/^/    /' >&2
  status=1
else
  echo "ok: a one-slot tagged block refused"
fi
elaborate 1 2
if [ "$elab_rc" -ne 0 ]; then
  echo "RED PROBE FAILED: a two-slot tagged block, the smallest legal one, was refused (exit $elab_rc)" >&2
  printf '%s\n' "$elab_out" | sed -e 's/^/    /' >&2
  status=1
else
  echo "ok: a two-slot tagged block accepted"
fi

exit $status
