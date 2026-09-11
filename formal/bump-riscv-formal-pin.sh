#!/bin/bash
# Runs `make test/monitor.v`, which executes upstream's own generate.py -- so this
# step is given no GH_TOKEN and its checkout carries no credential.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [ "$#" -ne 5 ]; then
  echo "usage: bump-riscv-formal-pin.sh <pin-sha> <upstream-sha> <branch> <title> <out-dir>" >&2
  exit 2
fi

PIN_SHA=$1
UPSTREAM_SHA=$2
BRANCH=$3
TITLE=$4
OUT_DIR=$5

if [ -n "${GH_TOKEN:-}" ]; then
  echo "bump-riscv-formal-pin.sh: GH_TOKEN is set; this step runs upstream code and must not hold one" >&2
  exit 2
fi

mkdir -p "$OUT_DIR"
UPSTREAM_URL="https://github.com/YosysHQ/riscv-formal.git"

CLONE_DIR=$(mktemp -d)
trap 'rm -rf "$CLONE_DIR"' EXIT
git clone --quiet --filter=blob:none "$UPSTREAM_URL" "$CLONE_DIR"

DIFFSTAT=$(git -C "$CLONE_DIR" diff --stat "$PIN_SHA..$UPSTREAM_SHA" -- checks/ insns/ monitor/)
FULLDIFF=$(git -C "$CLONE_DIR" diff "$PIN_SHA..$UPSTREAM_SHA" -- checks/ insns/ monitor/)
DIFF_LINES=$(printf '%s\n' "$FULLDIFF" | wc -l | tr -d ' ')

# git show, not the clone's working tree, which sits at whatever HEAD was at clone time.
RVFI_INSN_CHECK_REPORT="$CLONE_DIR/rvfi-insn-check-report.txt"
UPSTREAM_RVFI_INSN_CHECK="$CLONE_DIR/upstream-rvfi_insn_check.sv"
RVFI_INSN_CHECK_STATUS=0
if git -C "$CLONE_DIR" show "$UPSTREAM_SHA:checks/rvfi_insn_check.sv" \
    > "$UPSTREAM_RVFI_INSN_CHECK" 2>/dev/null; then
  python3 nano/formal/check-rvfi-insn-check.py \
    "$UPSTREAM_RVFI_INSN_CHECK" nano/formal/rvfi_insn_check.sv \
    > "$RVFI_INSN_CHECK_REPORT" 2>&1 || RVFI_INSN_CHECK_STATUS=$?
else
  RVFI_INSN_CHECK_STATUS=2
  echo "upstream no longer ships checks/rvfi_insn_check.sv at $UPSTREAM_SHA" \
    > "$RVFI_INSN_CHECK_REPORT"
fi
RVFI_INSN_CHECK_REPORT_LINES=$(wc -l < "$RVFI_INSN_CHECK_REPORT" | tr -d ' ')

git checkout -b "$BRANCH"

python3 - "$UPSTREAM_SHA" <<'EOF'
import pathlib, re, sys
new_sha = sys.argv[1]
p = pathlib.Path('formal/pin.mk')
text = p.read_text()
new_text, n = re.subn(
    r'(override RISCV_FORMAL_SHA := )[0-9a-f]{40}',
    lambda m: m.group(1) + new_sha,
    text,
)
if n != 1:
    sys.exit(f'expected exactly one RISCV_FORMAL_SHA line, replaced {n}')
p.write_text(new_text)
EOF

rm -rf formal/riscv-formal
make test/monitor.v

if git diff --quiet -- formal/pin.mk test/monitor.v; then
  echo "bumping the pin produced no diff in formal/pin.mk or test/monitor.v" >&2
  exit 1
fi

COMPARE_URL="https://github.com/YosysHQ/riscv-formal/compare/${PIN_SHA}...${UPSTREAM_SHA}"
printf '%s\n' "$TITLE" > "$OUT_DIR/title"
printf '%s\n' "$BRANCH" > "$OUT_DIR/branch"
{
  echo "pinned:   $PIN_SHA"
  echo "upstream: $UPSTREAM_SHA"
  echo "compare:  $COMPARE_URL"
  echo
  printf '%s\n' "$DIFFSTAT"
} > "$OUT_DIR/commit-trailer"

{
  echo "Upstream riscv-formal moved."
  echo
  echo "- pinned:   \`$PIN_SHA\`"
  echo "- upstream: \`$UPSTREAM_SHA\`"
  echo "- compare:  $COMPARE_URL"
  echo
  echo "\`test/monitor.v\` is regenerated against the new pin in this commit."
  echo "The existing gates decide whether the bump is safe: monitor-freshness,"
  echo "\`formal/check-genchecks.py\`, \`formal/check-complete-exclusions.py\`,"
  echo "and the generated riscv-formal checks themselves."
  echo
  echo "### nano/formal/rvfi_insn_check.sv (nanocpu's RV32E oracle patch)"
  echo
  if [ "$RVFI_INSN_CHECK_STATUS" -eq 0 ]; then
    echo "Still in sync with the new pin -- \`nano/formal/check-rvfi-insn-check.py\`"
    echo "reports no drift beyond the documented \`RISCV_FORMAL_E\` block."
  else
    echo "**STALE.** \`nano/formal/check-rvfi-insn-check.py\` reports a residual diff"
    echo "against the new pin, which is not regenerated here -- it is a hand-written"
    echo "fork, not a vendored copy. Re-apply the \`RISCV_FORMAL_E\` assumption block by"
    echo "hand before merging; \`monitor-freshness\` will not pass otherwise."
    echo
    # Tildes, not backticks: SystemVerilog's own preprocessor directives are backtick-prefixed.
    if [ "$RVFI_INSN_CHECK_REPORT_LINES" -le 300 ]; then
      echo '~~~diff'
      cat "$RVFI_INSN_CHECK_REPORT"
      echo '~~~'
    else
      echo "Report is ${RVFI_INSN_CHECK_REPORT_LINES} lines; run"
      echo '~~~'
      echo "python3 nano/formal/check-rvfi-insn-check.py <upstream file at $UPSTREAM_SHA> nano/formal/rvfi_insn_check.sv"
      echo '~~~'
      echo "locally against the new pin to see it."
    fi
  fi
  echo
  echo "### Diff under checks/, insns/, monitor/"
  echo
  if [ -z "$DIFFSTAT" ]; then
    echo "None -- this bump only touches other directories (e.g. cores/)."
  else
    echo '```'
    printf '%s\n' "$DIFFSTAT"
    echo '```'
    if [ "$DIFF_LINES" -le 300 ]; then
      echo
      echo "<details><summary>Full diff (${DIFF_LINES} lines)</summary>"
      echo
      echo '```diff'
      printf '%s\n' "$FULLDIFF"
      echo '```'
      echo "</details>"
    else
      echo
      echo "Full diff is ${DIFF_LINES} lines; see the compare link above."
    fi
  fi
} > "$OUT_DIR/issue-body.md"

echo "regenerated against $UPSTREAM_SHA on $BRANCH; body in $OUT_DIR/issue-body.md"
