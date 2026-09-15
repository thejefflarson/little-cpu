#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
exec bash "$root/soc/fetch_ahead/apply.sh" \
  "${1:?usage: soc/fetch_ahead/apply-decoupled.sh <output-dir>}" \
  "$root/soc/fetch_ahead/prototype-decoupled.patch"
