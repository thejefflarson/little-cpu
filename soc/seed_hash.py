#!/usr/bin/env python3
"""Reproducible high-entropy placer seeds: one per line, count decided by the caller.

Seeds 1..N are not an independent draw on nextpnr-ice40's placer: `rngstate = seed`
followed by five xorshift warm-up rounds is linear over GF(2), so seeds 1..16 span only
a 4-dimensional subspace of a 64-dimensional state (ADR-0170). `soc/soc_seed_search.sh`'s
default search draws from here instead: sha256 of a plain, reproducible string, truncated
to 28 bits so it sits well inside nextpnr's signed-int `--seed` argument.

Usage: seed_hash.py <count>
"""

import hashlib
import sys

def seed(index):
    digest = hashlib.sha256(f"little-cpu-soc-seed-{index}".encode()).hexdigest()
    return int(digest[:7], 16)

def main():
    if len(sys.argv) != 2:
        sys.exit("usage: seed_hash.py <count>")
    try:
        count = int(sys.argv[1])
    except ValueError:
        sys.exit(f"seed_hash.py: '{sys.argv[1]}' is not an integer count")
    if count < 1:
        sys.exit("seed_hash.py: count must be at least 1")
    for index in range(count):
        print(seed(index))

if __name__ == "__main__":
    main()
