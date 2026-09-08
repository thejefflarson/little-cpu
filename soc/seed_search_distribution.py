#!/usr/bin/env python3
"""Turn a `soc/soc_seed_search.sh` run's per-seed samples into the JSON
`soc/soc_pin.py write --distribution` records, so `soc/pin.json` carries the whole
sweep a seed was chosen from and not just the winner.

Usage: seed_search_distribution.py <samples file: '<seed> <mhz>' per line> <chosen seed>
                                    <seeds source, free text>
"""

import json
import sys

def main():
    if len(sys.argv) != 4:
        sys.exit("usage: seed_search_distribution.py <samples> <chosen-seed> <seeds-source>")
    samples_path, chosen_seed, seeds_source = sys.argv[1], int(sys.argv[2]), sys.argv[3]

    samples = []
    with open(samples_path) as handle:
        for line in handle:
            seed, mhz = line.split()
            samples.append({"seed": int(seed), "mhz": float(mhz)})

    print(json.dumps({
        "note": "Seeds 1..N are a structured lattice in nextpnr-ice40's xorshift "
                "RNG state (state(3) = state(1) XOR state(2) over GF(2)), not an "
                "independent sample -- read the spread here as one search's "
                "outcome, not as the design's placement distribution. See "
                "ADR-0171.",
        "seeds_source": seeds_source,
        "samples": samples,
        "chosen_seed": chosen_seed,
    }, indent=2))

if __name__ == "__main__":
    main()
