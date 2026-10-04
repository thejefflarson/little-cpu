#!/usr/bin/env python3
"""Refuse to publish a measured stamp the checked-out tree did not produce.

The publish job runs with write scopes and takes its stamp from an artifact, so it
re-derives what the stamp claims: every measured pair's `base` must be the commit
checked out, and its `digest` must equal soc/compare/product_digest.py over this tree.

Usage: product_verify.py STAMP --sha SHA [--repo DIR]
Exit:  0 every measured pair verified, 1 a pair did not verify, 2 refused
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from product_check import BASE_RE, REFUSED, load, refuse  # noqa: E402
from product_digest import content_digest  # noqa: E402


def problems(stamp, sha, digest):
    pairs = stamp.get("pairs")
    if not isinstance(pairs, dict):
        return ["the stamp has no pairs"]
    found = []
    measured = 0
    for name, pair in sorted(pairs.items()):
        if not isinstance(pair, dict) or pair.get("status") != "measured":
            continue
        measured += 1
        if pair.get("base") != sha:
            found.append(f"{name}: base is {pair.get('base')!r}, not the checked-out {sha}")
        if pair.get("digest") != digest:
            found.append(f"{name}: digest is {pair.get('digest')!r}, this tree's is {digest}")
    if not measured:
        found.append("the stamp has no measured pair")
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stamp")
    parser.add_argument("--sha", required=True)
    parser.add_argument("--repo", default=".")
    args = parser.parse_args()
    if not BASE_RE.fullmatch(args.sha):
        refuse(f"*** --sha '{args.sha}' is not a 40-character commit SHA")
    found = problems(load(args.stamp), args.sha, content_digest(args.repo))
    for line in found:
        print(f"*** {line}", file=sys.stderr)
    if found:
        sys.exit(1)
    print(f"every measured pair in {args.stamp} was taken at {args.sha} over this tree")


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as exc:
        print(f"*** {exc!r}", file=sys.stderr)
        sys.exit(REFUSED)
