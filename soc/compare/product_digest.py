#!/usr/bin/env python3
"""The content digest a cross-core product stamp is keyed on.

A stamp that names a commit cannot be checked after a squash merge deletes the
commit, so soc/compare/product.json records a digest of the bytes the
measurement reads instead, the way soc/soc_pin.py keys soc/pin.json. Blob
bytes survive a squash merge; commit ids do not.

The digest is over the working tree, not HEAD: a stamp taken over uncommitted
edits and a stamp taken over the same edits once committed read the same bytes
and so carry the same digest. Untracked files that git does not ignore count,
because a new rtl/ file is an input the moment it exists.

Usage: product_digest.py [--repo DIR]     prints sha256:<hex>
Exit:  0 printed, 2 refused
"""

import argparse
import hashlib
import os
import subprocess
import sys

REFUSED = 2

# Everything a measurement reads: the cores, the harness, the benchmark sources,
# the build rules and the riscv-formal pin that supplies VexRiscv.
INPUT_PATHS = ("rtl", "soc/compare", "test/bench", "Makefile", "formal/pin.mk")

# Files under INPUT_PATHS that the stamp itself feeds. Digesting them makes the
# stamp stale on its own output: the weekly refresh rewrites the stamp and
# docs/comparison.md, and CYCLE_FLOOR is hand-edited from the stamp.
EXCLUDED = ("soc/compare/product.json", "soc/compare/CYCLE_FLOOR",
            "docs/comparison.md")

def refuse(message):
    print(message, file=sys.stderr)
    sys.exit(REFUSED)

def input_files(repo):
    command = ["git", "-C", repo, "ls-files", "-z", "--cached", "--others",
               "--exclude-standard", "--"] + list(INPUT_PATHS)
    try:
        out = subprocess.run(command, capture_output=True, check=True).stdout
    except FileNotFoundError:
        refuse("*** no git on PATH, so the measured inputs cannot be listed.")
    except subprocess.CalledProcessError as exc:
        refuse(f"*** git could not list the measured inputs in '{repo}': "
               f"{exc.stderr.decode(errors='replace').strip()}")
    names = {name.decode() for name in out.split(b"\0") if name}
    return sorted(names - set(EXCLUDED))

def file_hash(path):
    if os.path.islink(path):
        return "link:" + hashlib.sha256(os.readlink(path).encode()).hexdigest()
    try:
        with open(path, "rb") as handle:
            return hashlib.sha256(handle.read()).hexdigest()
    except FileNotFoundError:
        return "absent"
    except OSError as exc:
        refuse(f"*** cannot read measured input {path}: {exc}")

def content_digest(repo):
    """`sha256:<hex>` over every measured input's path and bytes. A tracked file
    deleted from the working tree hashes as `absent`, so deleting it moves the
    digest rather than dropping out of it unnoticed.
    """
    lines = "".join(f"{file_hash(os.path.join(repo, name))}  {name}\n"
                    for name in input_files(repo))
    return "sha256:" + hashlib.sha256(lines.encode()).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", default=".")
    print(content_digest(parser.parse_args().repo))

if __name__ == "__main__":
    main()
