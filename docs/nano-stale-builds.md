# Nano simulator images are keyed on a digest of their inputs

`nano/stamp.mk` defines `nano_stamp`, and every simulator image in `nano/tb.mk` depends on a stamp
it creates.

## Why mtimes are not enough

Make rebuilds a product when a prerequisite is newer than it. Two kinds of change escape that
test:

- **An input older than its product.** A saved copy put back with `cp -p`, `mv` or `rsync -a`
  carries its old mtime, so a changed source reads as unchanged.
- **An input that ties with its product.** GNU Make 3.81 (macOS) compares whole seconds, so an
  edit in the same second a build finished reads as no change.

## How the stamp works

The stamp's file NAME holds a digest of the image's inputs and its defines. A changed digest names
a file that does not exist, which make must create, and so it rebuilds everything above it.

- Creating a stamp deletes the other stamps of its family, so returning to an earlier digest is a
  rebuild too rather than a hit on a stale file.
- The rule waits one second before creating the stamp, which keeps the stamp newer than a product
  built in the same second.

`$(eval $(call nano_stamp,VAR,inputs,defines))` sets `VAR` to the stamp's path; `inputs` lists the
files the digest covers and `defines` the preprocessor flags.

## The grader

`nano/tb/nano_stale_build_probe.sh`, run by `make nano-stale-build-test` on `make test`'s path,
checks two things, each with a red direction:

1. **Content, not mtime, decides a rebuild.** A throwaway Makefile builds an image, rewrites its
   input with an mtime from the year 2000, changes a define, and rewrites the input again; the
   stamped image must reflect every change. The red direction is the same scenario against an
   unstamped rule, which must go stale — if it does not, the scenario no longer reproduces the
   defect.
2. **Every image is stamped.** `make -qp` over `nano/tb.mk` must show a `.stamp` prerequisite on
   each of the eight simulator images. The red direction drops one stamp from `tb.mk` and requires
   the check to name that image.

Both run in a temporary directory and touch no tracked file.
