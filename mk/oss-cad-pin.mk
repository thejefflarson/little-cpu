# The OSS CAD Suite release this repo builds, proves and times with, and the one place it is
# written down. `make cad-setup` fetches the asset for this machine, refuses one whose SHA-256
# is not listed here, and unpacks it next to the other tools. A move to a newer release is a
# change to this file, its digests and an ADR, never a side effect of a new day.
ifneq ($(filter command line environment,$(foreach v,OSS_CAD_TAG OSS_CAD_YOSYS,$(origin $v))),)
$(error OSS_CAD_TAG and OSS_CAD_YOSYS cannot be set from the command line or the \
  environment: they pin the yosys, nextpnr, icetime and sby every build, proof and \
  timing run goes through. Change them in mk/oss-cad-pin.mk, together with the digests)
endif
override OSS_CAD_TAG := 2026-11-09

# The first line `yosys -V` prints for that release, up to the end of the commit id: the
# trailing `-dirty` and compiler text differ between the platforms' builds of one release.
override OSS_CAD_YOSYS := Yosys 0.70+360 (git sha1 5e740962d

ifeq ($(shell printf '%s' '$(OSS_CAD_TAG)' | grep -cE '^[0-9]{4}-[0-9]{2}-[0-9]{2}$$'),0)
$(error OSS_CAD_TAG must be a release date like 2026-11-09, not a branch, a moving tag \
  or a range: '$(OSS_CAD_TAG)')
endif

# Names embed the release on purpose: bumping the tag without adding digests makes the
# lookup empty, and cad-setup refuses rather than fetches.
OSS_CAD_SHA256_oss-cad-suite-darwin-arm64-20261109 := 024cd74ba5d8bc1891c7b5ec1f1d0fc307429f0ac7fb1bd1f1c2e1d1083d92c4
OSS_CAD_SHA256_oss-cad-suite-linux-arm64-20261109  := 39f1dcc67591517a0f60e8b586c6f244dd6c7bb0ad19f6e6676bccc43483bbb5
OSS_CAD_SHA256_oss-cad-suite-linux-x64-20261109    := 256b113c88b8da8b363e739dd97babe89eefbfdbb84373ccae0666ae19ac304d
