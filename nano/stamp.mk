# A stamp named by a digest of its inputs rebuilds what mtimes miss; docs/nano-stale-builds.md
# says why, and nano/tb/nano_stale_build_probe.sh grades it.
NANO_DIGEST := $(shell if command -v shasum >/dev/null 2>&1; then echo 'shasum -a 256'; else echo sha256sum; fi)

define nano_stamp
$(1) := $(BUILD)/$(1).$$(firstword $$(shell { $(NANO_DIGEST) $(2) && printf '%s\n' '$(3)'; } | $(NANO_DIGEST))).stamp
$$($(1)):
	@mkdir -p $$(dir $$@); rm -f $(BUILD)/$(1).*.stamp; sleep 1; : > $$@
endef
