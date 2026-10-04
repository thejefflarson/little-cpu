# Mtimes alone cannot see a changed input that is older than its product (a saved copy put back
# with cp -p, mv or rsync -a), or one that ties with it (make 3.81 reads whole seconds, so an edit
# in the second a build finished reads as no change). So each simulator image also depends on a
# stamp whose NAME holds a digest of its inputs and defines: a changed digest names a file that
# does not exist, which make must create and so rebuilds everything above it. Creating a stamp
# deletes the others of its family, so returning to an earlier digest is a rebuild too, and the
# second's wait before creation keeps the stamp newer than a product built in the same second.
# nano/tb/nano_stale_build_probe.sh grades it.
NANO_DIGEST := $(shell if command -v shasum >/dev/null 2>&1; then echo 'shasum -a 256'; else echo sha256sum; fi)

# $(1) names the variable that receives the stamp's path, $(2) lists the inputs, $(3) the defines.
define nano_stamp
$(1) := $(BUILD)/$(1).$$(firstword $$(shell { $(NANO_DIGEST) $(2) && printf '%s\n' '$(3)'; } | $(NANO_DIGEST))).stamp
$$($(1)):
	@mkdir -p $$(dir $$@); rm -f $(BUILD)/$(1).*.stamp; sleep 1; : > $$@
endef
