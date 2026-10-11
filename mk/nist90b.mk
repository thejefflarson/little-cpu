# NIST's SP 800-90B reference tools, pinned the way nano/nano.mk pins the sky130 liberty, and the
# assessment of the iCESugar-Pro's seed source that runs them against the board.
ifneq ($(filter command line environment,$(origin NIST_90B_COMMIT)),)
$(error NIST_90B_COMMIT cannot be set from the command line or the environment: it pins \
  bytes this repo executes. Change it in mk/nist90b.mk, together with the SHA-256 digest \
  below it)
endif
override NIST_90B_COMMIT := 68ed165fd7a3eeef26b87a546ba23f338e82a3f3

ifeq ($(shell printf '%s' '$(NIST_90B_COMMIT)' | grep -cE '^[0-9a-f]{40}$$'),0)
$(error NIST_90B_COMMIT must be a full 40-hex commit id, not a branch or tag: '$(NIST_90B_COMMIT)')
endif

override NIST_90B_SHA256 := 067946993b73de947c435c48d811f5f9b2ce9084d8408512f263515816e19582

NIST_90B_DIR := $(TOOL_CACHE)/sp800-90b
NIST_90B_BIN := $(NIST_90B_DIR)/bin

.PHONY: nist90b-setup
nist90b-setup:
	@./soc/nist90b_setup.sh '$(NIST_90B_COMMIT)' '$(NIST_90B_SHA256)' '$(NIST_90B_DIR)' '$(SAIL_DOWNLOAD_DIR)'

.PHONY: nist90b-grade-test entropy-stream-test
nist90b-grade-test:
	@python3 ./test/nist90b_grade_test.py

entropy-stream-test:
	@python3 ./test/entropy_stream_test.py

# The multi-hour board run, off `make test`. It loads SRAM only, never the flash. ENTROPY_NIST_LOCK, when
# set, is a directory made around each load-and-read and removed after, for a bench shared with other work.
# ENTROPY_NIST_RESUME=1 keeps a capture already taken and continues a partly filled restart matrix.
ENTROPY_NIST_RESTARTS ?= 1000
ENTROPY_NIST_SAMPLES  ?= 1000000
ENTROPY_NIST_FLOOR    ?= 0.5
ENTROPY_NIST_OUT      ?= $(BUILD)/nist
ENTROPY_NIST_LOCK     ?=
ENTROPY_NIST_RESUME   ?=

define entropy_nist_run
@python3 soc/entropy_nist.py --tag $(1) --core-pad $(2) --out '$(ENTROPY_NIST_OUT)' \
  --bin '$(NIST_90B_BIN)' --samples $(ENTROPY_NIST_SAMPLES) --restarts $(ENTROPY_NIST_RESTARTS) \
  --floor $(ENTROPY_NIST_FLOOR) $(if $(ENTROPY_NIST_LOCK),--lock '$(ENTROPY_NIST_LOCK)') $(if $(ENTROPY_NIST_RESUME),--resume)
endef

.PHONY: entropy-nist-pll entropy-nist-pad entropy-nist
entropy-nist-pll: nist90b-setup
	$(call entropy_nist_run,pll,0)

entropy-nist-pad: nist90b-setup
	$(call entropy_nist_run,pad,1)

entropy-nist: entropy-nist-pll entropy-nist-pad
