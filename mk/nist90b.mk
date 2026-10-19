# NIST SP 800-90B tools, pinned like nano/nano.mk pins the liberty; the board run is off `make test`.
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

.PHONY: nist90b-grade-test entropy-stream-test entropy-upduino-test
nist90b-grade-test:
	@python3 ./test/nist90b_grade_test.py

entropy-stream-test:
	@python3 ./test/entropy_stream_test.py

entropy-upduino-test:
	@python3 ./test/entropy_upduino_test.py

ENTROPY_NIST_RESTARTS ?= 1000
ENTROPY_NIST_SAMPLES  ?= 1000000
ENTROPY_NIST_FLOOR    ?= 0.5
ENTROPY_NIST_OUT      ?= $(BUILD)/nist
ENTROPY_NIST_LOCK     ?= $(TOOL_CACHE)/icesugar.lock
ENTROPY_NIST_RESUME   ?=

define entropy_nist_run
@soc/icelink_eject.sh
@python3 soc/entropy_nist.py --tag $(1) --core-pad $(2) --out '$(ENTROPY_NIST_OUT)' \
  --bin '$(NIST_90B_BIN)' --samples $(ENTROPY_NIST_SAMPLES) --restarts $(ENTROPY_NIST_RESTARTS) \
  --floor $(ENTROPY_NIST_FLOOR) --lock '$(ENTROPY_NIST_LOCK)' $(if $(ENTROPY_NIST_RESUME),--resume)
endef

.PHONY: entropy-nist-pll entropy-nist-pad entropy-nist
entropy-nist-pll: nist90b-setup
	$(call entropy_nist_run,pll,0)

entropy-nist-pad: nist90b-setup
	$(call entropy_nist_run,pad,1)

entropy-nist: entropy-nist-pll entropy-nist-pad

ENTROPY_UPDUINO_OSC  ?= internal
ENTROPY_UPDUINO_FAB  ?= 0
ENTROPY_UPDUINO_LOCK ?= $(TOOL_CACHE)/upduino.lock

.PHONY: entropy-nist-upduino
entropy-nist-upduino: nist90b-setup
	@soc/icelink_eject.sh
	@python3 soc/entropy_nist.py --board upduino --tag upduino_$(ENTROPY_UPDUINO_OSC)_f$(ENTROPY_UPDUINO_FAB) \
	  --osc $(ENTROPY_UPDUINO_OSC) --fab-log2 $(ENTROPY_UPDUINO_FAB) --out '$(ENTROPY_NIST_OUT)' \
	  --bin '$(NIST_90B_BIN)' --samples $(ENTROPY_NIST_SAMPLES) --restarts $(ENTROPY_NIST_RESTARTS) \
	  --floor $(ENTROPY_NIST_FLOOR) --lock '$(ENTROPY_UPDUINO_LOCK)' $(if $(ENTROPY_NIST_RESUME),--resume)
