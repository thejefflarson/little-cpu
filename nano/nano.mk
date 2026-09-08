# The sky130hd liberty nanocpu synthesises against, pinned the way formal/pin.mk pins riscv-formal.
ifneq ($(filter command line environment,$(origin NANO_LIBERTY_COMMIT)),)
$(error NANO_LIBERTY_COMMIT cannot be set from the command line or the environment: it \
  pins bytes this repo executes. Change it in nano/nano.mk, together with the SHA-256 \
  digest below it)
endif
override NANO_LIBERTY_COMMIT := b5dceb3c08fd30c9a1c5c0587bc6a57e0392080a

ifeq ($(shell printf '%s' '$(NANO_LIBERTY_COMMIT)' | grep -cE '^[0-9a-f]{40}$$'),0)
$(error NANO_LIBERTY_COMMIT must be a full 40-hex commit id, not a branch or tag: \
  '$(NANO_LIBERTY_COMMIT)')
endif

override NANO_LIBERTY_SHA256 := ec0e1067a35c8bf20b11e58d1e8ac53326067e4dac84a125cc1b917a3518d0d9
override NANO_LIBERTY_URL := https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD-flow-scripts/$(NANO_LIBERTY_COMMIT)/flow/platforms/sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib

NANO_LIBERTY_DIR := $(TOOL_CACHE)/sky130
NANO_LIBERTY     := $(NANO_LIBERTY_DIR)/sky130_fd_sc_hd__tt_025C_1v80.lib

.PHONY: nano-liberty-setup
nano-liberty-setup:
	@set -e; \
	if command -v shasum >/dev/null 2>&1; then sha='shasum -a 256'; \
	elif command -v sha256sum >/dev/null 2>&1; then sha='sha256sum'; \
	else \
	  echo "neither shasum nor sha256sum is on PATH; refusing to fetch a" >&2; \
	  echo "liberty file this machine cannot verify." >&2; exit 1; \
	fi; \
	if [ -f '$(NANO_LIBERTY)' ] && \
	   [ "$$($$sha '$(NANO_LIBERTY)' | cut -d ' ' -f 1)" = '$(NANO_LIBERTY_SHA256)' ]; then \
	  echo "$(NANO_LIBERTY) already verified."; exit 0; \
	fi; \
	mkdir -p '$(NANO_LIBERTY_DIR)'; \
	tmp=$$(mktemp '$(NANO_LIBERTY_DIR)'/.download.XXXXXX); \
	echo "fetching $(NANO_LIBERTY_URL)"; \
	curl -fsSL -o "$$tmp" '$(NANO_LIBERTY_URL)'; \
	got=$$($$sha "$$tmp" | cut -d ' ' -f 1); \
	if [ "$$got" != '$(NANO_LIBERTY_SHA256)' ]; then \
	  echo "liberty SHA-256 MISMATCH -- refusing to keep it:" >&2; \
	  echo "  expected : $(NANO_LIBERTY_SHA256)" >&2; \
	  echo "  actual   : $$got" >&2; \
	  rm -f "$$tmp"; exit 1; \
	fi; \
	echo "sha256 ok: $$got"; \
	mv "$$tmp" '$(NANO_LIBERTY)'

# A ratchet, not a pin -- unlike NANO_LIBERTY_COMMIT it is meant to move, but only in a
# reviewed commit that edits this line, never from the command line or the environment:
# `NANO_MAX_UM2=nan` would otherwise reach area_report.py's `>` comparison, which is
# false against a non-finite value on either side of it.
ifneq ($(filter command line environment,$(origin NANO_MAX_UM2)),)
$(error NANO_MAX_UM2 cannot be set from the command line or the environment: it is a \
  ratchet, and raising it needs a reason in the commit that edits nano/nano.mk)
endif
# The donor's own measured figure, at c55efd6.
override NANO_MAX_UM2 := 84291

NANO_SRCS := nano/nano.v

.PHONY: nano-area
nano-area: | nano-liberty-setup
	@nano/srcs_guard.sh $(NANO_SRCS); rc=$$?; \
	if [ $$rc -eq 2 ]; then exit 0; fi; \
	if [ $$rc -ne 0 ]; then exit $$rc; fi; \
	yosys -p "$$(nano/synth_script.sh '$(NANO_LIBERTY)' $(NANO_SRCS))" \
	  > nano/area.synth.log 2>&1 || { tail -40 nano/area.synth.log; exit 1; }; \
	python3 nano/area_report.py nano/area.json --liberty '$(NANO_LIBERTY)' \
	  --liberty-sha256 '$(NANO_LIBERTY_SHA256)' --max-um2 '$(NANO_MAX_UM2)'
