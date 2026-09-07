# The sky130hd liberty nanocpu synthesises against, pinned the way formal/pin.mk pins riscv-formal: a raw URL at a commit, SHA-256-verified before anything reads it.
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

# 84510 = 84291 + 219: the donor's measured area and the churn band measured on this tree across six functionally identical spellings of it.
NANO_MAX_UM2 := 84510

NANO_SRCS := nano/nano.v

nano/area.json: $(NANO_SRCS)
	@yosys -p 'read_verilog -sv $(NANO_SRCS); hierarchy -auto-top; synth; dfflibmap -liberty $(NANO_LIBERTY); abc -liberty $(NANO_LIBERTY); tee -o $@ stat -liberty $(NANO_LIBERTY) -json' > nano/area.synth.log 2>&1 || { tail -40 nano/area.synth.log; exit 1; }

.PHONY: nano-area
nano-area: nano/area.json
	@python3 nano/area_report.py $< --liberty $(NANO_LIBERTY) \
	  --liberty-sha256 $(NANO_LIBERTY_SHA256) --max-um2 $(NANO_MAX_UM2)
