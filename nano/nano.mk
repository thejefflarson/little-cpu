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

# `dfflibmap` maps flip-flops only; a latch left generic is priced at zero by `stat -liberty` without this techmap stub.
override NANO_LATCHMAP_SHA256 := 5f28f158599cd6548fdf302fa3dbe2e306cf0aba431884f1c9b6305589cd560f
override NANO_LATCHMAP_URL := https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD-flow-scripts/$(NANO_LIBERTY_COMMIT)/flow/platforms/sky130hd/cells_latch_hd.v
NANO_LATCHMAP := $(NANO_LIBERTY_DIR)/cells_latch_hd.v

.PHONY: nano-liberty-setup
nano-liberty-setup:
	@set -e; \
	if command -v shasum >/dev/null 2>&1; then sha='shasum -a 256'; \
	elif command -v sha256sum >/dev/null 2>&1; then sha='sha256sum'; \
	else \
	  echo "neither shasum nor sha256sum is on PATH; refusing to fetch a" >&2; \
	  echo "liberty file this machine cannot verify." >&2; exit 1; \
	fi; \
	fetch() { \
	  url=$$1; dest=$$2; digest=$$3; \
	  if [ -f "$$dest" ] && [ "$$($$sha "$$dest" | cut -d ' ' -f 1)" = "$$digest" ]; then \
	    echo "$$dest already verified."; return 0; \
	  fi; \
	  mkdir -p "$$(dirname "$$dest")"; \
	  tmp=$$(mktemp "$$(dirname "$$dest")"/.download.XXXXXX); \
	  echo "fetching $$url"; \
	  curl -fsSL -o "$$tmp" "$$url"; \
	  got=$$($$sha "$$tmp" | cut -d ' ' -f 1); \
	  if [ "$$got" != "$$digest" ]; then \
	    echo "SHA-256 MISMATCH for $$dest -- refusing to keep it:" >&2; \
	    echo "  expected : $$digest" >&2; \
	    echo "  actual   : $$got" >&2; \
	    rm -f "$$tmp"; return 1; \
	  fi; \
	  echo "sha256 ok: $$got"; \
	  mv "$$tmp" "$$dest"; \
	}; \
	rc=0; \
	fetch '$(NANO_LIBERTY_URL)' '$(NANO_LIBERTY)' '$(NANO_LIBERTY_SHA256)' || rc=1; \
	fetch '$(NANO_LATCHMAP_URL)' '$(NANO_LATCHMAP)' '$(NANO_LATCHMAP_SHA256)' || rc=1; \
	exit $$rc

# A ratchet, moved only in a reviewed commit: `NANO_MAX_UM2=nan` would otherwise beat area_report.py's `>` comparison, which is false against any non-finite value. Stepped for the QSPI front end: nano.v + nano/qspi.v, wired through nano/area_top.v since yosys's auto-top drops one of two unconnected modules, measure 60,859.6 um2 against nano.v alone's unchanged 45,712.6.
override NANO_MAX_UM2 := 63000

NANO_SRCS := nano/nano.v nano/qspi.v nano/area_top.v

.PHONY: nano-area
nano-area:
	@nano/srcs_guard.sh $(NANO_SRCS); rc=$$?; \
	if [ $$rc -eq 2 ]; then exit 0; fi; \
	if [ $$rc -ne 0 ]; then exit $$rc; fi; \
	$(MAKE) --no-print-directory nano-liberty-setup; \
	yosys -p "$$(nano/synth_script.sh '$(NANO_LIBERTY)' $(NANO_SRCS))" \
	  > nano/area.synth.log 2>&1 || { tail -40 nano/area.synth.log; exit 1; }; \
	python3 nano/area_report.py nano/area.json --liberty '$(NANO_LIBERTY)' \
	  --liberty-sha256 '$(NANO_LIBERTY_SHA256)' --max-um2 '$(NANO_MAX_UM2)'

# Area and delay both come out of one synthesis run per register-file build; no ratchet, since this ranks RTL versions rather than gating either figure.
.PHONY: nano-timing
nano-timing:
	@nano/srcs_guard.sh $(NANO_SRCS); rc=$$?; \
	if [ $$rc -eq 2 ]; then exit 0; fi; \
	if [ $$rc -ne 0 ]; then exit $$rc; fi; \
	$(MAKE) --no-print-directory nano-liberty-setup; \
	yosys -p "$$(nano/timing_script.sh '$(NANO_LIBERTY)' '$(NANO_LATCHMAP)' nano/timing.flops.json '' $(NANO_SRCS))" \
	  > nano/timing.flops.log 2>&1 & pid_flops=$$!; \
	yosys -p "$$(nano/timing_script.sh '$(NANO_LIBERTY)' '$(NANO_LATCHMAP)' nano/timing.latches.json NANO_LATCH_RF $(NANO_SRCS))" \
	  > nano/timing.latches.log 2>&1 & pid_latches=$$!; \
	rc=0; \
	wait $$pid_flops || { tail -40 nano/timing.flops.log; rc=1; }; \
	wait $$pid_latches || { tail -40 nano/timing.latches.log; rc=1; }; \
	[ $$rc -eq 0 ] || exit 1; \
	python3 nano/timing_report.py --liberty '$(NANO_LIBERTY)' \
	  --liberty-sha256 '$(NANO_LIBERTY_SHA256)' \
	  --variant flops:nano/timing.flops.log:nano/timing.flops.json \
	  --variant latches:nano/timing.latches.log:nano/timing.latches.json \
	  --flow-correlation nano/timing_flow_correlation.json
