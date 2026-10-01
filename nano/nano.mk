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

# The cells the Tiny Tapeout flow's LibreLane excludes from synthesis: open_pdks' two lists at the PDK commit that flow resolves.
ifneq ($(filter command line environment,$(origin NANO_PDK_COMMIT)),)
$(error NANO_PDK_COMMIT cannot be set from the command line or the environment: change it \
  in nano/nano.mk, together with the two SHA-256 digests below it)
endif
override NANO_PDK_COMMIT := 8afc8346a57fe1ab7934ba5a6056ea8b43078e71
override NANO_NO_SYNTH_SHA256 := 8bd5ee6d949870fd389d177d4b987eeb4d22b55614eea3de8a8f6705fd8982be
override NANO_DRC_EXCLUDE_SHA256 := 8785391a0540d4b96b52b242dc57bf337860e607eed09a25664673f710a9afb7
override NANO_PDK_CELLS_URL := https://raw.githubusercontent.com/RTimothyEdwards/open_pdks/$(NANO_PDK_COMMIT)/sky130/openlane/sky130_fd_sc_hd

NANO_NO_SYNTH       := $(NANO_LIBERTY_DIR)/no_synth.cells
NANO_DRC_EXCLUDE    := $(NANO_LIBERTY_DIR)/drc_exclude.cells
NANO_EXCLUDED_CELLS := $(NANO_LIBERTY_DIR)/nano_excluded.cells

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
	fetch '$(NANO_PDK_CELLS_URL)/no_synth.cells' '$(NANO_NO_SYNTH)' '$(NANO_NO_SYNTH_SHA256)' || rc=1; \
	fetch '$(NANO_PDK_CELLS_URL)/drc_exclude.cells' '$(NANO_DRC_EXCLUDE)' '$(NANO_DRC_EXCLUDE_SHA256)' || rc=1; \
	if [ $$rc -eq 0 ]; then \
	  cat '$(NANO_NO_SYNTH)' '$(NANO_DRC_EXCLUDE)' | sort -u > '$(NANO_EXCLUDED_CELLS)'; \
	fi; \
	exit $$rc

# A ratchet, moved only in a reviewed commit: `NANO_MAX_UM2=nan` would otherwise beat area_report.py's `>` comparison, which is false against any non-finite value. Re-derived when the instrument's flip-flop exclusions started taking effect: 75,082.0 um2 against 73,137.6 on the same RTL, with the prior step's 2,034.3 um2 of headroom. A ranking between RTL versions, never a fit, which only a flow run with gate-level simulation says.
override NANO_MAX_UM2 := 77100

NANO_SRCS := nano/nano.v nano/qspi.v nano/uart.v nano/gpio.v nano/bus.v \
             nano/tt/src/tt_um_thejefflarson_nanocpu.v

.PHONY: nano-area
nano-area:
	@nano/srcs_guard.sh $(NANO_SRCS); rc=$$?; \
	if [ $$rc -eq 2 ]; then exit 0; fi; \
	if [ $$rc -ne 0 ]; then exit $$rc; fi; \
	$(MAKE) --no-print-directory nano-liberty-setup; \
	yosys -p "$$(nano/synth_script.sh '$(NANO_LIBERTY)' '$(NANO_EXCLUDED_CELLS)' $(NANO_SRCS))" \
	  > nano/area.synth.log 2>&1 || { tail -40 nano/area.synth.log; exit 1; }; \
	python3 nano/area_report.py nano/area.json --liberty '$(NANO_LIBERTY)' \
	  --liberty-sha256 '$(NANO_LIBERTY_SHA256)' --max-um2 '$(NANO_MAX_UM2)' \
	  --excluded '$(NANO_EXCLUDED_CELLS)'

# Area and delay both come out of one synthesis run; no ratchet, since this ranks RTL versions against each other rather than gating either figure.
.PHONY: nano-timing
nano-timing:
	@nano/srcs_guard.sh $(NANO_SRCS); rc=$$?; \
	if [ $$rc -eq 2 ]; then exit 0; fi; \
	if [ $$rc -ne 0 ]; then exit $$rc; fi; \
	$(MAKE) --no-print-directory nano-liberty-setup; \
	yosys -p "$$(nano/timing_script.sh '$(NANO_LIBERTY)' '$(NANO_EXCLUDED_CELLS)' nano/timing.flops.json $(NANO_SRCS))" \
	  > nano/timing.flops.log 2>&1 || { tail -40 nano/timing.flops.log; exit 1; }; \
	python3 nano/timing_report.py --liberty '$(NANO_LIBERTY)' \
	  --liberty-sha256 '$(NANO_LIBERTY_SHA256)' --excluded '$(NANO_EXCLUDED_CELLS)' \
	  --variant flops:nano/timing.flops.log:nano/timing.flops.json \
	  --flow-correlation nano/timing_flow_correlation.json

# The sky130_fd_sc_hd behavioral Verilog a gate-level simulation reads, pinned like the liberty above.
ifneq ($(filter command line environment,$(origin NANO_SKY130_VERILOG_COMMIT)),)
$(error NANO_SKY130_VERILOG_COMMIT cannot be set from the command line or the \
  environment: it pins bytes this repo executes. Change it in nano/nano.mk, \
  together with the SHA-256 digest below it)
endif
override NANO_SKY130_VERILOG_COMMIT := ac7fb61f06e6470b94e8afdf7c25268f62fbd7b1

ifeq ($(shell printf '%s' '$(NANO_SKY130_VERILOG_COMMIT)' | grep -cE '^[0-9a-f]{40}$$'),0)
$(error NANO_SKY130_VERILOG_COMMIT must be a full 40-hex commit id, not a branch or tag: \
  '$(NANO_SKY130_VERILOG_COMMIT)')
endif

override NANO_SKY130_VERILOG_SHA256 := c613384ff89ea065c0d91e31db223471d7d70e546f6972c3b96f2abb8e7a8faf
override NANO_SKY130_VERILOG_URL := https://codeload.github.com/google/skywater-pdk-libs-sky130_fd_sc_hd/tar.gz/$(NANO_SKY130_VERILOG_COMMIT)

NANO_SKY130_VERILOG_DIR := $(TOOL_CACHE)/sky130-fd-sc-hd-verilog

.PHONY: nano-sky130-verilog-setup
nano-sky130-verilog-setup:
	@./nano/sky130_verilog_setup.sh '$(NANO_SKY130_VERILOG_URL)' '$(NANO_SKY130_VERILOG_SHA256)' \
	  '$(NANO_SKY130_VERILOG_DIR)'
