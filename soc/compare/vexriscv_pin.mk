VEXRISCV_SHA     := c4b2a55b22f46afb760446cd69f3ca7c36eef778
VEXRISCV_SPINAL  := 1.13.0
VEXRISCV_V       := soc/compare/vexriscv/VexRiscv.v
VEXRISCV_GEN     := soc/compare/vexriscv/GenLittleCpuCompare.scala
VEXRISCV_SHA256  := 03ce8bafc0f9ed21167a0b16ddd7ca51d62ac6aed234c09ecef4923b2e5b9a52

.PHONY: vexriscv-pin-check
vexriscv-pin-check:
	@got=$$(shasum -a 256 $(VEXRISCV_V) | cut -d' ' -f1); \
	if [ "$$got" != "$(VEXRISCV_SHA256)" ]; then \
	  echo "*** $(VEXRISCV_V) is $$got, not the pinned $(VEXRISCV_SHA256)."; \
	  echo "*** That file is GENERATED output. If it needs to change, change"; \
	  echo "*** $(VEXRISCV_GEN), regenerate at VEXRISCV_SHA, and move the"; \
	  echo "*** digest here in the same commit -- a hand-edit forks a core"; \
	  echo "*** nobody can reproduce."; \
	  exit 1; \
	fi
	@echo "$(VEXRISCV_V): matches the pin, generated from $(VEXRISCV_GEN) at $(VEXRISCV_SHA)"
