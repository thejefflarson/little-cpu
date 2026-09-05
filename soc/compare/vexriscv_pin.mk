# The VexRiscv this harness compares against, and WHY IT IS GENERATED HERE
# rather than taken from the riscv-formal clone.
#
# riscv-formal's cores/VexRiscv/VexRiscv.v is FormalSimple -- its own
# VERIFICATION configuration. It has no MulPlugin, no CsrPlugin, and every
# hazard bypass disabled. Measured against this core's RV32IMAC_Zicsr_Zifencei
# _Zkt with traps, CSRs, a timer and executor-only forwarding, that is not a
# peer, and it distorts BOTH halves of the product at once: it flatters
# VexRiscv on period (no bypass network, no CSR file, no multiplier to place)
# and flatters this core on cycles (nothing forwards, so everything stalls).
# It measured 1021.9 cycles per Dhrystone. The configuration below measures
# 640.1 -- BETTER than this core's 731.1 -- so the cycle "win" the old harness
# reported was an artifact of the opponent's config and not a property of
# either design.
#
# A GENERATED ARTIFACT IS ONLY REPRODUCIBLE WITH ITS GENERATOR AND ITS CONFIG,
# so both are pinned: the upstream commit below, and the .scala beside the .v.
# Regenerating means checking out that commit, dropping the .scala into
# src/main/scala/vexriscv/demo/, and `sbt "runMain
# vexriscv.demo.GenLittleCpuCompare"`. It needs a JDK (17 works; upstream's
# README says 8) and sbt, which is why the OUTPUT is vendored: no contributor
# should need a JVM to run `make compare-timing`.
VEXRISCV_SHA     := c4b2a55b22f46afb760446cd69f3ca7c36eef778
VEXRISCV_SPINAL  := 1.13.0
VEXRISCV_V       := soc/compare/vexriscv/VexRiscv.v
VEXRISCV_GEN     := soc/compare/vexriscv/GenLittleCpuCompare.scala
VEXRISCV_SHA256  := 03ce8bafc0f9ed21167a0b16ddd7ca51d62ac6aed234c09ecef4923b2e5b9a52

# The vendored Verilog is graded against its own digest, the way COREMARK_PIN
# grades the vendored algorithm files: a hand-edit to generated output is a
# silent fork of a core nobody can regenerate.
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
