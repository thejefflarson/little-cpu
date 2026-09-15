print-%:
	@echo '$($*)'

TOOL_CACHE := $(if $(XDG_CACHE_HOME),$(XDG_CACHE_HOME),$(HOME)/.cache)/little-cpu

# TOOLS_ON_PATH opts out, for a caller that built PATH on purpose (probe fixtures do).
OSS_CAD_BIN   := $(TOOL_CACHE)/oss-cad-suite/bin
RISCV_GCC_BIN := $(TOOL_CACHE)/riscv-gcc/bin
ifndef TOOLS_ON_PATH
ifneq ($(wildcard $(OSS_CAD_BIN)/yosys),)
export PATH := $(OSS_CAD_BIN):$(PATH)
endif
ifneq ($(wildcard $(RISCV_GCC_BIN)/riscv-none-elf-gcc),)
export PATH := $(RISCV_GCC_BIN):$(PATH)
endif
endif
