print-%:
	@echo '$($*)'

TOOL_CACHE := $(if $(XDG_CACHE_HOME),$(XDG_CACHE_HOME),$(HOME)/.cache)/little-cpu

ifndef OSS_CAD_TAG
$(error mk/oss-cad-pin.mk has to be included before mk/toolchain.mk)
endif

OSS_CAD_PLATFORM_Darwin_arm64  := darwin-arm64
OSS_CAD_PLATFORM_Linux_aarch64 := linux-arm64
OSS_CAD_PLATFORM_Linux_x86_64  := linux-x64
OSS_CAD_PLATFORM := $(OSS_CAD_PLATFORM_$(shell uname -s)_$(shell uname -m))
OSS_CAD_ASSET    := oss-cad-suite-$(OSS_CAD_PLATFORM)-$(subst -,,$(OSS_CAD_TAG))
OSS_CAD_SHA256   := $(OSS_CAD_SHA256_$(OSS_CAD_ASSET))
OSS_CAD_DIR      := $(TOOL_CACHE)/oss-cad-suite-$(OSS_CAD_TAG)
OSS_CAD_BIN      := $(OSS_CAD_DIR)/bin

# TOOLS_ON_PATH opts out, for a caller that built PATH on purpose (probe fixtures do).
RISCV_GCC_BIN := $(TOOL_CACHE)/riscv-gcc/bin
ifndef TOOLS_ON_PATH
ifneq ($(wildcard $(OSS_CAD_BIN)/yosys),)
export PATH := $(OSS_CAD_BIN):$(PATH)
endif
ifneq ($(wildcard $(RISCV_GCC_BIN)/riscv-none-elf-gcc),)
export PATH := $(RISCV_GCC_BIN):$(PATH)
endif
endif
