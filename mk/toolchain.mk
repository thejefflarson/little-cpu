print-%:
	@echo '$($*)'

TOOL_CACHE := $(if $(XDG_CACHE_HOME),$(XDG_CACHE_HOME),$(HOME)/.cache)/little-cpu

# TOOLS_ON_PATH opts out, for a caller that built PATH on purpose (probe fixtures do).
OSS_CAD_BIN := $(TOOL_CACHE)/oss-cad-suite/bin
ifneq ($(wildcard $(OSS_CAD_BIN)/yosys),)
ifndef TOOLS_ON_PATH
export PATH := $(OSS_CAD_BIN):$(PATH)
endif
endif
