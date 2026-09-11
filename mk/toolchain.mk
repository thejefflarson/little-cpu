# Shared by the root Makefile and every sub-make that invokes yosys or sby directly
# (formal/Makefile, nano/formal/Makefile) -- `make -C formal ...` starts a fresh make
# with no memory of the root Makefile's PATH, so each includes this on its own.

# Outside the checkout, because a worktree gets tracked files only and a tool installed
# inside one is invisible from every other.
TOOL_CACHE := $(if $(XDG_CACHE_HOME),$(XDG_CACHE_HOME),$(HOME)/.cache)/little-cpu

# TOOLS_ON_PATH opts out, for a caller that built PATH on purpose (probe fixtures do).
OSS_CAD_BIN := $(TOOL_CACHE)/oss-cad-suite/bin
ifneq ($(wildcard $(OSS_CAD_BIN)/yosys),)
ifndef TOOLS_ON_PATH
export PATH := $(OSS_CAD_BIN):$(PATH)
endif
endif
