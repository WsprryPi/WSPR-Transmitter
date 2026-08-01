# Makefile for building most C++ or C++ and C projects

# Debug flag
DEBUG := -DDEBUG_WSPR_TRANSMIT
# C++ Version
CXXVER := 20
# Prefix for executing tests, "sudo" if required
SUDO := sudo
# Add any required submodules by their src path
SUBMODULE_SRCDIRS := $(wildcard ../../WSPR-Reference/src/wspr)
SUBMODULE_SRCDIRS += $(wildcard ../../Mailbox/src)
SUBMODULE_SRCDIRS += $(wildcard ../../Signal-Handler/src)
SUBMODULE_SRCDIRS += $(wildcard ../external)

# Get project name from Git, revert to directory name
#
# Project name from remote origin URL (fallback to 'unknown' if not available)
PRJ := $(shell git config --local remote.origin.url 2>/dev/null | sed -n 's#.*/\([^.]*\)\.git#\1#p')
# If PRJ is empty, fallback to current directory name
PRJ := $(or $(PRJ),$(shell basename $$(pwd)))
# If PRJ is "src", use the parent directory name
PRJ := $(shell [ "$(PRJ)" = "src" ] && basename $$(dirname $$(pwd)) || echo "$(PRJ)")
# Convert PRJ to lowercase for EXE_NAME
EXE_NAME := $(shell echo $(PRJ) | tr '[:upper:]' '[:lower:]')

# Output Items
OUT := $(EXE_NAME)					# Normal release binary
TEST_OUT :=	$(EXE_NAME)_test		# Debug/test binary
# Strip whitespace from comments
OUT := $(strip $(OUT))
TEST_OUT := $(strip $(TEST_OUT))

# Output directories
OBJ_DIR_RELEASE = build/obj/release
OBJ_DIR_DEBUG   = build/obj/debug
DEP_DIR         = build/dep
BIN_DIR		 	= build/bin

# ────────────────────────────────────────────────────────────────────────────────
# Find your project’s own .c/.cpp (this will pick up ./main.cpp or ./main.c):
LOCAL_C_SOURCES   := $(shell find . -type f -name '*.c')
LOCAL_CPP_SOURCES := $(shell find . -type f -name '*.cpp')
SI5351_TEST_SOURCE := ./si5351_backend_harness.cpp
STANDARD_FELD_TEST_SOURCE := ../tests/standard_feld_compiler_test.cpp
LOCAL_CPP_SOURCES := $(filter-out $(SI5351_TEST_SOURCE) $(STANDARD_FELD_TEST_SOURCE),$(LOCAL_CPP_SOURCES))

# Find in each submodule—but explicitly skip any “main” there:
SUBMODULE_C_SOURCES   := $(shell find $(SUBMODULE_SRCDIRS) -type f -name '*.c'   ! -name 'main.c')
SUBMODULE_CPP_SOURCES := $(shell find $(SUBMODULE_SRCDIRS) -type f -name '*.cpp' ! -name 'main.cpp')

# Combine them:
C_SOURCES   := $(LOCAL_C_SOURCES)   $(SUBMODULE_C_SOURCES)
CPP_SOURCES := $(LOCAL_CPP_SOURCES) $(SUBMODULE_CPP_SOURCES)

# Strip any leading ../../ from the submodule paths so your object rules work:
REL_C_SOURCES   := $(patsubst ../../%,%,$(C_SOURCES))
REL_CPP_SOURCES := $(patsubst ../../%,%,$(CPP_SOURCES))

# Regenerate your object‐lists exactly as before:
C_OBJECTS         := $(patsubst %.c,   $(OBJ_DIR_RELEASE)/%.o,$(REL_C_SOURCES))
CPP_OBJECTS       := $(patsubst %.cpp, $(OBJ_DIR_RELEASE)/%.o,$(REL_CPP_SOURCES))
C_DEBUG_OBJECTS   := $(patsubst %.c,   $(OBJ_DIR_DEBUG)/%.o,$(REL_C_SOURCES))
CPP_DEBUG_OBJECTS := $(patsubst %.cpp, $(OBJ_DIR_DEBUG)/%.o,$(REL_CPP_SOURCES))
SI5351_TEST_OBJECTS := \
	$(OBJ_DIR_DEBUG)/./si5351_backend_harness.o \
	$(OBJ_DIR_DEBUG)/./wspr_transmit_backend_si5351.o \
	$(OBJ_DIR_DEBUG)/./si5351_device.o \
	$(OBJ_DIR_DEBUG)/./si5351_planner.o
STANDARD_FELD_TEST_OUT := build/bin/standard_feld_compiler_test
STANDARD_FELD_GPIO_DRY_RUN_TEST_OUT := build/bin/standard_feld_gpio_dry_run_test
STANDARD_FELD_RPI_EXECUTION_TEST_OUT := build/bin/standard_feld_rpi_execution_test
STANDARD_FELD_GATE_TEST_OUT := build/bin/standard_feld_execution_gate_test
STANDARD_FELD_PROGRESS_BRIDGE_TEST_OUT := build/bin/standard_feld_progress_bridge_test
# ────────────────────────────────────────────────────────────────────────────────

# Linker Flags
LDFLAGS := -lpthread  -latomic
# Get packages for linker from PKG_CONFIG_PATH
# LDFLAGS += $(shell pkg-config --cflags --libs libgpiod)
# LDFLAGS += $(shell pkg-config --libs libgpiodcxx)
# Strip whitespace from LDFLAGS
LDFLAGS := $(strip $(LDFLAGS))

# Collect dependency files
DEPFILES := $(wildcard $(DEP_DIR)/**/*.d)
-include $(DEPFILES)

# Compiler Executables
CXX = g++
CC = gcc

# Allow threading using all available processors
MAKEFLAGS := -j$(nproc)

# Compiler Flags
#
# Common flags for both C and C++ compilers
COMMON_FLAGS := -Wall -Werror -fmax-errors=10 -MMD -MP

# C Flags
CFLAGS := $(COMMON_FLAGS)
# C Debug Flags
C_DEBUG_FLAGS := $(CFLAGS) -g $(DEBUG)	# Debug flags
# C Release Flags
C_RELEASE_FLAGS := $(CFLAGS) -O2		# Release optimized

# C++ Flags
CXXFLAGS := -Wno-psabi -lstdc++fs -std=c++$(CXXVER)
CXXFLAGS += $(COMMON_FLAGS) $(COMM_CXX_FLAGS)

# Include paths
CXXFLAGS += -I$(abspath .)
CXXFLAGS += $(foreach dir,$(SUBMODULE_SRCDIRS),-I$(abspath $(dir)))
CXXFLAGS += -I$(abspath ../../WSPR-Reference/src/wspr)

# C++ Debug Flags
CXX_DEBUG_FLAGS := $(CXXFLAGS) -g $(DEBUG)	# Debug flags
# C++ Release Flags
CXX_RELEASE_FLAGS := $(CXXFLAGS) -O2		# Release optimized

# Strip whitespaces
C_DEBUG_FLAGS := $(strip $(C_DEBUG_FLAGS))
C_RELEASE_FLAGS := $(strip $(C_RELEASE_FLAGS))
CXX_DEBUG_FLAGS := $(strip $(CXX_DEBUG_FLAGS))
CXX_RELEASE_FLAGS := $(strip $(CXX_RELEASE_FLAGS))

# Enable verbose output if VERBOSE=1 is specified during the build
VERBOSE ?= 0
ifeq ($(VERBOSE), 1)
	Q :=
else
	Q := @
endif

##
# Compile & link rules
##

# Generic debug compile (your own code)
$(OBJ_DIR_DEBUG)/%.o: %.c
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling (debug) $< into $@"
	$(Q)$(CC) $(C_DEBUG_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

$(OBJ_DIR_DEBUG)/%.o: %.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling (debug) $< into $@"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

# Auto‑generate debug compile rules for each submodule
define MAKE_DEBUG_SUBMOD_RULES
$(OBJ_DIR_DEBUG)/$(patsubst ../../%,%,$(1))/%.o: $(1)/%.cpp
	$(Q)mkdir -p $$(@D)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)echo "Compiling (debug) $$< into $$@"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) -MF $(DEP_DIR)/$$*.d -c $$< -o $$@

$(OBJ_DIR_DEBUG)/$(patsubst ../../%,%,$(1))/%.o: $(1)/%.c
	$(Q)mkdir -p $$(@D)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)echo "Compiling (debug) $$< into $$@"
	$(Q)$(CC)  $(C_DEBUG_FLAGS)   -MF $(DEP_DIR)/$$*.d -c $$< -o $$@
endef

$(foreach dir,$(SUBMODULE_SRCDIRS),$(eval $(call MAKE_DEBUG_SUBMOD_RULES,$(dir))))

# Link debug binary
build/bin/$(TEST_OUT): $(CPP_DEBUG_OBJECTS) $(C_DEBUG_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking debug: $(TEST_OUT)"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

# Link temporary Si5351 backend harness
build/bin/si5351_backend_harness: $(SI5351_TEST_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking debug: si5351_backend_harness"
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

$(STANDARD_FELD_TEST_OUT): ../tests/standard_feld_compiler_test.cpp \
		execution_plan_compiler.cpp standard_feld.cpp
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) $^ -o $@ $(LDFLAGS)

$(STANDARD_FELD_GPIO_DRY_RUN_TEST_OUT): \
		../tests/standard_feld_gpio_dry_run_test.cpp \
		../tests/rpi_standard_feld_dry_run.cpp \
		../tests/rpi_standard_feld_dry_run.hpp \
		execution_plan_compiler.cpp standard_feld.cpp \
		rpi_standard_feld_execution.cpp rpi_standard_feld_execution.hpp \
		execution_plan.hpp execution_plan_compiler.hpp \
		standard_feld.hpp standard_feld_asset.hpp \
		transmission_payloads.hpp transmission_request.hpp
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) \
		../tests/standard_feld_gpio_dry_run_test.cpp \
		../tests/rpi_standard_feld_dry_run.cpp \
		execution_plan_compiler.cpp standard_feld.cpp \
		rpi_standard_feld_execution.cpp \
		-o $@ $(LDFLAGS)

$(STANDARD_FELD_RPI_EXECUTION_TEST_OUT): \
		../tests/standard_feld_rpi_execution_test.cpp \
		rpi_standard_feld_execution.cpp rpi_standard_feld_execution.hpp \
		transmission_controller.cpp transmission_controller.hpp \
		transmission_backend.hpp wspr_transmit.hpp wspr_transmit_types.hpp \
		execution_plan_compiler.cpp execution_plan_compiler.hpp \
		standard_feld.cpp standard_feld.hpp standard_feld_asset.hpp \
		execution_plan.hpp transmission_payloads.hpp transmission_request.hpp
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) \
		../tests/standard_feld_rpi_execution_test.cpp \
		rpi_standard_feld_execution.cpp \
		transmission_controller.cpp \
		execution_plan_compiler.cpp standard_feld.cpp \
		-o $@ $(LDFLAGS)

$(STANDARD_FELD_GATE_TEST_OUT): ../tests/standard_feld_execution_gate_test.cpp \
		standard_feld_execution_gate.cpp standard_feld_execution_gate.hpp
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) -DSTANDARD_FELD_GATE_TEST_HOOKS $< standard_feld_execution_gate.cpp -o $@ $(LDFLAGS)

$(STANDARD_FELD_PROGRESS_BRIDGE_TEST_OUT): ../tests/standard_feld_progress_bridge_test.cpp rpi_standard_feld_progress_bridge.cpp rpi_standard_feld_progress_bridge.hpp rpi_standard_feld_execution.cpp rpi_standard_feld_execution.hpp execution_plan_compiler.cpp standard_feld.cpp
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CXX) $(CXX_DEBUG_FLAGS) -DSTANDARD_FELD_PROGRESS_TEST_HOOKS $< rpi_standard_feld_progress_bridge.cpp rpi_standard_feld_execution.cpp execution_plan_compiler.cpp standard_feld.cpp -o $@ $(LDFLAGS)

# Generic release compile (your own code)
$(OBJ_DIR_RELEASE)/%.o: %.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling (release) $< into $@"
	$(Q)$(CXX) $(CXX_RELEASE_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

$(OBJ_DIR_RELEASE)/%.o: %.c
	$(Q)mkdir -p $(dir $@)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $*)
	$(Q)echo "Compiling (release) $< into $@"
	$(Q)$(CC) $(C_RELEASE_FLAGS) -MF $(DEP_DIR)/$*.d -c $< -o $@

# Auto‑generate release compile rules for each submodule
define MAKE_RELEASE_SUBMOD_RULES
$(OBJ_DIR_RELEASE)/$(patsubst ../../%,%,$(1))/%.o: $(1)/%.cpp
	$(Q)mkdir -p $$(@D)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)echo "Compiling (release) $$< into $$@"
	$(Q)$(CXX) $(CXX_RELEASE_FLAGS) -MF $(DEP_DIR)/$$*.d -c $$< -o $$@

$(OBJ_DIR_RELEASE)/$(patsubst ../../%,%,$(1))/%.o: $(1)/%.c
	$(Q)mkdir -p $$(@D)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)mkdir -p $(DEP_DIR)/$(dir $$*)
	$(Q)echo "Compiling (release) $$< into $$@"
	$(Q)$(CC)  $(C_RELEASE_FLAGS)   -MF $(DEP_DIR)/$$*.d -c $$< -o $$@
endef

$(foreach dir,$(SUBMODULE_SRCDIRS),$(eval $(call MAKE_RELEASE_SUBMOD_RULES,$(dir))))

# Link release binary
build/bin/$(OUT): $(CPP_OBJECTS) $(C_OBJECTS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)echo "Linking release: $(OUT)"
	$(Q)$(CXX) $(CXX_RELEASE_FLAGS) $^ -o $@ $(LDFLAGS)

##
# Phony targets
##
.PHONY: clean release debug test si5351-test standard-feld-test standard-feld-gpio-dry-run-test standard-feld-rpi-execution-test standard-feld-execution-gate-test standard-feld-progress-bridge-test gdb lint macros help

clean:
	$(Q)echo "Cleaning up build artifacts."
	$(Q)rm -rf build

release: build/bin/$(OUT)
	$(Q)echo "Release build completed successfully."

debug: build/bin/$(TEST_OUT)
	$(Q)echo "Debug build completed successfully."

test: debug
	$(Q)$(SUDO) ./build/bin/$(TEST_OUT) --now --oneshot

si5351-test: build/bin/si5351_backend_harness
	$(Q)echo "Si5351 backend harness built successfully."

standard-feld-test: $(STANDARD_FELD_TEST_OUT)
	$(Q)./$(STANDARD_FELD_TEST_OUT)

standard-feld-gpio-dry-run-test: $(STANDARD_FELD_GPIO_DRY_RUN_TEST_OUT)
	$(Q)./$(STANDARD_FELD_GPIO_DRY_RUN_TEST_OUT)

standard-feld-rpi-execution-test: $(STANDARD_FELD_RPI_EXECUTION_TEST_OUT)
	$(Q)./$(STANDARD_FELD_RPI_EXECUTION_TEST_OUT)

standard-feld-execution-gate-test: $(STANDARD_FELD_GATE_TEST_OUT)
	$(Q)./$(STANDARD_FELD_GATE_TEST_OUT)

standard-feld-progress-bridge-test: $(STANDARD_FELD_PROGRESS_BRIDGE_TEST_OUT)
	$(Q)./$(STANDARD_FELD_PROGRESS_BRIDGE_TEST_OUT)

watchdog: debug
	$(Q)$(SUDO) env WSPR_TX_INJECT_WD_STALL=5 ./build/bin/$(TEST_OUT) --now --oneshot

gdb: debug
	$(Q)$(SUDO) gdb --args ./build/bin/$(TEST_OUT)

lint:
	$(Q)command -v cppcheck >/dev/null 2>&1 || { \
	  echo "Warning: cppcheck not installed."; exit 1; }
	$(Q)echo "Running cppcheck…"
	$(Q)cppcheck --std=c++$(CXXVER) --enable=all --inconclusive \
	      --force --inline-suppr --quiet $(CPP_SOURCES)

macros:
	$(Q)echo "Project macros:"
	$(Q)$(CXX) $(CXXFLAGS) -dM -E -x c++ /dev/null \
	      | grep -v '^#define _' || true

help:
	$(Q)echo "Available targets:"
	$(Q)echo "  release    Build optimized binary"
	$(Q)echo "  debug      Build debug binary"
	$(Q)echo "  test       Run tests"
	$(Q)echo "  si5351-test Build temporary Si5351 backend harness"
	$(Q)echo "  standard-feld-test Build and run backend-neutral Standard Feld tests"
	$(Q)echo "  watchdog   Run tests with watchdog fail"
	$(Q)echo "  gdb        Debug with gdb"
	$(Q)echo "  lint       Static analysis"
	$(Q)echo "  macros     Show macros"
	$(Q)echo "  clean      Remove build artifacts"
	$(Q)echo "  help       This message"
